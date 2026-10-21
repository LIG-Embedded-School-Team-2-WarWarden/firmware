#include "runtime.hpp"
#include "acquisition.hpp"
#include <fstream>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>
#include "client.hpp"
#include "motor.hpp"
#include "capture.hpp"
#include <algorithm>
#include <atomic>
#include <csignal>
#include <future>
#include <optional>
#include <thread>

using namespace device;
static volatile std::sig_atomic_t running=1;
static void signal_stop(int){running=0;}
struct Options {
    bool simulation=false,clock_verified=false;
    std::string role="eff",id="201",c2="127.0.0.1",uart,profile,calibration;
    unsigned short port=0,c2_port=5000;unsigned baud=115200;
    double uncertainty=0;Calibration axes;lidar::Config sensor;
    unsigned point_timeout_ms=2000,stop_timeout_ms=2000,settle_ms=150;
    double tolerance_deg=.5;
    std::optional<AcquisitionProfile> acquisition;
};
static Options options(int argc,char** argv,const std::string& role){
    std::map<std::string,std::string> args;bool simulation=false,clock=false;
    for(int i=1;i<argc;++i){const std::string key=argv[i];if(key=="--simulate"){simulation=true;continue;}if(key=="--clock-verified"){clock=true;continue;}
        if(key=="--help"){std::cout<<"mfs_obs | mfs_eff --simulate --c2-address IPv4 --c2-port 5000 [--device-id ID] [--listen-port PORT]\nReal Linux: --uart DEVICE --baud N --calibration-id ID --pan-zero N --tilt-zero N --ticks-per-turn N --pan-sign +/-1 --tilt-sign +/-1 --pan-min-ticks N --pan-max-ticks N --tilt-min-ticks N --tilt-max-ticks N --point-timeout-ms N --stop-timeout-ms N --settle-ms N --tolerance-deg N [--clock-verified --clock-uncertainty-ms N]\nReal OBS additionally: --sensor-ip IP --board-ip IP --sensor-profile ID [--acquisition-profile JSON], build with LIDAR_WITH_XTSDK=ON.\n";std::exit(0);}
        if(key.rfind("--",0)!=0 || i+1==argc)throw std::invalid_argument("unknown/incomplete option "+key);args[key]=argv[++i];}
    Options o;o.simulation=simulation;o.clock_verified=simulation||clock;
    auto get=[&](const char* key,const std::string& fallback={}){auto i=args.find(key);return i==args.end()?fallback:i->second;};
    auto required=[&](const char* key){const auto value=get(key);if(value.empty())throw std::invalid_argument(std::string("real device requires ")+key);return value;};
    o.role=role;if(args.count("--role"))throw std::invalid_argument("role is fixed by executable; use mfs_obs or mfs_eff");
    o.id=get("--device-id",o.role=="obs"?"101":"201");o.c2=get("--c2-address","127.0.0.1");
    const auto c2port=std::stoul(get("--c2-port","5000")),port=std::stoul(get("--listen-port","0"));
    if(c2port<1||c2port>65535||port>65535)throw std::invalid_argument("UDP port out of range");o.c2_port=static_cast<unsigned short>(c2port);o.port=static_cast<unsigned short>(port);
    if(simulation){if(o.c2!="127.0.0.1")throw std::invalid_argument("simulation is loopback only");o.uncertainty=1;o.axes.id="synthetic-"+o.role+"-zero-v1";o.profile="synthetic-organized-v1";}
    else{
        o.uart=required("--uart");o.baud=std::stoul(get("--baud","115200"));o.axes.id=required("--calibration-id");
        o.axes.pan_zero=std::stod(required("--pan-zero"));o.axes.tilt_zero=std::stod(required("--tilt-zero"));o.axes.ticks_per_turn=std::stod(required("--ticks-per-turn"));
        o.axes.pan_sign=std::stod(required("--pan-sign"));o.axes.tilt_sign=std::stod(required("--tilt-sign"));
        o.axes.pan_min=std::stoi(required("--pan-min-ticks"));o.axes.pan_max=std::stoi(required("--pan-max-ticks"));
        o.axes.tilt_min=std::stoi(required("--tilt-min-ticks"));o.axes.tilt_max=std::stoi(required("--tilt-max-ticks"));
        o.point_timeout_ms=std::stoul(required("--point-timeout-ms"));o.stop_timeout_ms=std::stoul(required("--stop-timeout-ms"));
        o.settle_ms=std::stoul(required("--settle-ms"));o.tolerance_deg=std::stod(required("--tolerance-deg"));
        if(clock)o.uncertainty=std::stod(required("--clock-uncertainty-ms"));
        if(o.role=="obs"){o.sensor.sensor_ip=required("--sensor-ip");o.sensor.board_ip=required("--board-ip");o.profile=required("--sensor-profile");}
    }
    if(args.count("--acquisition-profile")){
        if(simulation||o.role!="obs"||!clock)throw std::invalid_argument("acquisition profile requires real OBS and verified board clock");
        std::ifstream file(required("--acquisition-profile"));if(!file)throw std::invalid_argument("cannot read acquisition profile");
        const std::string json((std::istreambuf_iterator<char>(file)),{});google::protobuf::Struct data;
        if(!google::protobuf::util::JsonStringToMessage(json,&data).ok())throw std::invalid_argument("invalid acquisition JSON");
        auto text=[&](const char* k){const auto i=data.fields().find(k);if(i==data.fields().end()||i->second.kind_case()!=google::protobuf::Value::kStringValue||i->second.string_value().empty())throw std::invalid_argument(std::string("profile string required: ")+k);return i->second.string_value();};
        auto number=[&](const char* k){const auto i=data.fields().find(k);if(i==data.fields().end()||i->second.kind_case()!=google::protobuf::Value::kNumberValue||!std::isfinite(i->second.number_value()))throw std::invalid_argument(std::string("profile number required: ")+k);return i->second.number_value();};
        auto integer=[&](const char* k){const double n=number(k);if(n<0||n>255||n!=std::floor(n))throw std::invalid_argument("profile SDK enum must be uint8");return static_cast<std::uint32_t>(n);};
        AcquisitionProfile p;p.id=text("id");p.evidence=text("evidence");p.board_boot_id=text("board_boot_id");p.sensor_ip=text("sensor_ip");p.sensor_profile=text("sensor_profile");
        p.offset_seconds=number("offset_seconds");p.uncertainty_ms=number("uncertainty_ms");p.start_offset_ms=number("start_offset_ms");p.end_offset_ms=number("end_offset_ms");
        p.valid_from=parse_utc(text("valid_from"));p.valid_until=parse_utc(text("valid_until"));p.max_pose_gap_ms=number("max_pose_gap_ms");
        p.sync_type=integer("sync_type");p.sync_state=integer("sync_state");p.frame_protocol_version=integer("frame_protocol_version");
        std::ifstream boot_file("/proc/sys/kernel/random/boot_id");std::string boot;std::getline(boot_file,boot);
        if(boot.empty()||boot!=p.board_boot_id||p.sensor_ip!=o.sensor.sensor_ip||p.sensor_profile!=o.profile||p.id.size()>64||p.evidence.size()>1024||
           p.uncertainty_ms<1||p.uncertainty_ms<o.uncertainty||p.uncertainty_ms>100||p.start_offset_ms>0||p.end_offset_ms<0||p.start_offset_ms>=p.end_offset_ms||p.max_pose_gap_ms<=0||p.max_pose_gap_ms>250||p.valid_from>wall()||p.valid_until<=wall())throw std::invalid_argument("profile boot/sensor/time/uncertainty/interval mismatch");
        o.acquisition=std::move(p);
    }
    if(!std::isfinite(o.uncertainty)||o.uncertainty<0)throw std::invalid_argument("invalid verified clock uncertainty");
    if(!o.point_timeout_ms||o.point_timeout_ms>60000||!o.stop_timeout_ms||o.stop_timeout_ms>60000||!o.settle_ms||!std::isfinite(o.tolerance_deg)||o.tolerance_deg<=0)throw std::invalid_argument("invalid completion profile");
    return o;
}

class Application {
    Options options_;Motor motor_;std::unique_ptr<lidar::Capture> sensor_;Client client_;
    std::uint32_t installation_=0,mode_revision_=0;
    std::string control_,obs_control_,collection_;
    std::uint64_t epoch_=0,sequence_=0,obs_epoch_=0,obs_sequence_=0,frame_=0;
    bool gate_=false,obs_gate_=false,collecting_=false,measurement_fault_=false,disconnect_stop_=false;
    pb::ControlMode control_mode_=pb::CONTROL_MODE_UNSPECIFIED;
    pb::Result last_terminal_;
    std::string last_point_request_;
    std::uint64_t last_point_epoch_=0,last_point_sequence_=0;
    struct Motion {pb::Envelope request;double pan,tilt;Mono::time_point started,near_since;std::uint64_t initial_sample;bool stopping;};
    std::optional<Motion> motion_;
    std::optional<pb::Envelope> drain_;
    std::future<bool> measurement_;
    std::optional<pb::Envelope> measurement_request_;
    bool measurement_start_=false;
    bool shutdown_=false;
    Mono::time_point last_status_{};
    bool settled_=false;
    Sample previous_sample_,stable_sample_;
    PoseHistory pose_history_;
    std::uint64_t pose_generation_=0;
    std::optional<lidar::Frame> pending_frame_;
    double last_wall_=wall();Mono::time_point last_mono_=Mono::now();
    Mono::time_point stable_since_{};
    bool stopped()const{return settled_&&!motion_&&motor_.sample().pan_valid&&motor_.sample().tilt_valid;}
    pb::MotionState motion_state()const{
        if(!motor_.sample().pan_valid||!motor_.sample().tilt_valid)return pb::MOTION_STATE_UNKNOWN;
        return motion_||!settled_?pb::MOTION_STATE_MOVING:pb::MOTION_STATE_STOPPED;
    }
    void capabilities(){
        pb::Envelope e;auto* c=e.mutable_capability_report();c->set_device_id(options_.id);c->set_absolute_pointing(true);c->set_measured_angle_feedback(true);c->set_stop_supported(true);c->set_drain_supported(true);
        c->set_point_completion_timeout_ms(options_.point_timeout_ms);c->set_stop_completion_timeout_ms(options_.stop_timeout_ms);const auto& a=motor_.calibration();
        c->set_pan_min_deg(std::min(a.degrees(a.pan_min,true),a.degrees(a.pan_max,true)));c->set_pan_max_deg(std::max(a.degrees(a.pan_min,true),a.degrees(a.pan_max,true)));
        c->set_tilt_min_deg(std::min(a.degrees(a.tilt_min,false),a.degrees(a.tilt_max,false)));c->set_tilt_max_deg(std::max(a.degrees(a.tilt_min,false),a.degrees(a.tilt_max,false)));c->set_calibration_id(a.id);
        if(sensor_){c->add_supported_observation_modes(pb::OBSERVATION_MODE_FIXED_VIEW);c->set_pose_feedback_supported(true);c->set_lidar_measurement_stop_supported(true);c->set_sensor_data_profile_id(options_.profile);c->set_organized_frame_supported(true);}
        client_.send(e);
    }
    void status(){
        if(!client_.ready())return;
        pb::Envelope e;auto* s=e.mutable_device_status();s->set_device_id(options_.id);s->set_reported_at(utc());fill_angles(s->mutable_angles(),motor_.sample());
        s->set_device_state(motion_state()==pb::MOTION_STATE_UNKNOWN?pb::DEVICE_STATE_FAULT:motion_?pb::DEVICE_STATE_BUSY:pb::DEVICE_STATE_IDLE);
        s->set_motion_state(motion_state());s->set_active_installation_revision(installation_);s->set_estop_latch_state(pb::EMERGENCY_LATCH_STATE_RELEASED);
        s->set_clock_state(options_.clock_verified?pb::CLOCK_STATE_SYNCED:pb::CLOCK_STATE_UNKNOWN);if(options_.clock_verified)s->set_clock_uncertainty_ms(options_.uncertainty);
        if(motion_state()==pb::MOTION_STATE_UNKNOWN)s->set_fault_code(pb::REASON_CODE_ANGLE_READ_FAILED);
        if(sensor_)s->set_measurement_state(measurement_.valid()?(measurement_start_?pb::MEASUREMENT_STATE_STARTING:pb::MEASUREMENT_STATE_STOPPING):measurement_fault_?pb::MEASUREMENT_STATE_ERROR:collecting_?pb::MEASUREMENT_STATE_COLLECTING:pb::MEASUREMENT_STATE_STOPPED);
        client_.send(e);
        if(options_.role=="eff"){
            pb::Envelope p;auto* r=p.mutable_pointing_status();r->set_device_id(options_.id);if(!control_.empty())r->set_control_session_id(control_);r->set_control_epoch(epoch_);r->set_last_accepted_sequence(sequence_);r->set_motion_state(motion_state());
            fill_angles(r->mutable_angles(),motor_.sample());if(motor_.sample().pan_valid){r->set_pan_deg(motor_.sample().pan);r->set_pan_measured(true);}if(motor_.sample().tilt_valid){r->set_tilt_deg(motor_.sample().tilt);r->set_tilt_measured(true);}
            r->set_control_mode(control_mode_);r->set_gate_state(drain_?pb::GATE_STATE_DRAINING:gate_?pb::GATE_STATE_OPEN:pb::GATE_STATE_CLOSED);
            if(motion_){const auto& q=motion_->request.pointing_command();r->set_active_request_id(q.request_id());r->set_active_point_epoch(q.control_epoch());r->set_active_point_sequence(q.command_sequence());}
            if(!last_terminal_.request_id().empty()){r->mutable_last_terminal_result()->CopyFrom(last_terminal_);r->set_completion_verified(last_terminal_.result()==pb::REQUEST_RESULT_COMPLETED&&!motion_);}
            client_.send(p);
        }
    }
    void finish_motion(const pb::Envelope& request,pb::RequestResult value,pb::ReasonCode reason=pb::REASON_CODE_UNSPECIFIED){
        pb::Envelope e;pb::Result* out=nullptr;std::string id;
        if(request.has_pointing_command()){
            const auto& q=request.pointing_command();id=q.request_id();auto* r=e.mutable_pointing_result();r->set_control_session_id(q.control_session_id());r->set_control_epoch(q.control_epoch());r->set_command_sequence(q.command_sequence());r->set_actual_motion_state(motion_state());out=r->mutable_outcome();
            if(q.operation()==pb::PointingCommand::OPERATION_DRAIN&&!last_point_request_.empty()){r->set_drained_epoch(last_point_epoch_);r->set_drained_sequence(last_point_sequence_);r->set_drained_request_id(last_point_request_);r->set_last_command_result(last_terminal_.result());}
        }else{
            const auto& q=request.obs_motion_command();id=q.request_id();auto* r=e.mutable_obs_motion_result();r->set_obs_id(options_.id);r->set_obs_control_session_id(q.obs_control_session_id());r->set_control_epoch(q.control_epoch());r->set_command_sequence(q.command_sequence());r->set_actual_motion_state(motion_state());out=r->mutable_outcome();
        }
        result(out,id,value,reason);if(value!=pb::REQUEST_RESULT_ACCEPTED)last_terminal_=*out;client_.send(e,id);
    }
    void cancel_motion(){
        if(motion_){auto old=motion_->request;motion_.reset();finish_motion(old,pb::REQUEST_RESULT_CANCELLED,pb::REASON_CODE_SUPERSEDED);}
        if(drain_){auto old=*drain_;drain_.reset();finish_motion(old,pb::REQUEST_RESULT_CANCELLED,pb::REASON_CODE_SUPERSEDED);}
    }
    void motion_command(const pb::Envelope& e){
        const bool obs=e.has_obs_motion_command();
        const auto& p=e.pointing_command();const auto& q=e.obs_motion_command();
        const auto operation=obs?int(q.operation()):int(p.operation());
        const bool stop=operation==2,point=operation==1,get=obs?operation==4:operation==3,drain=obs?operation==3:operation==4;
        const std::string session=obs?q.obs_control_session_id():p.control_session_id();const auto epoch=obs?q.control_epoch():p.control_epoch(),seq=obs?q.command_sequence():p.command_sequence();
        auto& high=obs?obs_sequence_:sequence_;const auto active_epoch=obs?obs_epoch_:epoch_;
        if((obs!=(options_.role=="obs")) || session!=(obs?obs_control_:control_) || session.empty() || epoch!=active_epoch || !seq || seq<=high){client_.reject(e,pb::REASON_CODE_STALE_COMMAND);return;}
        if(!point&&!stop&&!get&&!drain){client_.reject(e,pb::REASON_CODE_INVALID_ARGUMENT);return;}
        const bool has_pan=obs?q.has_pan_deg():p.has_pan_deg(),has_tilt=obs?q.has_tilt_deg():p.has_tilt_deg();
        if(!point&&(has_pan||has_tilt||(obs?q.has_purpose()||q.has_collection_session_id()||q.has_object_id()||q.has_source_frame_id():p.has_collection_session_id()||p.has_object_id()||p.has_source_frame_id()))){client_.reject(e,pb::REASON_CODE_INVALID_MESSAGE);return;}
        if(point){
            const auto rev=obs?q.installation_revision():p.effector_installation_revision();const auto pan=obs?q.pan_deg():p.pan_deg(),tilt=obs?q.tilt_deg():p.tilt_deg();
            if(!installation_||rev!=installation_){client_.reject(e,pb::REASON_CODE_INSTALLATION_MISMATCH);return;}
            if(!(obs?obs_gate_:gate_) || (obs&&(collecting_||measurement_.valid()))){client_.reject(e,pb::REASON_CODE_BUSY);return;}
            if(!has_pan||!has_tilt||!std::isfinite(pan)||!std::isfinite(tilt)){client_.reject(e,pb::REASON_CODE_INVALID_ARGUMENT);return;}
            const auto expiry=obs?q.expires_at():p.expires_at();const auto issued=obs?q.issued_at():p.issued_at();
            if(expiry.empty()||parse_utc(expiry)<=wall()||parse_utc(issued)>wall()+.1||parse_utc(expiry)-parse_utc(issued)>1.001){client_.reject(e,pb::REASON_CODE_EXPIRED_COMMAND);return;}
            if(obs&&(q.observation_mode_revision()!=mode_revision_||q.purpose()!=pb::ObsMotionCommand::PURPOSE_FIXED_POSITION)){client_.reject(e,pb::REASON_CODE_MODE_UNSUPPORTED);return;}
            if(!obs&&p.origin()!=control_mode_){client_.reject(e,pb::REASON_CODE_PERMISSION_DENIED);return;}
            if(!obs&&p.origin()==pb::CONTROL_MODE_AUTO&&(!p.has_collection_session_id()||!p.has_object_id()||!p.has_source_device_id()||!p.has_source_connection_session_id()||!p.has_source_frame_id()||!p.has_observation_installation_revision())){client_.reject(e,pb::REASON_CODE_INVALID_MESSAGE);return;}
            if(!obs&&p.origin()==pb::CONTROL_MODE_MANUAL&&(p.has_collection_session_id()||p.has_object_id()||p.has_source_frame_id())){client_.reject(e,pb::REASON_CODE_INVALID_MESSAGE);return;}
            if(!motor_.in_range(pan,tilt)){client_.reject(e,pb::REASON_CODE_OUT_OF_RANGE);return;}
            cancel_motion();high=seq;settled_=false;stable_since_={};++pose_generation_;pending_frame_.reset();
            if(!motor_.move(pan,tilt)){client_.reject(e,pb::REASON_CODE_OUT_OF_RANGE);return;}
            motion_=Motion{e,pan,tilt,Mono::now(),{},motor_.sample().sequence,false};finish_motion(e,pb::REQUEST_RESULT_ACCEPTED);
            if(!obs){last_point_request_=p.request_id();last_point_epoch_=p.control_epoch();last_point_sequence_=p.command_sequence();}
            std::cout<<"POINT "<<options_.id<<' '<<seq<<' '<<pan<<' '<<tilt<<'\n'<<std::flush;
        }else if(stop){
            cancel_motion();high=seq;gate_=obs_gate_=false;settled_=false;stable_since_={};++pose_generation_;pending_frame_.reset();
            if(!motor_.hold()){client_.reject(e,pb::REASON_CODE_ANGLE_READ_FAILED);return;}
            motion_=Motion{e,motor_.sample().pan,motor_.sample().tilt,Mono::now(),{},motor_.sample().sequence,true};finish_motion(e,pb::REQUEST_RESULT_ACCEPTED);
            std::cout<<"STOP "<<options_.id<<' '<<seq<<'\n'<<std::flush;
        }else if(drain){high=seq;(obs?obs_gate_:gate_)=false;if(motion_){drain_=e;finish_motion(e,pb::REQUEST_RESULT_ACCEPTED);}else finish_motion(e,pb::REQUEST_RESULT_COMPLETED);}
        else{high=seq;finish_motion(e,pb::REQUEST_RESULT_COMPLETED);status();}
    }
    void command(const pb::Envelope& e){
        if(e.has_installation_set()){
            const auto& q=e.installation_set();
            if(!stopped()||collecting_||measurement_.valid()){client_.reject(e,pb::REASON_CODE_BUSY);return;}
            if(q.device_id()!=options_.id||!q.installation_revision()||q.axis_calibration_id()!=motor_.calibration().id||q.mount_rotation_size()!=9){client_.reject(e,pb::REASON_CODE_INSTALLATION_MISMATCH);return;}
            for(double v:q.mount_rotation())if(!std::isfinite(v)){client_.reject(e,pb::REASON_CODE_INVALID_ARGUMENT);return;}
            installation_=q.installation_revision();pb::Envelope out;auto* r=out.mutable_installation_result();result(r->mutable_outcome(),q.request_id(),pb::REQUEST_RESULT_COMPLETED);r->set_device_id(options_.id);r->set_active_installation_revision(installation_);client_.send(out,q.request_id());status();
        }else if(e.has_control_open()&&options_.role=="eff"){
            const auto& q=e.control_open();if(!stopped()){client_.reject(e,pb::REASON_CODE_BUSY);return;}
            if(!installation_||q.installation_revision()!=installation_){client_.reject(e,pb::REASON_CODE_INSTALLATION_MISMATCH);return;}
            if(q.control_epoch()<=epoch_||q.control_session_id().size()!=36||(q.mode()!=pb::CONTROL_MODE_AUTO&&q.mode()!=pb::CONTROL_MODE_MANUAL)){client_.reject(e,pb::REASON_CODE_STALE_COMMAND);return;}
            control_=q.control_session_id();epoch_=q.control_epoch();sequence_=0;gate_=true;control_mode_=q.mode();
            last_point_request_.clear();last_point_epoch_=last_point_sequence_=0;
            pb::Envelope out;auto* r=out.mutable_control_result();result(r->mutable_outcome(),q.request_id(),pb::REQUEST_RESULT_COMPLETED);r->set_active_control_session_id(control_);r->set_control_epoch(epoch_);client_.send(out,q.request_id());
        }else if(e.has_pointing_command()||e.has_obs_motion_command())motion_command(e);
        else if(e.has_obs_mode_command()&&sensor_){
            const auto& q=e.obs_mode_command();if(!stopped()||collecting_||measurement_.valid()){client_.reject(e,pb::REASON_CODE_BUSY);return;}
            if(q.mode()!=pb::OBSERVATION_MODE_FIXED_VIEW||q.has_follow_profile()){client_.reject(e,pb::REASON_CODE_MODE_UNSUPPORTED);return;}
            if(!installation_||q.installation_revision()!=installation_){client_.reject(e,pb::REASON_CODE_INSTALLATION_MISMATCH);return;}
            if(!q.control_epoch()||q.control_epoch()<obs_epoch_||q.observation_mode_revision()<=mode_revision_||q.obs_control_session_id().size()!=36||(q.control_epoch()==obs_epoch_&&q.obs_control_session_id()!=obs_control_)){client_.reject(e,pb::REASON_CODE_STALE_COMMAND);return;}
            if(q.control_epoch()!=obs_epoch_)obs_sequence_=0;obs_control_=q.obs_control_session_id();obs_epoch_=q.control_epoch();mode_revision_=q.observation_mode_revision();obs_gate_=true;
            pb::Envelope out;auto* r=out.mutable_obs_mode_result();result(r->mutable_outcome(),q.request_id(),pb::REQUEST_RESULT_COMPLETED);r->set_obs_id(options_.id);r->set_observation_mode_revision(mode_revision_);r->set_mode(q.mode());r->set_actual_state(motion_state());r->set_mode_applied_at(utc());client_.send(out,q.request_id());
        }else if(e.has_measurement_command()&&sensor_){
            const auto& q=e.measurement_command();
            if(measurement_.valid()){client_.reject(e,pb::REASON_CODE_BUSY);return;}
            if(!installation_||q.installation_revision()!=installation_){client_.reject(e,pb::REASON_CODE_INSTALLATION_MISMATCH);return;}
            const bool start=q.operation()==pb::MeasurementCommand::OPERATION_START;
            if(q.collection_session_id().size()!=36||(!start&&q.operation()!=pb::MeasurementCommand::OPERATION_STOP)){client_.reject(e,pb::REASON_CODE_INVALID_ARGUMENT);return;}
            if(start&&(!stopped()||!mode_revision_||collecting_)){client_.reject(e,pb::REASON_CODE_BUSY);return;}
            if(!start&&q.collection_session_id()!=collection_){client_.reject(e,pb::REASON_CODE_UNKNOWN_SESSION);return;}
            pending_frame_.reset();
            if(start)collection_=q.collection_session_id();else collecting_=false;
            measurement_start_=start;measurement_request_=e;
            pb::Envelope out;auto* r=out.mutable_measurement_result();result(r->mutable_outcome(),q.request_id(),pb::REQUEST_RESULT_ACCEPTED);r->set_collection_session_id(q.collection_session_id());r->set_actual_state(start?pb::MEASUREMENT_STATE_STARTING:pb::MEASUREMENT_STATE_STOPPING);client_.send(out,q.request_id());
            measurement_=std::async(std::launch::async,[this,start]{return start?sensor_->start():sensor_->stop();});
        }else client_.reject(e,pb::REASON_CODE_UNSUPPORTED_TYPE);
    }
    void progress(){
        const double current_wall=wall();const auto current_mono=Mono::now();
        if(std::abs((current_wall-last_wall_)-std::chrono::duration<double>(current_mono-last_mono_).count())>.25){
            pose_history_.clear();pending_frame_.reset();options_.acquisition.reset();options_.clock_verified=false;
        }
        last_wall_=current_wall;last_mono_=current_mono;
        const bool fresh=motor_.poll();
        if(fresh){const auto& s=motor_.sample();const auto now=Mono::now();
            if(s.pan_valid&&s.tilt_valid&&previous_sample_.pan_valid&&previous_sample_.tilt_valid&&s.pan_at-previous_sample_.pan_at<=.25&&s.tilt_at-previous_sample_.tilt_at<=.25&&std::abs(s.pan-previous_sample_.pan)<=options_.tolerance_deg&&std::abs(s.tilt-previous_sample_.tilt)<=options_.tolerance_deg){
                if(stable_since_==Mono::time_point{} || std::abs(s.pan-stable_sample_.pan)>options_.tolerance_deg || std::abs(s.tilt-stable_sample_.tilt)>options_.tolerance_deg){
                    stable_since_=now;stable_sample_=s;settled_=false;
                }
                if(now-stable_since_>=std::chrono::milliseconds(options_.settle_ms))settled_=true;
            }else{stable_since_={};settled_=false;}
            previous_sample_=s;
        }
        if(fresh&&motion_){
            const auto& s=motor_.sample();const auto now=Mono::now();
            if(!s.pan_valid||!s.tilt_valid||now-motion_->started>std::chrono::milliseconds(motion_->stopping?options_.stop_timeout_ms:options_.point_timeout_ms)){
                const auto reason=!s.pan_valid||!s.tilt_valid?pb::REASON_CODE_ANGLE_READ_FAILED:pb::REASON_CODE_COMPLETION_TIMEOUT;
                auto q=motion_->request;motion_.reset();settled_=false;stable_since_={};motor_.hold();gate_=obs_gate_=false;finish_motion(q,pb::REQUEST_RESULT_FAILED,reason);
            }else if(s.sequence>motion_->initial_sample&&std::abs(s.pan-motion_->pan)<=options_.tolerance_deg&&std::abs(s.tilt-motion_->tilt)<=options_.tolerance_deg){
                if(motion_->near_since==Mono::time_point{})motion_->near_since=now;
                if(settled_&&now-motion_->near_since>=std::chrono::milliseconds(options_.settle_ms)){auto q=motion_->request;motion_.reset();finish_motion(q,pb::REQUEST_RESULT_COMPLETED);status();}
            }else motion_->near_since={};
        }
        if(fresh){const auto& s=motor_.sample();pose_history_.add({wall(),s.pan_at,s.tilt_at,s.pan,s.tilt,s.pan_valid&&s.tilt_valid,stopped(),pose_generation_});}
        if(drain_&&!motion_){auto q=*drain_;drain_.reset();finish_motion(q,pb::REQUEST_RESULT_COMPLETED);}
        if(measurement_.valid()&&measurement_.wait_for(std::chrono::milliseconds(0))==std::future_status::ready){
            bool ok=false;try{ok=measurement_.get();}catch(const std::exception&){ok=false;}
            collecting_=ok&&measurement_start_&&!disconnect_stop_;measurement_fault_=!ok;
            if(measurement_request_&&client_.ready()){
                const auto q=measurement_request_->measurement_command();pb::Envelope out;auto* r=out.mutable_measurement_result();result(r->mutable_outcome(),q.request_id(),ok?pb::REQUEST_RESULT_COMPLETED:pb::REQUEST_RESULT_FAILED,ok?pb::REASON_CODE_UNSPECIFIED:pb::REASON_CODE_INTERNAL_ERROR);r->set_collection_session_id(q.collection_session_id());r->set_actual_state(ok?(collecting_?pb::MEASUREMENT_STATE_COLLECTING:pb::MEASUREMENT_STATE_STOPPED):pb::MEASUREMENT_STATE_UNKNOWN);client_.send(out,q.request_id());
            }measurement_request_.reset();status();
        }
        if(disconnect_stop_&&sensor_&&!measurement_.valid()){
            disconnect_stop_=false;measurement_start_=false;measurement_=std::async(std::launch::async,[this]{return sensor_->stop();});
        }
    }
    std::size_t packet_size(pb::Envelope e)const{
        e.set_version_major(1);e.set_sender_id(options_.id);e.set_recipient_id("C2");e.set_connection_session_id(client_.session());e.set_message_id(std::string(36,'0'));e.set_sent_at(utc());return e.ByteSizeLong();
    }
    void raw_frame(lidar::Frame f,const CapturedPose& captured){
        const auto count=std::uint64_t(f.width)*f.height;if(!count||count>500000||count!=f.points.size())throw std::runtime_error("invalid SDK frame slot count");
        const auto collection=collection_;const auto session=client_.session();const auto pose_id=uuid();++frame_;
        pb::Envelope pose;auto* p=pose.mutable_obs_pose();p->set_obs_id(options_.id);p->set_pose_id(pose_id);p->set_collection_session_id(collection);p->set_installation_revision(installation_);p->set_observation_mode_revision(mode_revision_);p->set_motion_state(motion_state());
        const bool measured=captured.quality==PoseQuality::Verified;p->set_pose_valid(measured);
        if(f.captured_at)p->set_captured_at(utc(*f.captured_at));
        if(measured){p->set_pan_measured(true);p->set_tilt_measured(true);p->set_pan_deg(captured.pan);p->set_tilt_deg(captured.tilt);}
        client_.send(pose);
        pb::PointCloudChunk meta;meta.set_collection_session_id(collection);meta.set_frame_id(frame_);meta.set_source_frame_id(f.source_id);meta.set_installation_revision(installation_);meta.set_observation_mode_revision(mode_revision_);meta.set_pose_id(pose_id);meta.set_width(f.width);meta.set_height(f.height);meta.set_total_points(static_cast<std::uint32_t>(count));meta.set_sensor_data_profile_id(options_.profile);meta.set_motion_compensation(pb::PointCloudChunk::MOTION_COMPENSATION_NONE);
        if(f.captured_start)meta.set_captured_start_at(utc(*f.captured_start));if(f.captured_at)meta.set_captured_at(utc(*f.captured_at));if(f.captured_end)meta.set_captured_end_at(utc(*f.captured_end));
        if(!f.mapping_id.empty())meta.set_clock_mapping_id(f.mapping_id);if(f.uncertainty_ms)meta.set_clock_uncertainty_ms(*f.uncertainty_ms);
        auto* sdk=meta.mutable_sdk_time();sdk->set_seconds(f.sdk_seconds);sdk->set_nanoseconds(f.sdk_nanoseconds);sdk->set_sync_type(f.sync_type);sdk->set_sync_state(f.sync_state);sdk->set_frame_protocol_version(f.protocol_version);
        std::vector<pb::PointCloudChunk> chunks;auto chunk=meta;chunk.set_chunk_count(8192);chunk.set_chunk_index(8191);
        for(std::size_t i=0;i<count;++i){const auto& v=f.points[i];auto* point=chunk.add_points();point->set_pixel_index(static_cast<std::uint32_t>(i));point->set_sdk_depth_raw(v.depth);const bool valid=v.xyz_valid&&std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);point->set_xyz_valid(valid);if(valid){point->set_x(v.x);point->set_y(v.y);point->set_z(v.z);}
            pb::Envelope test;test.mutable_point_cloud_chunk()->CopyFrom(chunk);
            if(packet_size(test)>1200){auto copied=chunk.points(chunk.points_size()-1);chunk.mutable_points()->RemoveLast();if(!chunk.points_size())throw std::runtime_error("frame metadata exceeds datagram budget");chunks.push_back(chunk);chunk=meta;chunk.set_chunk_count(8192);chunk.set_chunk_index(8191);chunk.add_points()->CopyFrom(copied);}
        }
        if(chunk.points_size())chunks.push_back(std::move(chunk));if(chunks.size()>8192)throw std::runtime_error("frame exceeds chunk count limit");
        std::size_t total_bytes=0;for(const auto& value:chunks)total_bytes+=value.ByteSizeLong();
        if(total_bytes>8*1024*1024)throw std::runtime_error("frame exceeds 8 MiB assembly limit");
        for(std::size_t i=0;i<chunks.size();++i){if(!collecting_||collection_!=collection||client_.session()!=session)return;chunks[i].set_chunk_count(static_cast<std::uint32_t>(chunks.size()));chunks[i].set_chunk_index(static_cast<std::uint32_t>(i));pb::Envelope e;e.mutable_point_cloud_chunk()->CopyFrom(chunks[i]);client_.send(e);if(i%128==127)client_.step();}
    }
public:
    explicit Application(Options o,device::CaptureFactory factory):options_(std::move(o)),motor_(options_.simulation,options_.axes,options_.uart,options_.baud),sensor_(factory?factory(options_.simulation,options_.sensor):nullptr),client_({options_.id,options_.c2,options_.role=="obs"?pb::ROLE_OBS:pb::ROLE_EFF,options_.port,options_.c2_port}){
        motor_.poll();
        client_.command=[this](const auto& e){command(e);};
        client_.connected=[this](bool changed){if(changed){installation_=mode_revision_=0;control_.clear();obs_control_.clear();epoch_=sequence_=obs_epoch_=obs_sequence_=frame_=0;}capabilities();status();std::cout<<"REGISTERED "<<options_.id<<" boot="<<client_.boot<<" session="<<client_.session()<<'\n'<<std::flush;};
        client_.disconnected=[this]{++pose_generation_;pose_history_.clear();pending_frame_.reset();motor_.hold();settled_=false;stable_since_={};motion_.reset();drain_.reset();gate_=obs_gate_=false;collecting_=false;measurement_request_.reset();disconnect_stop_=bool(sensor_);};
    }
    ~Application(){shutdown();}
    void shutdown()noexcept{
        if(shutdown_)return;shutdown_=true;
        try{motor_.hold();collecting_=false;if(measurement_.valid())measurement_.wait();if(sensor_)sensor_->stop();}catch(const std::exception&){}
    }
    void run(){
        while(running){client_.step();progress();if(client_.ready()){
            if(Mono::now()-last_status_>=std::chrono::milliseconds(500)){last_status_=Mono::now();status();}
            if(sensor_&&collecting_){
                if(auto f=sensor_->latest()){
                    if(options_.acquisition)map_acquisition(*f,*options_.acquisition,wall());
                    pending_frame_=std::move(*f);
                }
                if(pending_frame_){
                    const double gap=options_.acquisition?options_.acquisition->max_pose_gap_ms:200;
                    const auto captured=pose_history_.qualify(*pending_frame_,wall(),gap,options_.tolerance_deg,pose_generation_);
                    if(captured.quality!=PoseQuality::Pending){auto f=std::move(*pending_frame_);pending_frame_.reset();raw_frame(std::move(f),captured);}
                }
            }
        }}
        shutdown();
    }
};
int device::run_device(int argc,char** argv,const std::string& role,CaptureFactory factory){
    std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);
    try{Application app(options(argc,argv,role),std::move(factory));app.run();return 0;}
    catch(const std::exception& error){std::cerr<<"device error: "<<error.what()<<'\n';return 1;}
}
