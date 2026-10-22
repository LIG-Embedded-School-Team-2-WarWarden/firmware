#pragma once
#include "mfs.pb.h"
#include <array>
#include <chrono>
#include <cstring>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#ifdef GetMessage
#undef GetMessage
#endif
#else
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace device {
namespace pb = mfs::icd::v1;
using Mono = std::chrono::steady_clock;
inline double wall() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
inline std::string utc(double seconds=wall()) {
    const auto ms=static_cast<long long>(seconds*1000);
    const auto value=static_cast<std::time_t>(ms/1000);
    std::tm t{};
#ifdef _WIN32
    gmtime_s(&t,&value);
#else
    gmtime_r(&value,&t);
#endif
    std::ostringstream out; out<<std::put_time(&t,"%Y-%m-%dT%H:%M:%S")<<'.'<<std::setw(3)<<std::setfill('0')<<ms%1000<<'Z';return out.str();
}
inline double parse_utc(const std::string& s) {
    if(s.size()!=24 || s[10]!='T' || s[19]!='.' || s.back()!='Z')throw std::invalid_argument("UTC milliseconds required");
    std::tm t{};std::istringstream in(s.substr(0,19));in>>std::get_time(&t,"%Y-%m-%dT%H:%M:%S");
    if(in.fail())throw std::invalid_argument("invalid UTC");
#ifdef _WIN32
    const auto value=_mkgmtime(&t);
#else
    const auto value=timegm(&t);
#endif
    return static_cast<double>(value)+std::stoi(s.substr(20,3))/1000.;
}
inline std::string uuid() {
    static std::mt19937_64 random(std::random_device{}());
    std::array<unsigned char,16> b{};for(auto& v:b)v=static_cast<unsigned char>(random());
    b[6]=(b[6]&15)|64;b[8]=(b[8]&63)|128;
    std::ostringstream s;s<<std::hex<<std::setfill('0');for(unsigned i=0;i<16;++i){if(i==4||i==6||i==8||i==10)s<<'-';s<<std::setw(2)<<unsigned(b[i]);}return s.str();
}
inline void result(pb::Result* out,const std::string& request,pb::RequestResult value,pb::ReasonCode reason=pb::REASON_CODE_UNSPECIFIED) {
    out->set_request_id(request);out->set_result(value);out->set_reported_at(utc());
    if(value!=pb::REQUEST_RESULT_ACCEPTED)out->set_completed_at(out->reported_at());
    if(reason!=pb::REASON_CODE_UNSPECIFIED)out->set_reason_code(reason);
}

class Client {
public:
    struct Config { std::string id,c2_ip;pb::Role role;unsigned short listen_port=0,c2_port=5000;bool laser=false; };
    std::function<void(const pb::Envelope&)> command;
    std::function<void(bool)> connected; // true only when connectionSessionId changes
    std::function<void()> disconnected;
    const std::string boot=uuid();
    explicit Client(Config config):config_(std::move(config)) {
#ifdef _WIN32
        WSADATA wsa{};if(WSAStartup(MAKEWORD(2,2),&wsa))throw std::runtime_error("WSAStartup failed");
#endif
        socket_=::socket(AF_INET,SOCK_DGRAM,0);
        if(socket_==invalid)throw std::runtime_error("UDP socket failed");
        sockaddr_in local{};local.sin_family=AF_INET;local.sin_port=htons(config_.listen_port);
        if(::bind(socket_,reinterpret_cast<sockaddr*>(&local),sizeof(local)))throw std::runtime_error("UDP bind failed");
        remote_.sin_family=AF_INET;remote_.sin_port=htons(config_.c2_port);
        if(inet_pton(AF_INET,config_.c2_ip.c_str(),&remote_.sin_addr)!=1)throw std::invalid_argument("IPv4 C2 address required");
        int len=sizeof(local);
#ifdef _WIN32
        getsockname(socket_,reinterpret_cast<sockaddr*>(&local),&len);
#else
        socklen_t n=sizeof(local);getsockname(socket_,reinterpret_cast<sockaddr*>(&local),&n);
#endif
        port_=ntohs(local.sin_port);registration_id_=uuid();
    }
    ~Client(){
#ifdef _WIN32
        closesocket(socket_);WSACleanup();
#else
        close(socket_);
#endif
    }
    bool ready()const{return ready_;}
    const std::string& session()const{return session_;}
    unsigned short port()const{return port_;}
    void send(pb::Envelope e,const std::string& request={}) {
        e.set_version_major(1);e.set_version_minor(0);e.set_sender_id(config_.id);e.set_recipient_id("C2");
        if(ready_)e.set_connection_session_id(session_);
        e.set_message_id(uuid());e.set_sent_at(utc());
        std::string bytes=e.SerializeAsString();if(bytes.size()>1200)throw std::length_error("Envelope exceeds 1200 bytes");
        raw_send(bytes);
        if(!request.empty()) { auto it=cache_.find(request);if(it!=cache_.end())it->second.response=std::move(bytes); }
    }
    void reject(const pb::Envelope& request,pb::ReasonCode reason) {
        pb::Envelope e;pb::Result* out=nullptr;std::string id;
        switch(request.body_case()) {
        case pb::Envelope::kLaserCommand: {const auto& q=request.laser_command();id=q.request_id();auto* r=e.mutable_laser_result();r->set_control_session_id(q.control_session_id());r->set_control_epoch(q.control_epoch());r->set_command_sequence(q.command_sequence());r->set_actual_laser_state(pb::LASER_STATE_UNKNOWN);out=r->mutable_outcome();break;}
        case pb::Envelope::kInstallationSet: {const auto& q=request.installation_set();id=q.request_id();auto* r=e.mutable_installation_result();r->set_device_id(config_.id);out=r->mutable_outcome();break;}
        case pb::Envelope::kControlOpen: {const auto& q=request.control_open();id=q.request_id();auto* r=e.mutable_control_result();r->set_active_control_session_id(q.control_session_id());r->set_control_epoch(q.control_epoch());out=r->mutable_outcome();break;}
        case pb::Envelope::kPointingCommand: {const auto& q=request.pointing_command();id=q.request_id();auto* r=e.mutable_pointing_result();r->set_control_session_id(q.control_session_id());r->set_control_epoch(q.control_epoch());r->set_command_sequence(q.command_sequence());r->set_actual_motion_state(pb::MOTION_STATE_UNKNOWN);out=r->mutable_outcome();break;}
        case pb::Envelope::kObsMotionCommand: {const auto& q=request.obs_motion_command();id=q.request_id();auto* r=e.mutable_obs_motion_result();r->set_obs_id(config_.id);r->set_obs_control_session_id(q.obs_control_session_id());r->set_control_epoch(q.control_epoch());r->set_command_sequence(q.command_sequence());r->set_actual_motion_state(pb::MOTION_STATE_UNKNOWN);out=r->mutable_outcome();break;}
        case pb::Envelope::kObsModeCommand: {const auto& q=request.obs_mode_command();id=q.request_id();auto* r=e.mutable_obs_mode_result();r->set_obs_id(config_.id);r->set_observation_mode_revision(q.observation_mode_revision());r->set_mode(q.mode());r->set_actual_state(pb::MOTION_STATE_UNKNOWN);r->set_mode_applied_at(utc());out=r->mutable_outcome();break;}
        case pb::Envelope::kMeasurementCommand: {const auto& q=request.measurement_command();id=q.request_id();auto* r=e.mutable_measurement_result();r->set_collection_session_id(q.collection_session_id());r->set_actual_state(pb::MEASUREMENT_STATE_UNKNOWN);out=r->mutable_outcome();break;}
        default:return;
        }
        result(out,id,pb::REQUEST_RESULT_REJECTED,reason);send(e,id);
    }
    void step() {
        const auto now=Mono::now();
        if(ready_ && now-alive_>std::chrono::seconds(2))lose_session();
        if(!ready_ && now-last_register_>=std::chrono::milliseconds(200)) {
            last_register_=now;
            if(registration_packet_.empty()){
                pb::Envelope e;e.set_version_major(1);e.set_sender_id(config_.id);e.set_recipient_id("C2");e.set_message_id(uuid());e.set_sent_at(utc());
                auto* r=e.mutable_register_request();r->set_request_id(registration_id_);r->set_device_id(config_.id);r->set_role(config_.role);r->set_boot_id(boot);r->set_receive_port(port_);if(config_.laser)r->add_capabilities(pb::CAPABILITY_LASER_CONTROL);registration_packet_=e.SerializeAsString();
            }raw_send(registration_packet_);
        }
        if(ready_ && now-last_heartbeat_>=std::chrono::milliseconds(500)){
            last_heartbeat_=now;pb::Envelope e;auto* h=e.mutable_heartbeat();h->set_boot_id(boot);h->set_sequence(++heartbeat_);send(e);
        }
        for(auto i=cache_.begin();i!=cache_.end();)if(now-i->second.created>std::chrono::seconds(60))i=cache_.erase(i);else++i;
        fd_set readable;FD_ZERO(&readable);FD_SET(socket_,&readable);timeval timeout{0,10000};
        if(select(static_cast<int>(socket_)+1,&readable,nullptr,nullptr,&timeout)<=0)return;
        std::array<char,1201> data{};sockaddr_in sender{};
#ifdef _WIN32
        int length=sizeof(sender);
#else
        socklen_t length=sizeof(sender);
#endif
        const auto size=recvfrom(socket_,data.data(),static_cast<int>(data.size()),0,reinterpret_cast<sockaddr*>(&sender),&length);
        if(size<=0 || size>1200 || sender.sin_addr.s_addr!=remote_.sin_addr.s_addr || sender.sin_port!=remote_.sin_port)return;
        pb::Envelope e;if(!e.ParseFromArray(data.data(),static_cast<int>(size)) || e.version_major()!=1 || e.version_minor()!=0 || e.sender_id()!="C2" || e.recipient_id()!=config_.id || e.body_case()==pb::Envelope::BODY_NOT_SET)return;
        if(e.message_id().size()!=36)return;
        try{parse_utc(e.sent_at());}catch(const std::exception&){return;}
        if(e.has_register_result()){
            const auto& r=e.register_result();if(r.outcome().request_id()!=registration_id_ || r.boot_id()!=boot || r.device_id()!=config_.id || r.outcome().result()!=pb::REQUEST_RESULT_COMPLETED || r.connection_session_id().empty())return;
            if(ready_)return;
            const bool changed=session_!=r.connection_session_id();if(changed){cache_.clear();heartbeat_=c2_sequence_=0;}
            session_=r.connection_session_id();c2_boot_=r.c2_boot_id();ready_=true;alive_=now;if(connected)connected(changed);return;
        }
        if(e.has_session_required()){if(e.session_required().device_id()==config_.id)lose_session();return;}
        if(!ready_ || e.connection_session_id()!=session_)return;
        if(e.has_heartbeat()){const auto& h=e.heartbeat();if(h.boot_id()==c2_boot_ && h.sequence()>c2_sequence_){c2_sequence_=h.sequence();alive_=now;}return;}
        const auto* body=e.GetDescriptor()->FindFieldByNumber(static_cast<int>(e.body_case()));
        const auto& message=e.GetReflection()->GetMessage(e,body);
        const auto* field=message.GetDescriptor()->FindFieldByName("request_id");if(!field)return;
        const auto id=message.GetReflection()->GetString(message,field);if(id.size()!=36)return;
        const auto raw=std::to_string(static_cast<int>(e.body_case()))+":"+message.SerializeAsString();auto found=cache_.find(id);
        if(found!=cache_.end()){
            if(found->second.request!=raw){const auto saved=found->second.response;reject(e,pb::REASON_CODE_ID_CONFLICT);found->second.response=saved;}
            else if(!found->second.response.empty())raw_send(found->second.response);
            return;
        }
        if(cache_.size()>=1024){reject(e,pb::REASON_CODE_BUSY);return;}
        cache_[id]={raw,{},now};
        try { if(command)command(e);else reject(e,pb::REASON_CODE_UNSUPPORTED_TYPE); }
        catch(const std::exception& error){std::cerr<<"command error: "<<error.what()<<'\n';if(disconnected)disconnected();reject(e,pb::REASON_CODE_INTERNAL_ERROR);}
    }
private:
#ifdef _WIN32
    using Socket=SOCKET;static constexpr Socket invalid=INVALID_SOCKET;
#else
    using Socket=int;static constexpr Socket invalid=-1;
#endif
    struct Cached { std::string request,response;Mono::time_point created; };
    Config config_;Socket socket_=invalid;sockaddr_in remote_{};unsigned short port_=0;
    std::string session_,c2_boot_,registration_id_,registration_packet_;bool ready_=false;
    std::uint64_t heartbeat_=0,c2_sequence_=0;
    Mono::time_point alive_=Mono::now(),last_register_{},last_heartbeat_{};
    std::map<std::string,Cached> cache_;
    void raw_send(const std::string& data){if(sendto(socket_,data.data(),static_cast<int>(data.size()),0,reinterpret_cast<const sockaddr*>(&remote_),sizeof(remote_))<0)throw std::runtime_error("UDP send failed");}
    void lose_session(){if(ready_ && disconnected)disconnected();ready_=false;registration_id_=uuid();registration_packet_.clear();}
};
}
