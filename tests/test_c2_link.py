"""Cross-language C2/Python <-> firmware/C++ UDP tests; no board access."""
import argparse
import copy
import datetime as dt
import json
import os
import pathlib
import queue
import socket
import subprocess
import sys
import threading
import time
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--c2-root',type=pathlib.Path,default=ROOT.parent/'ai-generated-c2')
parser.add_argument('--obs',type=pathlib.Path,required=True)
parser.add_argument('--eff',type=pathlib.Path,required=True)
parser.add_argument('--detector',type=pathlib.Path)
args,remaining=parser.parse_known_args()
args.obs=args.obs.resolve()
args.eff=args.eff.resolve()
args.detector=(args.detector or args.c2_root/'build/live/Release/c2_detector_worker.exe').resolve()
sys.path.insert(0,str(args.c2_root))
from team_mvp.contract import pb,uid,utc,encode,decode,envelope,outcome
from team_mvp.engine import Engine
from team_mvp.cloud import Detector

def wait(predicate,timeout=6):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.015)
    raise AssertionError('condition timed out')

class Native:
    def __init__(self,role,port,uart=None):
        self.lines=[]
        command=[str(args.obs if role=='obs' else args.eff),'--c2-port',str(port)]
        if uart is None:command+=['--simulate']
        else:command+=['--uart',uart,'--calibration-id','synthetic-'+role+'-zero-v1',
            '--pan-zero','2048','--tilt-zero','2048','--ticks-per-turn','4096','--pan-sign','1','--tilt-sign','1',
            '--pan-min-ticks','1024','--pan-max-ticks','3072','--tilt-min-ticks','1536','--tilt-max-ticks','2560',
            '--point-timeout-ms','2000','--stop-timeout-ms','2000','--settle-ms','150','--tolerance-deg','0.5',
            '--clock-verified','--clock-uncertainty-ms','1']
        self.process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8')
        def read():
            for line in self.process.stdout:self.lines.append(line.strip())
        self.reader=threading.Thread(target=read,daemon=True);self.reader.start()
    def close(self):
        if self.process.poll() is None:self.process.terminate()
        self.process.wait(timeout=5);self.reader.join(timeout=1);self.process.stdout.close()

class EndToEnd(unittest.TestCase):
    def setUp(self):
        config=json.loads((args.c2_root/'configs/team-demo.json').read_text(encoding='utf-8-sig'))
        self.engine=Engine(config,Detector(args.detector),port=0)
        self.obs=Native('obs',self.engine.address[1]);self.eff=Native('eff',self.engine.address[1])
        self.addCleanup(self.obs.close);self.addCleanup(self.eff.close);self.addCleanup(self.engine.close)
        wait(lambda:self.engine.snapshot()['devices'].get('201',{}).get('installed') and self.engine.snapshot()['observation']=='CURRENT')
    def aim(self):
        def start():
            with self.engine.lock:
                if self.engine.cloud.state!='CURRENT':return False
                self.engine.aim('201');return True
        wait(start)
        wait(lambda:sum(line.startswith('POINT ') for line in self.eff.lines)>=3)
    def stop(self):
        self.engine.stop('201');wait(lambda:self.engine.snapshot()['controls']['201']['state']=='STOPPED')
    def test_raw_detection_aim_stop_and_measurement_restart(self):
        self.aim()
        self.assertTrue(self.engine.snapshot()['devices']['201']['angles_current'])
        self.stop();count=sum(line.startswith('POINT ') for line in self.eff.lines)
        old=self.engine.peers['101']['collection'];self.engine.collect(False)
        wait(lambda:self.engine.peers['101'].get('measurement_result')=='COMPLETED' and not self.engine.peers['101']['collecting'])
        self.engine.collect(True);wait(lambda:self.engine.snapshot()['observation']=='CURRENT')
        self.assertNotEqual(old,self.engine.peers['101']['collection'])
        self.assertEqual(sum(line.startswith('POINT ') for line in self.eff.lines),count)
        self.assertFalse([e for e in self.engine.events if e['event']=='runtime_error'])
    def test_console_observer_move_stops_aim_then_restarts_collection(self):
        self.aim()
        old=self.engine.peers['101']['collection']
        self.engine.obs_point(20,10)
        wait(lambda:self.engine.snapshot()['observer_action']['state']=='COMPLETED')
        wait(lambda:self.engine.snapshot()['observation']=='DISPLAY_ONLY')
        snapshot=self.engine.snapshot()
        self.assertEqual(snapshot['controls']['201']['state'],'STOPPED')
        self.assertNotEqual(old,self.engine.peers['101']['collection'])
        self.assertAlmostEqual(snapshot['devices']['101']['pan'],20)
        with self.assertRaises(ValueError):self.engine.aim('201')
        self.engine.obs_point(0,0)
        wait(lambda:self.engine.snapshot()['observer_action']['state']=='COMPLETED')
        wait(lambda:self.engine.snapshot()['observation']=='CURRENT')
        self.engine.obs_stop()
        wait(lambda:self.engine.snapshot()['observer_action']['state']=='COMPLETED')
        self.assertFalse(self.engine.snapshot()['devices']['101']['collecting'])
        # STOP closes the OBS gate; the next CLI move must open a new epoch/mode.
        self.engine.obs_point(0,0)
        wait(lambda:self.engine.snapshot()['observer_action']['state']=='COMPLETED')
        wait(lambda:self.engine.snapshot()['observation']=='CURRENT')

    def test_observer_stop_cancels_pending_move_without_restarting_collection(self):
        self.engine.obs_point(20,10)
        wait(lambda:self.engine.snapshot()['observer_action']['state']=='MOVING')
        self.engine.obs_stop()
        wait(lambda:self.engine.snapshot()['observer_action']['state']=='COMPLETED')
        self.assertFalse(self.engine.snapshot()['devices']['101']['collecting'])
        count=sum(line.startswith('POINT ') for line in self.obs.lines)
        time.sleep(.35)
        self.assertEqual(sum(line.startswith('POINT ') for line in self.obs.lines),count)
        self.assertFalse(self.engine.snapshot()['devices']['101']['collecting'])

    def test_observer_process_loss_stops_effector(self):
        self.aim();self.obs.close()
        wait(lambda:self.engine.snapshot()['controls']['201']['state']=='STOPPED')
        count=sum(line.startswith('POINT ') for line in self.eff.lines);time.sleep(.3)
        self.assertEqual(sum(line.startswith('POINT ') for line in self.eff.lines),count)

class ConsoleProcess(unittest.TestCase):
    def test_actual_cli_prompt_observer_aim_and_stop(self):
        reserve=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);reserve.bind(('127.0.0.1',0));port=reserve.getsockname()[1];reserve.close()
        console=subprocess.Popen([sys.executable,str(args.c2_root/'scripts/run_team_mvp.py'),'--asset-port',str(port),'--detector',str(args.detector)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8')
        lines=queue.Queue()
        def read():
            for line in console.stdout:lines.put(line.strip())
        reader=threading.Thread(target=read,daemon=True);reader.start()
        def close():
            if console.poll() is None:
                try:console.stdin.write('quit\n');console.stdin.flush();console.wait(timeout=5)
                except (OSError,subprocess.TimeoutExpired):console.terminate();console.wait(timeout=5)
            reader.join(timeout=1);console.stdin.close();console.stdout.close()
        obs=eff=None
        try:
            self.assertIn('Team ICD',lines.get(timeout=5))
            obs=Native('obs',port);eff=Native('eff',port)
            def command(value):
                console.stdin.write(value+'\n');console.stdin.flush()
                return lines.get(timeout=5).removeprefix('c2> ')
            def status(predicate):
                latest={}
                def check():
                    nonlocal latest
                    latest=json.loads(command('status'))
                    self.assertIsNone(console.poll())
                    return predicate(latest)
                wait(check,8);return latest
            status(lambda s:s['devices'].get('201',{}).get('installed') and s['observation']=='CURRENT')
            self.assertIn('OBS:',command('obs point 10 5'))
            status(lambda s:s['observer_action']['state']=='COMPLETED')
            report=json.loads(command('obs status'));self.assertAlmostEqual(report['device']['pan'],10)
            command('obs point 0 0')
            status(lambda s:s['observer_action']['state']=='COMPLETED' and s['observation']=='CURRENT')
            self.assertIn('OPENING',command('aim 201'))
            status(lambda s:s['controls'].get('201',{}).get('sequence',0)>=2)
            command('stop 201');status(lambda s:s['controls']['201']['state']=='STOPPED')
            command('obs stop');status(lambda s:s['observer_action']['state']=='COMPLETED' and not s['devices']['101']['collecting'])
            command('quit');console.wait(timeout=5);self.assertEqual(console.returncode,0)
        finally:
            close()
            if obs:obs.close()
            if eff:eff.close()

class WirePeer:
    def __init__(self,role,uart=None):
        self.role=role;self.id='101' if role=='obs' else '201';self.boot=uid();self.session=uid();self.sequence=0;self.last_hb=0
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('127.0.0.1',0));self.sock.settimeout(.03)
        self.native=Native(role,self.sock.getsockname()[1],uart);self.address=None;self.messages=[]
        end=time.monotonic()+5
        while time.monotonic()<end:
            try:
                data,address=self.sock.recvfrom(65536);e=decode(data,'C2')
            except socket.timeout:continue
            if e.HasField('register_request'):
                q=e.register_request;self.address=(address[0],q.receive_port)
                self.send(pb.RegisterResult(outcome=outcome(q.request_id),device_id=self.id,boot_id=q.boot_id,connection_session_id=self.session,c2_boot_id=self.boot),session=False)
                break
        if not self.address:raise AssertionError('native registration timed out: '+repr(self.native.lines))
        self.until(lambda e:e.HasField('device_status') and e.device_status.motion_state==pb.MOTION_STATE_STOPPED)
        self.send(pb.InstallationSet(request_id=uid(),device_id=self.id,installation_revision=1,
            position_m=pb.Position(),sensor_offset_m=pb.Position(),mount_rotation=[1,0,0,0,1,0,0,0,1],axis_calibration_id='synthetic-'+role+'-zero-v1'))
        self.until(lambda e:e.HasField('installation_result') and e.installation_result.outcome.result==pb.REQUEST_RESULT_COMPLETED)
    def send(self,body,session=True,packet=None):
        value=packet or encode(envelope('C2',self.id,body,self.session if session else None))
        self.sock.sendto(value,self.address);return value
    def until(self,predicate,timeout=4):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            now=time.monotonic()
            if now-self.last_hb>=.25:
                self.last_hb=now;self.sequence+=1;self.send(pb.Heartbeat(boot_id=self.boot,sequence=self.sequence))
            try:data,_=self.sock.recvfrom(65536)
            except socket.timeout:continue
            e=decode(data,'C2');self.messages.append(e)
            if predicate(e):return e
        raise AssertionError('native result timed out: '+repr(self.native.lines))
    def result(self,request,kind):
        return self.until(lambda e:e.WhichOneof('body')==kind and getattr(e,kind).outcome.request_id==request and getattr(e,kind).outcome.result!=pb.REQUEST_RESULT_ACCEPTED)
    def close(self):self.native.close();self.sock.close()

class DeviceContract(unittest.TestCase):
    def test_drain_finishes_last_point_and_closes_gate(self):
        peer=WirePeer('eff');self.addCleanup(peer.close);control=uid()
        open_request=pb.ControlOpen(request_id=uid(),control_session_id=control,control_epoch=1,installation_revision=1,mode=pb.CONTROL_MODE_MANUAL)
        peer.send(open_request);peer.result(open_request.request_id,'control_result')
        expiry=(dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=1)).isoformat(timespec='milliseconds').replace('+00:00','Z')
        move=pb.PointingCommand(request_id=uid(),control_session_id=control,control_epoch=1,command_sequence=1,operation=pb.PointingCommand.OPERATION_POINT,issued_at=utc(),expires_at=expiry,pan_deg=10,tilt_deg=5,effector_installation_revision=1,origin=pb.CONTROL_MODE_MANUAL)
        peer.send(move)
        peer.until(lambda e:e.HasField('pointing_result') and e.pointing_result.outcome.request_id==move.request_id and e.pointing_result.outcome.result==pb.REQUEST_RESULT_ACCEPTED)
        drain=pb.PointingCommand(request_id=uid(),control_session_id=control,control_epoch=1,command_sequence=2,operation=pb.PointingCommand.OPERATION_DRAIN,issued_at=utc())
        peer.send(drain);result=peer.result(drain.request_id,'pointing_result').pointing_result
        self.assertEqual(result.outcome.result,pb.REQUEST_RESULT_COMPLETED)
        self.assertEqual((result.drained_request_id,result.drained_sequence,result.drained_epoch),(move.request_id,1,1))
        after=copy.deepcopy(move);after.request_id=uid();after.command_sequence=3;peer.send(after)
        self.assertEqual(peer.result(after.request_id,'pointing_result').pointing_result.outcome.reason_code,pb.REASON_CODE_BUSY)
        self.assertEqual(sum(line.startswith('POINT ') for line in peer.native.lines),1)

    def test_effector_duplicate_conflict_and_stop_fence(self):
        peer=WirePeer('eff');self.addCleanup(peer.close);control=uid()
        q=pb.ControlOpen(request_id=uid(),control_session_id=control,control_epoch=1,installation_revision=1,mode=pb.CONTROL_MODE_MANUAL)
        peer.send(q);self.assertEqual(peer.result(q.request_id,'control_result').control_result.outcome.result,pb.REQUEST_RESULT_COMPLETED)
        expiry=(dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=1)).isoformat(timespec='milliseconds').replace('+00:00','Z')
        move=pb.PointingCommand(request_id=uid(),control_session_id=control,control_epoch=1,command_sequence=1,operation=pb.PointingCommand.OPERATION_POINT,issued_at=utc(),expires_at=expiry,pan_deg=10,tilt_deg=5,effector_installation_revision=1,origin=pb.CONTROL_MODE_MANUAL)
        packet=peer.send(move);peer.send(move,packet=packet)
        self.assertEqual(peer.result(move.request_id,'pointing_result').pointing_result.outcome.result,pb.REQUEST_RESULT_COMPLETED)
        self.assertEqual(sum(line.startswith('POINT ') for line in peer.native.lines),1)
        conflict=copy.deepcopy(move);conflict.pan_deg=15;peer.send(conflict)
        self.assertEqual(peer.result(move.request_id,'pointing_result').pointing_result.outcome.reason_code,pb.REASON_CODE_ID_CONFLICT)
        stop=pb.PointingCommand(request_id=uid(),control_session_id=control,control_epoch=1,command_sequence=2,operation=pb.PointingCommand.OPERATION_STOP,issued_at=utc())
        peer.send(stop);self.assertEqual(peer.result(stop.request_id,'pointing_result').pointing_result.actual_motion_state,pb.MOTION_STATE_STOPPED)
        stale=copy.deepcopy(move);stale.request_id=uid();peer.send(stale)
        self.assertEqual(peer.result(stale.request_id,'pointing_result').pointing_result.outcome.reason_code,pb.REASON_CODE_STALE_COMMAND)
        self.assertEqual(sum(line.startswith('POINT ') for line in peer.native.lines),1)
    def test_obs_uart_motion_is_separate_from_measurement(self):
        peer=WirePeer('obs');self.addCleanup(peer.close);control=uid()
        mode=pb.ObsModeCommand(request_id=uid(),obs_control_session_id=control,control_epoch=1,observation_mode_revision=1,mode=pb.OBSERVATION_MODE_FIXED_VIEW,installation_revision=1)
        peer.send(mode);peer.result(mode.request_id,'obs_mode_result')
        expiry=(dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=1)).isoformat(timespec='milliseconds').replace('+00:00','Z')
        move=pb.ObsMotionCommand(request_id=uid(),obs_control_session_id=control,control_epoch=1,command_sequence=1,operation=pb.ObsMotionCommand.OPERATION_POINT,purpose=pb.ObsMotionCommand.PURPOSE_FIXED_POSITION,issued_at=utc(),expires_at=expiry,pan_deg=20,tilt_deg=10,observation_mode_revision=1,installation_revision=1)
        peer.send(move)
        peer.until(lambda e:e.HasField('obs_motion_result') and e.obs_motion_result.outcome.request_id==move.request_id and e.obs_motion_result.outcome.result==pb.REQUEST_RESULT_ACCEPTED)
        # Acceptance of a move is not a stopped-position confirmation.
        premature=pb.MeasurementCommand(request_id=uid(),operation=pb.MeasurementCommand.OPERATION_START,collection_session_id=uid(),installation_revision=1)
        peer.send(premature)
        rejected=peer.result(premature.request_id,'measurement_result').measurement_result.outcome
        self.assertEqual(rejected.reason_code,pb.REASON_CODE_BUSY)
        self.assertFalse(any(e.HasField('point_cloud_chunk') for e in peer.messages))
        self.assertEqual(peer.result(move.request_id,'obs_motion_result').obs_motion_result.outcome.result,pb.REQUEST_RESULT_COMPLETED)
        status=peer.until(lambda e:e.HasField('device_status') and abs(e.device_status.angles.pan_deg-20)<.01)
        self.assertFalse(status.device_status.measurement_state==pb.MEASUREMENT_STATE_COLLECTING)
        collection=uid();start=pb.MeasurementCommand(request_id=uid(),operation=pb.MeasurementCommand.OPERATION_START,collection_session_id=collection,installation_revision=1)
        peer.send(start);peer.result(start.request_id,'measurement_result')
        chunk=peer.until(lambda e:e.HasField('point_cloud_chunk')).point_cloud_chunk
        self.assertEqual(chunk.total_points,chunk.width*chunk.height)
        busy=copy.deepcopy(move);busy.request_id=uid();busy.command_sequence=2;peer.send(busy)
        self.assertEqual(peer.result(busy.request_id,'obs_motion_result').obs_motion_result.outcome.reason_code,pb.REASON_CODE_BUSY)

@unittest.skipUnless(os.name=='posix', 'requires Linux PTY and the real UART backend')
class NativeUart(unittest.TestCase):
    def setUp(self):
        from uart_plant import Plant
        self.plant=Plant();self.addCleanup(self.plant.close)
        self.peer=WirePeer('eff',self.plant.path);self.addCleanup(self.peer.close)
        self.control=uid()
        q=pb.ControlOpen(request_id=uid(),control_session_id=self.control,control_epoch=1,installation_revision=1,mode=pb.CONTROL_MODE_MANUAL)
        self.peer.send(q);self.peer.result(q.request_id,'control_result')
    def move(self):
        expiry=(dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=1)).isoformat(timespec='milliseconds').replace('+00:00','Z')
        q=pb.PointingCommand(request_id=uid(),control_session_id=self.control,control_epoch=1,command_sequence=1,operation=pb.PointingCommand.OPERATION_POINT,issued_at=utc(),expires_at=expiry,pan_deg=10,tilt_deg=5,effector_installation_revision=1,origin=pb.CONTROL_MODE_MANUAL)
        self.peer.send(q)
        self.peer.until(lambda e:e.HasField('pointing_result') and e.pointing_result.outcome.request_id==q.request_id and e.pointing_result.outcome.result==pb.REQUEST_RESULT_ACCEPTED)
        return q
    def test_real_uart_move_feedback_and_stop(self):
        q=self.move()
        r=self.peer.result(q.request_id,'pointing_result').pointing_result
        self.assertEqual(r.outcome.result,pb.REQUEST_RESULT_COMPLETED)
        self.assertAlmostEqual(self.plant.pan,2048+10*4096/360,delta=1)
        self.assertAlmostEqual(self.plant.tilt,2048+5*4096/360,delta=1)
        self.assertGreater(self.plant.moves,0)
        stop=pb.PointingCommand(request_id=uid(),control_session_id=self.control,control_epoch=1,command_sequence=2,operation=pb.PointingCommand.OPERATION_STOP,issued_at=utc())
        self.peer.send(stop)
        self.assertEqual(self.peer.result(stop.request_id,'pointing_result').pointing_result.actual_motion_state,pb.MOTION_STATE_STOPPED)
    def test_lost_uart_feedback_cannot_complete_point(self):
        self.plant.respond=False
        q=self.move()
        r=self.peer.result(q.request_id,'pointing_result').pointing_result
        self.assertEqual(r.outcome.result,pb.REQUEST_RESULT_FAILED)
        self.assertEqual(r.outcome.reason_code,pb.REASON_CODE_ANGLE_READ_FAILED)

if __name__=='__main__':unittest.main(argv=[sys.argv[0]]+remaining,verbosity=2)
