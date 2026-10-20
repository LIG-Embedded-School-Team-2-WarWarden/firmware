"""Linux PTY motor-controller fixture. Runs the production UART protocol, no hardware."""
import os
import pty
import select
import threading

class Plant:
    def __init__(self):
        self.master,self.slave=pty.openpty()
        self.path=os.ttyname(self.slave)
        self.pan=self.tilt=2048
        self.pan_target=self.tilt_target=2048
        self.respond=True;self.moves=0;self.running=True
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        pending=b''
        while self.running:
            if not select.select([self.master],[],[],.05)[0]:continue
            try:pending+=os.read(self.master,256)
            except OSError:return
            while len(pending)>=4:
                if pending[:2]!=b'\xff\xff':pending=pending[1:];continue
                n=pending[3]+4
                if len(pending)<n:break
                packet,pending=pending[:n],pending[n:]
                if sum(packet[2:])&255!=255:continue
                if n==22 and packet[4:7]==bytes([131,42,6]):
                    self.pan_target=int.from_bytes(packet[8:10],'little')
                    self.tilt_target=int.from_bytes(packet[15:17],'little');self.moves+=1
                elif n==10 and packet[4:7]==bytes([130,56,8]) and self.respond:
                    def advance(value,target):return min(target,value+20) if value<target else max(target,value-20)
                    self.pan=advance(self.pan,self.pan_target);self.tilt=advance(self.tilt,self.tilt_target)
                    for axis,ticks in [(2,self.tilt),(1,self.pan)]:
                        reply=bytes([255,255,axis,10,0,ticks&255,ticks>>8,0,0,0,0,0,0])
                        os.write(self.master,reply+bytes([(~sum(reply[2:]))&255]))
    def close(self):
        self.running=False;self.thread.join(timeout=1)
        os.close(self.master);os.close(self.slave)
