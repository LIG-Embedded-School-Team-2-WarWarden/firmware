"""Run the real UART parser against a Linux pseudo-terminal, no motor required."""
import ctypes
import os
import pty
import select
import sys
import threading
import time
import unittest

driver=ctypes.CDLL(sys.argv.pop(1))
driver.test_motor_open.argtypes=[ctypes.c_char_p]
driver.test_motor_move.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int]
driver.test_motor_read.argtypes=[ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_int)]

def response(axis,ticks,corrupt=False):
    data=[255,255,axis,10,0,ticks&255,ticks>>8,0,0,0,0,0,0]
    data.append((~sum(data[2:]))&255)
    if corrupt:data[-1]^=1
    return bytes(data)

class Uart(unittest.TestCase):
    def setUp(self):
        self.master,self.slave=pty.openpty()
        self.assertEqual(driver.test_motor_open(os.ttyname(self.slave).encode()),1)
    def tearDown(self):
        driver.test_motor_close();os.close(self.master);os.close(self.slave)
    def read(self,replies):
        def serve():
            request=b''
            while len(request)<10:
                if not select.select([self.master],[],[],1)[0]:return
                request+=os.read(self.master,10-len(request))
            for packet in replies:
                os.write(self.master,packet[:5]);time.sleep(.005);os.write(self.master,packet[5:])
        worker=threading.Thread(target=serve);worker.start()
        pan,tilt=ctypes.c_int(),ctypes.c_int();start=time.monotonic()
        flags=driver.test_motor_read(ctypes.byref(pan),ctypes.byref(tilt))
        elapsed=time.monotonic()-start;worker.join(timeout=2)
        self.assertLess(elapsed,.6)
        return flags,pan.value,tilt.value
    def test_split_responses_in_reverse_axis_order(self):
        self.assertEqual(self.read([response(2,2500),response(1,1900)]),(3,1900,2500))
    def test_checksum_failure_is_not_zero_angle(self):
        flags,_,tilt=self.read([response(1,1900,True),response(2,2500)])
        self.assertEqual(flags,2);self.assertEqual(tilt,2500)
    def test_move_packet_matches_existing_motor_protocol(self):
        self.assertEqual(driver.test_motor_move(1900,2500,200,300),1)
        self.assertTrue(select.select([self.master],[],[],1)[0])
        packet=os.read(self.master,64)
        self.assertEqual(len(packet),22)
        self.assertEqual(packet[:8],bytes([255,255,254,18,131,42,6,1]))
        self.assertEqual(packet[8:14],bytes([108,7,0,0,200,0]))
        self.assertEqual(packet[14:21],bytes([2,196,9,0,0,44,1]))
        self.assertEqual(sum(packet[2:])&255,255)
    def test_unknown_axis_is_ignored(self):
        flags,pan,tilt=self.read([response(3,1234),response(1,1900),response(2,2500)])
        self.assertEqual((flags,pan,tilt),(3,1900,2500))
    def test_garbage_before_response_is_resynchronized(self):
        self.assertEqual(self.read([b'bad bytes'+response(1,1900),response(2,2500)]),(3,1900,2500))
    def test_old_buffered_responses_are_not_fresh_samples(self):
        os.write(self.master,response(1,1900)+response(2,2500))
        time.sleep(.02)
        self.assertEqual(self.read([])[0],0)
    def test_missing_responses_are_bounded(self):
        self.assertEqual(self.read([])[0],0)

if __name__=='__main__':unittest.main(verbosity=2)
