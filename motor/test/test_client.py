import os
import pathlib
import socket
import subprocess
import sys
import threading
import unittest
from unittest.mock import patch
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from client import (Client, encode, decode, EF01, EF02, EF03, EF04, EF05,
                        FE01, FE02, FE03, FE04, FE05, make_id, split_id, ECS, FCS, RDR)


class CodecTests(unittest.TestCase):
    def test_asset_ids_unsigned_sequences(self):
        for request, response in ((EF01,FE01),(EF02,FE02),(EF03,FE03),(EF04,FE04),(EF05,FE05)):
            fields=(1,30,180) if response==FE02 else (0,1) if response==FE04 else (0,)
            data=encode(response,65535,32768,fields)
            self.assertEqual(data[:2],response.to_bytes(2,'big'))
            reply=decode(data)
            self.assertEqual((reply['id'],reply['tx'],reply['rx']),(response,65535,32768))
        self.assertEqual(encode(FE01,1,2,(0,)).hex(),'fe01000100020000')
        self.assertEqual(split_id(EF03),(ECS,FCS,3))
        self.assertEqual(make_id(ECS,RDR,1),0xED01)
        self.assertEqual(split_id(0xDE01),(RDR,ECS,1))
        for args in ((16,FCS,1),(ECS,-1,1),(ECS,FCS,256)):
            with self.assertRaises(ValueError): make_id(*args)
        for header in ('0000','8000','0001','0002','0003','0004','0005','ffff','fffe','fffd','fffc','fffb','ed01','de01','ef06'):
            with self.assertRaises(ValueError): decode(bytes.fromhex(header+'000100000000'))

    def test_golden_and_lengths(self):
        self.assertEqual(encode(EF03, 1, values=(30, 180)).hex(), 'ef0300010000001e00b40000')
        for mid, fields, size in ((EF01, (), 8), (EF02, (), 8), (EF03, (30,180), 12),
                                  (EF04, (1,), 8), (EF05, (), 8), (FE01,(0,),8),
                                  (FE02,(1,-1,180),12), (FE03,(0,),8),
                                  (FE04,(0,1),12), (FE05,(0,),8)):
            data=encode(mid,65535, values=fields)
            self.assertEqual(len(data),size)
            self.assertEqual(decode(data)['id'],mid)
            for bad in (data[:-1],data+b'\0'):
                with self.assertRaises(ValueError): decode(bad)
        self.assertEqual(decode(encode(FE02,0,values=(1,-1,180)))['elevation'],-1)

    def test_invalid_reply(self):
        for bad in (b'', b'\0'*8, encode(FE03,1,values=(99,)),
                    encode(FE04,1,values=(0,2)), bytes.fromhex('fe0400010000000000010001')):
            with self.assertRaises(ValueError): decode(bad)


class HeartbeatTests(unittest.TestCase):
    def test_three_consecutive_losses_reconnect(self):
        c=Client('127.0.0.1')
        c.connected=True
        try:
            with patch.object(c,'request',side_effect=[TimeoutError(),TimeoutError(),TimeoutError(),{'result':0}]) as req:
                self.assertIsNone(c.poll_once()); self.assertIsNone(c.poll_once())
                self.assertEqual(c.poll_once(),{'result':0})
                self.assertTrue(c.connected); self.assertEqual(c.missed,0)
                self.assertEqual([x.args[0] for x in req.call_args_list],[EF02]*3+[EF01])
        finally: c.close()

    def test_success_resets_loss_count(self):
        c=Client('127.0.0.1'); c.connected=True
        try:
            with patch.object(c,'request',side_effect=[TimeoutError(),{'flags':1},TimeoutError()]):
                c.poll_once(); c.poll_once(); c.poll_once()
                self.assertEqual(c.missed,1)
        finally: c.close()


class UdpTests(unittest.TestCase):
    def test_retry_matching_and_wrap(self):
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server.bind(('127.0.0.1',0)); server.settimeout(.1)
        stop=threading.Event(); seen=[]
        def serve():
            while not stop.is_set():
                try: data,peer=server.recvfrom(1024)
                except socket.timeout: continue
                seen.append(data)
                if len(seen)==1: continue
                q=decode(data)
                server.sendto(encode(FE03,0,(q['tx']+1)&65535,(0,)),peer)
                server.sendto(encode(FE03,1,q['tx'],(1,)),peer)
        t=threading.Thread(target=serve); t.start()
        c=Client(*server.getsockname(),timeout=.04); c.sequence=65535; c.connected=True
        try:
            self.assertEqual(c.command(EF03,(30,180))['result'],1)
            self.assertEqual(seen[0],seen[1]); self.assertEqual(c.sequence,0)
        finally:
            c.close(); stop.set(); t.join(); server.close()

    @unittest.skipUnless(os.environ.get('ICD_TEST_EXE'),'Set ICD_TEST_EXE for actual C++ interoperability')
    def test_cpp_controller_interoperability(self):
        proc=subprocess.Popen([os.environ['ICD_TEST_EXE'],'--bridge'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
        def exchange(data):
            proc.stdin.write(data.hex()+'\n'); proc.stdin.flush()
            return decode(bytes.fromhex(proc.stdout.readline().strip()))
        try:
            self.assertEqual(exchange(encode(EF01,0))['result'],0)
            status=exchange(encode(EF02,1))
            self.assertEqual((status['elevation'],status['azimuth']),(30,180))
            move=encode(EF03,2,values=(31,181))
            response=exchange(move)
            self.assertEqual(response['result'],0)
            self.assertEqual(exchange(move),response)
            self.assertEqual(exchange(encode(EF05,3))['result'],1)
            self.assertEqual(exchange(encode(EF04,4,values=(2,)))['result'],2)
            self.assertEqual(exchange(encode(EF04,5,values=(0,)))['laser'],0)
        finally:
            proc.stdin.close(); proc.wait(timeout=5); proc.stdout.close()
            self.assertEqual(proc.returncode,0)


if __name__=='__main__': unittest.main()
