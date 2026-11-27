import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest
import threading
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'pc'))

class MonitorTests(unittest.TestCase):
    def test_independent_sequence_spaces_and_latest_state(self):
        import monitor as m
        r=m.Receiver()
        r.feed(b'P,100,10,0,1000,20,1\nN,1,10,2,IDLE,0,0,1571,200,300,0,0\nI,101,10,0,0\nD,10,ok\nA,2,OK,accepted\n')
        self.assertEqual(r.snapshot()['frames'],5)
        self.assertEqual(r.snapshot()['missing'],0)
        self.assertEqual(r.latest_nav.cmd_id,2)
        self.assertEqual(r.latest_ack.reason,'accepted')
        self.assertEqual(r.diagnostics['D'].fields,('10','ok'))
    def test_commands_are_not_queued_while_disconnected(self):
        import monitor as m
        r=m.Receiver()
        with self.assertRaises(ConnectionError): r.send('STOP')
        r.set_status(True,'test')
        r.send('STOP')
        self.assertEqual(r.commands.get_nowait(),b'STOP\n')
        r.send('RESET')
        r.set_status(False,'disconnected')
        self.assertTrue(r.commands.empty())
    def test_demo_needs_no_optional_packages(self):
        path=Path(__file__).resolve().parents[2]/'pc'/'monitor.py'
        result=subprocess.run([sys.executable,'-B',str(path),'--demo','--headless','--duration','0.15'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('DEMO',result.stdout)
        self.assertIn('frames',result.stdout)


class IntegratedSequenceTests(unittest.TestCase):
    def test_point_and_imu_have_independent_sequences(self):
        import monitor as m
        r=m.Receiver()
        r.feed(b'P,0,10,0,1000,20,1\nI,0,10,0,0\nP,1,20,0,1000,20,1\nI,1,20,0,0\n')
        self.assertEqual(r.events.qsize(),4)
        self.assertEqual(r.snapshot()['missing'],0)

if __name__ == '__main__': unittest.main()

