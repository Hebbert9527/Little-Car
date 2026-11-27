import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'pc'))
import protocol as p

class ProtocolTests(unittest.TestCase):
    def test_navigation_and_ack(self):
        frame = p.parse_line(b'N,3,50,9,TRACKING,100,-200,1571,400,500,2,0')
        self.assertEqual((frame.x_mm, frame.y_mm, frame.target_y_mm), (100, -200, 500))
        self.assertEqual(p.parse_line(b'A,9,OK,accepted').status, 'OK')
    def test_diagnostics_are_preserved_without_guessing_schema(self):
        for kind in (b'D', b'T', b'B'):
            frame = p.parse_line(kind + b',100,name,12,34')
            self.assertEqual(frame.fields, ('100', 'name', '12', '34'))
    def test_invalid_navigation(self):
        for raw in (b'N,3,50,9,INVALID,0,0,0,0,0,0,0',
                    b'N,3,50,9,IDLE,nan,0,0,0,0,0,0', b'D,\xff', b'A,9,OK'):
            with self.assertRaises(ValueError): p.parse_line(raw)
    def test_fragmentation_and_recovery(self):
        d = p.StreamDecoder(max_line=100)
        self.assertEqual(d.feed(b'N,3,50,9,IDLE,0'), [])
        self.assertEqual(len(d.feed(b',0,1571,0,0,0,0\nP,4,50,9000,1000,20,1\n')), 2)
        self.assertEqual(d.feed(b'x'*102 + b'\nI,5,50,0,0\n')[0].seq, 5)
        self.assertEqual(d.bad_lines, 1)
    def test_command_encoding(self):
        self.assertEqual(p.command('GOTO,0.3,-0.2'), b'GOTO,0.300000,-0.200000\n')
        self.assertEqual(p.command('stop'), b'STOP\n')
        self.assertEqual(p.command('RESET'), b'RESET\n')
        for text in ('GOTO,nan,1', 'GOTO,1,inf', 'STOP\nRESET', 'NOPE', 'GOTO,1'):
            with self.assertRaises(ValueError): p.command(text)
    def test_sequences_wrap(self):
        s=p.SequenceTracker()
        for n in (0xfffffffe, 0xffffffff, 1): self.assertTrue(s.observe(n))
        self.assertEqual(s.missing, 1)
        self.assertFalse(s.observe(1))
        self.assertFalse(s.observe(0))


class DiagnosticTests(unittest.TestCase):
    def test_schedule_and_bus_diagnostic_fields(self):
        frame=p.parse_line(b'S,100,10,9,1,0,1,0')
        self.assertEqual(frame.kind,'S')
        self.assertEqual(frame.fields[1:3],('10','9'))
    def test_task_frames_keep_each_task(self):
        import monitor as m
        r=m.Receiver()
        r.feed(b'T,100,1,IMU,5,1024,10,1000,200,50,0,400\nT,200,2,OLED,2,1024,1,2000,300,10,0,500\n')
        self.assertEqual(set(r.tasks),{'1','2'})

class FirmwareBoundsTests(unittest.TestCase):
    def test_command_coordinates_match_firmware_two_metre_bounds(self):
        with self.assertRaises(ValueError): p.command('GOTO,2.01,0')
        self.assertEqual(p.command('GOTO,2,-2'),b'GOTO,2.000000,-2.000000\n')



class RoundedImuTests(unittest.TestCase):
    def test_rounded_pi_heading_is_valid(self):
        self.assertEqual(p.parse_line(b'I,0,10,18000,0').heading_cdeg,18000)

if __name__ == '__main__': unittest.main()

