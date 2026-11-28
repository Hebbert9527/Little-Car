import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'pc'))
import protocol as p

class ProtocolTests(unittest.TestCase):
    def test_radar_uses_map_heading_and_right_side(self):
        import math
        for angle,heading,x,y in ((0,math.pi/2,0,1), (9000,math.pi/2,1,0),
                                  (0,-math.pi/2,0,-1), (9000,0,0,-1)):
            actual_x,actual_y=p.radar_world_axes(angle,1000,heading)
            self.assertAlmostEqual(actual_x,x)
            self.assertAlmostEqual(actual_y,y)

    def test_maze_command_and_map_row(self):
        self.assertEqual(p.command('maze,0,1'), b'MAZE,0.000000,1.000000\n')
        row=b'M,2,48,-15,22,'+b'#.?'+b'.'*46
        self.assertEqual(p.parse_line(row).fields[:4], ('2','48','-15','22'))
        for invalid in (row[:-1], row.replace(b'48',b'49'), row+b'x'):
            with self.assertRaises(ValueError): p.parse_line(invalid)
    def test_full_five_cm_row_fragmentation(self):
        row=b'M,4294967295,212,-40,40,50,106,4294967295,'+b'.'*212+b'#\n'
        decoder=p.StreamDecoder()
        frames=[]
        for i in range(0,len(row),17): frames.extend(decoder.feed(row[i:i+17]))
        self.assertEqual(decoder.bad_lines,0)
        self.assertEqual(len(frames),1)
        self.assertEqual(len(frames[0].fields[-1]),213)

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
        self.assertEqual(p.command('SPEED,0.18'), b'SPEED,0.180\n')
        for text in ('SPEED,0.07','SPEED,0.401','SPEED,nan'):
            with self.assertRaises(ValueError): p.command(text)
        self.assertEqual(p.command('GOTO,0.3,-0.2'), b'GOTO,0.300000,-0.200000\n')
        self.assertEqual(p.command('stop'), b'STOP\n')
        self.assertEqual(p.command('RESET'), b'RESET\n')
        self.assertEqual(p.command('return'), b'RETURN\n')
        self.assertEqual(p.command('RETURN'), b'RETURN\n')
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
    def test_command_coordinates_match_firmware_five_metre_bounds(self):
        for verb in ('GOTO','MAZE'):
            for x,y in ((5,-5),(-5,5),(0,5),(5,5)):
                self.assertEqual(p.command(f'{verb},{x},{y}'),f'{verb},{x:.6f},{y:.6f}\n'.encode())
            for x,y in ((5.001,0),(0,-5.001),(float('nan'),0)):
                with self.assertRaises(ValueError): p.command(f'{verb},{x},{y}')



class RoundedImuTests(unittest.TestCase):
    def test_rounded_pi_heading_is_valid(self):
        self.assertEqual(p.parse_line(b'I,0,10,18000,0').heading_cdeg,18000)

if __name__ == '__main__': unittest.main()

