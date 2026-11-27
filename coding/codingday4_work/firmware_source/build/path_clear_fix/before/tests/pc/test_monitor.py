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
    def test_ble_connection_error_remains_visible_during_retry(self):
        import contextlib
        import io
        from types import SimpleNamespace
        from unittest.mock import patch
        import monitor as m
        stop=threading.Event()
        receiver=m.Receiver()
        class MissingServiceClient:
            def __init__(self,*args,**kwargs): pass
            async def __aenter__(self):
                raise RuntimeError('UART service unavailable')
            async def __aexit__(self,*args): pass
        async def finish_retry(_): stop.set()
        errors=io.StringIO()
        with patch.dict(sys.modules,{'bleak':SimpleNamespace(BleakClient=MissingServiceClient)}), \
                patch('asyncio.sleep',finish_retry), contextlib.redirect_stderr(errors):
            m.ble_source(receiver,stop,SimpleNamespace(ble='test-device'))
        self.assertFalse(receiver.snapshot()['connected'])
        self.assertIn('RuntimeError: UART service unavailable',receiver.snapshot()['status'])
        self.assertIn('UART service unavailable',errors.getvalue())

    def test_demo_needs_no_optional_packages(self):
        path=Path(__file__).resolve().parents[2]/'pc'/'monitor.py'
        result=subprocess.run([sys.executable,'-B',str(path),'--demo','--headless','--duration','0.15'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('DEMO',result.stdout)
        self.assertIn('frames',result.stdout)


class RunHistoryTests(unittest.TestCase):
    def test_reset_and_new_command_do_not_join_paths(self):
        import monitor as m
        history=m.RunHistory()
        history.update(m.Navigation(0,50,1,'TRACKING',0,0,1571,0,1000,0,0))
        history.update(m.Navigation(1,100,1,'ARRIVED',-125,991,1567,0,1000,0,0))
        history.update(m.Ack(2,'STOPPED','STOP_LATCHED'))
        self.assertEqual(list(history.points),[(0.0,0.0),(-0.125,0.991)])
        history.update(m.Ack(3,'OK','RESET_PENDING'))
        history.update(m.Navigation(2,150,0,'STOPPED',0,0,1571,0,0,0,0))
        self.assertFalse(history.points)
        history.update(m.Navigation(3,200,4,'TRACKING',0,0,1571,0,1500,0,0))
        history.update(m.Navigation(4,250,4,'ARRIVED',115,1567,1626,0,1500,0,0))
        self.assertEqual(list(history.points),[(0.0,0.0),(0.115,1.567)])
        history.update(m.Navigation(5,300,5,'TRACKING',115,1567,1626,0,2500,0,0))
        self.assertEqual(list(history.points),[(0.115,1.567)])


class IntegratedSequenceTests(unittest.TestCase):
    def test_point_and_imu_have_independent_sequences(self):
        import monitor as m
        r=m.Receiver()
        r.feed(b'P,0,10,0,1000,20,1\nI,0,10,0,0\nP,1,20,0,1000,20,1\nI,1,20,0,0\n')
        self.assertEqual(r.events.qsize(),4)
        self.assertEqual(r.snapshot()['missing'],0)

    def test_navigation_reboot_allows_delayed_imu_and_clears_old_state(self):
        import monitor as m
        r=m.Receiver()
        r.feed(b'N,200,10000,2,ARRIVED,0,1400,1571,0,1400,27,0\n'
               b'I,150,10000,2000,0\nP,170,10000,0,1000,20,1\n'
               b'A,2,OK,accepted\nD,10000,ok\n')
        r.feed(b'N,0,50,0,IDLE,0,0,1571,0,0,0,0\n')
        self.assertEqual(r.boot_epoch,1)
        self.assertEqual(r.events.qsize(),1)
        self.assertIsNone(r.latest_imu)
        self.assertIsNone(r.latest_ack)
        self.assertFalse(r.diagnostics)
        r.feed(b'I,0,6500,100,0\nP,0,6550,0,1000,20,1\n')
        self.assertEqual(r.latest_imu.ms,6500)
        self.assertEqual(r.events.qsize(),3)

if __name__ == '__main__': unittest.main()



class MapStatusTests(unittest.TestCase):
    def test_fault_remains_frozen_after_imu_recovers(self):
        import monitor as m
        nav=m.Navigation(1,100,4,'FAULT',-153,522,1747,-1750,1300,0,1)
        fresh,title=m.map_display_status(nav,True,99.9,100)
        self.assertFalse(fresh)
        self.assertIn('FAULT 1',title)
        self.assertIn('sensor / pose invalid',title)
    def test_stale_and_recovered_grid(self):
        import monitor as m
        self.assertFalse(m.map_display_status(None,True,97,100)[0])
        self.assertIn('3.0s',m.map_display_status(None,True,97,100)[1])
        self.assertTrue(m.map_display_status(None,True,99.9,100)[0])
        self.assertFalse(m.map_display_status(None,False,99.9,100)[0])
        self.assertIn('No mapped scan',m.map_display_status(None,True,None,100)[1])

class PersistentMapTests(unittest.TestCase):
    def test_saved_walls_survive_clear_rows_and_sensor_loss(self):
        import monitor as m
        from protocol import parse_line
        history=m.MapHistory()
        history.update(parse_line(b'M,1,53,0,0,100,53,0,'+b'.'*53+b'#'+b'.'*53),100)
        self.assertEqual(history.occupied(),[(0.0,0.0)])
        # The transport is current, but the source scan is ten seconds old.
        history.update(parse_line(b'M,1,53,0,0,100,53,10000,'+b'.'*107),110)
        self.assertEqual(history.occupied(),[(0.0,0.0)])
        self.assertFalse(m.map_display_status(None,True,history.scan_time,110)[0])
        self.assertFalse(m.map_display_status(None,False,history.scan_time,110)[0])
        history.update(parse_line(b'M,2,53,0,0,100,53,4294967295,'+b'?'*107),111)
        self.assertFalse(history.occupied())
        self.assertIsNone(history.scan_time)

    def test_extended_geometry_and_old_firmware(self):
        import monitor as m
        from protocol import parse_line
        history=m.MapHistory()
        history.update(parse_line(b'M,1,103,10,-20,100,53,0,'+b'.'*103+b'#...'),100)
        self.assertEqual(history.occupied(),[(4.99,5.02)])
        history.clear()
        self.assertFalse(history.occupied())
        history.update(parse_line(b'M,1,24,0,0,'+b'.'*24+b'#'+b'.'*24),100)
        self.assertEqual(history.occupied(),[(0.0,0.0)])

    def test_new_map_rows_reject_bad_geometry(self):
        from protocol import parse_line
        good=b'M,1,54,0,0,200,27,123,'+b'.'*55
        self.assertEqual(parse_line(good).fields[-1],'.'*55)
        for bad in (good[:-1],good.replace(b',54,',b',55,'),
                    good.replace(b',200,',b',0,'),good.replace(b',123,',b',-1,')):
            with self.assertRaises(ValueError): parse_line(bad)

class ResetMapTests(unittest.TestCase):
    def test_reset_discards_queued_old_rows_then_rebuilds(self):
        import monitor as m
        from protocol import parse_line
        history=m.MapHistory()
        def row(epoch,wall):
            return parse_line(f'M,{epoch},53,0,0,100,53,0,'.encode()+b'.'*53+wall+b'.'*53)
        history.update(row(7,b'#'),100)
        history.reset_pending()
        self.assertFalse(history.occupied())
        history.update(row(7,b'#'),101)  # queued before RESET
        self.assertFalse(history.occupied())
        history.update(row(8,b'.'),102)
        self.assertFalse(history.occupied())
        history.update(row(7,b'#'),103)  # late old version must not return
        self.assertFalse(history.occupied())
        history.update(row(8,b'#'),104)
        self.assertEqual(history.occupied(),[(0.0,0.0)])

    def test_epoch_wrap_and_reboot(self):
        import monitor as m
        from protocol import parse_line
        history=m.MapHistory()
        def row(epoch):
            return parse_line(f'M,{epoch},53,0,0,100,53,0,'.encode()+b'#'*107)
        history.update(row(4294967295),100)
        history.reset_pending()
        history.update(row(0),101)
        self.assertEqual(len(history.occupied()),107)
        history.clear()  # reboot permits an unrelated version
        history.update(row(0),102)
        self.assertEqual(len(history.occupied()),107)
