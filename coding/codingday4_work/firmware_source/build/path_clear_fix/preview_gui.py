from pathlib import Path
import sys,time,threading
from types import SimpleNamespace
from unittest.mock import patch
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'pc'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent
import monitor
receiver=monitor.Receiver(); receiver.set_status(True,'OFFLINE PREVIEW')
for row in range(53,71):
    cells=['?']*107
    cells[47]=cells[60]='#'
    receiver.feed(f'M,1,{row},0,0,100,53,2500,{"".join(cells)}\n'.encode())
receiver.feed(b'N,1,10000,3,FAULT,0,700,1571,0,1800,0,1\nI,1,10000,0,0\n')
frames=[]
class Animation:
    def __init__(self,fig,update,**kwargs): frames.append(update)
def show():
    frames[0](0)
    fig=plt.gcf(); nav,radar=fig.axes[:2]
    assert nav.get_xlim()==(-5.0,5.0) and nav.get_ylim()==(-5.0,5.0)
    assert radar.get_xlim()==(-5.0,5.0) and radar.get_ylim()==(-5.0,5.0)
    assert len(nav.collections[0].get_offsets())==36
    assert nav.collections[0].get_alpha()==1.0
    assert 'FAULT 1' in nav.get_title()
    fig.canvas.draw()
    # Clicking a goal only fills the command box; sending remains deliberate.
    x,y=nav.transData.transform((1.1,1.2))
    fig.canvas.callbacks.process('button_press_event',MouseEvent('button_press_event',fig.canvas,x,y,button=1))
    assert receiver.commands.empty()
    assert any('Selected' in t.get_text() for t in fig.texts)
    # Exercise the full-range button, then restore the five-metre view.
    button=fig.axes[-1]
    for expected in ((-2.5,2.5),(-5.0,5.0)):
        x,y=button.transAxes.transform((.5,.5))
        for kind in ('button_press_event','button_release_event'):
            fig.canvas.callbacks.process(kind,MouseEvent(kind,fig.canvas,x,y,button=1))
        assert nav.get_xlim()==expected
    fig.savefig(root/'build/path_clear_fix/ui_preview.png',dpi=140)
    plt.close(fig)
with patch('matplotlib.animation.FuncAnimation',Animation),patch.object(plt,'show',show):
    monitor.run_gui(receiver,threading.Event(),SimpleNamespace(range_m=5.0,demo=False,duration=0))
print('GUI +/-5m preview passed: saved walls, fault status, goal selection without send, both views.')
