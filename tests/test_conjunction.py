import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from conjunction import State, risk_index, miss_distance_km

def test_far_clear():
    r = risk_index(State(50,0,0,0,0,0), thresh_km=5)
    assert r["status"]=="CLEAR"
def test_close_watch():
    r = risk_index(State(2,0,0,0.1,0,0), thresh_km=5)
    assert r["status"] in ("WATCH","CRITICAL")
    assert miss_distance_km(State(3,4,0,0,0,0))==5.0

if __name__=="__main__":
    test_far_clear(); test_close_watch(); print("ok")
