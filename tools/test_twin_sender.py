# test_twin_sender.py ver 5
"""Send POC_TWIN packets containing both muscle lengths and Festo pressure fields."""
from __future__ import annotations
import argparse, json, math, socket, sys, time
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from poc_platform import PocPlatform

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=10020)
ap.add_argument("--poc-root", required=True)
a = ap.parse_args()
platform = PocPlatform(a.poc_root)
cfg = platform.cfg
kin = platform.kin
wire_platform = str(platform.platform_metadata.get("platform_name", platform.selected_platform))
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); seq=0; t0=time.monotonic()
print(f"Sending length + pressure fields to {a.host}:{a.port}; Ctrl-C to stop")
print(f"Platform: {platform.selected_platform} -> {wire_platform}")
print("Use L for muscle_length_mm mode or F for Festo pressure mode in the visualizer")
try:
    while True:
        t=time.monotonic()-t0
        pose=np.array([20*math.sin(t*.35), 15*math.sin(t*.27), 35*math.sin(t*.45), math.radians(4)*math.sin(t*.31), math.radians(4)*math.sin(t*.39), math.radians(4)*math.sin(t*.23)])
        _pts, actuator=kin.inverse_kinematics(pose,return_lengths=True)
        lengths=(np.asarray(actuator)-cfg.FIXED_HARDWARE_LENGTH).tolist()
        pressures=[3000.0 + 900.0*math.sin(t*0.45 + i*math.pi/3.0) for i in range(6)]
        msg={
            "protocol":"POC_TWIN","version":1,"sequence":seq,
            "timestamp_ms":int(time.time()*1000),"platform":wire_platform,
            "muscle_length_mm":lengths,
            "command_pressure_mbar":pressures,
            "dtop_load_kg_per_muscle":40.0,
        }
        sock.sendto(json.dumps(msg,separators=(",",":")).encode(),(a.host,a.port)); seq=(seq+1)&0xffffffff; time.sleep(.05)
except KeyboardInterrupt:
    pass
