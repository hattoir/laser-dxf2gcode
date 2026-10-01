"""診断用(2026-10-01): レーザーが途中で消えるかを見る「長い直線」3 本。

    python jobs/diagnostics/make_long_lines.py

紙の試験で「最初の 1 辺(約 0.5 秒)だけ焼けて、あとは焦げすら付かない」が起きたため。
100mm の直線を 1 本ずつ、出力 20% / 50% / 80%、1500mm/min(各 4 秒)で焼く。線の間は 5 秒冷ます。
燃えにくいよう速めにしてある。線が途中で途切れたら、その位置と出力で、レーザー側の故障か判断する。
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from src.config import load_machine  # noqa: E402
from src.gcode import verify_gcode  # noqa: E402

X0, X1, FEED = 20.0, 120.0, 1500
LINES = [(120.0, 200), (130.0, 500), (140.0, 800)]   # (Y, S)
g = ["; diagnostic: does the laser cut out? three 100 mm lines, M4, 1500 mm/min, 1 pass",
     "; Y120: 20% / Y130: 50% / Y140: 80%",
     "G21 G90 G94", "M5", "S0", "G0 X0.000 Y0.000 S0"]
for i, (y, s) in enumerate(LINES):
    g += [f"; line {i + 1}: S{s}", f"G0 X{X0:.3f} Y{y:.3f} S0", f"M4 S{s}",
          f"G1 X{X1:.3f} Y{y:.3f} F{FEED}", "M5"]
    if i < len(LINES) - 1:
        g.append("G4 P5")
g += ["G0 X0.000 Y0.000 S0", "M5"]
text = "\n".join(g) + "\n"
frame = "\n".join(["; frame for long_lines.gcode: laser OFF", "G21 G90 G94", "M5", "S0",
                   f"G0 X{X0:.3f} Y120.000 S0", f"G0 X{X1:.3f} Y120.000 S0", f"G0 X{X1:.3f} Y140.000 S0",
                   f"G0 X{X0:.3f} Y140.000 S0", f"G0 X{X0:.3f} Y120.000 S0", "G0 X0.000 Y0.000 S0", "M5"]) + "\n"
m = load_machine()
verify_gcode(text, m)
verify_gcode(frame, m)
out = ROOT / "out" / "checks"
out.mkdir(parents=True, exist_ok=True)
(out / "long_lines.gcode").write_text(text, encoding="utf-8", newline="\n")
(out / "long_lines_frame.gcode").write_text(frame, encoding="utf-8", newline="\n")
print("long_lines.gcode / long_lines_frame.gcode: verify OK")
