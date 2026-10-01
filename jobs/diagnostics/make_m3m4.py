"""診断用: M4(速度に応じて出力)と M3(一定出力)で同じ線を焼き比べる G-code。

通常のツールは M3 を禁止しているので、このファイルは手で組み立て、
M3 を M4 に置き換えたものを通常の安全検証に通したうえで、M3 の前後も個別に検査する。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from src.config import load_machine  # noqa: E402
from src.gcode import strip_comment, verify_gcode  # noqa: E402

X0, X1 = 120.0, 150.0
LINES = [  # (Y, モード, 速度)
    (80.0, "M4", 400), (88.0, "M3", 400), (96.0, "M4", 200), (104.0, "M3", 200),
]

g = ["; diagnostic: M4 (dynamic) vs M3 (constant) at 100% - one 30 mm line each, 1 pass",
     "; Y80: M4 F400 / Y88: M3 F400 / Y96: M4 F200 / Y104: M3 F200",
     "G21 G90 G94", "M5", "S0", "G0 X0.000 Y0.000 S0"]
for i, (y, mode, f) in enumerate(LINES):
    g += [f"; line {i + 1}: {mode} S1000 F{f}",
          f"G0 X{X0:.3f} Y{y:.3f} S0", f"{mode} S1000", f"G1 X{X1:.3f} Y{y:.3f} F{f}", "M5"]
    if i < len(LINES) - 1:
        g.append("G4 P3")
g += ["G0 X0.000 Y0.000 S0", "M5"]
text = "\n".join(g) + "\n"

frame = "\n".join(["; frame for m3_m4.gcode: laser OFF", "G21 G90 G94", "M5", "S0",
                   f"G0 X{X0:.3f} Y80.000 S0", f"G0 X{X1:.3f} Y80.000 S0", f"G0 X{X1:.3f} Y104.000 S0",
                   f"G0 X{X0:.3f} Y104.000 S0", f"G0 X{X0:.3f} Y80.000 S0", "G0 X0.000 Y0.000 S0", "M5"]) + "\n"

m = load_machine()
# 1) M3 を M4 に置き換えて、通常の安全検証(G0 の S0、発振中の G0/G4 なし、末尾 M5、エリア内)
verify_gcode(text.replace("M3 S", "M4 S"), m)
verify_gcode(frame, m)
# 2) M3 の直後は必ず G1(1 行)、その次は必ず M5
cmds = [c for c in (strip_comment(l) for l in text.splitlines()) if c]
for i, c in enumerate(cmds):
    if c.startswith("M3"):
        assert cmds[i + 1].startswith("G1 ") and cmds[i + 2] == "M5", cmds[i:i + 3]
assert len(re.findall(r"^M3 ", text, re.M)) == 2

out = ROOT / "out" / "checks"
out.mkdir(parents=True, exist_ok=True)
(out / "m3_m4.gcode").write_text(text, encoding="utf-8", newline="\n")
(out / "m3_m4_frame.gcode").write_text(frame, encoding="utf-8", newline="\n")
print("written + checked: out/checks/m3_m4.gcode, m3_m4_frame.gcode")
print(text)
