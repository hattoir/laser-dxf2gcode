"""診断用(2026-10-01): 炎が出にくい速さでの M3/M4 比較と、紙での出力の健全性確認。

    python jobs/diagnostics/make_checks2.py

1. out/checks/m3_m4_fast.gcode
   100%・1 周の 30mm の線 4 本(線の間は 5 秒冷ます):
   Y112 = M4 1200 / Y120 = M3 1200 / Y128 = M4 800 / Y136 = M3 800 [mm/min]
   (前回の m3_m4.gcode は 400mm/min の 1 本目で火焔検知が反応して止まったため、速くした)
2. out/checks/paper_test.gcode
   クラフト紙(0.2mm)を、メーカー公式の条件で切る: 30mm 角 2 つ
   A = 80% / 4000mm/min / 1 周(公式: クラフト紙 0.2mm)、B = 50% / 4000mm/min / 1 周(参考)
   公式の条件で紙が切れなければ、機械の出力そのもの(レンズ・レーザー)が落ちている疑いが強い
"""
import contextlib
import io
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import ezdxf  # noqa: E402

from src.cli import main  # noqa: E402
from src.config import load_machine  # noqa: E402
from src.gcode import strip_comment, verify_gcode  # noqa: E402

OUT = ROOT / "out" / "checks"
OUT.mkdir(parents=True, exist_ok=True)
m = load_machine()

# ---------------- 1. M3/M4 比較(速め) ----------------
X0, X1 = 120.0, 150.0
LINES = [(112.0, "M4", 1200), (120.0, "M3", 1200), (128.0, "M4", 800), (136.0, "M3", 800)]
g = ["; diagnostic: M4 (dynamic) vs M3 (constant), 100%, one 30 mm line each, 1 pass",
     "; Y112: M4 F1200 / Y120: M3 F1200 / Y128: M4 F800 / Y136: M3 F800",
     "G21 G90 G94", "M5", "S0", "G0 X0.000 Y0.000 S0"]
for i, (y, mode, f) in enumerate(LINES):
    g += [f"; line {i + 1}: {mode} S1000 F{f}",
          f"G0 X{X0:.3f} Y{y:.3f} S0", f"{mode} S1000", f"G1 X{X1:.3f} Y{y:.3f} F{f}", "M5"]
    if i < len(LINES) - 1:
        g.append("G4 P5")
g += ["G0 X0.000 Y0.000 S0", "M5"]
text = "\n".join(g) + "\n"
ys = [y for y, _, _ in LINES]
frame = "\n".join(["; frame for m3_m4_fast.gcode: laser OFF", "G21 G90 G94", "M5", "S0",
                   f"G0 X{X0:.3f} Y{min(ys):.3f} S0", f"G0 X{X1:.3f} Y{min(ys):.3f} S0",
                   f"G0 X{X1:.3f} Y{max(ys):.3f} S0", f"G0 X{X0:.3f} Y{max(ys):.3f} S0",
                   f"G0 X{X0:.3f} Y{min(ys):.3f} S0", "G0 X0.000 Y0.000 S0", "M5"]) + "\n"
verify_gcode(text.replace("M3 S", "M4 S"), m)      # M3 以外は通常の安全検証
verify_gcode(frame, m)
cmds = [c for c in (strip_comment(l) for l in text.splitlines()) if c]
for i, c in enumerate(cmds):
    if c.startswith("M3"):
        assert cmds[i + 1].startswith("G1 ") and cmds[i + 2] == "M5", cmds[i:i + 3]
assert len(re.findall(r"^M3 ", text, re.M)) == 2
(OUT / "m3_m4_fast.gcode").write_text(text, encoding="utf-8", newline="\n")
(OUT / "m3_m4_fast_frame.gcode").write_text(frame, encoding="utf-8", newline="\n")
print("m3_m4_fast.gcode: verify OK")

# ---------------- 2. 紙での出力確認(通常のツールで生成) ----------------
prof = OUT / "paper_test_profile.yaml"
prof.write_text("""# 紙での出力確認用(Creality 公式: クラフト紙 0.2mm = 80% / 4000mm/min / 1 周)
paper_test_profile:
  kerf: 0.0                  # 【仮定値】出力確認が目的なので補正しない
  default_layer: k80
  layers:
    k80:
      power: 80              # 【公式値】Creality の推奨条件(この機械での実測ではない)
      feed: 4000             # 【公式値】同上
      passes: 1              # 【公式値】同上
      through_cut: true
      kerf_compensation: false
      order: 10
      air_assist: true
    k50:
      power: 50              # 【仮定値】参考
      feed: 4000             # 【仮定値】
      passes: 1              # 【仮定値】
      through_cut: true
      kerf_compensation: false
      order: 11
      air_assist: true
""", encoding="utf-8", newline="\n")
doc = ezdxf.new("R2010")
doc.header["$INSUNITS"] = 4
msp = doc.modelspace()
for x0, layer in ((10.0, "k80"), (50.0, "k50")):
    doc.layers.add(layer)
    msp.add_lwpolyline([(x0, 10), (x0 + 30, 10), (x0 + 30, 40), (x0, 40)], close=True, dxfattribs={"layer": layer})
dxf = OUT / "paper_test.dxf"
doc.saveas(dxf)
buf = io.StringIO()
with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
    code = main(["convert", str(dxf), "-p", str(prof), "--pass-mode", "path", "--cooldown", "0", "--frame",
                 "-o", str(OUT / "paper_test.gcode")])
assert code == 0, buf.getvalue()
t = (OUT / "paper_test.gcode").read_text(encoding="utf-8")
verify_gcode(t, m)
print("paper_test.gcode: verify OK /", sorted(set(re.findall(r"M4 S(\d+)", t))), "/ 範囲 X 10〜80, Y 10〜40")
