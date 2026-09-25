#!/usr/bin/env python3
"""Convierte una traza CSV de vrc_sim en un HTML autónomo con la ronda animada.

Uso: python3 trace_viewer.py traza.csv [salida.html]
"""
import csv
import json
import re
import sys
from pathlib import Path

PAGE = """<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Repetición de ronda</title>
<style>
:root { --bg:#f6f6f3; --fg:#1d1d1b; --muted:#6b6b66; --grid:#dcdcd6; --panel:#ffffff; }
@media (prefers-color-scheme: dark) { :root { --bg:#161615; --fg:#ececea; --muted:#9a9a94; --grid:#2c2c2a; --panel:#1f1f1e; } }
body { margin:0; background:var(--bg); color:var(--fg); font:14px/1.4 system-ui, sans-serif; }
main { max-width:760px; margin:0 auto; padding:16px; }
h1 { font-size:18px; margin:0 0 4px; }
.sub { color:var(--muted); margin:0 0 12px; }
svg { width:100%; height:auto; background:var(--panel); border-radius:8px; display:block; }
.controls { display:flex; gap:10px; align-items:center; margin:12px 0; flex-wrap:wrap; }
input[type=range] { flex:1; min-width:160px; }
button, select { font:inherit; padding:4px 10px; }
.status { font-family:ui-monospace, monospace; font-size:12px; white-space:pre; color:var(--muted); }
</style></head><body><main>
<h1 id="title"></h1><p class="sub">Verdad física de la simulación. Rovers: rectángulo con frente marcado; estela de los últimos 20 s.</p>
<svg id="view" viewBox="-80 -80 1020 1020"></svg>
<div class="controls">
  <button id="play">▶</button>
  <input id="seek" type="range" min="0" value="0">
  <select id="speed"><option>1</option><option>4</option><option selected>10</option><option>30</option></select>
</div>
<div class="status" id="status"></div>
</main><script>
const DATA = __DATA__;
const colors = { green:'#2e9e44', red:'#d23a2e', blue:'#2f63d8' };
const roverColors = ['#e08a00', '#8a4fd8'];
const svg = document.getElementById('view'), NS = 'http://www.w3.org/2000/svg';
const W = DATA.cols * DATA.cell, H = DATA.rows * DATA.cell;
document.getElementById('title').textContent = DATA.name;
function el(tag, attrs, parent = svg) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); parent.appendChild(e); return e; }
el('rect', { x:-70, y:-70, width:W + 140, height:H + 140, fill:'none', stroke:'var(--muted)', 'stroke-dasharray':'6 6' });
for (let i = 0; i <= DATA.cols; i += 5) el('line', { x1:i * DATA.cell, y1:0, x2:i * DATA.cell, y2:H, stroke:'var(--grid)' });
for (let j = 0; j <= DATA.rows; j += 5) el('line', { x1:0, y1:j * DATA.cell, x2:W, y2:j * DATA.cell, stroke:'var(--grid)' });
el('rect', { x:0, y:0, width:W, height:H, fill:'none', stroke:'var(--muted)' });
for (const d of DATA.depots) {
  const horizontal = Math.min(d.row, DATA.rows - d.row) <= Math.min(d.col, DATA.cols - d.col);
  const w = (horizontal ? 10 : 7.5) * DATA.cell, h = (horizontal ? 7.5 : 10) * DATA.cell;
  el('rect', { x:d.col * DATA.cell - w / 2, y:d.row * DATA.cell - h / 2, width:w, height:h, fill:colors[d.color], opacity:0.18, stroke:colors[d.color] });
}
const trails = DATA.rovers.map((id, k) => el('polyline', { fill:'none', stroke:roverColors[k], 'stroke-width':3, opacity:0.45 }));
const cubes = DATA.cubes.map(c => el('rect', { width:60, height:60, x:-30, y:-30, fill:colors[c] || '#888', stroke:'var(--fg)', 'stroke-width':1 }));
const rovers = DATA.rovers.map((id, k) => {
  const g = el('g', {});
  el('rect', { x:-60, y:-50, width:120, height:100, rx:8, fill:roverColors[k], opacity:0.85 }, g);
  el('rect', { x:60, y:-38, width:17, height:76, fill:roverColors[k] }, g);
  el('text', { x:-10, y:8, 'font-size':34, fill:'#fff' }, g).textContent = id;
  return g;
});
const seek = document.getElementById('seek'); seek.max = DATA.frames.length - 1;
const status = document.getElementById('status');
function draw(i) {
  const f = DATA.frames[i];
  DATA.rovers.forEach((id, k) => {
    const r = f.r[k];
    rovers[k].setAttribute('transform', `translate(${r[0]} ${r[1]}) rotate(${-r[2]})`);
    const pts = []; for (let j = Math.max(0, i - 400); j <= i; j += 4) pts.push(DATA.frames[j].r[k][0] + ',' + DATA.frames[j].r[k][1]);
    trails[k].setAttribute('points', pts.join(' '));
  });
  DATA.cubes.forEach((c, k) => cubes[k].setAttribute('transform', `translate(${f.c[k][0]} ${f.c[k][1]})`));
  status.textContent = `t=${(f.t / 1000).toFixed(1)} s  ${f.p}\\n` + DATA.rovers.map((id, k) => `rover ${id}: ${f.r[k][3]}  ${f.r[k][4]}`).join('\\n');
}
let playing = false, last = 0;
document.getElementById('play').onclick = e => { playing = !playing; e.target.textContent = playing ? '❚❚' : '▶'; last = performance.now(); if (playing) requestAnimationFrame(tick); };
seek.oninput = () => draw(+seek.value);
function tick(now) {
  if (!playing) return;
  const speed = +document.getElementById('speed').value;
  const step = Math.floor((now - last) * speed / DATA.dt);
  if (step > 0) { last = now; seek.value = Math.min(+seek.max, +seek.value + step); draw(+seek.value); }
  if (+seek.value >= +seek.max) { playing = false; document.getElementById('play').textContent = '▶'; return; }
  requestAnimationFrame(tick);
}
draw(0);
</script></body></html>
"""


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    source = Path(sys.argv[1])
    target = Path(sys.argv[2]) if len(sys.argv) > 2 else source.with_suffix(".html")
    lines = source.read_text().splitlines()
    header = lines[0].lstrip("# ")
    name, grid_part, depot_part = [p.strip() for p in header.split("|")]
    cols, rows = grid_part.split()[1].split("x")
    cell = float(grid_part.split()[4])
    depots = []
    for item in depot_part.split()[1:]:
        color, col, row = item.split(":")
        depots.append({"color": color, "col": float(col), "row": float(row)})
    table = list(csv.DictReader(lines[1:]))
    rover_ids = sorted({k[1:].split("_")[0] for k in table[0] if k.startswith("r") and k.endswith("_mode")})
    # Columnas de rover: r<id>_x. Ojo: "red_x" también empieza con "r".
    cube_colors = [k[:-2] for k in table[0] if k.endswith("_x") and not re.fullmatch(r"r\d+_x", k)]
    frames = []
    for row in table:
        frames.append({
            "t": int(float(row["t_ms"])),
            "p": row["phase"],
            "r": [[round(float(row[f"r{i}_x"]), 1), round(float(row[f"r{i}_y"]), 1), round(float(row[f"r{i}_th"]), 1),
                   row[f"r{i}_mode"], row[f"r{i}_motion"]] for i in rover_ids],
            "c": [[round(float(row[f"{c}_x"]), 1), round(float(row[f"{c}_y"]), 1)] for c in cube_colors],
        })
    dt = frames[1]["t"] - frames[0]["t"] if len(frames) > 1 else 50
    data = {"name": name, "cols": int(cols), "rows": int(rows), "cell": cell, "depots": depots,
            "rovers": rover_ids, "cubes": cube_colors, "frames": frames, "dt": dt}
    target.write_text(PAGE.replace("__DATA__", json.dumps(data, separators=(",", ":"))))
    print(target)


if __name__ == "__main__":
    main()
