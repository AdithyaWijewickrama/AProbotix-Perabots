"""
build_site.py - builds the GuardianBot version-comparison website from versions/*/spec.yaml.

    python build_site.py            -> versions/_site/index.html (+ img/), viewer.html (+ models/, lib/)
viewer.html shows each version's PCB, printed parts and laser-cut decks in 3-D, with downloads. Its models
come from versions/<folder>/models/manifest.json (made by export_models.py); the files the manifest names are
copied here from wherever they live in the repo.
GitHub Actions (.github/workflows/versions-site.yml at the repo root) runs this on every push that
changes the versions folder and publishes _site to GitHub Pages, so the page updates by itself.
To add a version: make a new folder vX.Y_name/ with spec.yaml (+ images/iso.jpg) and push.
Needs: pip install pyyaml
"""
import glob
import html
import json
import os
import re
import shutil

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                  # "Phase 02 - Final Round"
OUT = os.path.join(HERE, "_site")


def vkey(v):
    return [int(x) for x in re.findall(r"\d+", v["id"])]


def load():
    vs = []
    for f in glob.glob(os.path.join(HERE, "*", "spec.yaml")):
        d = yaml.safe_load(open(f, encoding="utf-8"))
        d["folder"] = os.path.basename(os.path.dirname(f))
        imgs = sorted(glob.glob(os.path.join(os.path.dirname(f), "images", "*.jpg")) +
                      glob.glob(os.path.join(os.path.dirname(f), "images", "*.png")))
        d["images"] = []
        for p in imgs:
            name = "%s_%s" % (d["id"], os.path.basename(p))
            d["images"].append("img/" + name)
            shutil.copy(p, os.path.join(OUT, "img", name))
        b = d.get("budget") or {}
        lkr = b.get("total") if b.get("currency") == "LKR" else b.get("total_lkr_approx")
        d["_budget_lkr"] = lkr
        d["_models"] = os.path.exists(os.path.join(os.path.dirname(f), "models", "manifest.json"))
        vs.append(d)
    vs.sort(key=vkey)
    return vs


PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>GuardianBot versions - PeraBots 2026 AProbotix</title>
<style>
.viz-root {
  color-scheme: light;
  --surface-0: #f4f3f0; --surface-1: #fcfcfb; --border: #dcdad4;
  --text-primary: #0b0b0b; --text-secondary: #52514e; --text-muted: #77756f;
  --series-1: #2a78d6; --series-1-soft: #2a78d633; --grid: #e7e5df;
  --good: #008300; --bad: #c43c3b;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) .viz-root {
    color-scheme: dark;
    --surface-0: #121211; --surface-1: #1a1a19; --border: #33322f;
    --text-primary: #f0efec; --text-secondary: #c3c2b7; --text-muted: #9a998f;
    --series-1: #3987e5; --series-1-soft: #3987e540; --grid: #2a2a28;
    --good: #0ca30c; --bad: #e66767;
  }
}
:root[data-theme="dark"] .viz-root {
  color-scheme: dark;
  --surface-0: #121211; --surface-1: #1a1a19; --border: #33322f;
  --text-primary: #f0efec; --text-secondary: #c3c2b7; --text-muted: #9a998f;
  --series-1: #3987e5; --series-1-soft: #3987e540; --grid: #2a2a28;
  --good: #0ca30c; --bad: #e66767;
}
* { box-sizing: border-box; }
.viewlink { display: inline-block; margin-top: 8px; color: var(--series-1); text-decoration: none; font-weight: 600; }
.viewlink:hover { text-decoration: underline; }
body { margin: 0; }
.viz-root { background: var(--surface-0); color: var(--text-primary); min-height: 100vh;
  font: 15px/1.5 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
main { max-width: 1180px; margin: 0 auto; padding: 28px 20px 60px; }
h1 { font-size: 26px; margin: 0 0 4px; }
h2 { font-size: 18px; margin: 34px 0 12px; }
.sub { color: var(--text-secondary); margin: 0 0 18px; }
.card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 10px; padding: 14px; }
.grid { display: grid; gap: 12px; grid-template-columns: repeat(auto-fill, minmax(250px, 1fr)); }
.vcard img { width: 100%; height: 150px; object-fit: contain; background: #fff; border-radius: 6px; }
.vcard .noimg { height: 150px; display: flex; align-items: center; justify-content: center; color: var(--text-muted);
  border: 1px dashed var(--border); border-radius: 6px; }
.vcard h3 { margin: 10px 0 2px; font-size: 16px; }
.tag { display: inline-block; font: 12px ui-monospace, monospace; padding: 1px 6px; border-radius: 4px;
  border: 1px solid var(--border); color: var(--text-secondary); }
.latest { border-color: var(--series-1); }
.latest .tag { border-color: var(--series-1); color: var(--text-primary); }
.kv { display: grid; grid-template-columns: auto 1fr; gap: 2px 10px; font-size: 13px; margin-top: 8px; }
.kv span:nth-child(odd) { color: var(--text-muted); }
.status { font-size: 12.5px; color: var(--text-secondary); margin-top: 4px; }
.charts { display: grid; gap: 12px; grid-template-columns: repeat(auto-fill, minmax(330px, 1fr)); }
.chart h3 { font-size: 14.5px; margin: 0 0 2px; }
.chart .note { font-size: 12px; color: var(--text-muted); margin-bottom: 6px; }
svg text { fill: var(--text-secondary); font-size: 12px; }
svg .val { fill: var(--text-primary); }
svg .na { fill: var(--text-muted); font-style: italic; }
svg rect.bar { fill: var(--series-1); }
svg rect.hit { fill: transparent; cursor: default; }
svg rect.hit:hover + rect.bar, svg g.row:hover rect.bar { opacity: .85; }
.tip { position: fixed; pointer-events: none; background: var(--surface-1); border: 1px solid var(--border);
  border-radius: 6px; padding: 6px 8px; font-size: 12.5px; box-shadow: 0 4px 14px #0002; display: none; z-index: 9; }
.pick { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; margin-bottom: 10px; }
select, button { font: inherit; padding: 5px 8px; border-radius: 6px; border: 1px solid var(--border);
  background: var(--surface-1); color: var(--text-primary); }
table { border-collapse: collapse; width: 100%; font-size: 13.5px; }
th, td { text-align: left; vertical-align: top; padding: 7px 9px; border-bottom: 1px solid var(--border); }
th { color: var(--text-secondary); font-weight: 600; width: 180px; }
.cmp td { width: calc((100% - 180px) / 2); }
.diff-add { color: var(--good); } .diff-del { color: var(--bad); text-decoration: line-through; }
.better { color: var(--good); font-weight: 600; } .worse { color: var(--bad); }
.wrap { overflow-x: auto; }
ul { margin: 0; padding-left: 18px; }
footer { margin-top: 40px; color: var(--text-muted); font-size: 12.5px; }
.theme { float: right; }
</style>
</head>
<body>
<div class="viz-root">
<main>
  <button class="theme" id="theme">Light / dark</button>
  <h1>GuardianBot - every version side by side</h1>
  <p class="sub">PeraBots 2026 University Final, team AProbotix. Speeds are from the same Webots arena with a neutral
  judge supervisor (Section 1 = robot enters the maze, Section 2 = robot leaves the maze, END = reaches the END point).
  Latest version: <b id="latestname"></b>.
  <a class="viewlink" href="viewer.html">Open the PCBs, printed parts and decks in 3-D &rarr;</a></p>

  <h2>Versions</h2>
  <div class="grid" id="cards"></div>

  <h2>Compare in numbers</h2>
  <div class="charts" id="charts"></div>

  <h2>Compare two versions</h2>
  <div class="card">
    <div class="pick">
      <label>A <select id="selA"></select></label>
      <label>B <select id="selB"></select></label>
      <span class="status">Green = better in B, red = worse. Sensors: green added in B, struck out removed.</span>
    </div>
    <div class="wrap"><table class="cmp" id="cmp"></table></div>
  </div>

  <h2>All data (table view)</h2>
  <div class="card wrap"><table id="all"></table></div>
  <footer>Built from <code>versions/*/spec.yaml</code> by <code>versions/build_site.py</code> on __BUILT__.
  Add a folder with a spec.yaml and push - GitHub Actions rebuilds this page.</footer>
</main>
<div class="tip" id="tip"></div>
</div>
<script>
const V = __DATA__;
const $ = s => document.querySelector(s);
const esc = s => String(s ?? "").replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const fmt = (v, u) => v == null ? "not measured" : (u === "Rs" ? "Rs " + Math.round(v).toLocaleString("en-US") : v.toLocaleString("en-US") + (u ? " " + u : ""));
const latest = V[V.length - 1];
$("#latestname").textContent = latest.id + " " + latest.name;
document.getElementById("theme").onclick = () => {
  const r = document.documentElement, dark = matchMedia("(prefers-color-scheme: dark)").matches;
  const cur = r.dataset.theme || (dark ? "dark" : "light");
  r.dataset.theme = cur === "dark" ? "light" : "dark";
};

// ---- cards
$("#cards").innerHTML = V.map(v => `
  <div class="card vcard ${v === latest ? "latest" : ""}">
    ${v.images.length ? `<img src="${esc(v.images.find(i => i.includes("iso")) || v.images[0])}" alt="${esc(v.name)} render">` : `<div class="noimg">no 3-D model</div>`}
    <h3>${esc(v.id)} ${esc(v.name)}</h3>
    <span class="tag">${esc(v.tag)}</span> ${v === latest ? '<span class="tag">latest</span>' : ""}
    <div class="status">${esc(v.status)}</div>
    <div class="kv">
      <span>Size</span><span>${v.body?.size_mm ? v.body.size_mm.join(" × ") + " mm" : "-"}</span>
      <span>Weight</span><span>${fmt(v.weight_g, "g")}</span>
      <span>Budget</span><span>${fmt(v._budget_lkr, "Rs")}</span>
      <span>END reached</span><span>${fmt(v.simulation?.end_s, "s")}</span>
      <span>PCB</span><span>${esc(v.pcb?.type)}</span>
    </div>
    ${v._models ? `<a class="viewlink" href="viewer.html?v=${encodeURIComponent(v.id)}">3-D PCB, parts &amp; files &rarr;</a>` : ""}
  </div>`).join("");

// ---- bar charts: one measure each, one series (no legend), value at the bar end, tooltip on hover
const tip = $("#tip");
function bars(title, note, get, unit, lowerBetter) {
  const rows = V.map(v => ({v, x: get(v)}));
  const max = Math.max(...rows.map(r => r.x || 0)) || 1;
  const W = 330, L = 64, R = 92, RH = 26, H = rows.length * RH + 8;
  let s = `<svg viewBox="0 0 ${W} ${H}" width="100%" role="img" aria-label="${esc(title)}">`;
  s += `<line x1="${L}" y1="2" x2="${L}" y2="${H - 4}" stroke="var(--grid)"/>`;
  rows.forEach((r, i) => {
    const y = 6 + i * RH, w = r.x == null ? 0 : Math.max(2, (W - L - R) * r.x / max);
    s += `<g class="row" data-t="${esc(r.v.id + " " + r.v.name)}" data-val="${esc(fmt(r.x, unit))}">`;
    s += `<rect class="hit" x="0" y="${y - 3}" width="${W}" height="${RH}"/>`;
    s += `<text x="${L - 8}" y="${y + 13}" text-anchor="end">${esc(r.v.id)}</text>`;
    if (r.x != null) {
      s += `<rect class="bar" x="${L}" y="${y + 2}" width="${w}" height="14" rx="4"/>`;
      s += `<text class="val" x="${L + w + 6}" y="${y + 13}">${esc(fmt(r.x, unit))}</text>`;
    } else s += `<text class="na" x="${L + 6}" y="${y + 13}">not measured</text>`;
    s += `</g>`;
  });
  s += `</svg>`;
  return `<div class="card chart"><h3>${esc(title)}</h3><div class="note">${esc(note)}${lowerBetter ? " Shorter bar is better." : ""}</div>${s}</div>`;
}
$("#charts").innerHTML = [
  bars("Full mission time", "Start to END point in the simulation; no bar = did not finish.", v => v.simulation?.end_s ?? null, "s", true),
  bars("Section 2 finished (leaves the maze)", "Seconds from the start.", v => v.simulation?.maze_s ?? null, "s", true),
  bars("Budget to buy", "LKR; USD versions converted at about Rs 300 per dollar.", v => v._budget_lkr ?? null, "Rs", true),
  bars("Weight", "Grams, from the model or the parts list.", v => v.weight_g ?? null, "g", false),
  bars("Height", "mm (rule limit 200 mm).", v => v.body?.height_mm ?? null, "mm", false),
  bars("Collisions", "Body-height contacts counted by the judge.", v => v.simulation?.collisions ?? null, "", true),
].join("");
document.querySelectorAll("g.row").forEach(g => {
  g.addEventListener("mousemove", e => {
    tip.style.display = "block";
    tip.innerHTML = `<b>${g.dataset.t}</b><br>${g.dataset.val}`;
    tip.style.left = Math.min(e.clientX + 14, innerWidth - 220) + "px"; tip.style.top = (e.clientY + 12) + "px";
  });
  g.addEventListener("mouseleave", () => tip.style.display = "none");
});

// ---- compare two versions
const opts = V.map((v, i) => `<option value="${i}">${esc(v.id + " " + v.name)}</option>`).join("");
$("#selA").innerHTML = opts; $("#selB").innerHTML = opts;
$("#selA").value = Math.max(0, V.length - 2); $("#selB").value = V.length - 1;
function cmpNum(a, b, lowerBetter) {
  if (a == null || b == null || a === b) return "";
  return (lowerBetter ? b < a : b > a) ? "better" : "worse";
}
function list(x) { return Array.isArray(x) ? x : (x ? [x] : []); }
function isoimg(v) {
  const p = v.images.find(i => i.includes("iso"));
  return p ? `<img src="${esc(p)}" style="max-width:100%;max-height:160px">` : "-";
}
function pcbimg(v) {
  const p = v.images.find(i => i.includes("pcb"));
  return p ? `<img src="${esc(p)}" style="max-width:100%;max-height:200px">` : esc(v.pcb?.type || "-");
}
function render() {
  const A = V[$("#selA").value], B = V[$("#selB").value];
  const row = (k, a, b, cls) => `<tr><th>${k}</th><td>${a}</td><td class="${cls || ""}">${b}</td></tr>`;
  const num = (k, f, u, lb) => row(k, esc(fmt(f(A), u)), esc(fmt(f(B), u)), cmpNum(f(A), f(B), lb));
  const sA = list(A.sensors), sB = list(B.sensors);
  const sens = row("Sensors", "<ul>" + sA.map(s => `<li class="${sB.includes(s) ? "" : "diff-del"}">${esc(s)}</li>`).join("") + "</ul>",
                              "<ul>" + sB.map(s => `<li class="${sA.includes(s) ? "" : "diff-add"}">${esc(s)}</li>`).join("") + "</ul>");
  $("#cmp").innerHTML =
    `<tr><th></th><td><b>${esc(A.id + " " + A.name)}</b></td><td><b>${esc(B.id + " " + B.name)}</b></td></tr>` +
    row("Picture", isoimg(A), isoimg(B)) +
    row("PCB picture", pcbimg(A), pcbimg(B)) +
    row("Summary", esc(A.summary), esc(B.summary)) +
    row("Status", esc(A.status), esc(B.status)) +
    row("Body", esc(A.body?.shape), esc(B.body?.shape)) +
    row("Size", esc((A.body?.size_mm || []).join(" × ")) + " mm", esc((B.body?.size_mm || []).join(" × ")) + " mm") +
    num("Weight", v => v.weight_g ?? null, "g", true) +
    row("Gripper", esc(A.body?.gripper), esc(B.body?.gripper)) +
    row("Controllers", esc(list(A.controllers).join(", ")), esc(list(B.controllers).join(", "))) +
    row("Drive", esc(A.drive), esc(B.drive)) +
    row("Battery", esc(A.battery), esc(B.battery)) + sens +
    row("PCB", esc(A.pcb?.type), esc(B.pcb?.type)) +
    num("Budget", v => v._budget_lkr ?? null, "Rs", true) +
    row("Budget note", esc(A.budget?.note), esc(B.budget?.note)) +
    num("Section 1 done", v => v.simulation?.section1_s ?? null, "s", true) +
    num("Section 2 done", v => v.simulation?.maze_s ?? null, "s", true) +
    num("END reached", v => v.simulation?.end_s ?? null, "s", true) +
    num("Collisions", v => v.simulation?.collisions ?? null, "", true) +
    row("Simulation result", esc(A.simulation?.result), esc(B.simulation?.result)) +
    row("Git tag", `<code>${esc(A.tag)}</code>`, `<code>${esc(B.tag)}</code>`);
}
$("#selA").onchange = render; $("#selB").onchange = render; render();

// ---- full table
const cols = [["Version", v => v.id + " " + v.name], ["Tag", v => v.tag], ["Size (mm)", v => (v.body?.size_mm || []).join(" × ")],
  ["Weight", v => fmt(v.weight_g, "g")], ["Budget", v => fmt(v._budget_lkr, "Rs")], ["Controllers", v => list(v.controllers).join(", ")],
  ["PCB", v => v.pcb?.type], ["S1", v => fmt(v.simulation?.section1_s, "s")], ["S2", v => fmt(v.simulation?.maze_s, "s")],
  ["END", v => fmt(v.simulation?.end_s, "s")], ["Collisions", v => fmt(v.simulation?.collisions, "")], ["Result", v => v.simulation?.result]];
$("#all").innerHTML = "<tr>" + cols.map(c => `<th style="width:auto">${c[0]}</th>`).join("") + "</tr>" +
  V.map(v => "<tr>" + cols.map(c => `<td>${esc(c[1](v))}</td>`).join("") + "</tr>").join("");
</script>
</body>
</html>
"""

def build_viewer(vs):
    """viewer.html + models/<id>/... + lib/ (three.js, vendored so the page needs no CDN)."""
    models = []
    for v in vs:
        mf = os.path.join(HERE, v["folder"], "models", "manifest.json")
        if not os.path.exists(mf):
            continue
        man = json.load(open(mf, encoding="utf-8"))
        dst = os.path.join(OUT, "models", v["id"])
        os.makedirs(dst, exist_ok=True)

        def put(src):
            p = os.path.join(ROOT, src)
            if not os.path.exists(p):
                print("  missing:", src)
                return None
            shutil.copy(p, os.path.join(dst, os.path.basename(p)))
            return "models/%s/%s" % (v["id"], os.path.basename(p))
        items = []
        for it in man["items"]:
            it = dict(it)
            if it.get("src"):
                it["href"] = put(it["src"])
            if it.get("svg"):
                it["svgHref"] = put(it["svg"])
            if it.get("dxf"):
                it["dxfHref"] = put(it["dxf"])
            if it.get("href") or it.get("svgHref"):
                items.append(it)
        files = []
        for f in man["files"]:
            h = put(f["src"])
            if h:
                files.append({"name": f["name"], "href": h, "kb": f["kb"]})
        models.append({"id": v["id"], "name": v["name"], "items": items, "files": files})
    shutil.copytree(os.path.join(HERE, "viewer_lib"), os.path.join(OUT, "lib"))
    page = open(os.path.join(HERE, "viewer.html"), encoding="utf-8").read()
    page = page.replace("/*MODELS*/[]", json.dumps(models, ensure_ascii=False))
    open(os.path.join(OUT, "viewer.html"), "w", encoding="utf-8").write(page)
    n = sum(len(m["items"]) for m in models)
    print("viewer: %d versions, %d models" % (len(models), n))


if __name__ == "__main__":
    import datetime
    if os.path.isdir(OUT):
        shutil.rmtree(OUT)
    os.makedirs(os.path.join(OUT, "img"))
    vs = load()
    page = PAGE.replace("__DATA__", json.dumps(vs, ensure_ascii=False, default=str)) \
               .replace("__BUILT__", datetime.date.today().isoformat())
    open(os.path.join(OUT, "index.html"), "w", encoding="utf-8").write(page)
    open(os.path.join(OUT, ".nojekyll"), "w").write("")
    build_viewer(vs)
    print("built %d versions -> %s" % (len(vs), os.path.join(OUT, "index.html")))
