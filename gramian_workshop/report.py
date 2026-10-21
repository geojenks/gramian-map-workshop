"""Self-contained HTML reports (inline SVG, no dependencies) for the demos.

Reason cells carry a letter as well as a colour, every cell has a hover
tooltip, and each page carries its table as text, so nothing is read from
colour alone. Light and dark themes share one set of role tokens.
"""
import html

# reason -> (letter, css colour role, legend text); the label ink for each
# role is the same name with "-ink"
REASON_STYLE = {
    "met": ("✓", "--st-good", "met"),
    "strength": ("s", "--st-warn", "held; short on comparative strength (not scored)"),
    "cone_edge": ("e", "--st-serious", "at the friction-cone edge"),
    "friction": ("f", "--st-critical", "friction (push slips or pulls)"),
    "direction": ("d", "--st-dir", "direction (cannot push that way)"),
    "no_contact": ("–", "--st-none", "no contact / no force data"),
}

SEQ = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7",
       "#3987e5", "#2a78d6", "#256abf", "#1c5cab", "#184f95"]
# dark mode: low values recede toward the dark surface, high values brighten
SEQ_D = ["#26282b", "#0d366b", "#104281", "#184f95", "#1c5cab", "#256abf",
         "#2a78d6", "#3987e5", "#5598e7", "#6da7ec", "#86b6ef"]


def _ink(hexs, dark):
    r, g, b = (int(hexs[i:i + 2], 16) / 255 for i in (1, 3, 5))
    lum = 0.2126 * r + 0.7152 * g + 0.0722 * b
    return "#0b0b0b" if lum > 0.45 else "#ffffff"


def _seq_vars(steps, dark):
    return " ".join(f"--seq{i}:{h}; --seq{i}-ink:{_ink(h, dark)};"
                    for i, h in enumerate(steps))

CSS = """
:root { color-scheme: light;
  --surface:#fcfcfb; --page:#f9f9f7; --ink:#0b0b0b; --ink2:#52514e;
  --muted:#898781; --grid:#e1e0d9; --ring:rgba(11,11,11,0.10);
  --st-good:#0ca30c; --st-warn:#fab219; --st-serious:#ec835a;
  --st-critical:#d03b3b; --st-dir:#52514e; --st-none:#e1e0d9;
  --st-good-ink:#0b0b0b; --st-warn-ink:#0b0b0b; --st-serious-ink:#0b0b0b;
  --st-critical-ink:#ffffff; --st-dir-ink:#ffffff; --st-none-ink:#52514e;
  --made:#256abf; --broken:#e1e0d9; SEQ_LIGHT }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  color-scheme: dark; --surface:#1a1a19; --page:#0d0d0d; --ink:#f0efec;
  --ink2:#c3c2b7; --muted:#898781; --grid:#2c2c2a; --ring:rgba(255,255,255,0.10);
  --st-dir:#c3c2b7; --st-none:#383835; --made:#3987e5; --broken:#2c2c2a;
  --st-dir-ink:#0b0b0b; --st-none-ink:#c3c2b7; SEQ_DARK } }
:root[data-theme="dark"] { color-scheme: dark; --surface:#1a1a19;
  --page:#0d0d0d; --ink:#f0efec; --ink2:#c3c2b7; --muted:#898781;
  --grid:#2c2c2a; --ring:rgba(255,255,255,0.10); --st-dir:#c3c2b7;
  --st-none:#383835; --made:#3987e5; --broken:#2c2c2a;
  --st-dir-ink:#0b0b0b; --st-none-ink:#c3c2b7; SEQ_DARK }
body { background:var(--page); color:var(--ink); margin:0;
  font:14px/1.45 system-ui, -apple-system, Segoe UI, sans-serif; }
main { max-width:1100px; margin:0 auto; padding:24px 16px 48px; }
h1 { font-size:22px; margin:0 0 4px; } h2 { font-size:16px; margin:28px 0 6px; }
.caveat { color:var(--ink2); margin:0 0 16px; }
.card { background:var(--surface); border:1px solid var(--ring);
  border-radius:8px; padding:14px; overflow-x:auto; margin:10px 0; }
.legend { display:flex; flex-wrap:wrap; gap:6px 16px; color:var(--ink2);
  margin:8px 0 0; }
.sw { display:inline-block; width:14px; height:14px; border-radius:3px;
  vertical-align:-2px; margin-right:6px; text-align:center; font-size:10px;
  line-height:14px; color:#0b0b0b; }
svg text { fill:var(--ink2); font-size:11px; }
svg .lab { fill:var(--ink); }
svg .cl { font-size:11px; font-weight:600; pointer-events:none; }
svg rect.c { stroke:var(--surface); stroke-width:2; }
svg rect.c:hover { stroke:var(--ink); }
pre { font:12px/1.35 ui-monospace, SFMono-Regular, Menlo, monospace;
  color:var(--ink2); white-space:pre; overflow-x:auto; margin:0; }
details summary { cursor:pointer; color:var(--ink2); }
p.note { color:var(--ink2); max-width:780px; }
"""


CSS = (CSS.replace("SEQ_LIGHT", _seq_vars(SEQ, False))
       .replace("SEQ_DARK", _seq_vars(SEQ_D, True)))


def esc(s):
    return html.escape(str(s))


def legend_reasons():
    items = []
    for _r, (letter, role, text) in REASON_STYLE.items():
        items.append(f'<span><span class="sw" style="background:var({role});'
                     f'color:var({role}-ink)">'
                     f'{esc(letter)}</span>{esc(text)}</span>')
    return '<div class="legend">' + "".join(items) + "</div>"


def grid_svg(rows, cols, cell, label_w=230, cw=34, ch=22, top=22):
    """rows: list of row labels; cols: column labels; cell(r, c) ->
    (fill css, text, tooltip)."""
    w = label_w + cw * len(cols) + 8
    h = top + ch * len(rows) + 4
    out = [f'<svg width="{w}" height="{h}" role="img">']
    for j, c in enumerate(cols):
        out.append(f'<text x="{label_w + j * cw + cw / 2}" y="14" '
                   f'text-anchor="middle">{esc(c)}</text>')
    for i, r in enumerate(rows):
        y = top + i * ch
        out.append(f'<text class="lab" x="{label_w - 8}" y="{y + ch / 2 + 4}" '
                   f'text-anchor="end">{esc(r)}</text>')
        for j in range(len(cols)):
            fill, txt, tip, ink = cell(i, j)
            x = label_w + j * cw
            out.append(f'<g><rect class="c" x="{x}" y="{y}" width="{cw}" '
                       f'height="{ch}" rx="4" style="fill:{fill}">'
                       f'<title>{esc(tip)}</title></rect>'
                       f'<text class="cl" style="fill:{ink}" '
                       f'x="{x + cw / 2}" y="{y + ch / 2 + 4}" '
                       f'text-anchor="middle">{esc(txt)}</text></g>')
    out.append("</svg>")
    return "".join(out)


def raster_svg(made, held, label, width=720, rowh=12):
    """made: (ticks, n) bool; held: (ticks,) bool. One row per finger, then a
    held strip."""
    t, n = made.shape
    lw = 70
    sx = width / t
    h = (n + 2) * rowh + 26
    out = [f'<svg width="{lw + width + 8}" height="{h}" role="img">'
           f'<text class="lab" x="0" y="12">{esc(label)}</text>']
    y0 = 20
    for i in range(n):
        y = y0 + i * rowh
        out.append(f'<text x="{lw - 6}" y="{y + rowh - 3}" '
                   f'text-anchor="end">finger {i}</text>')
        k = 0
        while k < t:
            v = made[k, i]
            k2 = k
            while k2 < t and made[k2, i] == v:
                k2 += 1
            role = "--made" if v else "--broken"
            out.append(f'<rect x="{lw + k * sx}" y="{y + 1}" '
                       f'width="{(k2 - k) * sx}" height="{rowh - 2}" '
                       f'style="fill:var({role})"><title>finger {i} '
                       f'{"made" if v else "broken"}, ticks {k}-{k2 - 1}'
                       f'</title></rect>')
            k = k2
    y = y0 + (n + 0.5) * rowh
    out.append(f'<text x="{lw - 6}" y="{y + rowh - 3}" text-anchor="end">held</text>')
    k = 0
    while k < t:
        v = bool(held[k])
        k2 = k
        while k2 < t and bool(held[k2]) == v:
            k2 += 1
        role = "--st-good" if v else "--st-critical"
        out.append(f'<rect x="{lw + k * sx}" y="{y + 1}" '
                   f'width="{(k2 - k) * sx}" height="{rowh - 2}" '
                   f'style="fill:var({role})"><title>{"held" if v else "not held"}'
                   f', ticks {k}-{k2 - 1}</title></rect>')
        k = k2
    out.append("</svg>")
    return "".join(out)


def page(title, caveat, body):
    return ("<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
            f"<title>{esc(title)}</title><style>{CSS}</style></head><body><main>"
            f"<h1>{esc(title)}</h1><p class=\"caveat\">{esc(caveat)}</p>"
            f"{body}</main></body></html>")


def reason_cell(reason):
    """(fill, letter, ink) for a reason cell."""
    letter, role, _ = REASON_STYLE[reason]
    return f"var({role})", letter, f"var({role}-ink)"


def seq_fill(x):
    """x in [0, 1] -> (fill, ink) of the blue sequential step; near zero
    recedes toward the surface in either theme."""
    i = min(len(SEQ) - 1, max(0, int(round(x * (len(SEQ) - 1)))))
    return f"var(--seq{i})", f"var(--seq{i}-ink)"
