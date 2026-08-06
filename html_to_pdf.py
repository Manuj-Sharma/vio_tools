#!/usr/bin/env python3
"""Render the session report HTML to a tightly-spaced monospace PDF.

LibreOffice's writer_web filter triples the line spacing inside <pre> and
ignores line-height, and no pandoc/wkhtmltopdf/latex/groff-pdf is installed on
this box. matplotlib's PdfPages is, and gives exact control over leading, so
the report is flattened to text and typeset here instead.

  html_to_pdf.py IN.html OUT.pdf
"""
import html
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

SRC, DST = sys.argv[1], sys.argv[2]
WRAP = 96          # chars per line for prose
LINES_PER_PAGE = 60

raw = open(SRC, encoding="utf-8").read()
raw = re.sub(r"<head>.*?</head>", "", raw, flags=re.S)
raw = re.sub(r"<style>.*?</style>", "", raw, flags=re.S)

def clean(s):
    s = re.sub(r"<[^>]+>", "", s)
    s = html.unescape(s)
    return s.replace("—", "--").replace("–", "-").replace("’", "'") \
            .replace("“", '"').replace("”", '"').replace("·", "-") \
            .replace("±", "+/-").replace("≥", ">=").replace("µ", "u") \
            .replace("é", "e").replace("→", "->").replace("²", "^2")

def wrap(text, width, indent=""):
    out, line = [], indent
    for w in text.split():
        if len(line) + len(w) + 1 > width and line.strip():
            out.append(line.rstrip()); line = indent + w + " "
        else:
            line += w + " "
    if line.strip():
        out.append(line.rstrip())
    return out

# Walk the body in document order, keeping <pre> verbatim.
lines = []
body = raw[raw.index("<body>") + 6: raw.index("</body>")]
for chunk in re.split(r"(<pre>.*?</pre>)", body, flags=re.S):
    if chunk.startswith("<pre>"):
        lines.append("")
        for l in clean(chunk[5:-6]).split("\n"):
            if l.strip():
                lines.append("    " + l.rstrip())
        lines.append("")
        continue
    for m in re.finditer(r"<(h1|h2|h3|p|li|div)[^>]*>(.*?)</\1>", chunk, flags=re.S):
        tag, txt = m.group(1), clean(m.group(2)).strip()
        if not txt:
            continue
        if tag == "h1":
            lines += ["", "=" * WRAP] + wrap(txt, WRAP) + ["=" * WRAP]
        elif tag == "h2":
            lines += ["", ""] + wrap(txt, WRAP) + ["-" * WRAP]
        elif tag == "h3":
            lines += [""] + wrap(txt, WRAP)
        elif tag == "li":
            lines += wrap("* " + txt, WRAP, "")
        else:
            lines += [""] + wrap(txt, WRAP)

pages = [lines[i:i + LINES_PER_PAGE] for i in range(0, len(lines), LINES_PER_PAGE)]
with PdfPages(DST) as pdf:
    for n, page in enumerate(pages, 1):
        fig = plt.figure(figsize=(8.27, 11.69))          # A4 portrait
        fig.text(0.07, 0.955, "\n".join(page), family="monospace", fontsize=7.1,
                 va="top", ha="left", linespacing=1.32)
        fig.text(0.5, 0.028, f"{n} / {len(pages)}", family="monospace",
                 fontsize=7, ha="center", color="#666")
        pdf.savefig(fig); plt.close(fig)

print(f"{len(lines)} lines -> {len(pages)} pages -> {DST}")
