"""Convierte ``informe.md`` en ``informe.html`` (listo para imprimir a PDF).

El conversor es deliberadamente pequeño y sin dependencias: interpreta los
elementos de Markdown que usa el informe (títulos, párrafos, negritas, cursivas,
código en línea, listas, tablas, reglas y la directiva ``{{FIG:...}}``) y luego
inserta los SVG de las figuras en línea.

Uso:

    python docs/build_informe.py
    # genera docs/informe.html
"""

from __future__ import annotations

import html
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "informe.md"
OUT = HERE / "informe.html"
SENTINEL = "\x00STAR\x00"

CSS = """
@page { size: A4; margin: 16mm 15mm 15mm 15mm; }
* { box-sizing: border-box; }
body {
  font-family: "Georgia", "Times New Roman", serif;
  font-size: 10.2pt; line-height: 1.42; color: #1a1a1a; margin: 0;
  text-align: justify; hyphens: auto;
}
h1 { font-size: 18pt; margin: 0 0 2pt 0; text-align: left; }
h3 { font-size: 11.5pt; font-weight: normal; font-style: italic; color: #444;
     margin: 0 0 10pt 0; text-align: left; }
h2 { font-size: 12.5pt; margin: 14pt 0 4pt 0; border-bottom: 1px solid #bbb;
     padding-bottom: 2pt; page-break-after: avoid; }
p { margin: 0 0 6pt 0; }
ul, ol { margin: 0 0 6pt 0; padding-left: 16pt; }
li { margin-bottom: 2pt; }
code { font-family: "Consolas", monospace; font-size: 9pt; background: #f2f2f2;
       padding: 0 2px; border-radius: 2px; }
table { border-collapse: collapse; width: 100%; margin: 4pt 0 8pt 0;
        font-size: 9.2pt; page-break-inside: avoid; }
th, td { border: 1px solid #c8c8c8; padding: 2.5pt 5pt; }
th { background: #eef2f7; text-align: left; }
td { text-align: left; }
tr:nth-child(even) td { background: #fafafa; }
figure { margin: 8pt 0; page-break-inside: avoid; text-align: center; }
figure svg { width: 100%; height: auto; max-height: 92mm; }
figcaption { font-size: 8.6pt; color: #555; margin-top: 3pt; text-align: left; }
hr { border: none; border-top: 1px solid #ccc; margin: 10pt 0; }
.appendix { page-break-before: always; }
"""


def read_source(path: Path) -> str:
    """Lee el Markdown tolerando BOM y codificaciones ANSI/Windows.

    Muchos editores de Windows guardan el archivo en UTF-8 con BOM o en ANSI
    (cp1252). Leído siempre como UTF-8 sin BOM, el BOM rompería el primer título
    y una codificación ANSI provocaría un ``UnicodeDecodeError``. Aquí se prueban
    varias codificaciones y se avisa por consola de cuál se usó.
    """

    data = path.read_bytes()
    for encoding in ("utf-8-sig", "cp1252", "latin-1"):
        try:
            text = data.decode(encoding)
        except UnicodeDecodeError:
            continue
        if encoding != "utf-8-sig":
            print(f"Aviso: {path.name} no está en UTF-8 sin BOM; se leyó como {encoding}.")
        return text.lstrip("\ufeff")
    return data.decode("utf-8", "replace").lstrip("\ufeff")


def inline(text: str) -> str:
    """Aplica el formato en línea (negrita, cursiva, código) a ``text``."""

    text = text.replace("\\*", SENTINEL)
    text = html.escape(text, quote=False)
    text = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", text)
    text = re.sub(r"(?<!\*)\*(?!\s)(.+?)(?<!\s)\*(?!\*)", r"<em>\1</em>", text)
    text = re.sub(r"`(.+?)`", r"<code>\1</code>", text)
    return text.replace(SENTINEL, "*")


def render_table(rows: list[str]) -> str:
    cells = [[c.strip() for c in row.strip().strip("|").split("|")] for row in rows]
    head, body = cells[0], [r for r in cells[2:]]
    out = ["<table>", "<thead><tr>"]
    out += [f"<th>{inline(c)}</th>" for c in head]
    out += ["</tr></thead>", "<tbody>"]
    for row in body:
        out.append("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in row) + "</tr>")
    out.append("</tbody></table>")
    return "\n".join(out)


def render_figure(spec: str) -> str:
    rel, caption = spec.split("|", 1)
    svg_path = HERE / rel.strip()
    if not svg_path.exists():
        raise SystemExit(
            f"No se encontró la figura '{svg_path}'.\n"
            "Genera antes los resultados y las figuras:\n"
            "  python -m rover_ai.cli run-all\n"
            "  python docs/figuras/generar_figuras.py"
        )
    svg = svg_path.read_text(encoding="utf-8")
    return f'<figure>{svg}<figcaption>{inline(caption.strip())}</figcaption></figure>'


def convert(markdown: str) -> str:
    lines = markdown.splitlines()
    out: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        if not stripped:
            i += 1
            continue

        if stripped.startswith("{{FIG:"):
            out.append(render_figure(stripped[len("{{FIG:"):-2]))
            i += 1
            continue

        if stripped.startswith("|"):
            block = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                block.append(lines[i])
                i += 1
            out.append(render_table(block))
            continue

        if stripped.startswith("### "):
            out.append(f"<h3>{inline(stripped[4:])}</h3>")
            i += 1
            continue
        if stripped.startswith("## "):
            title = stripped[3:]
            cls = ' class="appendix"' if title.startswith("Apéndice") else ""
            out.append(f"<h2{cls}>{inline(title)}</h2>")
            i += 1
            continue
        if stripped.startswith("# "):
            out.append(f"<h1>{inline(stripped[2:])}</h1>")
            i += 1
            continue

        if stripped == "---":
            out.append("<hr>")
            i += 1
            continue

        if stripped.startswith("- "):
            items = []
            while i < len(lines) and lines[i].strip().startswith("- "):
                items.append(f"<li>{inline(lines[i].strip()[2:])}</li>")
                i += 1
            out.append("<ul>" + "".join(items) + "</ul>")
            continue

        if re.match(r"^\d+\.\s", stripped):
            items = []
            while i < len(lines) and re.match(r"^\d+\.\s", lines[i].strip()):
                items.append(f"<li>{inline(re.sub(r'^\d+\.\s', '', lines[i].strip()))}</li>")
                i += 1
            out.append("<ol>" + "".join(items) + "</ol>")
            continue

        # Párrafo: se acumulan líneas hasta una línea vacía o un bloque nuevo.
        para = [stripped]
        i += 1
        while i < len(lines):
            nxt = lines[i].strip()
            if not nxt or nxt.startswith(("|", "#", "{{FIG:", "- ")) or re.match(r"^\d+\.\s", nxt):
                break
            para.append(nxt)
            i += 1
        out.append(f"<p>{inline(' '.join(para))}</p>")

    body = "\n".join(out)
    return (
        "<!DOCTYPE html>\n<html lang=\"es\"><head><meta charset=\"utf-8\">"
        f"<title>Informe Proyecto 1 — Rover autónomo</title><style>{CSS}</style></head>"
        f"<body>{body}</body></html>\n"
    )


def main() -> None:
    OUT.write_text(convert(read_source(SRC)), encoding="utf-8")
    print("Generado:", OUT)


if __name__ == "__main__":
    main()
