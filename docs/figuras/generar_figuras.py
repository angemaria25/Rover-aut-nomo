"""Genera las figuras (SVG) del informe a partir de los resultados reales.

Uso (con el entorno del proyecto activado):

    python docs/figuras/generar_figuras.py

Figuras producidas (en la misma carpeta):

* ``fig1_entorno_ruta.svg``    entorno de la instancia semilla 23 + ruta planificada.
* ``fig2_busqueda.svg``        comparación de los algoritmos de búsqueda.
* ``fig3_planificacion.svg``   plan óptimo vs. metaheurísticas vs. azar, y éxito de las reglas.
* ``fig4_estructuras.svg``     costo por algoritmo según la estructura del terreno.

Todo se dibuja con la biblioteca estándar (sin dependencias externas) para que
las figuras sean reproducibles desde cero.
"""

from __future__ import annotations

import json
from pathlib import Path

from rover_ai.environment import EnvironmentConfig, generate_instance
from rover_ai.metaheuristics import hill_climbing_order, simulated_annealing_order
from rover_ai.models import Terrain
from rover_ai.planning import build_key_graph, charge_keys, comm_keys, plan_mission
from rover_ai.search import a_star

HERE = Path(__file__).resolve().parent
OUTPUTS = HERE.parent.parent / "outputs"

TERRAIN_COLOR = {
    Terrain.PLAIN: "#eef3e2",
    Terrain.STONE: "#d9d9d9",
    Terrain.ROCK: "#b9b9b9",
    Terrain.FINE_SAND: "#f3e3b8",
    Terrain.DUNE: "#e6cf9a",
    Terrain.OBSTACLE: "#4a4a4a",
}


def _header(width: float, height: float) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" font-family="Verdana, Segoe UI, sans-serif">'
    ]


def _cell_center(x: int, y: int, cell: float, margin: float) -> tuple[float, float]:
    return margin + (x + 0.5) * cell, margin + (y + 0.5) * cell


def figura_entorno_ruta() -> Path:
    """Entorno de la instancia semilla 23 con la ruta planificada encima."""

    cell = 30
    margin = 12
    config = EnvironmentConfig(width=12, height=12, n_pois=4, n_comm_zones=2, n_charging_stations=1)
    inst = generate_instance(23, config)
    artifacts = build_key_graph(inst)

    hc_order, hc_score = hill_climbing_order(inst, 23, artifacts=artifacts)
    sa_order, sa_score = simulated_annealing_order(inst, 23, artifacts=artifacts)
    order = sa_order if sa_score <= hc_score else hc_order
    plan, _ = plan_mission(inst, preferred_order=order, enforce_order=True, artifacts=artifacts)

    seq: list[tuple[int, int]] = [inst.base]
    for action in plan:
        if action.name == "move":
            seq.append(artifacts.key_positions[action.arg])

    cells_path: list[tuple[int, int]] = []
    for start, goal in zip(seq, seq[1:]):
        segment = a_star(inst, start, goal).path
        cells_path.extend(segment if not cells_path else segment[1:])

    width = cell * inst.width + 2 * margin
    height = cell * inst.height + 2 * margin
    svg = _header(width, height)

    for y in range(inst.height):
        for x in range(inst.width):
            color = TERRAIN_COLOR[inst.grid[y][x]]
            svg.append(
                f'<rect x="{margin + x * cell}" y="{margin + y * cell}" width="{cell}" height="{cell}" '
                f'fill="{color}" stroke="#ffffff" stroke-width="1"/>'
            )

    centers = [_cell_center(x, y, cell, margin) for x, y in cells_path]
    points = " ".join(f"{cx:.1f},{cy:.1f}" for cx, cy in centers)
    svg.append(
        f'<polyline points="{points}" fill="none" stroke="#1d4ed8" stroke-width="3" '
        f'stroke-linejoin="round" stroke-linecap="round" opacity="0.85"/>'
    )

    # Zonas de comunicación.
    for comm_id in comm_keys(artifacts):
        x, y = _cell_center(*artifacts.key_positions[comm_id], cell, margin)
        svg.append(f'<rect x="{x - 8:.1f}" y="{y - 8:.1f}" width="16" height="16" fill="#16a34a" stroke="#ffffff" stroke-width="1.5"/>')
        svg.append(f'<text x="{x:.1f}" y="{y + 3:.1f}" font-size="9" fill="#ffffff" text-anchor="middle">C</text>')

    # Estaciones de carga (además de la base).
    for charge_id in charge_keys(artifacts)[1:]:
        x, y = _cell_center(*artifacts.key_positions[charge_id], cell, margin)
        svg.append(f'<rect x="{x - 8:.1f}" y="{y - 8:.1f}" width="16" height="16" fill="#f59e0b" stroke="#ffffff" stroke-width="1.5"/>')
        svg.append(f'<text x="{x:.1f}" y="{y + 3:.1f}" font-size="9" fill="#ffffff" text-anchor="middle">R</text>')

    # Base (también punto de recarga).
    bx, by = _cell_center(*inst.base, cell, margin)
    svg.append(f'<circle cx="{bx:.1f}" cy="{by:.1f}" r="10" fill="#2563eb" stroke="#ffffff" stroke-width="1.5"/>')
    svg.append(f'<text x="{bx:.1f}" y="{by + 3.5:.1f}" font-size="10" fill="#ffffff" text-anchor="middle">B</text>')

    # POI en el orden de visita.
    visit_index = {poi_id: i + 1 for i, poi_id in enumerate(order)}
    for poi in inst.pois:
        x, y = _cell_center(*poi.position, cell, margin)
        svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="10" fill="#dc2626" stroke="#ffffff" stroke-width="1.5"/>')
        svg.append(
            f'<text x="{x:.1f}" y="{y + 3.5:.1f}" font-size="10" fill="#ffffff" '
            f'text-anchor="middle">{visit_index[poi.poi_id]}</text>'
        )

    svg.append("</svg>")
    path = HERE / "fig1_entorno_ruta.svg"
    path.write_text("\n".join(svg), encoding="utf-8")
    return path


def _bar_panel(x, y, w, h, title, cats, vals, color, fmt="{:.1f}", label_size=10):
    out = [f'<text x="{x + w / 2:.1f}" y="{y - 8:.1f}" font-size="12" font-weight="bold" text-anchor="middle">{title}</text>']
    bottom = y + h
    vmax = max(vals) if vals else 1.0
    slot = w / len(vals)
    bar_w = slot * 0.55
    for i, (cat, val) in enumerate(zip(cats, vals)):
        cx = x + slot * (i + 0.5)
        bh = (val / vmax) * (h - 26) if vmax else 0
        by = bottom - bh
        out.append(f'<rect x="{cx - bar_w / 2:.1f}" y="{by:.1f}" width="{bar_w:.1f}" height="{bh:.1f}" fill="{color}"/>')
        out.append(f'<text x="{cx:.1f}" y="{by - 4:.1f}" font-size="10" text-anchor="middle">{fmt.format(val)}</text>')
        out.append(f'<text x="{cx:.1f}" y="{bottom + 14:.1f}" font-size="{label_size}" text-anchor="middle">{cat}</text>')
    out.append(f'<line x1="{x:.1f}" y1="{bottom:.1f}" x2="{x + w:.1f}" y2="{bottom:.1f}" stroke="#888" stroke-width="1"/>')
    return out


def figura_busqueda() -> Path:
    """Comparación de los algoritmos de búsqueda (costo y nodos expandidos)."""

    data = json.loads((OUTPUTS / "summary.json").read_text(encoding="utf-8"))["search"]
    cost_names = ["bfs", "dijkstra", "a_star", "greedy"]
    cost_cats = ["BFS", "Dijkstra", "A*", "Voraz"]
    cost = [data[n]["cost"]["mean"] for n in cost_names]
    exp_names = ["bfs", "dfs", "dijkstra", "a_star", "greedy"]
    exp_cats = ["BFS", "DFS", "Dijkstra", "A*", "Voraz"]
    expanded = [data[n]["expanded"]["mean"] for n in exp_names]

    width, height = 760, 330
    svg = _header(width, height)
    svg.append('<text x="380" y="24" font-size="15" font-weight="bold" text-anchor="middle">'
               'Búsqueda en la rejilla (media de 5 instancias 16×16)</text>')
    svg += _bar_panel(40, 60, 320, 210, "Costo de la ruta (sin DFS)", cost_cats, cost, "#2563eb", "{:.1f}", 9)
    svg += _bar_panel(420, 60, 320, 210, "Nodos expandidos", exp_cats, expanded, "#dc2626", "{:.0f}", 9)
    svg.append("</svg>")
    path = HERE / "fig2_busqueda.svg"
    path.write_text("\n".join(svg), encoding="utf-8")
    return path


def figura_planificacion() -> Path:
    """Costo del plan por método y tasa de éxito de la política de reglas."""

    summary = json.loads((OUTPUTS / "summary.json").read_text(encoding="utf-8"))
    plan = summary["planning"]
    rules = summary["rules"]

    cats = ["Óptimo", "Hill C.", "Recocido", "Azar"]
    vals = [plan["optimal"]["plan_cost"]["mean"], plan["hill_climbing"]["plan_cost"]["mean"],
            plan["simulated_annealing"]["plan_cost"]["mean"], plan["random"]["plan_cost"]["mean"]]

    rcats = ["12x12\n4 POI", "14x14\n6 POI", "16x16\n5 POI"]
    rvals = [rules[c]["success_rate"]["mean"] * 100 for c in ("12x12_4poi", "14x14_6poi", "16x16_5poi")]

    width, height = 760, 340
    svg = _header(width, height)
    svg.append('<text x="380" y="26" font-size="15" font-weight="bold" text-anchor="middle">'
               'Planificación y política de reglas (módulo E)</text>')
    svg += _bar_panel(40, 60, 320, 210, "Costo del plan (media)", cats, vals, "#0d9488", "{:.1f}")
    svg += _bar_panel(420, 60, 320, 210, "Misiones completadas (%)", rcats, rvals, "#7c3aed", "{:.0f}")
    svg.append("</svg>")
    path = HERE / "fig3_planificacion.svg"
    path.write_text("\n".join(svg), encoding="utf-8")
    return path


def figura_estructuras() -> Path:
    """Costo medio por algoritmo según la estructura del terreno (perfiles)."""

    data = json.loads((OUTPUTS / "summary.json").read_text(encoding="utf-8"))["structures"]
    profiles = ["flat", "default", "rough"]
    algos = ["bfs", "dijkstra", "a_star", "greedy"]
    labels = ["BFS", "Dijkstra", "A*", "Voraz"]
    colors = ["#93c5fd", "#2563eb", "#111827", "#16a34a"]

    width, height = 760, 360
    left, top, w, h = 60, 70, 660, 220
    svg = _header(width, height)
    svg.append('<text x="380" y="26" font-size="15" font-weight="bold" text-anchor="middle">'
               'Costo por algoritmo según la estructura del terreno (14×14, 5 POI)</text>')

    matrix = [[data[p][a]["cost"]["mean"] for a in algos] for p in profiles]
    vmax = max(max(row) for row in matrix)
    group_w = w / len(profiles)
    bar_w = group_w / (len(algos) + 1)
    bottom = top + h
    for gi, profile in enumerate(profiles):
        gx = left + gi * group_w
        for si in range(len(algos)):
            val = matrix[gi][si]
            bh = (val / vmax) * (h - 30)
            bx = gx + (si + 0.5) * bar_w
            by = bottom - bh
            svg.append(f'<rect x="{bx:.1f}" y="{by:.1f}" width="{bar_w * 0.9:.1f}" height="{bh:.1f}" fill="{colors[si]}"/>')
            svg.append(f'<text x="{bx + bar_w * 0.45:.1f}" y="{by - 3:.1f}" font-size="8" text-anchor="middle">{val:.1f}</text>')
        svg.append(f'<text x="{gx + group_w / 2:.1f}" y="{bottom + 16:.1f}" font-size="11" text-anchor="middle">{profile}</text>')
    svg.append(f'<line x1="{left}" y1="{bottom}" x2="{left + w}" y2="{bottom}" stroke="#888"/>')

    for si, label in enumerate(labels):
        lx = left + si * (w / len(algos))
        svg.append(f'<rect x="{lx:.1f}" y="{32}" width="11" height="11" fill="{colors[si]}"/>')
        svg.append(f'<text x="{lx + 15:.1f}" y="{42}" font-size="10">{label}</text>')

    svg.append("</svg>")
    path = HERE / "fig4_estructuras.svg"
    path.write_text("\n".join(svg), encoding="utf-8")
    return path


def main() -> None:
    for builder in (figura_entorno_ruta, figura_busqueda, figura_planificacion, figura_estructuras):
        print("Generada:", builder().name)


if __name__ == "__main__":
    main()
