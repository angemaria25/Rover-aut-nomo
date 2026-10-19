"""Protocolo experimental: ejecuta los módulos y guarda resultados.

Todo lo que se compara aquí sigue el protocolo del curso:

* varias instancias generadas por semilla (distintos tamaños por módulo);
* varias corridas con semillas declaradas cuando el algoritmo es aleatorio;
* presupuesto de cómputo igual para los algoritmos comparados;
* reporte de media y dispersión (no de la mejor corrida).

Los resultados se escriben en ``outputs/`` (CSV) y ``outputs/summary.json``.
"""

from __future__ import annotations

import csv
import json
import statistics
from dataclasses import asdict
from math import isnan
from pathlib import Path
from time import perf_counter
from typing import Dict, List

from .environment import EnvironmentConfig, generate_instance
from .metaheuristics import (
    DEFAULT_EVALUATIONS,
    evaluate_order,
    hill_climbing_order,
    random_order,
    simulated_annealing_order,
)
from .planning import build_key_graph, plan_mission
from .rules import run_rule_agent
from .search import SEARCH_ALGORITHMS
from .simulation import execute_plan

# Semillas por defecto para el protocolo (declaradas en el informe).
SEARCH_SEEDS = [11, 23, 37, 41, 59]
META_SEEDS = [11, 23, 37, 41, 59]
PLANNING_SEEDS = [11, 23, 37, 41, 59]
RULES_SEEDS = list(range(1, 21))  # 20 instancias para medir la política de reglas
MISSION_SEED = 23  # semilla fija para la demo (reproducibilidad)
STRUCTURE_PROFILES = ["flat", "default", "rough"]  # estructuras de terreno a comparar


def _mean_std(values: List[float]) -> Dict[str, float]:
    """Media y desviación estándar poblacional de una lista de valores."""

    values = [v for v in values if v is not None and not (isinstance(v, float) and isnan(v))]
    if not values:
        return {"mean": float("nan"), "std": float("nan")}
    return {
        "mean": statistics.mean(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
    }


def benchmark_search(seeds: List[int], out_dir: Path) -> Path:
    """Compara los tres algoritmos de búsqueda sobre varias instancias."""

    out_dir.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, float | int | str]] = []

    for seed in seeds:
        instance = generate_instance(seed, EnvironmentConfig(width=16, height=16, n_pois=5, n_comm_zones=2))
        points = [instance.base] + [poi.position for poi in instance.pois]

        for i in range(len(points) - 1):
            start, goal = points[i], points[i + 1]
            for name, algorithm in SEARCH_ALGORITHMS.items():
                result = algorithm(instance, start, goal)
                rows.append(
                    {
                        "seed": seed,
                        "segment": i,
                        "algorithm": name,
                        "cost": result.cost,
                        "expanded": result.expanded,
                        "runtime_ms": result.runtime_ms,
                        "path_found": 1 if result.path else 0,
                    }
                )

    csv_path = out_dir / "search_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def benchmark_structures(seeds: List[int], out_dir: Path) -> Path:
    """Compara los algoritmos de búsqueda sobre distintas ESTRUCTURAS de terreno.

    La estructura (mezcla de terrenos) es un parámetro del generador; aquí se
    ejecuta el mismo algoritmo con el mismo tamaño pero con perfiles distintos
    ("flat", "default", "rough") para comprobar si la conducta cambia, tal como
    pide el curso (probar los algoritmos en sistemas diferentes).
    """

    out_dir.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, float | int | str]] = []

    for profile in STRUCTURE_PROFILES:
        for seed in seeds:
            instance = generate_instance(
                seed,
                EnvironmentConfig(width=14, height=14, n_pois=5, n_comm_zones=2, terrain_profile=profile),
            )
            points = [instance.base] + [poi.position for poi in instance.pois]

            for i in range(len(points) - 1):
                start, goal = points[i], points[i + 1]
                for name, algorithm in SEARCH_ALGORITHMS.items():
                    result = algorithm(instance, start, goal)
                    rows.append(
                        {
                            "profile": profile,
                            "seed": seed,
                            "segment": i,
                            "algorithm": name,
                            "cost": result.cost,
                            "expanded": result.expanded,
                            "runtime_ms": result.runtime_ms,
                            "path_found": 1 if result.path else 0,
                        }
                    )

    csv_path = out_dir / "structure_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def benchmark_metaheuristics(
    seeds: List[int],
    out_dir: Path,
    runs_per_seed: int = 8,
    max_evaluations: int = DEFAULT_EVALUATIONS,
) -> Path:
    """Compara Hill Climbing y Recocido Simulado con el mismo presupuesto.

    Para cada corrida se guarda la calidad del orden (función objetivo del
    módulo) y el costo real del plan que resulta al imponer ese orden, de modo
    que se puede discutir la brecha entre la función de evaluación y el plan.
    """

    out_dir.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, float | int | str]] = []

    for seed in seeds:
        instance = generate_instance(seed, EnvironmentConfig(width=14, height=14, n_pois=4, n_comm_zones=2))
        artifacts = build_key_graph(instance)
        for run in range(runs_per_seed):
            local_seed = seed * 100 + run

            t0 = perf_counter()
            hc_order, hc_score = hill_climbing_order(instance, local_seed, max_evaluations, artifacts)
            hc_ms = (perf_counter() - t0) * 1000

            t0 = perf_counter()
            sa_order, sa_score = simulated_annealing_order(instance, local_seed, max_evaluations=max_evaluations, artifacts=artifacts)
            sa_ms = (perf_counter() - t0) * 1000

            for algo, order, score, ms in (
                ("hill_climbing", hc_order, hc_score, hc_ms),
                ("simulated_annealing", sa_order, sa_score, sa_ms),
            ):
                _, plan_cost = plan_mission(instance, preferred_order=order, enforce_order=True, artifacts=artifacts)
                rows.append(
                    {
                        "seed": seed,
                        "run": run,
                        "algorithm": algo,
                        "objective": score,
                        "plan_cost": plan_cost,
                        "runtime_ms": ms,
                    }
                )

    csv_path = out_dir / "metaheuristic_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def benchmark_planning(seeds: List[int], out_dir: Path, random_runs: int = 6) -> Path:
    """Compara el plan óptimo con el plan guiado por la metaheurística.

    * ``optimal``: planificador libre (A* en el espacio de planes), cota de
      referencia.
    * ``hill_climbing`` / ``simulated_annealing``: plan forzado al orden que
      propone cada metaheurística.
    * ``random``: plan forzado a órdenes aleatorios (línea base).
    """

    out_dir.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, float | int | str]] = []

    for seed in seeds:
        instance = generate_instance(seed, EnvironmentConfig(width=12, height=12, n_pois=4, n_comm_zones=2, n_charging_stations=1))
        artifacts = build_key_graph(instance)

        t0 = perf_counter()
        _, optimal_cost = plan_mission(instance, enforce_order=False, artifacts=artifacts)
        rows.append(
            {
                "seed": seed,
                "method": "optimal",
                "order": "",
                "plan_cost": optimal_cost,
                "runtime_ms": (perf_counter() - t0) * 1000,
            }
        )

        hc_order, _ = hill_climbing_order(instance, seed, artifacts=artifacts)
        _, hc_cost = plan_mission(instance, preferred_order=hc_order, enforce_order=True, artifacts=artifacts)
        rows.append({"seed": seed, "method": "hill_climbing", "order": "|".join(hc_order), "plan_cost": hc_cost, "runtime_ms": 0.0})

        sa_order, _ = simulated_annealing_order(instance, seed, artifacts=artifacts)
        _, sa_cost = plan_mission(instance, preferred_order=sa_order, enforce_order=True, artifacts=artifacts)
        rows.append({"seed": seed, "method": "simulated_annealing", "order": "|".join(sa_order), "plan_cost": sa_cost, "runtime_ms": 0.0})

        for run in range(random_runs):
            order, _ = random_order(instance, seed * 100 + run, artifacts)
            _, cost = plan_mission(instance, preferred_order=order, enforce_order=True, artifacts=artifacts)
            rows.append({"seed": seed, "method": "random", "order": "|".join(order), "plan_cost": cost, "runtime_ms": 0.0})

    csv_path = out_dir / "planning_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def benchmark_rules(seeds: List[int], out_dir: Path) -> Path:
    """Mide la política basada en reglas (módulo E) sobre varias instancias.

    Para cada configuración se reporta la tasa de éxito (planes de reglas que
    logran transmitir todas las muestras) y el costo medio.

    :param seeds: lista de semillas (cada una es una instancia distinta).
    :param out_dir: carpeta de salida.
    """

    out_dir.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, float | int | str]] = []

    configs = [
        ("12x12_4poi", EnvironmentConfig(width=12, height=12, n_pois=4, n_comm_zones=2, n_charging_stations=1)),
        ("14x14_6poi", EnvironmentConfig(width=14, height=14, n_pois=6, n_comm_zones=2, n_charging_stations=2)),
        ("16x16_5poi", EnvironmentConfig(width=16, height=16, n_pois=5, n_comm_zones=2, n_charging_stations=2)),
    ]

    for name, config in configs:
        for seed in seeds:
            instance = generate_instance(seed, config)
            artifacts = build_key_graph(instance)
            _, execution = run_rule_agent(instance, artifacts)
            rows.append(
                {
                    "config": name,
                    "seed": seed,
                    "success": 1 if execution.success else 0,
                    "total_cost": execution.total_cost,
                }
            )

    csv_path = out_dir / "rules_results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def mission_demo(seed: int, out_dir: Path) -> Path:
    """Demo de la tubería completa para una instancia (reproducible).

    Metaheurísticas -> orden de POI -> planificador (que impone el orden y usa
    búsqueda para cada movimiento) -> simulación/validación. Se comparan además
    el plan libre (óptimo) y la política basada en reglas (módulo E).
    """

    out_dir.mkdir(parents=True, exist_ok=True)
    instance = generate_instance(seed, EnvironmentConfig(width=12, height=12, n_pois=4, n_comm_zones=2, n_charging_stations=1))
    artifacts = build_key_graph(instance)

    hc_order, hc_score = hill_climbing_order(instance, seed, artifacts=artifacts)
    sa_order, sa_score = simulated_annealing_order(instance, seed, artifacts=artifacts)
    best_order = sa_order if sa_score <= hc_score else hc_order

    plan, symbolic_cost = plan_mission(instance, preferred_order=best_order, enforce_order=True, artifacts=artifacts)
    execution = execute_plan(instance, plan, artifacts)

    free_plan, free_cost = plan_mission(instance, enforce_order=False, artifacts=artifacts)
    free_execution = execute_plan(instance, free_plan, artifacts)

    rule_plan, rule_execution = run_rule_agent(instance, artifacts)

    output = {
        "seed": seed,
        "configuration": {"width": instance.width, "height": instance.height, "n_pois": len(instance.pois)},
        "metaheuristic": {
            "hill_climbing": {"order": hc_order, "objective": hc_score},
            "simulated_annealing": {"order": sa_order, "objective": sa_score},
            "selected_order": best_order,
        },
        "enforced_plan": {
            "symbolic_cost": symbolic_cost,
            "execution": asdict(execution),
            "actions": [f"{a.name}:{a.arg}" for a in plan],
        },
        "optimal_free_plan": {
            "symbolic_cost": free_cost,
            "execution": asdict(free_execution),
        },
        "rule_based_policy": {
            "execution": asdict(rule_execution),
            "actions": [f"{a.name}:{a.arg}" for a in rule_plan],
        },
    }

    json_path = out_dir / "mission_demo.json"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(output, f, ensure_ascii=False, indent=2, default=str)
    return json_path


def summarize_results(out_dir: Path) -> Path:
    """Agrega los CSV en ``summary.json`` (medias y desviaciones)."""

    out_dir.mkdir(parents=True, exist_ok=True)
    summary: Dict[str, Dict] = {
        "search": {},
        "structures": {},
        "metaheuristics": {},
        "planning": {},
        "rules": {},
    }

    search_path = out_dir / "search_results.csv"
    if search_path.exists():
        by_algo: Dict[str, Dict[str, List[float]]] = {}
        with search_path.open("r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if int(row["path_found"]) != 1:
                    continue  # no se promedian costos de rutas inexistentes
                algo = row["algorithm"]
                bucket = by_algo.setdefault(algo, {"cost": [], "expanded": [], "runtime_ms": []})
                bucket["cost"].append(float(row["cost"]))
                bucket["expanded"].append(float(row["expanded"]))
                bucket["runtime_ms"].append(float(row["runtime_ms"]))
        for algo, metrics in by_algo.items():
            summary["search"][algo] = {k: _mean_std(v) for k, v in metrics.items()}

    structure_path = out_dir / "structure_results.csv"
    if structure_path.exists():
        by_profile: Dict[str, Dict[str, Dict[str, List[float]]]] = {}
        with structure_path.open("r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if int(row["path_found"]) != 1:
                    continue
                bucket = by_profile.setdefault(row["profile"], {}).setdefault(
                    row["algorithm"], {"cost": [], "expanded": [], "runtime_ms": []}
                )
                bucket["cost"].append(float(row["cost"]))
                bucket["expanded"].append(float(row["expanded"]))
                bucket["runtime_ms"].append(float(row["runtime_ms"]))
        for profile, by_algo in by_profile.items():
            summary["structures"][profile] = {algo: {k: _mean_std(v) for k, v in m.items()} for algo, m in by_algo.items()}

    meta_path = out_dir / "metaheuristic_results.csv"
    if meta_path.exists():
        by_algo = {}
        with meta_path.open("r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                algo = row["algorithm"]
                bucket = by_algo.setdefault(algo, {"objective": [], "plan_cost": [], "runtime_ms": []})
                bucket["objective"].append(float(row["objective"]))
                bucket["plan_cost"].append(float(row["plan_cost"]))
                bucket["runtime_ms"].append(float(row["runtime_ms"]))
        for algo, metrics in by_algo.items():
            summary["metaheuristics"][algo] = {k: _mean_std(v) for k, v in metrics.items()}

    planning_path = out_dir / "planning_results.csv"
    if planning_path.exists():
        by_method = {}
        with planning_path.open("r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                method = row["method"]
                bucket = by_method.setdefault(method, {"plan_cost": []})
                bucket["plan_cost"].append(float(row["plan_cost"]))
        for method, metrics in by_method.items():
            summary["planning"][method] = {k: _mean_std(v) for k, v in metrics.items()}

    rules_path = out_dir / "rules_results.csv"
    if rules_path.exists():
        by_config: Dict[str, Dict[str, List[float]]] = {}
        with rules_path.open("r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                bucket = by_config.setdefault(row["config"], {"success_rate": [], "total_cost": []})
                bucket["success_rate"].append(float(row["success"]))
                bucket["total_cost"].append(float(row["total_cost"]))
        for config, metrics in by_config.items():
            summary["rules"][config] = {k: _mean_std(v) for k, v in metrics.items()}

    out = out_dir / "summary.json"
    with out.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    return out


def run_all(out_dir: Path) -> Dict[str, Path]:
    """Ejecuta todo el protocolo experimental con semillas fijas.

    Imprime el progreso paso a paso para que se vea que el programa avanza
    (el protocolo completo tarda unos segundos porque ejecuta varias instancias
    y corridas).
    """

    print("  [1/7] Benchmark de busqueda (BFS, DFS, Dijkstra, A*, voraz)...")
    paths = {"search": benchmark_search(SEARCH_SEEDS, out_dir)}
    print("  [2/7] Benchmark de estructuras de terreno (perfiles flat/default/rough)...")
    paths["structures"] = benchmark_structures(SEARCH_SEEDS, out_dir)
    print("  [3/7] Benchmark de metaheuristicas (Hill Climbing y Recocido Simulado)...")
    paths["metaheuristics"] = benchmark_metaheuristics(META_SEEDS, out_dir)
    print("  [4/7] Benchmark de planificacion (optimo vs orden de metaheuristica)...")
    paths["planning"] = benchmark_planning(PLANNING_SEEDS, out_dir)
    print("  [5/7] Benchmark de la politica basada en reglas (modulo E)...")
    paths["rules"] = benchmark_rules(RULES_SEEDS, out_dir)
    print("  [6/7] Demo de la tuberia completa (semilla fija)...")
    paths["mission"] = mission_demo(MISSION_SEED, out_dir)
    print("  [7/7] Agregando resultados (medias y desviaciones)...")
    paths["summary"] = summarize_results(out_dir)
    return paths
