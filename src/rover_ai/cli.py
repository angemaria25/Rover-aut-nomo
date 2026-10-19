"""Interfaz de línea de comandos del proyecto.

Punto de entrada único para reproducir todos los resultados:

    python -m rover_ai.cli run-all

y comandos individuales para cada módulo.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from .environment import EnvironmentConfig, generate_instance
from .experiments import (
    benchmark_metaheuristics,
    benchmark_planning,
    benchmark_rules,
    benchmark_search,
    benchmark_structures,
    mission_demo,
    run_all,
    summarize_results,
)
from .planning import build_key_graph, plan_mission
from .metaheuristics import hill_climbing_order, simulated_annealing_order
from .rules import run_rule_agent
from .simulation import execute_plan

OUT_DIR = Path("outputs")


def _serialize_instance(instance) -> dict:
    """Convierte una instancia a un diccionario serializable a JSON."""

    return {
        "width": instance.width,
        "height": instance.height,
        "grid": [[terrain.value for terrain in row] for row in instance.grid],
        "base": list(instance.base),
        "comm_zones": [list(z) for z in sorted(instance.comm_zones)],
        "charging_stations": [list(z) for z in sorted(instance.charging_stations)],
        "pois": [{"id": p.poi_id, "position": list(p.position)} for p in instance.pois],
        "battery_capacity": instance.battery_capacity,
        "memory_capacity": instance.memory_capacity,
    }


def _cmd_save_instance(seed: int, out_path: Path, config: EnvironmentConfig) -> None:
    instance = generate_instance(seed, config)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(_serialize_instance(instance), f, ensure_ascii=False, indent=2)
    print(out_path)


def _cmd_plan_demo(seed: int, config: EnvironmentConfig) -> None:
    instance = generate_instance(seed, config)
    artifacts = build_key_graph(instance)
    plan, cost = plan_mission(instance, enforce_order=False, artifacts=artifacts)
    execution = execute_plan(instance, plan, artifacts)
    print(f"Plan libre (optimo): {len(plan)} acciones, costo simbolico {cost}")
    for action in plan:
        print(f"  {action.name}:{action.arg}")
    print(json.dumps(asdict(execution), ensure_ascii=False, indent=2, default=str))


def _cmd_rules_demo(seed: int, config: EnvironmentConfig) -> None:
    instance = generate_instance(seed, config)
    artifacts = build_key_graph(instance)

    meta_order, meta_score = simulated_annealing_order(instance, seed, artifacts=artifacts)
    _, plan_cost = plan_mission(instance, preferred_order=meta_order, enforce_order=True, artifacts=artifacts)

    print("Politica basada en reglas (encadenamiento hacia adelante):")
    rule_plan, rule_exec = run_rule_agent(instance, artifacts, trace=True)
    print(json.dumps(asdict(rule_exec), ensure_ascii=False, indent=2, default=str))
    print()
    print(f"Comparacion de costo total para la semilla {seed}:")
    print(f"  plan con orden de metaheuristica : {plan_cost}")
    print(f"  politica basada en reglas        : {rule_exec.total_cost}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Proyecto Rover Autónomo - Elementos de IA")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("run-all", help="Ejecuta el protocolo experimental completo")
    sub.add_parser("benchmark-search", help="Compara algoritmos de búsqueda")
    sub.add_parser("benchmark-structures", help="Compara estructuras de terreno (flat/default/rough)")
    sub.add_parser("benchmark-meta", help="Compara metaheurísticas")
    sub.add_parser("benchmark-planning", help="Compara plan optimo vs orden de metaheurística")
    sub.add_parser("benchmark-rules", help="Mide la politica basada en reglas (modulo E)")
    sub.add_parser("summarize", help="Agrega los CSV en outputs/summary.json")

    mission = sub.add_parser("mission-demo", help="Demo de la tubería completa")
    mission.add_argument("--seed", type=int, default=23)

    plan_demo = sub.add_parser("plan-demo", help="Muestra un plan simbólico y su validación")
    plan_demo.add_argument("--seed", type=int, default=23)

    rules_demo = sub.add_parser("rules-demo", help="Muestra la política basada en reglas (módulo E)")
    rules_demo.add_argument("--seed", type=int, default=23)

    save = sub.add_parser("save-instance", help="Guarda una instancia de ejemplo en data/")
    save.add_argument("--seed", type=int, default=23)
    save.add_argument("--out", type=str, default="data/instancia_ejemplo.json")

    args = parser.parse_args()
    config = EnvironmentConfig(width=12, height=12, n_pois=4, n_comm_zones=2, n_charging_stations=1)

    if args.command == "run-all":
        print("Ejecutando el protocolo experimental completo (tarda unos segundos)...\n")
        paths = run_all(OUT_DIR)
        print("\nListo. Resultados guardados en la carpeta 'outputs/':")
        for name, path in paths.items():
            print(f"  {name:>8}: {path}")
        print("\nLas medias y desviaciones estan en outputs/summary.json.")
    elif args.command == "benchmark-search":
        print(benchmark_search([11, 23, 37, 41, 59], OUT_DIR))
    elif args.command == "benchmark-structures":
        print(benchmark_structures([11, 23, 37, 41, 59], OUT_DIR))
    elif args.command == "benchmark-meta":
        print(benchmark_metaheuristics([11, 23, 37, 41, 59], OUT_DIR))
    elif args.command == "benchmark-planning":
        print(benchmark_planning([11, 23, 37, 41, 59], OUT_DIR))
    elif args.command == "benchmark-rules":
        print(benchmark_rules(list(range(1, 21)), OUT_DIR))
    elif args.command == "summarize":
        print(summarize_results(OUT_DIR))
    elif args.command == "mission-demo":
        print(mission_demo(args.seed, OUT_DIR))
    elif args.command == "plan-demo":
        _cmd_plan_demo(args.seed, config)
    elif args.command == "rules-demo":
        _cmd_rules_demo(args.seed, config)
    elif args.command == "save-instance":
        _cmd_save_instance(args.seed, Path(args.out), config)


if __name__ == "__main__":
    main()
