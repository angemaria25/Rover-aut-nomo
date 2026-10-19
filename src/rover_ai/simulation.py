"""Ejecución y validación concreta de un plan sobre una instancia.

El planificador produce una secuencia de acciones abstractas; este módulo la
ejecuta paso a paso comprobando **todas** las precondiciones (batería, memoria,
lugar correcto, muestra recolectada/analizada) y acumula el costo real. Si
alguna precondición falla, la ejecución se marca como fallida. Además calcula
la puntuación de la misión, que es el criterio que se quiere maximizar.
"""

from __future__ import annotations

from math import inf
from typing import List, Optional, Set

from .models import Action, MissionExecution, MissionInstance, mission_score
from .planning import PlanningArtifacts, build_key_graph, is_charging_key


def execute_plan(
    instance: MissionInstance,
    plan: List[Action],
    artifacts: Optional[PlanningArtifacts] = None,
) -> MissionExecution:
    """Ejecuta (valida) un plan y devuelve el resultado de la misión.

    Se mantiene un estado explícito (posición, batería, memoria y conjuntos de
    muestras recolectadas/analizadas/transmitidas) y cada acción comprueba sus
    precondiciones antes de aplicarse.
    """

    artifacts = artifacts or build_key_graph(instance)

    battery = instance.battery_capacity
    memory_used = 0
    total = 0.0
    at = "BASE"
    collected: Set[str] = set()
    analyzed: Set[str] = set()
    transmitted: Set[str] = set()
    poi_ids = {poi.poi_id for poi in instance.pois}

    def finish(success: bool) -> MissionExecution:
        return MissionExecution(
            plan=plan,
            total_cost=total,
            battery_left=battery,
            collected_count=len(collected),
            analyzed_count=len(analyzed),
            transmitted_count=len(transmitted),
            score=mission_score(instance, len(transmitted), total),
            success=success,
        )

    for action in plan:
        if action.name == "move" and action.arg:
            move_cost = artifacts.move_costs.get((at, action.arg), inf)
            if move_cost == inf or battery < move_cost:
                return finish(False)
            battery -= int(move_cost)
            total += move_cost
            at = action.arg

        elif action.name == "collect" and action.arg:
            if action.arg not in poi_ids or action.arg in collected:
                return finish(False)
            if memory_used >= instance.memory_capacity or battery < instance.collect_cost:
                return finish(False)
            battery -= instance.collect_cost
            memory_used += 1
            total += instance.collect_cost
            collected.add(action.arg)

        elif action.name == "analyze" and action.arg:
            if at != action.arg or action.arg not in collected or action.arg in analyzed:
                return finish(False)
            if battery < instance.analyze_cost:
                return finish(False)
            battery -= instance.analyze_cost
            total += instance.analyze_cost
            analyzed.add(action.arg)

        elif action.name == "transmit" and action.arg:
            if not at.startswith("COMM_") or action.arg not in analyzed or action.arg in transmitted:
                return finish(False)
            if battery < instance.transmit_cost:
                return finish(False)
            battery -= instance.transmit_cost
            memory_used = max(0, memory_used - 1)
            total += instance.transmit_cost
            transmitted.add(action.arg)

        elif action.name == "recharge":
            if not is_charging_key(at):
                return finish(False)
            battery = instance.battery_capacity
            total += instance.recharge_cost

        else:
            # Acción desconocida: plan inválido.
            return finish(False)

    success = transmitted == poi_ids
    return finish(success)
