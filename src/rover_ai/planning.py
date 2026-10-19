"""Módulo B (elegido): planificación simbólica de la misión.

El planificador modela el problema con acciones, precondiciones y efectos
(estilo STRIPS) sobre una abstracción "de puntos clave": las únicas posiciones
relevantes son la base, los POI y las zonas de comunicación. El costo de
moverse entre dos puntos clave lo resuelve el **módulo de búsqueda** (A*),
que es exactamente la integración que pide el enunciado: el planificador
decide la secuencia de alto nivel y la búsqueda resuelve cada desplazamiento
concreto.

Acciones modeladas:

* ``move(destino)``  : precondición = el destino es alcanzable y hay batería;
  efecto = cambia ``at`` y descuenta batería.
* ``collect(poi)``   : precondición = estar en el POI, no haberlo recolectado,
  memoria libre y batería; efecto = lo añade a ``collected`` y ocupa memoria.
* ``analyze(poi)``   : precondición = recolectado y batería; efecto = lo añade
  a ``analyzed``.
* ``transmit(poi)``  : precondición = estar en una zona de comunicación, el
  POI analizado y batería; efecto = lo añade a ``transmitted`` y libera memoria.
* ``recharge()``     : precondición = estar en la base; efecto = batería al
  máximo.

La prueba de meta es que todas las muestras hayan sido transmitidas, igual que
en el dominio del rover de la conferencia C2b (el objetivo no exige volver a la
base).
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from math import inf
from typing import Dict, List, Optional, Tuple

from .models import Action, MissionInstance, PlannerState
from .search import a_star


@dataclass
class PlanningArtifacts:
    """Estructuras auxiliares de una instancia para planificar.

    ``key_positions`` mapea claves lógicas (``BASE``, ``POI_i``, ``COMM_j``,
    ``CHARGE_j``) a celdas, y ``move_costs`` guarda el costo real (A*) de ir de
    una clave a otra. Se calcula una sola vez y se reutiliza en el planificador,
    las metaheurísticas y la simulación.
    """

    key_positions: Dict[str, Tuple[int, int]]
    move_costs: Dict[Tuple[str, str], float]


def build_key_graph(instance: MissionInstance) -> PlanningArtifacts:
    """Construye el grafo de puntos clave con costos de ruta reales (A*)."""

    key_positions: Dict[str, Tuple[int, int]] = {"BASE": instance.base}
    for poi in instance.pois:
        key_positions[poi.poi_id] = poi.position
    for idx, zone in enumerate(sorted(instance.comm_zones)):
        key_positions[f"COMM_{idx+1}"] = zone
    # Estaciones de carga (la base ya es un punto de recarga y tiene su clave).
    charge_stations = sorted(instance.charging_stations - {instance.base})
    for idx, station in enumerate(charge_stations):
        key_positions[f"CHARGE_{idx+1}"] = station

    move_costs: Dict[Tuple[str, str], float] = {}
    keys = list(key_positions.keys())
    for i, src in enumerate(keys):
        for j, dst in enumerate(keys):
            if i == j:
                move_costs[(src, dst)] = 0.0
                continue
            result = a_star(instance, key_positions[src], key_positions[dst])
            move_costs[(src, dst)] = result.cost

    return PlanningArtifacts(key_positions=key_positions, move_costs=move_costs)


def comm_keys(artifacts: PlanningArtifacts) -> List[str]:
    """Lista de claves lógicas que corresponden a zonas de comunicación."""

    return [k for k in artifacts.key_positions if k.startswith("COMM_")]


def charge_keys(artifacts: PlanningArtifacts) -> List[str]:
    """Lista de claves lógicas donde se puede recargar (base + estaciones)."""

    return ["BASE"] + [k for k in artifacts.key_positions if k.startswith("CHARGE_")]


def is_charging_key(key: str) -> bool:
    """Indica si en la clave lógica ``key`` se puede recargar."""

    return key == "BASE" or key.startswith("CHARGE_")


def _goal(state: PlannerState, poi_ids: List[str]) -> bool:
    """Meta: todas las muestras han sido transmitidas."""

    return len(state.transmitted) == len(poi_ids)


def _mst(nodes: List[str], move_costs: Dict[Tuple[str, str], float]) -> float:
    """Longitud del árbol de expansión mínimo sobre ``nodes``.

    Es una cota inferior de la longitud de cualquier recorrido que visite todos
    esos nodos, porque todo recorrido conexo contiene un árbol de expansión.
    """

    unique = list(dict.fromkeys(nodes))
    if len(unique) <= 1:
        return 0.0

    in_tree = {unique[0]}
    remaining = set(unique[1:])
    total = 0.0
    while remaining:
        best_cost = inf
        best_node = None
        for u in in_tree:
            for v in remaining:
                c = move_costs.get((u, v), inf)
                if c < best_cost:
                    best_cost, best_node = c, v
        if best_node is None:
            return inf
        total += best_cost
        in_tree.add(best_node)
        remaining.discard(best_node)
    return total


def _heuristic(
    state: PlannerState,
    instance: MissionInstance,
    poi_ids: List[str],
    artifacts: PlanningArtifacts,
) -> float:
    """Heurística admisible: acciones obligatorias + cota inferior de movimiento.

    Dos cotas inferiores, ambas admisibles (nunca sobreestiman el costo real):

    1. **Acciones**: para cada muestra sin transmitir hay que recolectarla,
       analizarla y transmitirla (si no se hizo ya).
    2. **Movimiento**: el rover aún tiene que visitar los POI pendientes y, si
       queda alguna muestra por transmitir, llegar a una zona de comunicación.
       La longitud del árbol de expansión mínimo sobre esos nodos (más la
       posición actual) es una cota inferior de ese desplazamiento.
    """

    total = 0.0
    need_visit: List[str] = []
    needs_comm = False

    for poi_id in poi_ids:
        if poi_id in state.transmitted:
            continue
        needs_comm = True
        if poi_id not in state.collected:
            total += instance.collect_cost + instance.analyze_cost + instance.transmit_cost
            need_visit.append(poi_id)
        elif poi_id not in state.analyzed:
            total += instance.analyze_cost + instance.transmit_cost
            need_visit.append(poi_id)
        else:
            total += instance.transmit_cost

    if needs_comm:
        nodes = [state.at] + need_visit
        comms = comm_keys(artifacts)
        if comms:
            total += min(_mst(nodes + [c], artifacts.move_costs) for c in comms)
        else:
            total += _mst(nodes, artifacts.move_costs)

    return total


def _ordered_pois(instance: MissionInstance, preferred_order: Optional[List[str]]) -> List[str]:
    """Orden de visita de los POI.

    Si el módulo de metaheurísticas (o el sistema de reglas) propone un orden,
    se respeta para los POI existentes; los que falten se añaden al final.
    """

    base_order = [poi.poi_id for poi in instance.pois]
    if not preferred_order:
        return base_order
    preferred = [p for p in preferred_order if p in base_order]
    missing = [p for p in base_order if p not in preferred]
    return preferred + missing


def _available_actions(
    state: PlannerState,
    instance: MissionInstance,
    poi_ids: List[str],
    artifacts: PlanningArtifacts,
    enforce_order: bool,
) -> List[Tuple[Action, PlannerState, float]]:
    """Genera las acciones aplicables en ``state`` con su transición y costo."""

    actions: List[Tuple[Action, PlannerState, float]] = []

    # --- move(destino) ---
    for destination in artifacts.key_positions:
        if destination == state.at:
            continue
        move_cost = artifacts.move_costs[(state.at, destination)]
        if move_cost == inf or move_cost > state.battery:
            continue
        actions.append(
            (
                Action("move", destination),
                PlannerState(
                    at=destination,
                    battery=state.battery - int(move_cost),
                    memory_used=state.memory_used,
                    collected=state.collected,
                    analyzed=state.analyzed,
                    transmitted=state.transmitted,
                ),
                move_cost,
            )
        )

    # --- collect(poi) ---
    # Si se respeta el orden propuesto, solo se puede recolectar el siguiente
    # POI pendiente de ese orden (así la metaheurística influye de verdad).
    next_poi = next((p for p in poi_ids if p not in state.collected), None)
    can_collect_here = state.at in poi_ids and state.at not in state.collected
    if enforce_order and next_poi is not None:
        can_collect_here = can_collect_here and state.at == next_poi
    can_collect_here = (
        can_collect_here
        and state.memory_used < instance.memory_capacity
        and state.battery >= instance.collect_cost
    )
    if can_collect_here:
        actions.append(
            (
                Action("collect", state.at),
                PlannerState(
                    at=state.at,
                    battery=state.battery - instance.collect_cost,
                    memory_used=state.memory_used + 1,
                    collected=state.collected.union({state.at}),
                    analyzed=state.analyzed,
                    transmitted=state.transmitted,
                ),
                float(instance.collect_cost),
            )
        )

    # --- analyze(poi) ---
    if state.at in state.collected and state.at not in state.analyzed and state.battery >= instance.analyze_cost:
        actions.append(
            (
                Action("analyze", state.at),
                PlannerState(
                    at=state.at,
                    battery=state.battery - instance.analyze_cost,
                    memory_used=state.memory_used,
                    collected=state.collected,
                    analyzed=state.analyzed.union({state.at}),
                    transmitted=state.transmitted,
                ),
                float(instance.analyze_cost),
            )
        )

    # --- transmit(poi) ---
    if state.at.startswith("COMM_") and state.battery >= instance.transmit_cost:
        for poi_id in sorted(state.analyzed - state.transmitted):
            actions.append(
                (
                    Action("transmit", poi_id),
                    PlannerState(
                        at=state.at,
                        battery=state.battery - instance.transmit_cost,
                        memory_used=max(0, state.memory_used - 1),
                        collected=state.collected,
                        analyzed=state.analyzed,
                        transmitted=state.transmitted.union({poi_id}),
                    ),
                    float(instance.transmit_cost),
                )
            )

    # --- recharge() ---
    if is_charging_key(state.at) and state.battery < instance.battery_capacity:
        actions.append(
            (
                Action("recharge", None),
                PlannerState(
                    at=state.at,
                    battery=instance.battery_capacity,
                    memory_used=state.memory_used,
                    collected=state.collected,
                    analyzed=state.analyzed,
                    transmitted=state.transmitted,
                ),
                float(instance.recharge_cost),
            )
        )

    return actions


def _reconstruct_actions(
    parent: Dict[PlannerState, Tuple[PlannerState, Action]], final_state: PlannerState
) -> List[Action]:
    """Reconstruye la lista de acciones desde el estado inicial hasta el meta."""

    actions: List[Action] = []
    current = final_state
    while current in parent:
        prev, action = parent[current]
        actions.append(action)
        current = prev
    actions.reverse()
    return actions


def plan_mission(
    instance: MissionInstance,
    preferred_order: Optional[List[str]] = None,
    enforce_order: bool = False,
    artifacts: Optional[PlanningArtifacts] = None,
) -> Tuple[List[Action], float]:
    """Planifica la misión completa mediante búsqueda A* en el espacio de planes.

    Args:
        instance: instancia de misión.
        preferred_order: orden de POI sugerido (por la metaheurística o reglas).
        enforce_order: si es ``True``, el planificador solo permite recolectar
            los POI en el orden sugerido (así el orden influye en la solución).
        artifacts: grafo de puntos clave ya calculado (evita recomputarlo).

    Returns:
        ``(plan, costo)``. Si no hay plan, ``([], inf)``.
    """

    artifacts = artifacts or build_key_graph(instance)
    poi_ids = _ordered_pois(instance, preferred_order)

    start = PlannerState(at="BASE", battery=instance.battery_capacity, memory_used=0)
    frontier: List[Tuple[float, int, PlannerState]] = []
    counter = 0
    heapq.heappush(frontier, (0.0, counter, start))

    g_cost: Dict[PlannerState, float] = {start: 0.0}
    parent: Dict[PlannerState, Tuple[PlannerState, Action]] = {}

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if _goal(current, poi_ids):
            return _reconstruct_actions(parent, current), g_cost[current]

        for action, nxt, c in _available_actions(current, instance, poi_ids, artifacts, enforce_order):
            tentative = g_cost[current] + c
            if tentative < g_cost.get(nxt, inf):
                g_cost[nxt] = tentative
                parent[nxt] = (current, action)
                counter += 1
                score = tentative + _heuristic(nxt, instance, poi_ids, artifacts)
                heapq.heappush(frontier, (score, counter, nxt))

    return [], inf
