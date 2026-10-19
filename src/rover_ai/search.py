"""Módulo A (obligatorio): búsqueda de rutas entre objetivos.

Implementa cinco algoritmos clásicos sobre el grafo implícito de la rejilla,
con costo variable según el terreno:

* ``bfs``: búsqueda no informada en anchura (minimiza el número de pasos, no el
  costo del terreno).
* ``dfs``: búsqueda no informada en profundidad (encuentra *una* ruta, sin
  garantía de optimalidad).
* ``dijkstra``: búsqueda no informada, óptima (costo uniforme).
* ``a_star``: búsqueda informada con heurística de Manhattan, óptima porque la
  heurística es admisible y consistente.
* ``greedy_best_first``: búsqueda voraz guiada solo por la heurística; rápida
  pero sin garantía de optimalidad (sirve de contraste).

Todos devuelven un :class:`SearchResult` con la ruta, el costo, el número de
nodos expandidos y el tiempo de ejecución, que es lo que compara el protocolo
experimental.
"""

from __future__ import annotations

import heapq
import time
from collections import deque
from math import inf
from typing import Dict, List, Tuple

from .environment import neighbors4
from .models import MissionInstance, Position, SearchResult, TERRAIN_COST


def _reconstruct(came_from: Dict[Position, Position], start: Position, goal: Position) -> List[Position]:
    """Reconstruye la ruta desde ``start`` hasta ``goal`` siguiendo los punteros."""

    if goal not in came_from and goal != start:
        return []
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _reconstruct_simple(came_from: Dict[Position, Position | None], start: Position, goal: Position) -> List[Position]:
    """Reconstruye la ruta cuando ``came_from[start]`` es ``None``."""

    if goal not in came_from:
        return []
    path = [goal]
    while path[-1] != start:
        parent = came_from[path[-1]]
        if parent is None:
            return []
        path.append(parent)
    path.reverse()
    return path


def _path_cost(instance: MissionInstance, path: List[Position]) -> float:
    """Costo real (según el terreno) de recorrer una ruta ya encontrada."""

    return float(sum(_step_cost(instance, p) for p in path[1:])) if path else inf


def _step_cost(instance: MissionInstance, position: Position) -> float:
    """Costo de entrar a una celda (según su terreno)."""

    x, y = position
    return float(TERRAIN_COST[instance.grid[y][x]])


def manhattan(a: Position, b: Position) -> int:
    """Distancia Manhattan, cota inferior admisible del costo restante.

    Como el costo mínimo de entrar a cualquier celda es 1, la distancia
    Manhattan nunca sobreestima el costo real y por tanto A* sigue siendo
    óptimo.
    """

    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def breadth_first_search(instance: MissionInstance, start: Position, goal: Position) -> SearchResult:
    """Búsqueda en anchura (BFS): ruta con el menor número de celdas.

    No pondera el terreno, así que minimiza pasos y no costo; se reporta el
    costo real de la ruta encontrada para poder compararla con las demás.
    """

    t0 = time.perf_counter()
    came_from: Dict[Position, Position | None] = {start: None}
    queue: deque[Position] = deque([start])
    expanded = 0

    while queue:
        current = queue.popleft()
        expanded += 1
        if current == goal:
            break
        for nxt in neighbors4(instance, current):
            if nxt not in came_from:
                came_from[nxt] = current
                queue.append(nxt)

    path = _reconstruct_simple(came_from, start, goal)
    runtime_ms = (time.perf_counter() - t0) * 1000
    return SearchResult(path=path, cost=_path_cost(instance, path), expanded=expanded, runtime_ms=runtime_ms)


def depth_first_search(instance: MissionInstance, start: Position, goal: Position) -> SearchResult:
    """Búsqueda en profundidad (DFS): encuentra una ruta, no necesariamente óptima."""

    t0 = time.perf_counter()
    came_from: Dict[Position, Position | None] = {start: None}
    stack: List[Position] = [start]
    expanded = 0

    while stack:
        current = stack.pop()
        expanded += 1
        if current == goal:
            break
        for nxt in reversed(list(neighbors4(instance, current))):
            if nxt not in came_from:
                came_from[nxt] = current
                stack.append(nxt)

    path = _reconstruct_simple(came_from, start, goal)
    runtime_ms = (time.perf_counter() - t0) * 1000
    return SearchResult(path=path, cost=_path_cost(instance, path), expanded=expanded, runtime_ms=runtime_ms)



def dijkstra(instance: MissionInstance, start: Position, goal: Position) -> SearchResult:
    """Búsqueda de costo uniforme (Dijkstra/UCS) con costos de terreno."""

    t0 = time.perf_counter()
    frontier: List[Tuple[float, int, Position]] = [(0.0, 0, start)]
    dist: Dict[Position, float] = {start: 0.0}
    came_from: Dict[Position, Position] = {}
    settled: set[Position] = set()
    expanded = 0
    counter = 0

    while frontier:
        cur_cost, _, current = heapq.heappop(frontier)
        if current in settled:
            continue
        settled.add(current)
        expanded += 1
        if current == goal:
            break

        for nxt in neighbors4(instance, current):
            candidate = cur_cost + _step_cost(instance, nxt)
            if candidate < dist.get(nxt, inf):
                dist[nxt] = candidate
                came_from[nxt] = current
                counter += 1
                heapq.heappush(frontier, (candidate, counter, nxt))

    path = _reconstruct(came_from, start, goal)
    runtime_ms = (time.perf_counter() - t0) * 1000
    return SearchResult(path=path, cost=dist.get(goal, inf), expanded=expanded, runtime_ms=runtime_ms)


def a_star(instance: MissionInstance, start: Position, goal: Position) -> SearchResult:
    """A* con heurística de Manhattan (admisible y consistente)."""

    t0 = time.perf_counter()
    frontier: List[Tuple[float, float, int, Position]] = [(float(manhattan(start, goal)), 0.0, 0, start)]
    g_cost: Dict[Position, float] = {start: 0.0}
    came_from: Dict[Position, Position] = {}
    settled: set[Position] = set()
    expanded = 0
    counter = 0

    while frontier:
        _, gcost, _, current = heapq.heappop(frontier)
        if current in settled:
            continue
        settled.add(current)
        expanded += 1
        if current == goal:
            break

        for nxt in neighbors4(instance, current):
            tentative = gcost + _step_cost(instance, nxt)
            if tentative < g_cost.get(nxt, inf):
                g_cost[nxt] = tentative
                came_from[nxt] = current
                counter += 1
                f = tentative + manhattan(nxt, goal)
                heapq.heappush(frontier, (f, tentative, counter, nxt))

    path = _reconstruct(came_from, start, goal)
    runtime_ms = (time.perf_counter() - t0) * 1000
    return SearchResult(path=path, cost=g_cost.get(goal, inf), expanded=expanded, runtime_ms=runtime_ms)


def greedy_best_first(instance: MissionInstance, start: Position, goal: Position) -> SearchResult:
    """Búsqueda voraz best-first guiada solo por la heurística de Manhattan."""

    t0 = time.perf_counter()
    frontier: List[Tuple[float, int, Position]] = [(float(manhattan(start, goal)), 0, start)]
    visited: set[Position] = {start}
    came_from: Dict[Position, Position] = {}
    g_cost: Dict[Position, float] = {start: 0.0}
    expanded = 0
    counter = 0

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current == goal:
            break
        expanded += 1

        for nxt in neighbors4(instance, current):
            if nxt in visited:
                continue
            visited.add(nxt)
            came_from[nxt] = current
            g_cost[nxt] = g_cost[current] + _step_cost(instance, nxt)
            counter += 1
            heapq.heappush(frontier, (float(manhattan(nxt, goal)), counter, nxt))

    path = _reconstruct(came_from, start, goal)
    runtime_ms = (time.perf_counter() - t0) * 1000
    return SearchResult(path=path, cost=g_cost.get(goal, inf), expanded=expanded, runtime_ms=runtime_ms)


# Registro de algoritmos disponibles: facilita comparar y extender el módulo.
SEARCH_ALGORITHMS = {
    "bfs": breadth_first_search,
    "dfs": depth_first_search,
    "dijkstra": dijkstra,
    "a_star": a_star,
    "greedy": greedy_best_first,
}
