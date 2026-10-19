"""Generación parametrizable del entorno por semilla.

El curso exige un generador de instancias por semilla (no un mapa fijo) porque
el protocolo experimental compara algoritmos sobre varias instancias. Este
módulo produce entornos reproducibles y **parametrizables** en tres sentidos:

* **tamaño** (``width``, ``height``), número de POI y de zonas de comunicación;
* **estructura**: la mezcla de terrenos se elige con ``terrain_profile`` y la
  densidad de obstáculos con ``obstacle_prob``;
* **infraestructura**: número de estaciones de carga (``n_charging_stations``).

Además garantiza que la instancia **tenga solución**: primero asegura que base,
POI, zonas de comunicación y estaciones de carga estén conectadas, y después
que todas sean alcanzables dentro de la capacidad de la batería (si no, habría
misiones imposibles que romperían las estadísticas).
"""

from __future__ import annotations

import heapq
import random
from collections import deque
from dataclasses import dataclass
from math import inf
from typing import Dict, Iterable, List, Set, Tuple

from .models import TERRAIN_COST, MissionInstance, Poi, Position, Terrain

# Perfiles de estructura del terreno: distribuciones de probabilidad sobre los
# terrenos transitables. Permiten comparar los algoritmos en "sistemas" (mapas)
# distintos sin cambiar el tamaño.
TERRAIN_PROFILES: Dict[str, Tuple[Tuple[Terrain, float], ...]] = {
    "flat": (
        (Terrain.PLAIN, 0.70),
        (Terrain.STONE, 0.15),
        (Terrain.ROCK, 0.05),
        (Terrain.FINE_SAND, 0.07),
        (Terrain.DUNE, 0.03),
    ),
    "default": (
        (Terrain.PLAIN, 0.40),
        (Terrain.STONE, 0.20),
        (Terrain.ROCK, 0.14),
        (Terrain.FINE_SAND, 0.16),
        (Terrain.DUNE, 0.10),
    ),
    "rough": (
        (Terrain.PLAIN, 0.20),
        (Terrain.STONE, 0.20),
        (Terrain.ROCK, 0.22),
        (Terrain.FINE_SAND, 0.18),
        (Terrain.DUNE, 0.20),
    ),
}


@dataclass(frozen=True)
class EnvironmentConfig:
    """Parámetros del generador de entornos (tamaño, estructura e infraestructura).

    ``terrain_profile`` elige la mezcla de terrenos (véase ``TERRAIN_PROFILES``),
    ``obstacle_prob`` controla la densidad de obstáculos y
    ``n_charging_stations`` añade estaciones de carga además de la base.
    """

    width: int = 12
    height: int = 12
    n_pois: int = 4
    n_comm_zones: int = 2
    n_charging_stations: int = 0
    obstacle_prob: float = 0.12
    terrain_profile: str = "default"


def _random_terrain(rng: random.Random, weights: Tuple[Tuple[Terrain, float], ...]) -> Terrain:
    """Muestrea un terreno transitable con los pesos del perfil elegido."""

    value = rng.random()
    acc = 0.0
    for terrain, prob in weights:
        acc += prob
        if value <= acc:
            return terrain
    return Terrain.PLAIN


def _sample_positions(rng: random.Random, width: int, height: int, k: int, forbidden: Set[Position]) -> List[Position]:
    """Muestrea ``k`` posiciones distintas evitando las prohibidas.

    Si ``k <= 0`` se devuelve vacío sin tocar el generador aleatorio, de modo que
    añadir parámetros que no se usan (p. ej. ``n_charging_stations=0``) no cambie
    las instancias generadas.
    """

    if k <= 0:
        return []
    candidates = [(x, y) for x in range(width) for y in range(height) if (x, y) not in forbidden]
    rng.shuffle(candidates)
    return candidates[:k]


def _reachable(grid: List[List[Terrain]], start: Position, width: int, height: int) -> Set[Position]:
    """Conjunto de celdas alcanzables desde ``start`` (movimiento en 4 direcciones)."""

    seen = {start}
    queue: deque[Position] = deque([start])
    while queue:
        x, y = queue.popleft()
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if (nx, ny) in seen or grid[ny][nx] == Terrain.OBSTACLE:
                continue
            seen.add((nx, ny))
            queue.append((nx, ny))
    return seen


def _ensure_connected(grid: List[List[Terrain]], base: Position, keys: Iterable[Position], width: int, height: int) -> None:
    """Garantiza que todas las celdas clave sean alcanzables desde la base.

    Si alguna quedó encerrada por obstáculos, se cava un corredor en L hasta la
    base convirtiendo los obstáculos del camino en terreno llano. La operación
    es determinista, así que no rompe la reproducibilidad por semilla.
    """

    for key in keys:
        if key in _reachable(grid, base, width, height):
            continue
        bx, by = base
        kx, ky = key
        # Tramo horizontal y luego vertical desde la base hasta la celda clave.
        for x in range(min(bx, kx), max(bx, kx) + 1):
            if grid[by][x] == Terrain.OBSTACLE:
                grid[by][x] = Terrain.PLAIN
        for y in range(min(by, ky), max(by, ky) + 1):
            if grid[y][kx] == Terrain.OBSTACLE:
                grid[y][kx] = Terrain.PLAIN


def _grid_cost(grid: List[List[Terrain]], start: Position, goal: Position, width: int, height: int) -> float:
    """Costo mínimo (según el terreno) de ir de ``start`` a ``goal`` en la rejilla."""

    if start == goal:
        return 0.0
    dist: Dict[Position, float] = {start: 0.0}
    heap: List[Tuple[float, Position]] = [(0.0, start)]
    while heap:
        cost, (x, y) = heapq.heappop(heap)
        if cost > dist.get((x, y), inf):
            continue
        if (x, y) == goal:
            return cost
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if not (0 <= nx < width and 0 <= ny < height) or grid[ny][nx] == Terrain.OBSTACLE:
                continue
            new_cost = cost + TERRAIN_COST[grid[ny][nx]]
            if new_cost < dist.get((nx, ny), inf):
                dist[(nx, ny)] = new_cost
                heapq.heappush(heap, (new_cost, (nx, ny)))
    return inf


def _carve_plain(grid: List[List[Terrain]], start: Position, goal: Position) -> None:
    """Abre un corredor en L de terreno llano entre dos celdas (en su lugar)."""

    sx, sy = start
    gx, gy = goal
    for x in range(min(sx, gx), max(sx, gx) + 1):
        grid[sy][x] = Terrain.PLAIN
    for y in range(min(sy, gy), max(sy, gy) + 1):
        grid[y][gx] = Terrain.PLAIN


def _ensure_battery_feasible(
    grid: List[List[Terrain]],
    base: Position,
    charge_points: Iterable[Position],
    keys: Iterable[Position],
    width: int,
    height: int,
    capacity: int,
) -> None:
    """Garantiza que **toda** celda clave sea alcanzable dentro de la batería.

    Se parte de los puntos de recarga (la base y las estaciones de carga). Si
    una celda clave queda a más de ``capacity`` de costo, se abre un corredor
    llano hasta el punto de recarga más cercano. Así ningún tablero generado es
    irresoluble por falta de batería.
    """

    sources = [base] + [c for c in charge_points if c != base]

    # 1) Cada estación de carga debe ser alcanzable desde la base.
    for station in sources[1:]:
        if _grid_cost(grid, base, station, width, height) > capacity:
            _carve_plain(grid, base, station)

    # 2) Cada celda clave debe ser alcanzable desde algún punto de recarga.
    for key in keys:
        cost_from = [(s, _grid_cost(grid, s, key, width, height)) for s in sources]
        best_source, best_cost = min(cost_from, key=lambda item: item[1])
        if best_cost > capacity:
            _carve_plain(grid, best_source, key)


def generate_instance(seed: int, config: EnvironmentConfig | None = None) -> MissionInstance:
    """Genera una instancia de misión reproducible a partir de una semilla.

    El mismo ``seed`` y la misma ``config`` producen siempre la misma
    instancia (requisito de reproducibilidad del curso).
    """

    config = config or EnvironmentConfig()
    rng = random.Random(seed)
    weights = TERRAIN_PROFILES[config.terrain_profile]
    battery_capacity = 100

    base = (0, 0)
    grid: List[List[Terrain]] = [[Terrain.PLAIN for _ in range(config.width)] for _ in range(config.height)]

    for y in range(config.height):
        for x in range(config.width):
            if (x, y) == base:
                continue
            if rng.random() < config.obstacle_prob:
                grid[y][x] = Terrain.OBSTACLE
            else:
                grid[y][x] = _random_terrain(rng, weights)

    forbidden = {base}
    comm_positions = _sample_positions(rng, config.width, config.height, config.n_comm_zones, forbidden)
    forbidden.update(comm_positions)
    charge_positions = _sample_positions(rng, config.width, config.height, config.n_charging_stations, forbidden)
    forbidden.update(charge_positions)
    poi_positions = _sample_positions(rng, config.width, config.height, config.n_pois, forbidden)

    for x, y in comm_positions + charge_positions + poi_positions:
        grid[y][x] = Terrain.PLAIN

    keys = comm_positions + charge_positions + poi_positions

    # Base, zonas de comunicación, estaciones de carga y POI deben ser
    # alcanzables y estar dentro del alcance de la batería (el corredor no toca
    # las celdas clave porque ya son llanas).
    _ensure_connected(grid, base, keys, config.width, config.height)
    _ensure_battery_feasible(
        grid, base, charge_positions, keys, config.width, config.height, battery_capacity
    )

    pois = [Poi(f"POI_{i+1}", pos) for i, pos in enumerate(poi_positions)]

    return MissionInstance(
        width=config.width,
        height=config.height,
        grid=grid,
        base=base,
        comm_zones=set(comm_positions),
        pois=pois,
        charging_stations={base, *charge_positions},
        battery_capacity=battery_capacity,
    )


def in_bounds(instance: MissionInstance, position: Position) -> bool:
    """Indica si una posición está dentro de la rejilla."""

    x, y = position
    return 0 <= x < instance.width and 0 <= y < instance.height


def neighbors4(instance: MissionInstance, position: Position) -> Iterable[Position]:
    """Vecinos transitables en 4 direcciones (excluye obstáculos y bordes)."""

    x, y = position
    for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
        np = (nx, ny)
        if in_bounds(instance, np) and instance.grid[ny][nx] != Terrain.OBSTACLE:
            yield np
