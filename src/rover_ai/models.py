"""Modelos de datos del proyecto Rover autónomo (Elementos de IA).

Este módulo concentra las estructuras que comparten todos los demás módulos:
los tipos de terreno y sus costos de desplazamiento, los puntos de interés
(POI), la definición de una instancia de misión, el estado abstracto del
planificador y las estructuras de resultado (búsqueda y ejecución).

Separar los datos de los algoritmos es lo que permite que el entorno sea
parametrizable y que los tres módulos (búsqueda, planificación y
metaheurísticas) trabajen sobre la misma representación.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, FrozenSet, List, Set, Tuple

# Una posición es una celda (x, y) de la rejilla.
Position = Tuple[int, int]


class Terrain(str, Enum):
    """Tipos de superficie del mapa.

    El costo de entrar a una celda depende de su terreno. ``OBSTACLE`` no es
    transitable y se modela con un costo prácticamente infinito.
    """

    PLAIN = "plain"
    STONE = "stone"
    ROCK = "rock"
    FINE_SAND = "fine_sand"
    DUNE = "dune"
    OBSTACLE = "obstacle"


# Costo de entrar a una celda según su terreno. El costo se paga al pisar la
# celda destino (así lo usan búsqueda y planificación de forma consistente).
TERRAIN_COST: Dict[Terrain, int] = {
    Terrain.PLAIN: 1,
    Terrain.STONE: 2,
    Terrain.ROCK: 3,
    Terrain.FINE_SAND: 4,
    Terrain.DUNE: 5,
    Terrain.OBSTACLE: 10**9,
}


@dataclass(frozen=True)
class Poi:
    """Punto de interés científico: una muestra que el rover debe procesar."""

    poi_id: str
    position: Position


@dataclass
class MissionInstance:
    """Instancia concreta de una misión del rover.

    Además del mapa, la base y las zonas de comunicación, fija los recursos
    (batería y memoria) y los costos de las acciones elementales. Todos los
    parámetros viven aquí para que el protocolo experimental pueda variarlos.
    """

    width: int
    height: int
    grid: List[List[Terrain]]
    base: Position
    comm_zones: Set[Position]
    pois: List[Poi]
    # Puntos donde se puede recargar (incluye siempre la base). Permiten que el
    # rover no tenga que volver siempre a la base cuando el mapa es grande.
    charging_stations: Set[Position] = field(default_factory=set)
    battery_capacity: int = 100
    memory_capacity: int = 3
    collect_cost: int = 10
    analyze_cost: int = 5
    transmit_cost: int = 5
    # Recargar no consume batería extra (la energía es un recurso, no un
    # costo); el peso del modelo está en que solo se puede recargar en la base o
    # en una estación de carga.
    recharge_cost: float = 0.0
    # Recompensa por cada muestra transmitida, usada en la puntuación de misión.
    transmit_reward: float = 100.0


@dataclass(frozen=True)
class Action:
    """Acción de alto nivel del plan simbólico (nombre + argumento opcional)."""

    name: str
    arg: str | None = None


@dataclass(frozen=True)
class PlannerState:
    """Estado del planificador simbólico.

    Guarda dónde está el rover (``at`` como clave lógica: ``BASE``, ``POI_i``,
    ``COMM_j`` o ``CHARGE_j``), la batería restante, la memoria ocupada y los
    conjuntos de muestras recolectadas, analizadas y transmitidas.
    """

    at: str
    battery: int
    memory_used: int
    collected: FrozenSet[str] = field(default_factory=frozenset)
    analyzed: FrozenSet[str] = field(default_factory=frozenset)
    transmitted: FrozenSet[str] = field(default_factory=frozenset)


@dataclass
class SearchResult:
    """Resultado de un algoritmo de búsqueda entre dos celdas."""

    path: List[Position]
    cost: float
    expanded: int
    runtime_ms: float


@dataclass
class MissionExecution:
    """Resultado de ejecutar (validar) un plan completo sobre una instancia."""

    plan: List[Action]
    total_cost: float
    battery_left: int
    collected_count: int
    analyzed_count: int
    transmitted_count: int
    score: float
    success: bool


def mission_score(instance: MissionInstance, transmitted_count: int, total_cost: float) -> float:
    """Puntuación de la misión.

    La orden del proyecto pide "maximizar la puntuación de la misión". Se
    define de forma transparente y monótona: una recompensa fija por cada
    muestra transmitida menos el costo total consumido. Con la recompensa fija
    (``transmit_reward``), maximizar la puntuación equivale a minimizar el
    costo para el mismo número de muestras transmitidas.
    """

    return instance.transmit_reward * transmitted_count - total_cost
