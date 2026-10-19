"""Módulo D (obligatorio): metaheurísticas para el orden de visita de los POI.

La decisión que se optimiza es el **orden** en que el rover visita los puntos
de interés. La función objetivo evalúa ese orden con los costos de ruta reales
(los mismos que resuelve el módulo de búsqueda mediante A*) más los costos de
las acciones obligatorias (recolectar, analizar, transmitir por POI) y la
vuelta final a la base pasando por una zona de comunicación.

Se implementan dos metaheurísticas que comparten el mismo operador de vecindad
(intercambio de dos posiciones) y el **mismo presupuesto de cómputo** (número
de evaluaciones), requisito del protocolo experimental:

* Hill Climbing estocástico (primera mejora).
* Recocido Simulado (Simulated Annealing).
"""

from __future__ import annotations

import math
import random
from typing import Dict, List, Optional, Sequence, Tuple

from .models import MissionInstance
from .planning import PlanningArtifacts, build_key_graph

# Presupuesto por defecto: mismo número de evaluaciones para las dos algoritmos.
DEFAULT_EVALUATIONS = 2000


def evaluate_order(instance: MissionInstance, order: Sequence[str], move_costs: Dict[Tuple[str, str], float]) -> float:
    """Costo estimado de visitar los POI en el orden dado.

    La ruta se construye respetando la **capacidad de memoria**: cada vez que
    se acumulan ``memory_capacity`` muestras sin transmitir, hay que desviarse a
    una zona de comunicación antes de continuar. Al final se transmiten las que
    queden y se termina en el punto de recarga más cercano (la base o una
    estación de carga). El costo usa las distancias reales (A*) más el costo de
    las acciones obligatorias por muestra, así que está alineado con el plan que
    ejecutará el planificador.
    """

    def cost(a: str, b: str) -> float:
        return move_costs.get((a, b), math.inf)

    comms = sorted({key for pair in move_costs for key in pair if key.startswith("COMM_")})
    charges = ["BASE"] + sorted({key for pair in move_costs for key in pair if key.startswith("CHARGE_")})

    if not order:
        return 0.0

    capacity = max(1, instance.memory_capacity)
    route: List[str] = ["BASE"]
    remaining = list(order)
    while remaining:
        chunk, remaining = remaining[:capacity], remaining[capacity:]
        route.extend(chunk)
        if remaining and comms:
            last = route[-1]
            route.append(min(comms, key=lambda c: cost(last, c)))

    last = route[-1]
    if comms:
        near_comm = min(comms, key=lambda c: cost(last, c))
        route.append(near_comm)
    route.append(min(charges, key=lambda c: cost(route[-1], c)))

    total = sum(cost(route[i], route[i + 1]) for i in range(len(route) - 1))
    # Acciones obligatorias por muestra: recolectar + analizar + transmitir.
    total += len(order) * (instance.collect_cost + instance.analyze_cost + instance.transmit_cost)
    return total


def _swap_neighbor(rng: random.Random, seq: List[str]) -> List[str]:
    """Vecindad: intercambia dos posiciones aleatorias de la secuencia."""

    if len(seq) < 2:
        return seq[:]
    i, j = rng.sample(range(len(seq)), 2)
    candidate = seq[:]
    candidate[i], candidate[j] = candidate[j], candidate[i]
    return candidate


def hill_climbing_order(
    instance: MissionInstance,
    seed: int,
    max_evaluations: int = DEFAULT_EVALUATIONS,
    artifacts: Optional[PlanningArtifacts] = None,
) -> Tuple[List[str], float]:
    """Hill Climbing estocástico (acepta el vecino solo si mejora).

    Se ejecuta el **mismo número de evaluaciones** que el recocido simulado
    para que la comparación sea justa.
    """

    rng = random.Random(seed)
    artifacts = artifacts or build_key_graph(instance)
    order = [poi.poi_id for poi in instance.pois]
    rng.shuffle(order)

    best = order
    best_score = evaluate_order(instance, best, artifacts.move_costs)

    for _ in range(max_evaluations):
        candidate = _swap_neighbor(rng, best)
        score = evaluate_order(instance, candidate, artifacts.move_costs)
        if score < best_score:
            best, best_score = candidate, score

    return best, best_score


def simulated_annealing_order(
    instance: MissionInstance,
    seed: int,
    initial_temp: float = 80.0,
    cooling: float = 0.99,
    max_evaluations: int = DEFAULT_EVALUATIONS,
    artifacts: Optional[PlanningArtifacts] = None,
) -> Tuple[List[str], float]:
    """Recocido Simulado con enfriamiento geométrico.

    Acepta vecinos peores con probabilidad ``exp(-delta / T)`` para escapar de
    óptimos locales. Usa el mismo presupuesto de evaluaciones que Hill Climbing.
    """

    rng = random.Random(seed)
    artifacts = artifacts or build_key_graph(instance)
    current = [poi.poi_id for poi in instance.pois]
    rng.shuffle(current)

    current_score = evaluate_order(instance, current, artifacts.move_costs)
    best, best_score = current[:], current_score
    temp = initial_temp

    for _ in range(max_evaluations):
        candidate = _swap_neighbor(rng, current)
        candidate_score = evaluate_order(instance, candidate, artifacts.move_costs)

        if candidate_score < current_score:
            current, current_score = candidate, candidate_score
        else:
            prob = math.exp(-(candidate_score - current_score) / max(temp, 1e-8))
            if rng.random() < prob:
                current, current_score = candidate, candidate_score

        if current_score < best_score:
            best, best_score = current[:], current_score

        temp *= cooling
        if temp < 1e-4:
            break

    return best, best_score


def random_order(
    instance: MissionInstance,
    seed: int,
    artifacts: Optional[PlanningArtifacts] = None,
) -> Tuple[List[str], float]:
    """Línea base: un orden aleatorio (sin optimización)."""

    rng = random.Random(seed)
    artifacts = artifacts or build_key_graph(instance)
    order = [poi.poi_id for poi in instance.pois]
    rng.shuffle(order)
    return order, evaluate_order(instance, order, artifacts.move_costs)


METAHEURISTICS = {
    "hill_climbing": hill_climbing_order,
    "simulated_annealing": simulated_annealing_order,
}
