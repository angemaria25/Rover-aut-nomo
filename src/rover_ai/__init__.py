"""Rover autónomo - Proyecto 1 (Elementos de Inteligencia Artificial).

Módulos:
    environment    - generación parametrizable del entorno por semilla.
    search         - algoritmos de búsqueda (BFS, DFS, Dijkstra, A*, voraz).
    planning       - planificación simbólica (acciones, precondiciones, efectos).
    metaheuristics - optimización del orden de visita de los POI.
    simulation     - ejecución y validación de planes + puntuación de misión.
    rules          - sistema basado en reglas (módulo opcional bonificable).
    experiments    - protocolo experimental y métricas.
"""

__all__ = [
    "environment",
    "search",
    "planning",
    "metaheuristics",
    "simulation",
    "rules",
    "experiments",
]
