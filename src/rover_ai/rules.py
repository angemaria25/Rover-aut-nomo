"""Módulo E (opcional, bonificable): sistema basado en reglas.

Implementa un pequeño **motor de encadenamiento hacia adelante** (forward
chaining) sobre hechos y reglas, y con él define la política de decisión del
rover: cuándo recargar, cuándo priorizar la transmisión, cuándo analizar y
cómo elegir el siguiente objetivo. El motor deriva hechos nuevos (los deseos
``want``) hasta alcanzar un punto fijo; después se elige, por prioridad, el
deseo que se traduce en una acción concreta del planificador.

Es deliberadamente simple: el objetivo de la bonificación es mostrar que se
entiende el mecanismo de reglas (hechos, premisas, conclusiones, conflicto y
resolución), no construir un sistema experto completo. La política se apoya en
los costos reales de ruta (módulo de búsqueda) para garantizar que el rover
siempre pueda volver a la base.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Set, Tuple

from .models import Action, MissionExecution, MissionInstance
from .planning import PlanningArtifacts, build_key_graph, charge_keys, is_charging_key
from .simulation import execute_plan

Fact = Tuple
Facts = Set[Fact]


@dataclass(frozen=True)
class Rule:
    """Regla de producción: si se cumple ``condition``, se concluye ``causes``.

    En un motor de reglas clásico las premisas son patrones; aquí se usa una
    función ``condition`` sobre el conjunto de hechos y una función
    ``causes`` que devuelve hechos nuevos. Es la misma idea (premisas ->
    conclusiones) adaptada a Python.
    """

    name: str
    condition: Callable[[Facts], bool]
    causes: Callable[[Facts], List[Fact]]


@dataclass
class RuleState:
    """Estado mutable que la política de reglas va actualizando."""

    at: str = "BASE"
    battery: int = 0
    memory_used: int = 0
    collected: Set[str] = field(default_factory=set)
    analyzed: Set[str] = field(default_factory=set)
    transmitted: Set[str] = field(default_factory=set)


def forward_chain(facts: Facts, rules: List[Rule], max_iterations: int = 100) -> Tuple[Facts, List[Tuple[str, Tuple[Fact, ...]]]]:
    """Encadenamiento hacia adelante hasta el punto fijo.

    Devuelve los hechos finales y la traza de reglas disparadas (para poder
    mostrar el razonamiento).
    """

    facts = set(facts)
    trace: List[Tuple[str, Tuple[Fact, ...]]] = []
    changed = True
    iterations = 0
    while changed and iterations < max_iterations:
        changed = False
        iterations += 1
        for rule in rules:
            if not rule.condition(facts):
                continue
            new_facts = [f for f in rule.causes(facts) if f not in facts]
            if new_facts:
                facts.update(new_facts)
                trace.append((rule.name, tuple(new_facts)))
                changed = True
    return facts, trace


def _cost(artifacts: PlanningArtifacts, a: str, b: str) -> float:
    return artifacts.move_costs.get((a, b), float("inf"))


def build_facts(instance: MissionInstance, state: RuleState, artifacts: PlanningArtifacts) -> Facts:
    """Traduce el estado del rover a hechos del sistema de reglas."""

    facts: Facts = {
        ("at", state.at),
        ("battery", state.battery),
        ("battery_capacity", instance.battery_capacity),
        ("memory_used", state.memory_used),
        ("memory_capacity", instance.memory_capacity),
    }
    facts.add(("at_base",) if state.at == "BASE" else ("at_other",))
    if is_charging_key(state.at):
        facts.add(("at_charge",))
        facts.add(("charge_location", state.at))
    if state.at.startswith("COMM_"):
        facts.add(("at_comm",))
        facts.add(("comm_location", state.at))

    for poi in instance.pois:
        pid = poi.poi_id
        if state.at == pid:
            facts.add(("at_poi", pid))
        if pid in state.transmitted:
            facts.add(("transmitted", pid))
        elif pid in state.analyzed:
            facts.add(("pending_transmit", pid))
        if pid in state.analyzed:
            facts.add(("analyzed", pid))
        elif pid in state.collected:
            facts.add(("collected_not_analyzed", pid))
        if pid in state.collected:
            facts.add(("collected", pid))
        else:
            facts.add(("uncollected", pid))

    if state.memory_used < instance.memory_capacity:
        facts.add(("free_memory",))
    if len(state.transmitted) == len(instance.pois):
        facts.add(("mission_complete",))

    # Margen de seguridad: costo de llegar al punto de recarga más cercano.
    facts.add(("cost_to_charge", min(_cost(artifacts, state.at, c) for c in charge_keys(artifacts))))
    return facts


def make_rules(instance: MissionInstance, artifacts: PlanningArtifacts) -> List[Rule]:
    """Construye las reglas de la política del rover (cerrando sobre la instancia).

    Las reglas de movimiento exigen una reserva de batería que garantiza que el
    rover siempre puede volver a la base y terminar las acciones locales del
    destino. Esa reserva es lo que evita que la política se quede sin energía.
    """

    comm_keys = sorted({k for pair in artifacts.move_costs for k in pair if k.startswith("COMM_")})
    charges = charge_keys(artifacts)

    def uncollected(facts: Facts) -> List[str]:
        return [f[1] for f in facts if f[0] == "uncollected"]

    def pending(facts: Facts) -> List[str]:
        return [f[1] for f in facts if f[0] == "pending_transmit"]

    def at(facts: Facts) -> str:
        return next(f[1] for f in facts if f[0] == "at")

    def battery(facts: Facts) -> int:
        return next(f[1] for f in facts if f[0] == "battery")

    def reserve_for(target: str) -> float:
        """Batería que hay que reservar tras llegar a ``target`` para poder volver.

        En un POI hay que recolectar y analizar antes de irse; en una zona de
        comunicación, transmitir. Después, en ambos casos, hay que poder llegar
        al punto de recarga más cercano (la base o una estación de carga).
        """

        if target.startswith("COMM_"):
            action_cost = instance.transmit_cost
        else:
            action_cost = instance.collect_cost + instance.analyze_cost
        retreat = min(_cost(artifacts, target, c) for c in charges)
        return action_cost + retreat

    def feasible(facts: Facts, target: str) -> bool:
        """¿Alcanza la batería para ir a ``target``, actuar allí y volver a la base?"""

        if target == at(facts):
            return False
        go = _cost(artifacts, at(facts), target)
        reserve = reserve_for(target)
        if go == float("inf") or reserve == float("inf"):
            return False
        return battery(facts) - go >= reserve

    def nearest(facts: Facts, candidates: List[str]) -> Optional[str]:
        candidates = [c for c in candidates if feasible(facts, c)]
        if not candidates:
            return None
        return min(candidates, key=lambda c: _cost(artifacts, at(facts), c))

    rules: List[Rule] = []

    # Recarga: en un punto de recarga con batería incompleta y trabajo pendiente.
    def recharge_cond(facts: Facts) -> bool:
        return ("at_charge",) in facts and battery(facts) < instance.battery_capacity and ("mission_complete",) not in facts

    rules.append(Rule("politica_recarga", recharge_cond, lambda f: [("want", "recharge")]))

    # Transmitir: en zona de comunicación con datos analizados sin transmitir.
    def transmit_cond(facts: Facts) -> bool:
        return ("at_comm",) in facts and len(pending(facts)) > 0

    def transmit_causes(facts: Facts) -> List[Fact]:
        return [("want", "transmit", pid) for pid in pending(facts)]

    rules.append(Rule("transmitir_datos", transmit_cond, transmit_causes))

    # Analizar: en un POI recolectado y aún sin analizar.
    def analyze_cond(facts: Facts) -> bool:
        return any(f[0] == "collected_not_analyzed" for f in facts) and any(f[0] == "at_poi" for f in facts)

    def analyze_causes(facts: Facts) -> List[Fact]:
        poi = next(f[1] for f in facts if f[0] == "at_poi")
        if ("collected_not_analyzed", poi) in facts:
            return [("want", "analyze", poi)]
        return []

    rules.append(Rule("analizar_muestra", analyze_cond, analyze_causes))

    # Recolectar: en un POI sin recolectar y con memoria libre.
    def collect_cond(facts: Facts) -> bool:
        return ("free_memory",) in facts and any(f[0] == "at_poi" and ("uncollected", f[1]) in facts for f in facts)

    def collect_causes(facts: Facts) -> List[Fact]:
        poi = next(f[1] for f in facts if f[0] == "at_poi")
        if ("uncollected", poi) in facts:
            return [("want", "collect", poi)]
        return []

    rules.append(Rule("recolectar_muestra", collect_cond, collect_causes))

    # Ir a transmitir: hay datos pendientes (o memoria llena) y no se está en zona de comunicación.
    def goto_comm_cond(facts: Facts) -> bool:
        return ("at_comm",) not in facts and (len(pending(facts)) > 0 or ("free_memory",) not in facts)

    def goto_comm_causes(facts: Facts) -> List[Fact]:
        target = nearest(facts, comm_keys)
        return [("want", "goto_comm", target)] if target else []

    rules.append(Rule("ir_a_transmitir", goto_comm_cond, goto_comm_causes))

    # Priorizar objetivo: ir al POI sin recolectar más cercano (con memoria libre).
    def goto_poi_cond(facts: Facts) -> bool:
        return ("free_memory",) in facts and len(uncollected(facts)) > 0

    def goto_poi_causes(facts: Facts) -> List[Fact]:
        target = nearest(facts, uncollected(facts))
        return [("want", "goto_poi", target)] if target else []

    rules.append(Rule("priorizar_objetivo", goto_poi_cond, goto_poi_causes))

    # Planificar la recarga: si queda trabajo pero ningún destino es factible,
    # hay que ir al punto de recarga más cercano (la reserva garantiza llegar).
    def return_cond(facts: Facts) -> bool:
        if ("at_charge",) in facts or ("mission_complete",) in facts:
            return False
        targets = uncollected(facts)
        if ("at_comm",) not in facts and (len(pending(facts)) > 0 or ("free_memory",) not in facts):
            targets = targets + comm_keys
        if not targets:
            return False
        return not any(feasible(facts, t) for t in targets)

    def return_causes(facts: Facts) -> List[Fact]:
        target = min(charges, key=lambda c: _cost(artifacts, at(facts), c))
        return [("want", "goto_charge", target)]

    rules.append(Rule("volver_a_recargar", return_cond, return_causes))

    # Arranque forzoso: en un punto de recarga con la batería llena, si ningún
    # destino pasa el filtro de reserva, se sale igualmente hacia el más cercano
    # (es el mejor intento posible desde el máximo de energía).
    def force_cond(facts: Facts) -> bool:
        if ("at_charge",) not in facts or ("mission_complete",) in facts:
            return False
        if battery(facts) < instance.battery_capacity:
            return False
        targets = uncollected(facts) + comm_keys
        return bool(targets) and not any(feasible(facts, t) for t in targets)

    def force_causes(facts: Facts) -> List[Fact]:
        here = at(facts)
        candidates = [c for c in uncollected(facts) if _cost(artifacts, here, c) != float("inf")]
        kind = "goto_poi"
        if not candidates:
            candidates = [c for c in comm_keys if _cost(artifacts, here, c) != float("inf")]
            kind = "goto_comm"
        if not candidates:
            return []
        target = min(candidates, key=lambda c: _cost(artifacts, here, c))
        return [("want", kind, target)]

    rules.append(Rule("salida_forzosa_base", force_cond, force_causes))

    return rules


# Orden de prioridad para resolver conflictos entre deseos derivados.
_WANT_PRIORITY: List[str] = [
    "recharge",
    "transmit",
    "analyze",
    "collect",
    "goto_charge",
    "goto_comm",
    "goto_poi",
]


def decide_action(instance: MissionInstance, state: RuleState, artifacts: PlanningArtifacts) -> Optional[Action]:
    """Ejecuta el motor de reglas y devuelve la acción de mayor prioridad."""

    facts = build_facts(instance, state, artifacts)
    facts, _trace = forward_chain(facts, make_rules(instance, artifacts))

    wants = sorted(
        (f for f in facts if f[0] == "want"),
        key=lambda f: _WANT_PRIORITY.index(f[1]),
    )
    for want in wants:
        kind = want[1]
        if kind == "recharge":
            return Action("recharge", None)
        if kind == "goto_charge":
            return Action("move", want[2])
        if kind == "transmit":
            return Action("transmit", want[2])
        if kind == "analyze":
            return Action("analyze", want[2])
        if kind == "collect":
            return Action("collect", want[2])
        if kind == "goto_comm":
            return Action("move", want[2])
        if kind == "goto_poi":
            return Action("move", want[2])
    return None


def _apply(instance: MissionInstance, state: RuleState, action: Action, artifacts: PlanningArtifacts) -> None:
    """Aplica una acción al estado (para que el motor decida la siguiente)."""

    if action.name == "move" and action.arg:
        state.battery -= int(_cost(artifacts, state.at, action.arg))
        state.at = action.arg
    elif action.name == "collect" and action.arg:
        state.battery -= instance.collect_cost
        state.memory_used += 1
        state.collected.add(action.arg)
    elif action.name == "analyze" and action.arg:
        state.battery -= instance.analyze_cost
        state.analyzed.add(action.arg)
    elif action.name == "transmit" and action.arg:
        state.battery -= instance.transmit_cost
        state.memory_used = max(0, state.memory_used - 1)
        state.transmitted.add(action.arg)
    elif action.name == "recharge":
        state.battery = instance.battery_capacity


def run_rule_agent(
    instance: MissionInstance,
    artifacts: Optional[PlanningArtifacts] = None,
    max_steps: int = 500,
    trace: bool = False,
) -> Tuple[List[Action], MissionExecution]:
    """Genera un plan con la política de reglas y lo valida con el simulador."""

    artifacts = artifacts or build_key_graph(instance)
    state = RuleState(at="BASE", battery=instance.battery_capacity)
    plan: List[Action] = []
    seen_signatures: Set[Tuple] = set()

    for _ in range(max_steps):
        signature = (state.at, state.battery, len(state.collected), len(state.analyzed), len(state.transmitted))
        if signature in seen_signatures:
            break  # no hay progreso: se detiene para no iterar sin avanzar
        seen_signatures.add(signature)

        action = decide_action(instance, state, artifacts)
        if action is None:
            break
        plan.append(action)
        if trace:
            print(f"  {action.name}:{action.arg}")
        _apply(instance, state, action, artifacts)

    execution = execute_plan(instance, plan, artifacts)
    return plan, execution
