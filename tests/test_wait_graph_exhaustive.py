"""Exhaust finite abstract graphs, not native fleet reachability or liveness."""
import itertools
from types import SimpleNamespace

import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.maneuver_admission import prospective_cycle


def reachable(adjacency, origin):
    visited = {origin}
    pending = [origin]
    while pending:
        for target in adjacency[pending.pop()]:
            if target not in visited:
                visited.add(target)
                pending.append(target)
    return visited


@pytest.mark.parametrize('representation', ['reported', 'paused', 'clearance', 'mixed'])
def test_every_four_vehicle_wait_graph_and_every_proposed_yield_set(representation):
    names = tuple('abcd')
    pairs = list(itertools.permutations(names, 2))
    states = {name: VehicleState(name, 0., 0., 0.) for name in names}
    proposals = {name: [frozenset(other for bit, other in enumerate(n for n in names if n != name)
                                 if mask & (1 << bit)) for mask in range(8)] for name in names}
    checked = 0
    for mask in range(1 << len(pairs)):
        local = SimpleNamespace(admission_dependencies=[], paused_by={}, clearance_pauses={})
        adjacency = {name: set() for name in names}
        for index, (source, target) in enumerate(pairs):
            if not mask & (1 << index):continue
            adjacency[source].add(target)
            kind = index % 3 if representation == 'mixed' else ('reported', 'paused', 'clearance').index(representation)
            if kind == 0:local.admission_dependencies.append(dict(vehicle=source, blocked_by=target))
            elif kind == 1:local.paused_by.setdefault(source, set()).add((target, 0))
            else:local.clearance_pauses.setdefault(source, set()).add(target)
        for applicant in names:
            # Adding yielding -> applicant closes a cycle exactly when
            # applicant can already reach at least one yielding owner.
            downstream = reachable(adjacency, applicant)
            for yielding in proposals[applicant]:
                expected = bool(downstream & yielding)
                result = prospective_cycle(local, applicant, yielding, states)
                assert bool(result) == expected, (mask, applicant, yielding, representation)
                if result:assert applicant in result and yielding.intersection(result)
                checked += 1
    assert checked == 131072


@pytest.mark.parametrize('count', range(2, 17))
def test_long_wait_chains_close_only_when_the_tail_yields_to_the_head(count):
    names = [f'Car_{i}' for i in range(count)]
    states = {name: VehicleState(name, 0., 0., 0.) for name in names}
    local = SimpleNamespace(admission_dependencies=[dict(vehicle=a, blocked_by=b) for a, b in zip(names, names[1:])],
                            paused_by={}, clearance_pauses={})
    assert prospective_cycle(local, names[0], {names[-1]}, states) == sorted(names)
    assert prospective_cycle(local, names[-1], {names[0]}, states) is None
