"""Center-based road occupancy plus future entrance commitments."""
from collections import defaultdict

from logistics_park.common.messages import RoadState, VehicleState
from logistics_park.map.road_graph import RoadGraph


class RoadCapacity:
    def __init__(self, graph: RoadGraph):
        self.capacities = {key: edge.storage_capacity for key,edge in graph.edges.items()}
        self.occupants: dict[str,set[str]] = defaultdict(set)
        self.committed: dict[str,set[str]] = defaultdict(set)
        self.max_effective: dict[str,int] = defaultdict(int)
        self.denied_attempts = 0

    def update(self, states: dict[str,VehicleState]) -> None:
        occupied: dict[str,set[str]] = defaultdict(set)
        for state in states.values():
            if state.edge_id is not None:
                if state.edge_id not in self.capacities:
                    raise ValueError('unknown occupied road')
                occupied[state.edge_id].add(state.vehicle_id)
        self.occupants = occupied
        # Conversion is atomic: a commitment becomes current occupancy.
        for edge_id,vehicles in self.committed.items():
            vehicles.difference_update(occupied[edge_id])
        self._observe()

    def road_state(self, edge_id: str) -> RoadState:
        return RoadState(edge_id,self.capacities[edge_id],len(self.occupants[edge_id]),len(self.committed[edge_id]))

    def reserve(self, vehicle_id: str, edge_id: str) -> bool:
        if not vehicle_id or edge_id not in self.capacities:
            raise ValueError('invalid vehicle or road commitment')
        if vehicle_id in self.occupants[edge_id] or vehicle_id in self.committed[edge_id]:
            return True
        if self.road_state(edge_id).is_saturated:
            self.denied_attempts += 1
            return False
        self.committed[edge_id].add(vehicle_id)
        self._observe()
        return True

    def release_commitment(self, vehicle_id: str, edge_id: str) -> None:
        self.committed[edge_id].discard(vehicle_id)

    def _observe(self) -> None:
        for edge_id in self.capacities:
            effective=len(self.occupants[edge_id])+len(self.committed[edge_id])
            self.max_effective[edge_id]=max(self.max_effective[edge_id],effective)

    @property
    def violations(self) -> int:
        return sum(len(self.occupants[key])+len(self.committed[key]) > capacity
                   for key,capacity in self.capacities.items())
