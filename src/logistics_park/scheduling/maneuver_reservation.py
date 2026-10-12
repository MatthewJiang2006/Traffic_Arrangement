"""Exclusive parking maneuvers, guarded by complete vehicle footprints."""
from logistics_park.safety.collision import OBB, overlaps


class ManeuverReservations:
    def __init__(self, zones: dict[str, OBB]):
        self.zones = dict(zones)
        self.owners: dict[str, str] = {}
        self.events: list[dict] = []

    def request(self, zone: str, vehicle: str, fleet: dict[str, OBB], t: float) -> bool:
        area = self.zones[zone]
        if vehicle not in fleet:
            raise ValueError("Reservation requires an observed vehicle footprint")
        blockers = sorted(n for n, body in fleet.items()
                          if n != vehicle and overlaps(area, body))
        conflicts = sorted(key for key in self.owners
                           if overlaps(area, self.zones[key])
                           and self.owners[key] != vehicle)
        if blockers or conflicts:
            self.events.append(dict(event="DENIED", zone=zone, vehicle=vehicle,
                                    blockers=blockers, conflicts=conflicts, t_s=t))
            return False
        self.owners[zone] = vehicle
        self.events.append(dict(event="GRANTED", zone=zone, vehicle=vehicle, t_s=t))
        return True

    def intruders(self, zone: str, fleet: dict[str, OBB]) -> list[str]:
        owner = self.owners.get(zone)
        if owner is None:
            return []
        return sorted(n for n, body in fleet.items()
                      if n != owner and overlaps(self.zones[zone], body))

    def release(self, zone: str, vehicle: str, fleet: dict[str, OBB], t: float) -> bool:
        if self.owners.get(zone) != vehicle:
            raise ValueError("Only the reservation owner can release it")
        if vehicle not in fleet:
            raise ValueError("Release requires an observed vehicle footprint")
        if overlaps(self.zones[zone], fleet[vehicle]):
            self.events.append(dict(event="TAIL_NOT_CLEAR", zone=zone,
                                    vehicle=vehicle, t_s=t))
            return False
        del self.owners[zone]
        self.events.append(dict(event="RELEASED", zone=zone, vehicle=vehicle, t_s=t))
        return True
