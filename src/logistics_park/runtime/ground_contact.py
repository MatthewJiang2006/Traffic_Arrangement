"""Classify observed ramp support without suppressing obstacle collisions."""

import math

ROAD_SURFACE_PREFIXES = ("SM_Road_", "SM_Floor_")


def supported_ramp_contact(row: dict, ramps: set[str], clearance_m: float | None = .12) -> bool:
    collision = row.get("collision")
    # AirSim may omit the actor name for a compound wheel contact. Infer only
    # vertical support on approved buffer meshes, never an unnamed side impact.
    anonymous_buffer_support = bool(
        clearance_m is None and collision == ""
        and len(row.get("collision_normal", [])) == 3
        and row["collision_normal"][2] < -.98
        and 0 <= row.get("collision_penetration_m", float("inf")) <= .01
        and len(row.get("wheel_surfaces", [])) == 6
        and all(s in ramps and s.startswith("Traffic") for s in row["wheel_surfaces"])
        and len(row.get("wheel_clearances_m", [])) == 6
        and all(c is not None and abs(c) <= .12 for c in row["wheel_clearances_m"]))
    known_ground = collision in ramps or anonymous_buffer_support
    if (not known_ground and collision and collision.startswith(ROAD_SURFACE_PREFIXES)
            and row.get("collision_penetration_m", float("inf")) <= .02):
        # A road or floor mesh under the chassis counts as ground only while every
        # wheel is demonstrably standing on a known surface; a bare road-mesh
        # impact, or a raised road edge, still never qualifies.
        known_ground = bool(row.get("wheel_surfaces")) and all(s in ramps for s in row["wheel_surfaces"])
    if not known_ground:
        return False
    normal = row.get("collision_normal", [])
    wheels = row.get("wheel_clearances_m", [])
    surfaces = row.get("wheel_surfaces", [])
    hits = row.get("body_hits", [])
    # Project acceptance permits tiny tire contact at a bevel adjoining an
    # approved curb buffer, while strict wheel-clearance acceptance is unchanged.
    # Require all six wheels supported, an observed buffer under the tires,
    # downward support within 35 degrees (the same seam bound as road tiles),
    # and at most 1 mm penetration.
    shallow_buffer_support = bool(
        clearance_m is None and collision and collision.startswith(ROAD_SURFACE_PREFIXES)
        and len(normal) == 3 and normal[2] < -math.cos(math.radians(35))
        and -.01 < row.get("collision_penetration_m", float("inf")) <= .001
        and len(wheels) == 6 and len(surfaces) == 6
        and all(s in ramps for s in surfaces)
        and any(s and s.startswith("Traffic") for s in surfaces)
        and all(c is not None and abs(c) <= .12 for c in wheels))
    # Project runs audit chassis bodies independently. A tiny tire contact
    # between two observed road tiles is support, even on their sloped seam;
    # retain strict wheel acceptance and reject lateral/deep mesh impacts.
    road_seam_support = bool(
        clearance_m is None and collision and collision.startswith(ROAD_SURFACE_PREFIXES)
        and len(normal) == 3 and normal[2] < -math.cos(math.radians(35))
        and 0 <= row.get("collision_penetration_m", float("inf")) <= .005
        and len(wheels) == 6 and len(surfaces) == 6
        and len(set(surfaces)) >= 2 and collision in surfaces
        and all(s in ramps and s.startswith(ROAD_SURFACE_PREFIXES) for s in surfaces)
        and all(c is not None and abs(c) <= .16 for c in wheels))
    return bool(
        len(normal) == 3 and (normal[2] < -.9 or shallow_buffer_support or road_seam_support)
        and row.get("collision_penetration_m", float("inf")) <= (.05 if clearance_m is None else .02)
        and len(wheels) == 6 and len(surfaces) == 6 and len(hits) == 3
        and (anonymous_buffer_support or collision in surfaces or
             (clearance_m is None and all(s in ramps for s in surfaces))) and not any(hits)
        and not any(row.get("parked_vehicle_overlaps", []))
        and (all(c is not None and abs(c) <= clearance_m for c in wheels)
             if clearance_m is not None else
             any((surface == collision or all(s in ramps for s in surfaces))
                 and c is not None and abs(c) <= .12
                 for surface,c in zip(surfaces,wheels))))
