"""Tasks always name their vehicle; seeded generators never assign vehicles."""
import random

from logistics_park.common.messages import Task, finite


def fixed_tasks(items: list[dict]) -> list[Task]:
    tasks = [Task(**item) for item in items]
    if len({task.task_id for task in tasks}) != len(tasks):
        raise ValueError("duplicate generated task IDs")
    return tasks


def random_tasks(vehicle_homes: dict[str, str], destinations: tuple[str, ...],
                 trips_per_vehicle: int, seed: int, release_interval: float = 0.0) -> list[Task]:
    finite(release_interval)
    if (type(trips_per_vehicle) is not int or trips_per_vehicle < 1 or not vehicle_homes
            or not destinations or len(set(destinations)) != len(destinations) or release_interval < 0):
        raise ValueError("invalid random task scenario")
    if any(home in destinations for home in vehicle_homes.values()):
        raise ValueError("transport destinations must differ from vehicle homes")
    rng = random.Random(seed)
    tasks = []
    for vehicle, home in sorted(vehicle_homes.items()):
        for trip in range(trips_per_vehicle):
            destination = rng.choice(destinations)
            release = trip*release_interval
            prefix = f"random_{vehicle}_{trip:03d}"
            tasks.extend([Task(prefix+"_out", vehicle, home, destination, release),
                          Task(prefix+"_return", vehicle, destination, home, release)])
    return tasks


def cyclic_tasks(vehicle_homes: dict[str, str], stops: tuple[str, ...], cycles: int,
                 release_interval: float = 0.0) -> list[Task]:
    finite(release_interval)
    if (type(cycles) is not int or cycles < 1 or not vehicle_homes or not stops
            or release_interval < 0):
        raise ValueError("invalid cyclic scenario")
    tasks = []
    for vehicle, home in sorted(vehicle_homes.items()):
        itinerary = (home,) + stops + (home,)
        if any(a == b for a,b in zip(itinerary, itinerary[1:])):
            raise ValueError("cyclic itinerary has identical consecutive stops")
        for cycle in range(cycles):
            for leg,(origin,destination) in enumerate(zip(itinerary,itinerary[1:])):
                tasks.append(Task(f"cyclic_{vehicle}_{cycle:03d}_{leg:03d}", vehicle,
                                  origin, destination, cycle*release_interval))
    return tasks
