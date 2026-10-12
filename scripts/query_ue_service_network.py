#!/usr/bin/env python3
"""Inspect directed service routes and queue constraints without controlling UE."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from logistics_park.routing.service_network import ServiceNetwork


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('origin', help='e.g. W5-loading or Q2')
    parser.add_argument('destination', help='e.g. W6-unloading')
    parser.add_argument('--include-pending', action='store_true', help='Include explicitly unvalidated access candidates')
    parser.add_argument('--occupied', nargs='*', default=[], help='Occupied service-node IDs')
    args = parser.parse_args()
    network = ServiceNetwork(ROOT / 'config/topology/industrialarea_ue_service_network.json')
    unknown = set([args.origin, args.destination, *args.occupied]) - set(network.services)
    if unknown:
        parser.error('unknown nodes: ' + ', '.join(sorted(unknown)))
    if args.origin == args.destination:
        parser.error('origin and destination must differ')
    route = network.route(args.origin, args.destination, occupied_nodes=set(args.occupied),
                          allow_pending_access=args.include_pending)
    # Queue candidates must obey the same live occupancy restrictions as the route.
    queues = network.queue_options(args.origin, args.destination, occupied_nodes=set(args.occupied),
                                   allow_pending_access=args.include_pending)
    print(json.dumps(dict(origin=args.origin, destination=args.destination,
        include_pending=args.include_pending, routing_enabled=False,
        destination_entry_ports=network.services[args.destination]['entry_road_ports'],
        route=route, waiting_options=queues), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
