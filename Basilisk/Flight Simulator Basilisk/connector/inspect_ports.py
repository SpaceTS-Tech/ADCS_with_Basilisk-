"""Print the ports of the flight software and the simulator, and save them.

Phase 1 of the AOCS connector: read both sides of the binding into blocks of
ports, the same blocks the GUI editor will draw.

Examples
--------
.. code-block:: text

    python inspect_ports.py
    python inspect_ports.py --software ../../ADCS_Software_v1_grt_rtw/ADCS_Software_v1.dll
    python inspect_ports.py --json output/ports.json --internal
    python inspect_ports.py --no-simulator --parameters
"""

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from aocs_connector.simulator_block import (  # noqa: E402
    DEFAULT_ENTRY, DEFAULT_FLIGHT_SOFTWARE_TAGS, DEFAULT_SIMULATOR_DIR,
    build_simulator_block)
from aocs_connector.software_block import (  # noqa: E402
    build_software_block, check_library)

DEFAULT_SOFTWARE = HERE.parents[1] / "ADCS_Software_v1_grt_rtw"


def print_software(block, show_parameters):
    meta = block.meta
    print(f"=== Software block: {block.name}  (signature {block.signature()})")
    print(f"    library     : {meta['library']}")
    print(f"    sample time : {meta['sample_time']} s")
    print(f"    headers     : {', '.join(meta['headers'])}")
    for title, ports in (("inputs", block.inputs), ("outputs", block.outputs)):
        print(f"  {title} ({len(ports)}):")
        for port in ports:
            label = f"  '{port.description}'" \
                if port.description and port.description != port.id else ""
            print(f"    {port.id:<24} {port.type_string():<18}"
                  f"@{port.extra['offset']:<8}{label}")
    for key in ("parameters", "signals"):
        items = meta.get(key, [])
        print(f"  {key}: {len(items)}"
              + ("" if show_parameters else "  (--parameters to list)"))
        if show_parameters:
            for item in items:
                shape = f"[{','.join(map(str, item['shape']))}]" \
                    if item["shape"] else ""
                print(f"    {item['id']:<40} {item['dtype']}{shape}")
    print("  library check:")
    for level, message in check_library(block):
        print(f"    [{level.upper():<5}] {message}")


def print_simulator(block):
    meta = block.meta
    print(f"\n=== Simulator block: {block.name}  (signature {block.signature()})")
    print(f"    entry  : {meta['entry']}")
    print(f"    tasks  : " + ", ".join(f"{k} @ {v} s"
                                      for k, v in meta["tasks"].items()))
    print(f"    modules: {', '.join(meta['modules'])}")
    print(f"    excluded flight-software module(s): "
          f"{', '.join(meta['flight_software_modules']) or 'none'}")

    def by_group(ports):
        groups = {}
        for port in ports:
            groups.setdefault(port.group, []).append(port)
        return groups

    print(f"  inputs ({len(block.inputs)} fields):")
    for group, ports in by_group(block.inputs).items():
        wiring = ports[0].extra["wiring"]
        note = wiring["status"]
        if wiring["source"]:
            note += f" <- {wiring['source']}"
        print(f"    {group}   [{ports[0].extra['payload']}]  ({note})")
        for port in ports:
            print(f"        .{port.extra['field']:<28} {port.type_string()}")

    print(f"  outputs ({len(block.outputs)} fields):")
    for group, ports in by_group(block.outputs).items():
        fsw = ports[0].extra["wiring"]["read_by_flight_software"]
        note = f"  (bridge reads: {', '.join(fsw)})" if fsw else ""
        print(f"    {group}   [{ports[0].extra['payload']}]{note}")
        for port in ports:
            print(f"        .{port.extra['field']:<28} {port.type_string()}")

    print("  connections made by the current hand-written bridge:")
    for source, target in meta["bridge_connections"]:
        print(f"    {source}  ->  {target}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--software", default=str(DEFAULT_SOFTWARE),
                        help="generated-code folder or the DLL inside it")
    parser.add_argument("--simulator-dir", default=str(DEFAULT_SIMULATOR_DIR))
    parser.add_argument("--entry", default=DEFAULT_ENTRY,
                        help="module:function that builds the scenario")
    parser.add_argument("--entry-kwargs", type=json.loads, default=None,
                        help="JSON keyword arguments for the entry function")
    parser.add_argument("--flight-software-tag", action="append",
                        help="ModelTag of a bridge module to leave out "
                             "(default: the software model name)")
    parser.add_argument("--internal", action="store_true",
                        help="also list simulator inputs wired internally")
    parser.add_argument("--parameters", action="store_true",
                        help="list tunable parameters and internal signals")
    parser.add_argument("--no-simulator", action="store_true",
                        help="only read the software side")
    parser.add_argument("--json", help="write both blocks to this file")
    args = parser.parse_args(argv)

    blocks = {}
    software = build_software_block(args.software)
    blocks["software"] = software
    print_software(software, args.parameters)

    if not args.no_simulator:
        tags = tuple(args.flight_software_tag or
                     (software.name, *DEFAULT_FLIGHT_SOFTWARE_TAGS))
        simulator = build_simulator_block(
            entry=args.entry, simulator_dir=args.simulator_dir,
            kwargs=args.entry_kwargs, flight_software_tags=tags,
            include_internal=args.internal)
        blocks["simulator"] = simulator
        print_simulator(simulator)

    if args.json:
        path = Path(args.json)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(
            {key: block.to_dict() for key, block in blocks.items()},
            indent=2))
        print(f"\nwrote {path}")


if __name__ == "__main__":
    main()
