"""Build the simulator block from a Basilisk scenario.

The scenario is **built but not run**: an entry function (by default
``adcs_sim.scenario.builder:build_simulation``) creates the simulation, then
Basilisk's own :meth:`SimBaseClass.GetMessageConnectionGraph` lists every
module's input and output messages and who is subscribed to what.  Each message
is expanded into one port per payload field, read from the payload class
itself -- so a model added to ``adcs_sim/models`` shows up here with no edit.

Which ports the block offers:

* **outputs** -- every output message field of every module.  These can feed
  software inputs.
* **inputs** -- input messages nothing in the simulator drives: unlinked ones,
  and ones currently fed by the hand-written flight-software bridge (the
  module named by ``flight_software_tags``, which the connector replaces).
  Inputs wired between two simulator modules are internal plumbing and are
  only listed with ``include_internal=True``.

Every port records how it is wired today (``extra["wiring"]``), so the
existing bridge's connections can be read straight off the block.
"""

import importlib
import sys
import warnings
from pathlib import Path

from aocs_connector.ports import Block, Port

#: The adcs_sim scenario next to this folder.
DEFAULT_SIMULATOR_DIR = Path(__file__).resolve().parents[2] / "simulator"
DEFAULT_ENTRY = "adcs_sim.scenario.builder:build_simulation"
#: ``library_path=None`` builds the simulator with no flight software, so
#: reading its ports never needs the DLL: the simulator stands on its own.
DEFAULT_ENTRY_KWARGS = {"rw_omega_rpm": [0.0, 0.0, 0.0],
                        "record_history": False,
                        "library_path": None}
#: ModelTag of the hand-written bridge module in adcs_sim.
DEFAULT_FLIGHT_SOFTWARE_TAGS = ("ADCS_Software_v1",)


def build_scenario(entry=DEFAULT_ENTRY, simulator_dir=DEFAULT_SIMULATOR_DIR,
                   kwargs=None):
    """Import ``module:function`` and call it; return ``(sim, result)``."""
    simulator_dir = str(Path(simulator_dir).resolve())
    if simulator_dir not in sys.path:
        sys.path.insert(0, simulator_dir)
    module_name, _, function_name = entry.partition(":")
    function = getattr(importlib.import_module(module_name), function_name)
    if kwargs is None:
        kwargs = DEFAULT_ENTRY_KWARGS if entry == DEFAULT_ENTRY else {}
    result = function(**kwargs)
    sim = getattr(result, "sim", result)
    if not hasattr(sim, "GetMessageConnectionGraph"):
        raise TypeError(f"{entry} returned {type(result).__name__}; expected a "
                        f"SimBaseClass or an object with a .sim attribute")
    return sim, result


# ---------------------------------------------------------------------------
# Payload fields
# ---------------------------------------------------------------------------

_SKIP_ATTRIBUTES = {"this", "thisown"}


def payload_fields(payload_type):
    """``[(field_path, dtype, shape)]`` for a Basilisk payload class name.

    Types come from a default-constructed payload: SWIG returns C ``double``
    as ``float``, integer types as ``int`` and arrays as (nested) lists.  The
    exact integer width is not exposed through SWIG, so integers are reported
    as ``int``.
    """
    from Basilisk.architecture import messaging

    payload_class = getattr(messaging, payload_type, None)
    if payload_class is None:
        return []
    return list(_fields_of(payload_class(), ""))


def _fields_of(payload, prefix):
    for name in sorted(dir(payload)):
        if name.startswith("_") or name in _SKIP_ATTRIBUTES:
            continue
        try:
            with warnings.catch_warnings(record=True) as raised:
                warnings.simplefilter("always")
                value = getattr(payload, name)
        except Exception:          # SWIG getters that need set-up data
            continue
        # Scheduled for removal: do not offer it.  Only a warning naming this
        # field counts -- unrelated ones (SWIG, imports) can fire here too.
        if any(f".{name} " in f"{w.message} " for w in raised):
            continue
        if callable(value):
            continue
        path = prefix + name
        if hasattr(value, "this"):                 # nested struct
            yield from _fields_of(value, path + ".")
            continue
        shape, leaf = _shape_of(value)
        yield path, _dtype_of(leaf), shape


def _shape_of(value):
    shape = []
    while isinstance(value, (list, tuple)):
        shape.append(len(value))
        if not value:
            return tuple(shape), None
        value = value[0]
    return tuple(shape), value


def _dtype_of(value):
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float64"
    if isinstance(value, str):
        return "char"
    return "unknown"


# ---------------------------------------------------------------------------
# Graph -> block
# ---------------------------------------------------------------------------

def _standalone_messages(root, module_ids, depth=3):
    """Python-created messages held by the scenario object, by attribute path.

    Messages written from Python (``messaging.XMsg().write(...)``), such as a
    configuration message, are not owned by any module, so the connection
    graph only sees them when they are passed in explicitly.
    """
    found, seen = {}, set()

    def visit(obj, path, level):
        if id(obj) in seen or level > depth:
            return
        seen.add(id(obj))
        type_name = type(obj).__name__
        if type_name.endswith("Msg") and hasattr(obj, "write") and \
                hasattr(obj, "read"):
            found[path] = obj
            return
        if hasattr(obj, "ModelTag") or id(obj) in module_ids:
            return                                  # modules are in the graph
        if isinstance(obj, dict):
            items = obj.items()
        elif isinstance(obj, (list, tuple)):
            items = enumerate(obj)
        elif hasattr(obj, "__dict__"):
            items = vars(obj).items()
        else:
            return
        for key, value in items:
            if isinstance(value, (str, bytes, int, float, bool)) or \
                    value is None:
                continue
            visit(value, f"{path}.{key}" if path else str(key), level + 1)

    visit(root, "", 0)
    return found


def message_graph(sim, extra_messages=None):
    """``sim.GetMessageConnectionGraph``, made deterministic.

    Basilisk's walker remembers visited objects by ``id()``.  SWIG hands out
    a fresh temporary proxy on every attribute access, so once one is freed a
    later proxy can reuse its id and be skipped as "already visited" -- in
    adcs_sim that silently dropped ``SpiceInterfaceData.planetStateOutMsgs``
    in about half of all runs.  Holding a reference to every object the
    walker visits, for the duration of the call, keeps the ids unique.
    """
    from Basilisk.utilities import SimulationBaseClass as base

    original = base._walkMessageLeaves
    alive = []

    def walk(obj, name, depth_remaining, visited):
        alive.append(obj)
        yield from original(obj, name, depth_remaining, visited)

    base._walkMessageLeaves = walk
    try:
        return sim.GetMessageConnectionGraph(
            extraMessages=extra_messages or None, includeRecorders=False)
    finally:
        base._walkMessageLeaves = original
        alive.clear()


def _module_names(sim, graph):
    """Unique, readable name per module record.

    ModelTags are not unique (adcs_sim tags the hub and the wheels both
    ``bsk-Sat``); clashing tags get the module's class name appended.
    """
    python_models = [model for task in sim.TaskList for model in task.TaskModels]
    tags = [record["tag"] for record in graph["modules"]]
    names = {}
    for record in graph["modules"]:
        name = record["tag"] or record["id"]
        if tags.count(record["tag"]) > 1:
            name = f"{record['tag']}:{_class_name(record, python_models)}"
        while name in names.values():
            name += "'"
        names[record["id"]] = name
    return names


def _class_name(record, python_models):
    endpoint_names = [e["name"] for e in record["inputs"] + record["outputs"]
                      if e["name"].isidentifier()]
    for model in python_models:
        if getattr(model, "ModelTag", None) != record["tag"]:
            continue
        if all(hasattr(model, name) for name in endpoint_names):
            return type(model).__name__
    return record["id"].split(":")[-1]


def build_simulator_block(entry=DEFAULT_ENTRY,
                          simulator_dir=DEFAULT_SIMULATOR_DIR, kwargs=None,
                          flight_software_tags=DEFAULT_FLIGHT_SOFTWARE_TAGS,
                          include_internal=False):
    """Build the scenario and describe it as a :class:`Block`."""
    sim, result = build_scenario(entry, simulator_dir, kwargs)

    module_ids = {id(m) for task in sim.TaskList for m in task.TaskModels}
    extra_messages = _standalone_messages(result, module_ids) \
        if result is not sim else {}
    graph = message_graph(sim, extra_messages)

    names = _module_names(sim, graph)
    fsw_owners = {record["id"] for record in graph["modules"]
                  if record["tag"] in flight_software_tags}
    owner_name = dict(names)
    for record in graph["standaloneMessages"]:
        owner_name[record["id"]] = "python"

    # Who feeds each input, and who reads each output.
    source_of, readers_of = {}, {}
    endpoint_by_id = {e["id"]: e for e in graph["inputs"] + graph["outputs"]}
    for edge in graph["edges"]:
        source_of[edge["target"]] = edge["source"]
        readers_of.setdefault(edge["source"], []).append(edge["target"])

    def endpoint_label(endpoint_id):
        endpoint = endpoint_by_id[endpoint_id]
        return f"{owner_name[endpoint['ownerId']]}.{endpoint['name']}"

    inputs, outputs, wrapper = [], [], []
    for endpoint in graph["outputs"]:
        if endpoint["ownerId"] in fsw_owners:
            continue
        readers = readers_of.get(endpoint["id"], [])
        fsw_readers = [r for r in readers
                       if endpoint_by_id[r]["ownerId"] in fsw_owners]
        wiring = {
            "read_by": [endpoint_label(r) for r in readers],
            "read_by_flight_software": [endpoint_by_id[r]["name"]
                                        for r in fsw_readers],
        }
        for reader in fsw_readers:
            wrapper.append((endpoint_label(endpoint["id"]),
                            endpoint_label(reader)))
        outputs += _message_ports(endpoint, owner_name, "out", wiring)

    for endpoint in graph["inputs"]:
        if endpoint["ownerId"] in fsw_owners:
            continue
        source = source_of.get(endpoint["id"])
        if source is None:
            status = "unresolved" if endpoint["isLinked"] else "open"
        elif endpoint_by_id[source]["ownerId"] in fsw_owners:
            status = "flight_software"
            wrapper.append((endpoint_label(source),
                            endpoint_label(endpoint["id"])))
        else:
            status = "internal"
        if status == "internal" and not include_internal:
            continue
        wiring = {"status": status,
                  "source": endpoint_label(source) if source else None}
        inputs += _message_ports(endpoint, owner_name, "in", wiring)

    tasks = {}
    for record in graph["modules"]:
        tasks[record["taskName"]] = record["taskPeriod"]

    return Block(
        kind="simulator",
        name="Basilisk",
        source=f"{entry} ({Path(simulator_dir).resolve()})",
        inputs=inputs,
        outputs=outputs,
        meta={
            "entry": entry,
            "simulator_dir": str(Path(simulator_dir).resolve()),
            "tasks": tasks,
            "modules": [names[r["id"]] for r in graph["modules"]
                        if r["id"] not in fsw_owners],
            "flight_software_modules": [names[r] for r in fsw_owners],
            "standalone_messages": sorted(extra_messages),
            #: (simulator endpoint, bridge endpoint) pairs the current
            #: hand-written bridge uses -- the wiring the connector replaces.
            "bridge_connections": sorted(wrapper),
        },
    )


def _message_ports(endpoint, owner_name, direction, wiring):
    owner = owner_name[endpoint["ownerId"]]
    group = f"{owner}.{endpoint['name']}"
    ports = []
    for field_path, dtype, shape in payload_fields(endpoint["payloadType"]):
        ports.append(Port(
            id=f"{group}.{field_path}",
            direction=direction,
            dtype=dtype,
            shape=shape,
            group=group,
            description=endpoint["payloadType"],
            extra={"module": owner, "message": endpoint["name"],
                   "payload": endpoint["payloadType"], "field": field_path,
                   "wiring": wiring},
        ))
    return ports


# ---------------------------------------------------------------------------
# Port manifest: the simulator's inputs/outputs saved to a file
# ---------------------------------------------------------------------------
#
# Building the scenario means importing Basilisk and constructing every model
# (SPICE kernels, star catalogue, ...) -- seconds of work just to list ports.
# The result only changes when the simulator's code does, so it is saved next
# to the simulator as ``simulator_ports.json`` and read back on later starts.
# The file carries a fingerprint of every ``.py`` file in the simulator package
# plus the build arguments; any edit there makes it stale and it is rebuilt
# automatically, so it can never describe an older simulator.

#: Manifest file, written into the simulator folder.
MANIFEST_NAME = "simulator_ports.json"
MANIFEST_FORMAT = "aocs-simulator-ports/1"


def manifest_path(simulator_dir=DEFAULT_SIMULATOR_DIR):
    return Path(simulator_dir).resolve() / MANIFEST_NAME


def simulator_fingerprint(entry, simulator_dir, kwargs, flight_software_tags,
                          include_internal):
    """Hash of the simulator's source code and the build arguments.

    Content, not timestamps: OneDrive and copies change mtimes freely.  Reading
    ~100 small files takes milliseconds.
    """
    import hashlib
    import json

    root = Path(simulator_dir).resolve()
    digest = hashlib.sha256()
    digest.update(json.dumps(
        [MANIFEST_FORMAT, entry, kwargs, list(flight_software_tags),
         include_internal], sort_keys=True, default=str).encode())
    package = entry.partition(":")[0].split(".")[0]
    for path in sorted((root / package).rglob("*.py")):
        digest.update(path.relative_to(root).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()[:24]


def load_simulator_block(entry=DEFAULT_ENTRY,
                         simulator_dir=DEFAULT_SIMULATOR_DIR, kwargs=None,
                         flight_software_tags=DEFAULT_FLIGHT_SOFTWARE_TAGS,
                         include_internal=False, refresh=False):
    """The simulator block, from the manifest when it is current.

    Returns ``(block, from_manifest)``.  Falls back to
    :func:`build_simulator_block` -- and rewrites the manifest -- when the
    file is missing, stale, unreadable, or ``refresh`` is set.
    """
    import json

    from aocs_connector.ports import block_from_dict

    root = Path(simulator_dir).resolve()
    if kwargs is None and entry == DEFAULT_ENTRY:
        kwargs = DEFAULT_ENTRY_KWARGS
    fingerprint = simulator_fingerprint(entry, root, kwargs,
                                        flight_software_tags, include_internal)
    path = manifest_path(root)

    if not refresh and path.is_file():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if (data.get("format") == MANIFEST_FORMAT
                    and data.get("fingerprint") == fingerprint):
                block = block_from_dict(data["block"])
                # Paths are re-derived, so a moved folder still loads.
                block.source = f"{entry} ({root})"
                block.meta["simulator_dir"] = str(root)
                return block, True
        except (OSError, ValueError, KeyError, TypeError):
            pass                     # unreadable: rebuild below

    block = build_simulator_block(entry, root, kwargs, flight_software_tags,
                                  include_internal)
    try:
        data = {"format": MANIFEST_FORMAT,
                "fingerprint": fingerprint,
                "note": "Generated by the AOCS connector from the simulator "
                        "code; rebuilt automatically when that code changes. "
                        "Safe to delete.",
                "block": block.to_dict()}
        path.write_text(json.dumps(data, indent=1), encoding="utf-8")
    except OSError:
        pass                         # read-only folder: just not cached
    return block, False
