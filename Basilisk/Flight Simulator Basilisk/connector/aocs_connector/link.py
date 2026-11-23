"""The link file: which simulator value feeds which software port, and how.

A link file (``*.aocslink``, JSON) is the whole binding:

* where the software package is and how the simulator is built,
* the conversion blocks and their parameters,
* the wires,
* the diagram layout (block positions, shown modules, expanded groups),
* the port signatures of both sides when it was saved.

Wire endpoints are strings:

==========================  ==============================================
``sw:<port>``               software port, e.g. ``sw:q_BI``
``sim:<port>``              simulator port, e.g. ``sim:SimpleNavigation.attOutMsg.sigma_BN``
``<node>:<port>``           conversion block port, e.g. ``n3:sigma``
==========================  ==============================================

Paths are stored relative to the link file, so a project folder can move.
:func:`validate` checks a document against freshly read blocks; it never
modifies the document, so a wire to a port that has disappeared stays in the
file (reported as broken) until the user removes it.
"""

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

from aocs_connector import conversions

FORMAT = "aocs-link/1"
SUFFIX = ".aocslink"


@dataclass
class Node:
    type: str
    params: dict
    pos: tuple = (0.0, 0.0)
    label: str = ""

    def spec(self):
        return conversions.TYPES[self.type]

    def input_names(self):
        return self.spec().input_names(self.params)

    def output_names(self):
        return list(self.spec().outputs)


@dataclass
class LinkDocument:
    software_path: str = ""
    #: True: no flight software is connected.  The simulator's actuator
    #: inputs stay at zero -- no control torque or force.  Saved as
    #: ``"none": true`` in the ``software`` entry.
    no_software: bool = False
    simulator: dict = field(default_factory=dict)
    nodes: dict = field(default_factory=dict)
    #: ``[(source, target)]`` -- source is an output, target an input.
    wires: list = field(default_factory=list)
    positions: dict = field(default_factory=dict)
    #: Simulator modules drawn although nothing is wired to them.
    shown_modules: list = field(default_factory=list)
    expanded_groups: list = field(default_factory=list)
    #: ``{block key: degrees clockwise}`` -- 90, 180 or 270; 0 is not stored.
    rotations: dict = field(default_factory=dict)
    saved_signatures: dict = field(default_factory=dict)
    path: str = ""

    # -- editing ------------------------------------------------------------

    def new_node_id(self):
        index = 1
        while f"n{index}" in self.nodes:
            index += 1
        return f"n{index}"

    def add_node(self, type_key, pos=(0.0, 0.0), params=None, node_id=None):
        node_id = node_id or self.new_node_id()
        spec = conversions.TYPES[type_key]
        self.nodes[node_id] = Node(type_key, {**spec.defaults(),
                                              **(params or {})}, tuple(pos))
        return node_id

    def remove_node(self, node_id):
        self.nodes.pop(node_id, None)
        self.wires = [w for w in self.wires
                      if owner(w[0]) != node_id and owner(w[1]) != node_id]

    def connect(self, source, target):
        """Add a wire; an input takes one wire, so any old one is replaced."""
        self.wires = [w for w in self.wires if w[1] != target]
        self.wires.append((source, target))

    def disconnect(self, wire):
        self.wires = [w for w in self.wires if tuple(w) != tuple(wire)]

    def source_of(self, target):
        for source, wired_target in self.wires:
            if wired_target == target:
                return source
        return None

    # -- files --------------------------------------------------------------

    def to_json(self):
        base = Path(self.path).parent if self.path else Path.cwd()
        simulator = dict(self.simulator)
        if simulator.get("simulator_dir"):
            simulator["simulator_dir"] = _relative(simulator["simulator_dir"],
                                                   base)
        return {
            "format": FORMAT,
            # The path is kept with "none", so switching the software back
            # on restores the same DLL.
            "software": {"path": _relative(self.software_path, base)
                         if self.software_path else "",
                         "signature": self.saved_signatures.get("software"),
                         **({"none": True} if self.no_software else {})},
            "simulator": {**simulator,
                          "signature": self.saved_signatures.get("simulator")},
            "nodes": {key: {"type": n.type, "params": n.params,
                            "pos": [round(n.pos[0], 1), round(n.pos[1], 1)],
                            **({"label": n.label} if n.label else {})}
                      for key, n in self.nodes.items()},
            "wires": [{"from": s, "to": t} for s, t in self.wires],
            "layout": {"positions": {k: [round(v[0], 1), round(v[1], 1)]
                                     for k, v in self.positions.items()},
                       "shown_modules": sorted(self.shown_modules),
                       "expanded_groups": sorted(self.expanded_groups),
                       "rotations": {k: v for k, v in
                                     sorted(self.rotations.items()) if v}},
        }

    def save(self, path=None, signatures=None):
        if path:
            self.path = str(Path(path).resolve())
        if signatures:
            self.saved_signatures = dict(signatures)
        Path(self.path).write_text(json.dumps(self.to_json(), indent=2),
                                   encoding="utf-8")

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        return cls.from_json(json.loads(path.read_text(encoding="utf-8")),
                             path)

    @classmethod
    def from_json(cls, data, path):
        """Rebuild a document; relative paths resolve against ``path``."""
        if data.get("format") != FORMAT:
            raise ValueError(f"{Path(path).name} is not an {FORMAT} file")
        base = Path(path).parent if path else Path.cwd()
        software = data.get("software", {})
        simulator = dict(data.get("simulator", {}))
        signatures = {"software": software.get("signature"),
                      "simulator": simulator.pop("signature", None)}
        if simulator.get("simulator_dir"):
            simulator["simulator_dir"] = str(
                (base / simulator["simulator_dir"]).resolve())
        layout = data.get("layout", {})
        return cls(
            software_path=str((base / software["path"]).resolve())
            if software.get("path") else "",
            no_software=bool(software.get("none", False)),
            simulator=simulator,
            nodes={key: Node(n["type"], n.get("params", {}),
                             tuple(n.get("pos", (0, 0))), n.get("label", ""))
                   for key, n in data.get("nodes", {}).items()},
            wires=[(w["from"], w["to"]) for w in data.get("wires", [])],
            positions={k: tuple(v) for k, v in
                       layout.get("positions", {}).items()},
            shown_modules=list(layout.get("shown_modules", [])),
            expanded_groups=list(layout.get("expanded_groups", [])),
            rotations={k: int(v) % 360 for k, v in
                       layout.get("rotations", {}).items()},
            saved_signatures=signatures,
            path=str(path) if path else "",
        )


def _relative(path, base):
    try:
        return Path(os.path.relpath(Path(path).resolve(), base)).as_posix()
    except ValueError:              # different drive on Windows
        return str(Path(path).resolve())


def owner(endpoint):
    """``"sw"``, ``"sim"`` or the node id of an endpoint."""
    return endpoint.split(":", 1)[0]


def port_name(endpoint):
    return endpoint.split(":", 1)[1]


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

@dataclass
class Issue:
    level: str          # "error", "warning", "info"
    message: str
    #: What to highlight: a wire tuple, a node id or an endpoint string.
    ref: object = None


@dataclass
class Report:
    issues: list = field(default_factory=list)
    #: wire tuple -> (level, message); level "ok" for a clean wire.
    wire_status: dict = field(default_factory=dict)
    #: endpoint -> (dtype, shape) for every conversion-block output.
    node_outputs: dict = field(default_factory=dict)

    def add(self, level, message, ref=None):
        self.issues.append(Issue(level, message, ref))

    def counts(self):
        return {level: sum(1 for i in self.issues if i.level == level)
                for level in ("error", "warning", "info")}


def check_types(source, target_port):
    """``(level, message)`` for a value of ``source`` spec into a port."""
    if source is None:
        return "warning", "type of the source is unknown"
    dtype, shape = source
    t_dtype, t_shape = target_port.dtype, tuple(target_port.shape)
    size = _size(shape)
    t_size = _size(t_shape)
    if tuple(shape) != t_shape:
        if len(shape) <= 1 and len(t_shape) <= 1 and size < t_size:
            return "info", (f"{_fmt(dtype, shape)} fills the first {size} of "
                            f"{t_size} elements; the rest are zero")
        if size == t_size and size > 0:
            return "info", f"reshaped {_fmt(dtype, shape)} → {t_shape}"
        return "error", (f"shape mismatch: {_fmt(dtype, shape)} into "
                         f"{_fmt(t_dtype, t_shape)}")
    if dtype != t_dtype and "unknown" not in (dtype, t_dtype):
        lossy = dtype.startswith("float") and not t_dtype.startswith("float")
        if lossy:
            return "warning", f"{dtype} converted to {t_dtype} (truncated)"
        return "ok", f"{dtype} converted to {t_dtype}"
    return "ok", ""


def _size(shape):
    count = 1
    for dim in shape:
        count *= dim
    return count


def _fmt(dtype, shape):
    return f"{dtype}[{','.join(map(str, shape))}]" if shape else dtype


def validate(doc, software=None, simulator=None):
    """Check a document against the blocks read from disk."""
    report = Report()
    sw_ports = {p.id: p for p in software.ports} if software else {}
    sim_ports = {p.id: p for p in simulator.ports} if simulator else {}

    def port_of(endpoint):
        kind, name = owner(endpoint), port_name(endpoint)
        if kind == "sw":
            return sw_ports.get(name)
        if kind == "sim":
            return sim_ports.get(name)
        return None

    def endpoint_exists(endpoint, direction):
        kind, name = owner(endpoint), port_name(endpoint)
        if kind in ("sw", "sim"):
            port = port_of(endpoint)
            return port is not None and port.direction == direction
        node = doc.nodes.get(kind)
        if node is None or node.type not in conversions.TYPES:
            return False
        names = node.output_names() if direction == "out" \
            else node.input_names()
        return name in names

    # 1. broken wires
    live_wires = []
    for wire in doc.wires:
        source, target = wire
        missing = [e for e, d in ((source, "out"), (target, "in"))
                   if not endpoint_exists(e, d)]
        if missing:
            side = " and ".join(missing)
            report.wire_status[wire] = ("error", f"port no longer exists: {side}")
            report.add("error", f"Broken wire {source} → {target}: "
                                f"{side} no longer exists", wire)
        else:
            live_wires.append(wire)
    feeds = {target: source for source, target in live_wires}

    # 2. conversion blocks, in dependency order
    def source_spec(endpoint):
        if owner(endpoint) in ("sw", "sim"):
            port = port_of(endpoint)
            return (port.dtype, tuple(port.shape)) if port else None
        return report.node_outputs.get(endpoint)

    pending = {key for key, n in doc.nodes.items()
               if n.type in conversions.TYPES}
    for key, node in doc.nodes.items():
        if node.type not in conversions.TYPES:
            report.add("error", f"{key}: unknown block type {node.type!r}", key)
    progress = True
    while pending and progress:
        progress = False
        for key in sorted(pending):
            node = doc.nodes[key]
            inputs = [f"{key}:{name}" for name in node.input_names()]
            upstream = [feeds.get(i) for i in inputs]
            if any(u is not None and owner(u) in pending and owner(u) != key
                   for u in upstream):
                continue                     # wait for upstream blocks
            pending.discard(key)
            progress = True
            unconnected = [i for i, u in zip(inputs, upstream) if u is None]
            for endpoint in unconnected:
                report.add("error", f"{_node_title(key, node)}: input "
                                    f"'{port_name(endpoint)}' is not connected",
                           key)
            specs = [source_spec(u) if u else None for u in upstream]
            outputs, error = conversions.infer_outputs(node.type, node.params,
                                                       specs)
            if outputs is None:
                if not unconnected:
                    report.add("error", f"{_node_title(key, node)}: {error}",
                               key)
                continue
            for name, spec in zip(node.output_names(), outputs):
                report.node_outputs[f"{key}:{name}"] = spec
    for key in sorted(pending):
        report.add("error", f"{_node_title(key, doc.nodes[key])} is part of "
                            f"a loop of conversion blocks", key)

    # 3. wires into software / simulator ports
    for wire in live_wires:
        source, target = wire
        target_port = port_of(target)
        if target_port is None:
            report.wire_status[wire] = ("ok", "")
            continue
        level, message = check_types(source_spec(source), target_port)
        report.wire_status[wire] = (level, message)
        if level in ("error", "warning"):
            report.add(level, f"{source} → {target}: {message}", wire)

    # 4. unconnected boundary ports
    for port in software.inputs if software else []:
        if f"sw:{port.id}" not in feeds:
            report.add("warning", f"Software input '{port.id}' is not "
                                  f"connected — it stays at zero",
                       f"sw:{port.id}")
    if doc.no_software:
        report.add("info", "No flight software: the simulator's actuator "
                           "inputs stay at zero — no control torque or force")
    for port in simulator.inputs if simulator and not doc.no_software else []:
        wiring = port.extra.get("wiring", {})
        if wiring.get("status") == "flight_software" and \
                f"sim:{port.id}" not in feeds:
            report.add("warning", f"Simulator input '{port.id}' was driven by "
                                  f"the old bridge and is not connected",
                       f"sim:{port.id}")

    # 5. timing and interface changes
    if software and simulator:
        step = software.meta.get("sample_time")
        periods = set(simulator.meta.get("tasks", {}).values())
        if step and periods and all(abs(step - p) > 1e-12 for p in periods):
            report.add("warning", f"Software step {step} s does not match any "
                                  f"simulator task period {sorted(periods)}")
    for key, block in (("software", software), ("simulator", simulator)):
        saved = doc.saved_signatures.get(key)
        if block and saved and saved != block.signature():
            report.add("info", f"The {key} interface changed since this file "
                               f"was saved — new ports are shown unconnected, "
                               f"removed ones as broken wires")
    return report


def _node_title(key, node):
    return f"{node.label or node.spec().label} ({key})"
