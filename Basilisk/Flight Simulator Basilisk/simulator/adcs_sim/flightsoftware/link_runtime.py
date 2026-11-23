"""Run a connector link file (``*.aocslink``) as the flight-software bridge.

The AOCS connector (``../connector``) is where the wiring between the
simulator and the flight software is drawn and saved.  This module reads that
file when the simulation is built and executes it every controller step, so a
saved change takes effect on the next run -- nothing in the generated code
folder (``ADCS_Software_v1_grt_rtw``) is edited or rebuilt.

Per step, inside :class:`~adcs_sim.flightsoftware.controller.GeneratedADCSController`:

.. code-block:: text

    sim:<module>.<message>.<field> --> conversion blocks --> sw:<input>   (ExtU)
                                                                 |
                                                            DLL step()
                                                                 |
    sim:<module>.<inMessage>.<field> <-- conversion blocks <-- sw:<output> (ExtY)

Simulator ports are named exactly as the connector names them: the same
Basilisk message graph and the same module-naming rules
(:mod:`aocs_connector.simulator_block`), so a wire saved in the editor finds
the same message here.  The conversion blocks are the connector's own
functions (:mod:`aocs_connector.conversions`) -- one implementation for both
the editor's type check and the run.

Simulator inputs:

* An input the link drives gets a message written by this runtime.  When the
  input was fed by the hand-written bridge's message (``rwMotorTorqueOutMsg``
  ...) that same message is written, so recorders and plots keep seeing what
  is actually applied.
* An input the old bridge drove but the link leaves unconnected is written
  with zeros -- unconnected means no command, as the editor warns.
* With **no flight software** (``"none": true``) every driven input is zero.

The link is checked when the simulation is built: a broken wire, an unknown
block, an unconnected block input or a loop stops the run with a list of the
problems, instead of flying a half-wired spacecraft.
"""

import ctypes
import re
import sys
from pathlib import Path

import numpy as np

from adcs_sim.config import paths


class LinkError(RuntimeError):
    """The link file cannot be run against this simulator and software."""


def _import_connector():
    """Import the connector package that sits next to the simulator."""
    connector_dir = str(paths.CONNECTOR_DIR)
    if connector_dir not in sys.path:
        sys.path.insert(0, connector_dir)
    try:
        from aocs_connector import conversions, link, simulator_block
    except ImportError as error:
        raise LinkError(
            f"The AOCS connector package was not found at {connector_dir}; it "
            "is needed to run a link file. Run with --no-link to use the "
            "hand-written bridge.") from error
    return conversions, link, simulator_block


def load_link(path):
    """Read a link file into a :class:`aocs_connector.link.LinkDocument`."""
    _, link, _ = _import_connector()
    path = Path(path)
    if not path.is_file():
        raise LinkError(f"Link file not found: {path}")
    try:
        return link.LinkDocument.load(path)
    except Exception as error:
        raise LinkError(f"Cannot read link file {path}: {error}") from error


def software_library(doc):
    """The DLL a link file selects, or ``None`` for no flight software."""
    if doc.no_software or not doc.software_path:
        return None
    path = Path(doc.software_path)
    if path.is_dir():
        # "Select software folder" in the editor: <model>.dll inside it.
        connector_dir = str(paths.CONNECTOR_DIR)
        if connector_dir not in sys.path:
            sys.path.insert(0, connector_dir)
        from aocs_connector.software_block import _locate
        _, _, library = _locate(path.resolve())
        if library is None:
            raise LinkError(f"No shared library found in {path} (selected in "
                            f"{Path(doc.path).name})")
        return library
    return path


# ---------------------------------------------------------------------------
# ctypes access to the software structs (sw:<port>)
# ---------------------------------------------------------------------------

_PART = re.compile(r"(\w+)((?:\[\d+\])*)$")


def _steps(path):
    """``"bus.x[2]"`` -> ``[("attr", "bus"), ("attr", "x"), ("item", 2)]``."""
    steps = []
    for part in path.split("."):
        match = _PART.match(part)
        if match is None:
            raise KeyError(path)
        steps.append(("attr", match.group(1)))
        steps += [("item", int(i)) for i in re.findall(r"\[(\d+)\]",
                                                        match.group(2))]
    return steps


def _parent(obj, path):
    steps = _steps(path)
    for kind, key in steps[:-1]:
        obj = getattr(obj, key) if kind == "attr" else obj[key]
    return obj, steps[-1]


def _ctypes_get(struct, path):
    parent, (kind, key) = _parent(struct, path)
    return getattr(parent, key) if kind == "attr" else parent[key]


def _check_ctypes_path(struct, path):
    try:
        _ctypes_get(struct, path)
        return True
    except (AttributeError, KeyError, IndexError, TypeError):
        return False


def read_software(struct, path):
    """Copy of a software struct field as a numpy array."""
    value = _ctypes_get(struct, path)
    if isinstance(value, ctypes.Array):
        return np.ctypeslib.as_array(value).copy()
    return np.asarray(value)


def write_software(struct, path, value):
    """Write a numpy value into a software struct field (in the DLL)."""
    parent, (kind, key) = _parent(struct, path)
    current = getattr(parent, key) if kind == "attr" else parent[key]
    value = np.asarray(value)
    if isinstance(current, ctypes.Array):
        _fill(np.ctypeslib.as_array(current), value)
        return
    if value.size != 1:
        raise ValueError(f"{value.size} values into the scalar {path}")
    scalar = value.reshape(()).item()
    if isinstance(current, bool):
        scalar = bool(scalar)
    elif isinstance(current, int):
        scalar = int(scalar)
    else:
        scalar = float(scalar)
    if kind == "attr":
        setattr(parent, key, scalar)
    else:
        parent[key] = scalar


def _fill(view, value):
    """Write ``value`` into the array ``view`` by the editor's shape rules.

    Same shapes: copied.  Same size: reshaped.  A shorter vector into a
    vector: fills the first elements, the rest zero.  Anything else raises.
    """
    if value.shape == view.shape:
        view[...] = value
    elif value.size == view.size and value.size > 0:
        view[...] = value.reshape(view.shape)
    elif value.ndim <= 1 and view.ndim <= 1 and value.size < view.size:
        view[...] = 0
        view[:value.size] = value.ravel()
    else:
        raise ValueError(f"shape {value.shape} does not fit {view.shape}")


# ---------------------------------------------------------------------------
# Basilisk payload access (sim:<port>)
# ---------------------------------------------------------------------------

def _payload_get(payload, field):
    value = payload
    for name in field.split("."):
        value = getattr(value, name)
    return value


def _payload_set(payload, field, value):
    """Set one payload field from a numpy value, keeping its type and shape.

    The complete SWIG property is assigned: slice-assigning a returned list
    only changes a temporary copy and leaves the payload at zero.
    """
    names = field.split(".")
    parent = payload
    for name in names[:-1]:
        parent = getattr(parent, name)
    current = getattr(parent, names[-1])
    value = np.asarray(value)
    if isinstance(current, (list, tuple)):
        template = np.asarray(current)
        target = np.zeros(template.shape,
                          dtype=float if template.dtype.kind == "f"
                          else template.dtype)
        _fill(target, value)
        setattr(parent, names[-1], target.tolist())
        return
    if value.size != 1:
        raise ValueError(f"{value.size} values into the scalar field {field}")
    scalar = value.reshape(()).item()
    if isinstance(current, bool):
        scalar = bool(scalar)
    elif isinstance(current, int):
        scalar = int(scalar)
    elif isinstance(current, float):
        scalar = float(scalar)
    setattr(parent, names[-1], scalar)


def _message_classes(payload_type):
    """``(messaging module, payload class, Msg class, Reader class)``."""
    from Basilisk.architecture import messaging

    base = payload_type[:-len("Payload")] if payload_type.endswith("Payload") \
        else payload_type
    try:
        return (getattr(messaging, payload_type), getattr(messaging, base),
                getattr(messaging, base + "Reader"))
    except AttributeError as error:
        raise LinkError(f"Basilisk has no message classes for "
                        f"{payload_type}") from error


class _SimSource:
    """A simulator output message, read once per step."""

    def __init__(self, endpoint):
        _, _, reader_class = _message_classes(endpoint["payloadType"])
        self.reader = reader_class()
        self.reader.subscribeTo(endpoint["message"])
        self.payload_type = endpoint["payloadType"]

    def read(self):
        return self.reader()


class _SimTarget:
    """A simulator input message this runtime writes each step."""

    def __init__(self, label, payload_type, message, created):
        self.label = label
        self.payload_class, _, _ = _message_classes(payload_type)
        self.message = message
        #: True when the runtime created the message (not the old bridge's).
        self.created = created
        #: ``[(field, source endpoint)]``; empty = written as zeros.
        self.fields = []


# ---------------------------------------------------------------------------
# The runtime
# ---------------------------------------------------------------------------

class LinkRuntime:
    """A link document bound to a built simulation and its flight software.

    Build with :meth:`bind` after the scenario's messages are connected; the
    controller then calls :meth:`write_software_inputs` before and
    :meth:`publish` after every software step.
    """

    def __init__(self, doc):
        self.doc = doc
        self.path = Path(doc.path) if doc.path else None
        conversions, link, simulator_block = _import_connector()
        self._conversions = conversions
        self._link = link
        self._simulator_block = simulator_block
        #: target endpoint -> source endpoint
        self.feeds = {target: source for source, target in doc.wires}
        self._sim_sources = {}          # message group -> _SimSource
        self._sim_source_fields = {}    # endpoint -> (group, field)
        self._sw_inputs = []            # [(port, source endpoint)]
        self._targets = []              # [_SimTarget]
        self._node_order = []
        self._retained = []

    # -- binding ------------------------------------------------------------

    def bind(self, bundle, library, flight_software_tags=("ADCS_Software_v1",)):
        """Resolve every wire against the simulation and the software."""
        sb = self._simulator_block
        sim = bundle.sim
        module_ids = {id(m) for task in sim.TaskList for m in task.TaskModels}
        extra = sb._standalone_messages(bundle, module_ids)
        graph = sb.message_graph(sim, extra)
        names = sb._module_names(sim, graph)
        owner_name = dict(names)
        for record in graph["standaloneMessages"]:
            owner_name[record["id"]] = "python"
        fsw_owners = {r["id"] for r in graph["modules"]
                      if r["tag"] in flight_software_tags}
        self._retained.append(graph)

        endpoint_by_id = {e["id"]: e for e in graph["inputs"] + graph["outputs"]}
        source_of = {edge["target"]: edge["source"] for edge in graph["edges"]}

        def group_of(endpoint):
            return f"{owner_name[endpoint['ownerId']]}.{endpoint['name']}"

        outputs = {group_of(e): e for e in graph["outputs"]
                   if e["ownerId"] not in fsw_owners}
        inputs = {group_of(e): e for e in graph["inputs"]
                  if e["ownerId"] not in fsw_owners}

        problems = []

        def split(port, groups):
            """``(group, field)`` for a simulator port id, longest group first."""
            best = None
            for group in groups:
                if port.startswith(group + ".") and \
                        (best is None or len(group) > len(best)):
                    best = group
            return (best, port[len(best) + 1:]) if best else (None, None)

        owner, port_name = self._link.owner, self._link.port_name
        conversions = self._conversions

        # Endpoints, by kind.
        for source, target in self.doc.wires:
            for endpoint, direction in ((source, "out"), (target, "in")):
                kind, name = owner(endpoint), port_name(endpoint)
                if kind == "sw":
                    struct = library.outputs if direction == "out" \
                        else library.inputs
                    if not _check_ctypes_path(struct, name):
                        side = "output" if direction == "out" else "input"
                        problems.append(f"{endpoint}: the software has no "
                                        f"{side} '{name}'")
                elif kind == "sim":
                    groups = outputs if direction == "out" else inputs
                    group, field = split(name, groups)
                    if group is None:
                        problems.append(f"{endpoint}: no such simulator "
                                        f"{'output' if direction == 'out' else 'input'}")
                        continue
                    payload_class, _, _ = _message_classes(
                        groups[group]["payloadType"])
                    try:
                        _payload_get(payload_class(), field)
                    except AttributeError:
                        problems.append(f"{endpoint}: {groups[group]['payloadType']} "
                                        f"has no field '{field}'")
                else:
                    node = self.doc.nodes.get(kind)
                    if node is None:
                        problems.append(f"{endpoint}: no block '{kind}'")
                    elif node.type not in conversions.TYPES:
                        problems.append(f"{kind}: unknown block type "
                                        f"{node.type!r}")
                    else:
                        valid = node.output_names() if direction == "out" \
                            else node.input_names()
                        if name not in valid:
                            problems.append(f"{endpoint}: block {kind} has no "
                                            f"{'output' if direction == 'out' else 'input'} "
                                            f"'{name}'")

        # Blocks that feed something must have every input connected.
        used = self._used_nodes()
        for key in sorted(used):
            node = self.doc.nodes.get(key)
            if node is None or node.type not in conversions.TYPES:
                continue
            for name in node.input_names():
                if f"{key}:{name}" not in self.feeds:
                    problems.append(f"{node.label or node.spec().label} "
                                    f"({key}): input '{name}' is not connected")
        try:
            self._node_order = self._topological(used)
        except LinkError as error:
            problems.append(str(error))

        if problems:
            raise LinkError(
                f"The link file {self.path} cannot be run "
                f"({len(problems)} problem{'s' * (len(problems) != 1)}):\n  - "
                + "\n  - ".join(problems)
                + "\nOpen it in the connector (Problems panel), fix and save; "
                  "or run with --no-link to use the hand-written bridge.")

        # Simulator sources: one reader per message.
        for source in {s for s, _ in self.doc.wires}:
            if owner(source) != "sim":
                continue
            group, field = split(port_name(source), outputs)
            if group not in self._sim_sources:
                self._sim_sources[group] = _SimSource(outputs[group])
            self._sim_source_fields[source] = (group, field)

        # Software inputs.
        self._sw_inputs = [(port_name(t), s) for s, t in self.doc.wires
                           if owner(t) == "sw"]

        # Simulator inputs: driven by the link, or by the old bridge.
        targets = {}
        for source, target in self.doc.wires:
            if owner(target) != "sim":
                continue
            group, field = split(port_name(target), inputs)
            if group not in targets:
                targets[group] = self._make_target(
                    group, inputs[group], source_of, endpoint_by_id, fsw_owners)
            targets[group].fields.append((field, source))
        for group, endpoint in inputs.items():
            if group in targets:
                continue
            source = source_of.get(endpoint["id"])
            if source is not None and \
                    endpoint_by_id[source]["ownerId"] in fsw_owners:
                # Old bridge input the link leaves unconnected: zero.
                targets[group] = self._make_target(
                    group, endpoint, source_of, endpoint_by_id, fsw_owners)
        self._targets = list(targets.values())
        return self

    def _make_target(self, group, endpoint, source_of, endpoint_by_id,
                     fsw_owners):
        source = source_of.get(endpoint["id"])
        if source is not None and endpoint_by_id[source]["ownerId"] in fsw_owners:
            # Keep writing the bridge's own message: recorders read it.
            return _SimTarget(group, endpoint["payloadType"],
                              endpoint_by_id[source]["message"], created=False)
        _, message_class, _ = _message_classes(endpoint["payloadType"])
        message = message_class()
        endpoint["message"].subscribeTo(message)
        self._retained.append(message)
        return _SimTarget(group, endpoint["payloadType"], message, created=True)

    def _used_nodes(self):
        """Blocks that something downstream reads."""
        owner = self._link.owner
        used, stack = set(), [t for _, t in self.doc.wires
                              if owner(t) in ("sw", "sim")]
        while stack:
            endpoint = stack.pop()
            source = self.feeds.get(endpoint)
            if source is None:
                continue
            key = owner(source)
            if key in ("sw", "sim") or key in used:
                continue
            used.add(key)
            node = self.doc.nodes.get(key)
            if node is not None and node.type in self._conversions.TYPES:
                stack += [f"{key}:{name}" for name in node.input_names()]
        return used

    def _topological(self, keys):
        owner = self._link.owner
        order, state = [], {}

        def visit(key, chain):
            if state.get(key) == "done":
                return
            if state.get(key) == "active":
                raise LinkError("conversion blocks form a loop: "
                                + " -> ".join(chain + [key]))
            state[key] = "active"
            node = self.doc.nodes[key]
            for name in node.input_names():
                source = self.feeds.get(f"{key}:{name}")
                if source is not None and owner(source) in keys:
                    visit(owner(source), chain + [key])
            state[key] = "done"
            order.append(key)

        for key in sorted(keys):
            if self.doc.nodes.get(key) is not None and \
                    self.doc.nodes[key].type in self._conversions.TYPES:
                visit(key, [])
        return order

    # -- evaluation ---------------------------------------------------------

    def _value(self, endpoint, memo, library):
        if endpoint in memo:
            return memo[endpoint]
        kind, name = self._link.owner(endpoint), self._link.port_name(endpoint)
        if kind == "sim":
            group, field = self._sim_source_fields[endpoint]
            payload_key = ("payload", group)
            if payload_key not in memo:
                memo[payload_key] = self._sim_sources[group].read()
            value = np.asarray(_payload_get(memo[payload_key], field))
            memo[endpoint] = value
            return value
        if kind == "sw":
            value = read_software(library.outputs, name)
            memo[endpoint] = value
            return value
        node = self.doc.nodes[kind]
        arguments = [self._value(self.feeds[f"{kind}:{n}"], memo, library)
                     for n in node.input_names()]
        try:
            results = node.spec().function(node.params, *arguments)
        except Exception as error:
            raise LinkError(f"block {node.label or node.spec().label} ({kind}) "
                            f"failed: {type(error).__name__}: {error}") from error
        for output, result in zip(node.output_names(), results):
            memo[f"{kind}:{output}"] = np.asarray(result)
        return memo[endpoint]

    def write_software_inputs(self, library):
        """Fill the software's inputs (ExtU) from the simulator, pre-step."""
        memo = {}
        for port, source in self._sw_inputs:
            value = self._value(source, memo, library)
            try:
                write_software(library.inputs, port, value)
            except (ValueError, TypeError) as error:
                raise LinkError(f"wire {source} -> sw:{port}: {error}") from error

    def publish(self, library, current_sim_nanos, module_id, zero=False):
        """Write the simulator inputs from the software's outputs, post-step.

        ``zero`` writes every driven input as zeros (no flight software).
        """
        memo = {}
        for target in self._targets:
            payload = target.payload_class()
            if not zero:
                for field, source in target.fields:
                    value = self._value(source, memo, library)
                    try:
                        _payload_set(payload, field, value)
                    except (ValueError, TypeError) as error:
                        raise LinkError(f"wire {source} -> sim:{target.label}."
                                        f"{field}: {error}") from error
            target.message.write(payload, current_sim_nanos, module_id)

    # -- reporting ----------------------------------------------------------

    def describe(self):
        """Printable lines for the run banner."""
        lines = [f"Wiring: link file {self.path} "
                 f"({len(self.doc.wires)} wires, {len(self.doc.nodes)} blocks)"]
        zeroed = [t.label for t in self._targets if not t.fields]
        if zeroed:
            lines.append("  not connected in the link, held at zero: "
                         + ", ".join(sorted(zeroed)))
        return lines
