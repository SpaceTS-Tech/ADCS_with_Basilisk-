"""Port and block descriptions shared by both sides of the connector.

A :class:`Block` is what the GUI will draw: a named box with input and output
:class:`Port` objects.  Both extractors -- the flight software (from its
generated C headers) and the simulator (from the Basilisk message graph) --
produce the same structure, so everything downstream (the editor, the link
file, the runtime) only ever deals with this one shape.

Direction is always seen **from the block**:

* ``"in"``  -- the block consumes this value (a software inport, a Basilisk
  input message field).
* ``"out"`` -- the block produces it (a software outport, a Basilisk output
  message field).

A wire therefore always runs from an ``out`` port to an ``in`` port.
"""

import hashlib
import json
from dataclasses import asdict, dataclass, field


@dataclass
class Port:
    """One connectable value."""

    #: Unique within its block, e.g. ``"q_BI"`` or ``"bsk-Sat.scStateOutMsg.sigma_BN"``.
    id: str
    direction: str
    #: Element type, numpy-style: ``float64``, ``uint8``, ``bool``, ``int``...
    dtype: str
    #: Array shape; ``()`` for a scalar.
    shape: tuple = ()
    #: Collapsible group in the editor (bus name, Basilisk message).
    group: str = ""
    #: Human-readable origin, e.g. the Simulink block path.
    description: str = ""
    #: Side-specific details (byte offset, payload type, wiring status...).
    extra: dict = field(default_factory=dict)

    @property
    def size(self):
        """Number of elements."""
        count = 1
        for dim in self.shape:
            count *= dim
        return count

    def signature(self):
        """What a saved connection depends on: identity, type and shape."""
        return (self.id, self.direction, self.dtype, tuple(self.shape))

    def type_string(self):
        if not self.shape:
            return self.dtype
        return f"{self.dtype}[{','.join(str(d) for d in self.shape)}]"


@dataclass
class Block:
    """A box in the connector: one side of the binding."""

    kind: str          # "software" or "simulator"
    name: str
    source: str        # folder, DLL or entry point the block was read from
    inputs: list = field(default_factory=list)
    outputs: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)

    @property
    def ports(self):
        return self.inputs + self.outputs

    def port(self, port_id):
        for candidate in self.ports:
            if candidate.id == port_id:
                return candidate
        raise KeyError(f"{self.name} has no port {port_id!r}")

    def signature(self):
        """Hash of every port signature.

        Stored in the link file so a reload can tell at once whether the
        software or simulator interface changed since the wiring was saved.
        """
        text = json.dumps(sorted(p.signature() for p in self.ports))
        return hashlib.sha256(text.encode()).hexdigest()[:16]

    def to_dict(self):
        data = asdict(self)
        data["signature"] = self.signature()
        return data


def block_from_dict(data):
    """Inverse of :meth:`Block.to_dict`."""
    def ports(items):
        return [Port(**{**item, "shape": tuple(item["shape"])}) for item in items]

    return Block(kind=data["kind"], name=data["name"], source=data["source"],
                 inputs=ports(data["inputs"]), outputs=ports(data["outputs"]),
                 meta=data.get("meta", {}))
