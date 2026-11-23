"""Conversion blocks: what sits on a wire between simulator and software.

Every block is a small pure function over numpy arrays.  The same function
serves twice:

* in the editor, it runs once on zeros to work out the block's output type
  and shape, so a wire can be checked before anything is simulated;
* in the runtime (phase 2), it runs every step to convert the data.

A block type declares its input and output port names and its parameters.
Parameters are plain JSON values, so they save straight into the link file.
"""

import importlib
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Param:
    name: str
    kind: str          # "float", "int", "text", "dtype"
    default: object
    help: str = ""


@dataclass
class ConversionType:
    key: str
    label: str
    category: str
    inputs: list
    outputs: list
    function: object             # (params, *inputs) -> tuple of outputs
    params: list = field(default_factory=list)
    help: str = ""
    #: Number of inputs set by a parameter (Mux), else fixed.
    variable_inputs: str = ""

    def input_names(self, params):
        if self.variable_inputs:
            count = max(1, int(params.get(self.variable_inputs, 2)))
            return [f"u{i + 1}" for i in range(count)]
        return list(self.inputs)

    def defaults(self):
        return {p.name: p.default for p in self.params}


# ---------------------------------------------------------------------------
# Attitude maths (numpy, same conventions as Basilisk RigidBodyKinematics)
# ---------------------------------------------------------------------------

def mrp_to_quaternion(sigma):
    """MRP -> quaternion, scalar first (Basilisk ``MRP2EP``)."""
    s = np.asarray(sigma, dtype=float)
    s2 = s @ s
    return np.concatenate(([(1.0 - s2) / (1.0 + s2)], 2.0 * s / (1.0 + s2)))


def quaternion_to_mrp(q):
    """Quaternion, scalar first -> MRP, short rotation (Basilisk ``EP2MRP``)."""
    q = np.asarray(q, dtype=float)
    if q[0] < 0:
        q = -q
    return q[1:] / (1.0 + q[0])


def mrp_to_dcm(sigma):
    """MRP -> direction cosine matrix [BN] (Basilisk ``MRP2C``)."""
    s = np.asarray(sigma, dtype=float)
    s2 = s @ s
    tilde = np.array([[0, -s[2], s[1]], [s[2], 0, -s[0]], [-s[1], s[0], 0]])
    return np.eye(3) + (8 * tilde @ tilde - 4 * (1 - s2) * tilde) / (1 + s2) ** 2


# ---------------------------------------------------------------------------
# Block functions
# ---------------------------------------------------------------------------

def _indices(text):
    """``"0,1,2"`` or ``"0:3"`` -> index array."""
    text = str(text).strip()
    if ":" in text:
        parts = [int(p) if p.strip() else None for p in text.split(":")]
        return slice(*parts)
    return [int(p) for p in text.split(",") if p.strip()]


def _select(params, u):
    return (np.asarray(u)[_indices(params["indices"])],)


def _place(params, u):
    out = np.zeros(int(params["length"]))
    u = np.atleast_1d(np.asarray(u, dtype=float))
    start = int(params["offset"])
    out[start:start + u.size] = u.ravel()
    return (out,)


def _python_function(params, *inputs):
    module_name, _, function_name = str(params["function"]).partition(":")
    function = getattr(importlib.import_module(module_name), function_name)
    result = function(*inputs)
    return (np.asarray(result),)


def _expression(params, *inputs):
    names = {f"u{i + 1}": np.asarray(v) for i, v in enumerate(inputs)}
    if inputs:
        names["u"] = names["u1"]
    names["np"] = np
    return (np.asarray(eval(str(params["expression"]), {"__builtins__": {}},
                            names)),)


TYPES = {t.key: t for t in [
    ConversionType(
        "gain", "Gain", "Math", ["u"], ["y"],
        lambda p, u: (float(p["k"]) * np.asarray(u),),
        [Param("k", "float", 1.0, "y = k * u")],
        "Multiply by a constant. k = -1 flips the sign."),
    ConversionType(
        "mrp_to_quat", "MRP → Quaternion", "Attitude", ["sigma"], ["q"],
        lambda p, s: (mrp_to_quaternion(s),),
        help="Basilisk MRP to quaternion, scalar first (MRP2EP)."),
    ConversionType(
        "quat_to_mrp", "Quaternion → MRP", "Attitude", ["q"], ["sigma"],
        lambda p, q: (quaternion_to_mrp(q),),
        help="Quaternion, scalar first, to Basilisk MRP (EP2MRP)."),
    ConversionType(
        "rotate_to_body", "Inertial → Body (MRP)", "Attitude",
        ["sigma_BN", "v_N"], ["v_B"],
        lambda p, s, v: (mrp_to_dcm(s) @ np.asarray(v, dtype=float),),
        help="Rotate an inertial vector into body axes: v_B = [BN(σ)] v_N."),
    ConversionType(
        "rotate_to_inertial", "Body → Inertial (MRP)", "Attitude",
        ["sigma_BN", "v_B"], ["v_N"],
        lambda p, s, v: (mrp_to_dcm(s).T @ np.asarray(v, dtype=float),),
        help="Rotate a body vector into inertial axes: v_N = [BN(σ)]ᵀ v_B."),
    ConversionType(
        "quat_swap", "Quat scalar first ↔ last", "Attitude", ["q"], ["q"],
        lambda p, q: (np.roll(np.asarray(q, dtype=float),
                              -1 if p["to"] == "last" else 1),),
        [Param("to", "text", "last", "'last' or 'first'")],
        help="Reorder a quaternion between scalar-first and scalar-last."),
    ConversionType(
        "select", "Select", "Routing", ["u"], ["y"], _select,
        [Param("indices", "text", "0,1,2", "e.g. 0,1,2 or 0:3")],
        "Pick elements of a vector."),
    ConversionType(
        "place", "Place into array", "Routing", ["u"], ["y"], _place,
        [Param("length", "int", 36, "output length"),
         Param("offset", "int", 0, "first index written")],
        "Write a vector into a zero array (e.g. 3 torques into motorTorque[36])."),
    ConversionType(
        "mux", "Mux", "Routing", [], ["y"],
        lambda p, *u: (np.concatenate([np.atleast_1d(v) for v in u]),),
        [Param("inputs", "int", 2, "number of inputs")],
        "Concatenate inputs into one vector.", variable_inputs="inputs"),
    ConversionType(
        "cast", "Cast", "Math", ["u"], ["y"],
        lambda p, u: (np.asarray(u).astype(p["dtype"]),),
        [Param("dtype", "dtype", "float64", "numpy type, e.g. uint8")],
        "Change the element type."),
    ConversionType(
        "constant", "Constant", "Sources", [], ["y"],
        lambda p: (np.asarray(eval(str(p["value"]), {"__builtins__": {}},
                                   {"np": np})),),
        [Param("value", "text", "0.0", "number or list, e.g. [0, 0, 1]")],
        "A fixed value."),
    ConversionType(
        "expression", "Expression", "Custom", [], ["y"], _expression,
        [Param("inputs", "int", 1, "number of inputs u1..un"),
         Param("expression", "text", "u", "numpy expression in u1, u2...")],
        "Any numpy expression of the inputs (u is u1).",
        variable_inputs="inputs"),
    ConversionType(
        "python_function", "Python function", "Custom", [], ["y"],
        _python_function,
        [Param("inputs", "int", 1, "number of inputs"),
         Param("function", "text", "module:function",
               "importable module:function, called with the inputs")],
        "Call a function from the simulator code, e.g. a sensor or actuator "
        "model.", variable_inputs="inputs"),
]}


def infer_outputs(type_key, params, input_specs):
    """Output ``[(dtype, shape)]`` of a block, from its input specs.

    Runs the block once on zero arrays.  Returns None when an input is
    unknown or the block cannot run (bad parameter, import error); the
    error text is returned as the second value.
    """
    block = TYPES[type_key]
    if any(spec is None for spec in input_specs):
        return None, "an input is not connected"
    try:
        zeros = [np.zeros(shape, dtype=_numpy_dtype(dtype))
                 for dtype, shape in input_specs]
        outputs = block.function(params, *zeros)
    except Exception as error:      # surfaced in the Problems panel
        return None, f"{type(error).__name__}: {error}"
    return [(_dtype_label(np.asarray(o).dtype), tuple(np.shape(o)))
            for o in outputs], ""


def _numpy_dtype(label):
    if label in ("int", "unknown", "pointer"):
        return np.int64 if label == "int" else np.float64
    if label == "char":
        return np.uint8
    return np.dtype(label)


def _dtype_label(dtype):
    return "bool" if dtype == np.bool_ else str(dtype)
