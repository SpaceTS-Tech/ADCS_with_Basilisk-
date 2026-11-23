"""Build the flight-software block from a Simulink Coder (GRT) package.

The user points at either the generated-code folder or the DLL inside it.  The
ports come from the generated headers, never from a hand-kept list:

==================  ===============================================
Generated struct    Becomes
==================  ===============================================
``ExtU_<model>_T``  block inputs  (root inports)
``ExtY_<model>_T``  block outputs (root outports)
``P_<model>_T``     ``meta["parameters"]`` -- tunable, set before a run
``B_<model>_T``     ``meta["signals"]``    -- internal, read-only probes
==================  ===============================================

Nested bus structs are flattened to dotted port ids (``bus.field``).  Every
port records its byte offset, so the runtime can bind it straight to the DLL's
global (``<model>_U`` / ``<model>_Y``) with ctypes.

:func:`check_library` then loads the DLL and confirms the entry points and
data symbols exist, and -- when the DLL exports its own struct sizes -- that
the layout read from the headers matches the compiled one.
"""

import ctypes
import re
from pathlib import Path

from aocs_connector import c_header
from aocs_connector.ports import Block, Port

LIBRARY_SUFFIXES = (".dll", ".so", ".dylib")

_EXTU_TYPEDEF = re.compile(r"\}\s*ExtU_(\w+)_T\s*;")
_LOCAL_INCLUDE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.M)
_ROOT_PORT_COMMENT = re.compile(
    r"\b(\w+)\s*(?:\[[^\]]*\]\s*)*;\s*/\*\s*'<Root>/([^']*)'")
_STEP_SIZE = re.compile(r"Timing\.stepSize0\s*=\s*([-+0-9.eE]+)\s*;")
_SAMPLE_TIME_COMMENT = re.compile(r"Sample time:\s*\[\s*([-+0-9.eE]+)s")

#: Exported helpers some packages provide that report ``sizeof`` of the I/O
#: structs.  ``{model}`` is replaced by the model name.  The ``ADCS_*`` names
#: are the ones adcs_basilisk_wrapper.c exports today.
SIZE_FUNCTIONS = {
    "inputs": ("{model}_get_inputs_size", "ADCS_get_inputs_size"),
    "outputs": ("{model}_get_outputs_size", "ADCS_get_outputs_size"),
    "signals": ("{model}_get_block_signals_size",
                "ADCS_get_block_signals_size"),
}


class SoftwarePackage:
    """A located generated-code package and the types read from it."""

    def __init__(self, path):
        self.path = Path(path).resolve()
        self.folder, self.model, self.library = _locate(self.path)
        self.header = self.folder / f"{self.model}.h"
        self.header_files = _header_closure(self.header)
        texts = [f.read_text(errors="replace") for f in self.header_files]
        self.types = c_header.HeaderTypes.from_texts(texts)
        self._header_text = self.header.read_text(errors="replace")

    # -- generated names ----------------------------------------------------

    def struct_type(self, kind):
        """ctypes structure for ``ExtU``/``ExtY``/``P``/``B``, or None."""
        name = f"{kind}_{self.model}_T"
        try:
            return self.types.ctype(name)
        except KeyError:
            return None

    def symbol(self, kind):
        """Name of the global holding a struct, from its ``extern`` line."""
        match = re.search(
            rf"extern\s+{kind}_{self.model}_T\s+(\w+)\s*;", self._header_text)
        return match.group(1) if match else f"{self.model}_{kind[-1]}"

    def entry_points(self):
        return {name: f"{self.model}_{name}"
                for name in ("initialize", "step", "terminate")}

    def sample_time(self):
        """Base step [s] from the generated source, or None if not found."""
        source = self.folder / f"{self.model}.c"
        if not source.is_file():
            return None
        text = source.read_text(errors="replace")
        match = _STEP_SIZE.search(text) or _SAMPLE_TIME_COMMENT.search(text)
        return float(match.group(1)) if match else None

    def root_port_names(self):
        """``{field: Simulink block name}`` from the header comments."""
        return {m.group(1): m.group(2)
                for m in _ROOT_PORT_COMMENT.finditer(self._header_text)}


def _locate(path):
    """Return ``(folder, model, library_or_None)`` for a folder or a DLL."""
    if path.is_file():
        if path.suffix.lower() not in LIBRARY_SUFFIXES + (".h",):
            raise ValueError(f"{path} is not a shared library or header")
        folder, hint = path.parent, path.stem
    elif path.is_dir():
        folder, hint = path, None
    else:
        raise FileNotFoundError(path)

    # A model's main header is exactly <model>.h; this skips copies such as
    # "<model>-Swapnil.h" that declare the same structs.
    models = []
    for header in sorted(folder.glob("*.h")):
        match = _EXTU_TYPEDEF.search(header.read_text(errors="replace"))
        if match and header.stem == match.group(1):
            models.append(match.group(1))
    if hint in models:
        model = hint
    elif len(models) == 1:
        model = models[0]
    elif not models:
        raise ValueError(f"no Simulink Coder model header (ExtU_<model>_T) "
                         f"found in {folder}")
    else:
        raise ValueError(f"several models in {folder}: {', '.join(models)}; "
                         f"select the DLL instead of the folder")

    library = path if path.is_file() and path.suffix.lower() in \
        LIBRARY_SUFFIXES else None
    if library is None:
        for suffix in LIBRARY_SUFFIXES:
            candidate = folder / f"{model}{suffix}"
            if candidate.is_file():
                library = candidate
                break
    return folder, model, library


def _header_closure(header, seen=None):
    """The header plus every local header it includes, in include order.

    Included headers come first, so types are declared before they are used.
    Headers that do not ship with the package (``tmwtypes.h``...) are skipped.
    """
    if seen is None:
        seen = set()
    if header in seen or not header.is_file():
        return []
    seen.add(header)
    ordered = []
    for name in _LOCAL_INCLUDE.findall(header.read_text(errors="replace")):
        ordered.extend(_header_closure(header.parent / name, seen))
    ordered.append(header)
    return ordered


def _ports(struct_type, direction, labels):
    ports = []
    for leaf in c_header.flatten(struct_type):
        top = leaf.path.split(".")[0].split("[")[0]
        ports.append(Port(
            id=leaf.path,
            direction=direction,
            dtype=leaf.dtype,
            shape=leaf.shape,
            group=top if "." in leaf.path else "",
            description=labels.get(top, ""),
            extra={"offset": leaf.offset, "nbytes": leaf.nbytes},
        ))
    return ports


def build_software_block(path, include_internals=True):
    """Read a generated-code folder (or its DLL) into a :class:`Block`."""
    package = SoftwarePackage(path)
    labels = package.root_port_names()
    inputs_type = package.struct_type("ExtU")
    outputs_type = package.struct_type("ExtY")
    if inputs_type is None or outputs_type is None:
        raise ValueError(f"{package.header} declares no ExtU/ExtY structs")

    meta = {
        "model": package.model,
        "folder": str(package.folder),
        "library": str(package.library) if package.library else None,
        "sample_time": package.sample_time(),
        "entry_points": package.entry_points(),
        "symbols": {"inputs": package.symbol("ExtU"),
                    "outputs": package.symbol("ExtY"),
                    "parameters": package.symbol("P"),
                    "signals": package.symbol("B")},
        "struct_sizes": {"inputs": ctypes.sizeof(inputs_type),
                         "outputs": ctypes.sizeof(outputs_type)},
        "headers": [f.name for f in package.header_files],
        "opaque_types": package.types.opaque,
    }
    if include_internals:
        for kind, key in (("P", "parameters"), ("B", "signals")):
            struct_type = _safe_struct(package, kind)
            if struct_type is None:
                continue
            meta["struct_sizes"][key] = ctypes.sizeof(struct_type)
            meta[key] = [
                {"id": leaf.path, "dtype": leaf.dtype, "shape": leaf.shape,
                 "offset": leaf.offset}
                for leaf in c_header.flatten(struct_type)]

    return Block(
        kind="software",
        name=package.model,
        source=str(package.path),
        inputs=_ports(inputs_type, "in", labels),
        outputs=_ports(outputs_type, "out", labels),
        meta=meta,
    )


def _safe_struct(package, kind):
    try:
        return package.struct_type(kind)
    except TypeError:          # contains an opaque type; not needed for I/O
        return None


def check_library(block):
    """Load the block's DLL and verify it matches the headers.

    Returns a list of ``(level, message)`` with level ``"ok"``, ``"warn"`` or
    ``"error"``.  Never raises for a mismatch -- the caller decides.
    """
    meta = block.meta
    if not meta.get("library"):
        return [("error", "no shared library found next to the headers")]
    try:
        library = ctypes.CDLL(meta["library"])
    except OSError as error:
        return [("error", f"cannot load {meta['library']}: {error}")]

    findings = []
    for role, symbol in {**meta["entry_points"], **meta["symbols"]}.items():
        level = "ok" if hasattr(library, symbol) else (
            "error" if role in ("initialize", "step", "inputs", "outputs")
            else "warn")
        findings.append((level, f"symbol {symbol} ({role})"
                         + ("" if level == "ok" else " not exported")))

    for key, names in SIZE_FUNCTIONS.items():
        expected = meta["struct_sizes"].get(key)
        if expected is None:
            continue
        for name in names:
            name = name.format(model=meta["model"])
            if hasattr(library, name):
                function = getattr(library, name)
                function.restype = ctypes.c_uint
                actual = int(function())
                level = "ok" if actual == expected else "error"
                findings.append((level, f"sizeof {key}: headers {expected} B, "
                                        f"DLL {actual} B ({name})"))
                break
        else:
            findings.append(("warn", f"sizeof {key}: DLL exports no size "
                                     f"function, layout not cross-checked"))
    return findings
