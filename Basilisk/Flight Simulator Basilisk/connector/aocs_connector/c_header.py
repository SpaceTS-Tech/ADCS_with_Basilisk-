"""Read struct layouts out of generated C headers -- no compiler needed.

Simulink-generated headers include files that do not ship with the generated
code (``tmwtypes.h``, ``rt_logging.h``...), so they cannot be handed straight
to a C parser.  Instead this module:

1. strips comments and preprocessor lines from the headers,
2. splits what is left into top-level declarations, keeping only type
   declarations (``typedef`` / ``struct`` / ``union`` / ``enum``) and dropping
   repeats left behind by include guards,
3. parses them with :mod:`pycparser`, declaring any type it does not know as
   an opaque struct and retrying -- those are only ever used behind pointers
   in structures we do not need (``RTWLogInfo *`` in the real-time model),
4. turns the structs into :mod:`ctypes` structures, which gives exact field
   offsets and sizes with the platform's own alignment rules.

The Simulink base types (``real_T``, ``uint8_T``...) and the ``<stdint.h>``
names are known up front, because their defining header is part of MATLAB, not
of the generated code.
"""

import ctypes
import re
from dataclasses import dataclass, field

from pycparser import c_ast, c_parser
from pycparser.c_parser import ParseError

#: Names whose ctypes type is fixed regardless of what the headers say.
BASE_TYPES = {
    # C keywords (normalised word order, see _normalise_identifier)
    "double": ctypes.c_double, "float": ctypes.c_float,
    "long double": ctypes.c_longdouble,
    "char": ctypes.c_char, "signed char": ctypes.c_byte,
    "unsigned char": ctypes.c_ubyte,
    "short": ctypes.c_short, "unsigned short": ctypes.c_ushort,
    "int": ctypes.c_int, "unsigned int": ctypes.c_uint,
    "long": ctypes.c_long, "unsigned long": ctypes.c_ulong,
    "long long": ctypes.c_longlong, "unsigned long long": ctypes.c_ulonglong,
    "_Bool": ctypes.c_bool, "void": None,
    # <stdint.h>, <stdbool.h>, <stddef.h>
    "bool": ctypes.c_bool,
    "int8_t": ctypes.c_int8, "uint8_t": ctypes.c_uint8,
    "int16_t": ctypes.c_int16, "uint16_t": ctypes.c_uint16,
    "int32_t": ctypes.c_int32, "uint32_t": ctypes.c_uint32,
    "int64_t": ctypes.c_int64, "uint64_t": ctypes.c_uint64,
    "size_t": ctypes.c_size_t, "ptrdiff_t": ctypes.c_ssize_t,
    # Simulink tmwtypes.h
    "real_T": ctypes.c_double, "real64_T": ctypes.c_double,
    "real32_T": ctypes.c_float, "time_T": ctypes.c_double,
    "int8_T": ctypes.c_int8, "uint8_T": ctypes.c_uint8,
    "int16_T": ctypes.c_int16, "uint16_T": ctypes.c_uint16,
    "int32_T": ctypes.c_int32, "uint32_T": ctypes.c_uint32,
    "int64_T": ctypes.c_int64, "uint64_T": ctypes.c_uint64,
    "boolean_T": ctypes.c_bool,
    "char_T": ctypes.c_char, "uchar_T": ctypes.c_ubyte,
    "byte_T": ctypes.c_char,
    "int_T": ctypes.c_int, "uint_T": ctypes.c_uint,
    "ulong_T": ctypes.c_ulong, "ulonglong_T": ctypes.c_ulonglong,
    "pointer_T": ctypes.c_void_p,
}

#: numpy-style element name for each ctypes scalar.
_DTYPE_NAMES = {
    ctypes.c_double: "float64", ctypes.c_float: "float32",
    ctypes.c_bool: "bool", ctypes.c_char: "char",
    ctypes.c_void_p: "pointer",
}


def dtype_name(ctype):
    """numpy-style name of a ctypes scalar (``c_int32`` -> ``int32``)."""
    # c_longdouble is c_double where the platform makes them the same size
    # (MSVC/MinGW), so it is only told apart when it is genuinely wider.
    if ctype is ctypes.c_longdouble and             ctypes.sizeof(ctype) > ctypes.sizeof(ctypes.c_double):
        return "longdouble"
    if ctype in _DTYPE_NAMES:
        return _DTYPE_NAMES[ctype]
    if issubclass(ctype, ctypes._SimpleCData) and ctype._type_ in "bBhHiIlLqQnN":
        signed = ctype._type_.islower()
        return f"{'' if signed else 'u'}int{8 * ctypes.sizeof(ctype)}"
    raise TypeError(f"no dtype name for {ctype!r}")


# ---------------------------------------------------------------------------
# Text clean-up and chunking
# ---------------------------------------------------------------------------

_COMMENT = re.compile(r"/\*.*?\*/|//[^\n]*", re.S)
_TYPEDEF_NAME = re.compile(r"(\w+)\s*(?:\[[^\]]*\]\s*)*;\s*$")
_TAG_DEFINITION = re.compile(r"^(struct|union|enum)\s+(\w+)\s*\{")


def strip_preprocessor(text):
    """Remove comments, line continuations and ``#`` directives."""
    text = _COMMENT.sub(" ", text)
    text = text.replace("\\\n", " ")
    return "\n".join(line for line in text.splitlines()
                     if not line.lstrip().startswith("#"))


def split_declarations(text):
    """Split C text into top-level declarations (each ending in ``;``)."""
    chunks, depth, start = [], 0, 0
    for index, char in enumerate(text):
        if char in "{(":
            depth += 1
        elif char in "})":
            depth -= 1
        elif char == ";" and depth == 0:
            chunk = " ".join(text[start:index + 1].split())
            if chunk != ";":
                chunks.append(chunk)
            start = index + 1
    return chunks


def type_declarations(texts):
    """Keep the type declarations of several header texts, de-duplicated.

    Include guards normally stop a typedef from being seen twice; with the
    preprocessor gone that has to be done here.  The first definition wins.
    """
    seen_typedefs, seen_tags, kept = set(BASE_TYPES), set(), []
    for text in texts:
        for chunk in split_declarations(strip_preprocessor(text)):
            if not chunk.startswith(("typedef", "struct", "union", "enum")):
                continue
            if chunk.startswith("typedef"):
                match = _TYPEDEF_NAME.search(chunk)
                if match is None:          # function-pointer typedef etc.
                    continue
                if match.group(1) in seen_typedefs:
                    continue
                seen_typedefs.add(match.group(1))
            tag = _TAG_DEFINITION.match(chunk.removeprefix("typedef ").lstrip())
            if tag is not None:
                key = (tag.group(1), tag.group(2))
                if key in seen_tags:
                    continue
                seen_tags.add(key)
            kept.append(chunk)
    return kept


_C_WORDS = {
    "const", "volatile", "static", "extern", "register", "restrict", "inline",
    "signed", "unsigned", "char", "short", "int", "long", "float", "double",
    "void", "_Bool", "struct", "union", "enum", "typedef", "sizeof",
}
_MEMBER_TYPE = re.compile(r"^([A-Za-z_]\w*)[\s*(]+[A-Za-z_*(]")


def declared_typedef_names(chunks):
    """Every name the chunks declare with ``typedef``."""
    return {m.group(1) for c in chunks if c.startswith("typedef")
            for m in [_TYPEDEF_NAME.search(c)] if m}


def unknown_type_names(chunks, known):
    """Type names the chunks use but nothing declares.

    These come from headers outside the package (``RTWLogInfo`` from
    ``rt_logging.h``...).  Declaring them opaque lets the rest parse; laying
    out a struct that holds one by value then fails, which only matters if
    that struct is actually needed.
    """
    unknown = set()
    for chunk in chunks:
        if "{" in chunk:
            body = chunk[chunk.find("{") + 1:chunk.rfind("}")]
        else:
            body = chunk.removeprefix("typedef")
        for member in re.split(r"[;{}]", body):
            words = member.split()
            while words and words[0] in ("const", "volatile", "typedef",
                                         "static", "extern"):
                words.pop(0)
            if not words or words[0] in _C_WORDS:
                continue
            match = _MEMBER_TYPE.match(" ".join(words))
            if match and match.group(1) not in known:
                unknown.add(match.group(1))
    return sorted(unknown)


def _preamble(names, opaque):
    # pycparser only needs to know these are type names; their real layout
    # comes from BASE_TYPES.
    lines = [f"typedef int {name};" for name in sorted(names)
             if name.isidentifier() and name not in _C_WORDS]
    lines += [f"typedef struct __opaque_{name} {name};" for name in opaque]
    return "\n".join(lines)


def parse_declarations(chunks):
    """Parse type declarations with pycparser, tolerating unknown types.

    Returns ``(ast, opaque_names, dropped_chunks)``.
    """
    chunks = list(chunks)
    known = set(BASE_TYPES) | declared_typedef_names(chunks)
    opaque = unknown_type_names(chunks, known)
    header = _preamble(BASE_TYPES, opaque)
    parser = c_parser.CParser()
    try:
        return parser.parse("\n".join([header, *chunks]), "<headers>"), \
            opaque, []
    except ParseError:
        pass
    # Something else is unparsable.  Find which declarations fail on their
    # own (every type name pre-declared) and leave them out.
    names_only = _preamble(known, opaque)
    dropped = []
    for chunk in chunks:
        try:
            parser.parse(names_only + "\n" + chunk, "<chunk>")
        except ParseError:
            dropped.append(chunk)
    kept = [c for c in chunks if c not in dropped]
    return parser.parse("\n".join([header, *kept]), "<headers>"), \
        opaque, dropped


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

@dataclass
class Leaf:
    """A scalar or array-of-scalar field flattened out of a struct."""

    path: str
    ctype: object
    shape: tuple
    offset: int

    @property
    def dtype(self):
        return dtype_name(self.ctype)

    @property
    def nbytes(self):
        count = 1
        for dim in self.shape:
            count *= dim
        return count * ctypes.sizeof(self.ctype)


@dataclass
class HeaderTypes:
    """Every type declared in a set of headers, resolvable to ctypes."""

    typedefs: dict = field(default_factory=dict)
    tags: dict = field(default_factory=dict)
    opaque: list = field(default_factory=list)
    dropped: list = field(default_factory=list)
    _cache: dict = field(default_factory=dict)

    @classmethod
    def from_texts(cls, texts):
        ast, opaque, dropped = parse_declarations(type_declarations(texts))
        types = cls(opaque=opaque, dropped=dropped)
        for node in ast.ext:
            if isinstance(node, c_ast.Typedef):
                types.typedefs[node.name] = node.type
            elif isinstance(node, c_ast.Decl):
                types._register_tag(node.type)
        for node in types.typedefs.values():
            types._register_tag(getattr(node, "type", None))
        return types

    def _register_tag(self, node):
        """Remember a struct/union definition so tag references resolve."""
        if isinstance(node, (c_ast.Struct, c_ast.Union)) and node.name and \
                node.decls is not None:
            self.tags[(type(node).__name__, node.name)] = node

    # -- resolution ---------------------------------------------------------

    def ctype(self, name):
        """ctypes type of a typedef or base type name."""
        if name in BASE_TYPES:
            return BASE_TYPES[name]
        if name in self._cache:
            return self._cache[name]
        if name not in self.typedefs:
            raise KeyError(f"unknown type {name!r}")
        resolved = self._resolve(self.typedefs[name], hint=name)
        self._cache[name] = resolved
        return resolved

    def _resolve(self, node, hint="anon"):
        if isinstance(node, c_ast.TypeDecl):
            return self._resolve(node.type, hint)
        if isinstance(node, c_ast.IdentifierType):
            return self.ctype(_normalise_identifier(node.names))
        if isinstance(node, c_ast.PtrDecl):
            return ctypes.c_void_p
        if isinstance(node, c_ast.ArrayDecl):
            return self._resolve(node.type, hint) * _eval_dim(node.dim)
        if isinstance(node, c_ast.Enum):
            return ctypes.c_int
        if isinstance(node, (c_ast.Struct, c_ast.Union)):
            return self._struct(node, hint)
        raise TypeError(f"cannot lay out {type(node).__name__} ({hint})")

    def _struct(self, node, hint):
        kind = type(node).__name__
        if node.decls is None:                      # reference to a tag
            key = (kind, node.name)
            if key not in self.tags:
                raise TypeError(f"{kind.lower()} {node.name} is opaque "
                                f"(its definition is not in the headers)")
            node = self.tags[key]
        cache_key = (kind, node.name or id(node))
        if cache_key in self._cache:
            return self._cache[cache_key]
        base = ctypes.Structure if kind == "Struct" else ctypes.Union
        cls = type(node.name or hint, (base,), {})
        self._cache[cache_key] = cls                # allows self-reference
        cls._fields_ = [(decl.name, self._resolve(decl.type, decl.name))
                        for decl in node.decls]
        return cls


def _normalise_identifier(words):
    """Canonical spelling of a multi-word C type (``long unsigned int``...)."""
    words = list(words)
    if len(words) == 1 and words[0] not in ("signed", "unsigned"):
        return words[0]
    unsigned = "unsigned" in words
    signed = "signed" in words
    words = [w for w in words if w not in ("signed", "unsigned")]
    longs = words.count("long")
    if "char" in words:
        if unsigned:
            return "unsigned char"
        return "signed char" if signed else "char"
    if "short" in words:
        base = "short"
    elif "double" in words:
        return "long double" if longs else "double"
    elif longs == 2:
        base = "long long"
    elif longs == 1:
        base = "long"
    else:
        base = "int"
    return f"unsigned {base}" if unsigned else base


def _eval_dim(node):
    if isinstance(node, c_ast.Constant):
        return int(node.value.rstrip("uUlL"), 0)
    if isinstance(node, c_ast.BinaryOp):
        left, right = _eval_dim(node.left), _eval_dim(node.right)
        return {"*": left * right, "+": left + right, "-": left - right,
                "/": left // right}[node.op]
    if isinstance(node, c_ast.UnaryOp) and node.op == "sizeof":
        raise TypeError("sizeof() array dimensions are not supported")
    raise TypeError(f"cannot evaluate array dimension {node!r}")


def flatten(struct_type, prefix="", base_offset=0, max_struct_array=64):
    """Yield a :class:`Leaf` for every scalar/array field of a ctypes struct.

    Nested structs become dotted paths (``bus.field``); arrays of structs are
    expanded element by element up to ``max_struct_array``.
    """
    for name, field_type in struct_type._fields_:
        offset = base_offset + getattr(struct_type, name).offset
        path = f"{prefix}{name}"
        shape = []
        element = field_type
        while issubclass(element, ctypes.Array):
            shape.append(element._length_)
            element = element._type_
        if issubclass(element, (ctypes.Structure, ctypes.Union)):
            if not shape:
                yield from flatten(element, path + ".", offset)
                continue
            count = 1
            for dim in shape:
                count *= dim
            for index in range(min(count, max_struct_array)):
                yield from flatten(element, f"{path}[{index}].",
                                   offset + index * ctypes.sizeof(element))
            continue
        yield Leaf(path, element, tuple(shape), offset)
