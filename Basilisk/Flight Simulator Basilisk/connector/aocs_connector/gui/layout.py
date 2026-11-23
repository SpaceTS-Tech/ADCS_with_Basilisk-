"""Automatic block placement, Simulink style.

Columns run left to right in signal order::

    sensor modules | conversions ... | SOFTWARE | ... conversions | actuator modules

A conversion block's column is its distance (in blocks) from the software, so
chains such as ``limit -> voltage_to_dipole`` spread out instead of stacking.
Within a column, blocks are ordered by the software port their signal
reaches and placed level with it where space allows, which keeps most wires
nearly horizontal.
"""

from aocs_connector.gui.canvas import HEADER, ROW


def arrange(specs, wires, owner_key, software_rows, measure):
    """Return ``{node key: (x, y)}``.

    specs          -- NodeSpec list
    wires          -- [(source endpoint, target endpoint)]
    owner_key      -- endpoint -> node key drawn on the canvas, or None
    software_rows  -- software endpoint -> row index on the software block
    measure        -- NodeSpec -> (width, height)
    """
    keys = [s.key for s in specs]
    edges = []
    for source, target in wires:
        a, b = owner_key(source), owner_key(target)
        if a in keys and b in keys:
            edges.append((a, b, source, target))
    conversions = {s.key for s in specs if s.kind == "conversion"}

    def distances(forward):
        """Steps from the software block along (or against) the wires."""
        dist, frontier = {"sw": 0}, ["sw"]
        while frontier:
            nxt = []
            for key in frontier:
                for a, b, _, _ in edges:
                    origin, other = (a, b) if forward else (b, a)
                    if origin == key and other in conversions and \
                            other not in dist:
                        dist[other] = dist[key] + 1
                        nxt.append(other)
            frontier = nxt
        dist.pop("sw")
        return dist

    left, right = distances(False), distances(True)
    column = {"sw": 0}
    for key in conversions:
        if key in left:
            column[key] = -left[key]
        elif key in right:
            column[key] = right[key]
        else:
            column[key] = -1
    depth_left = max([-c for c in column.values() if c < 0] or [0])
    depth_right = max([c for c in column.values() if c > 0] or [0])
    for spec in specs:
        if spec.kind != "module":
            continue
        sources = any(a == spec.key for a, *_ in edges)
        sinks = any(b == spec.key for _, b, *_ in edges)
        column[spec.key] = depth_right + 1 if sinks and not sources \
            else -(depth_left + 1)

    memo = {}

    def anchor(key, seen=()):
        """Row of the software port this block's signal is tied to."""
        if key in memo:
            return memo[key]
        if key in seen:
            return 999
        rows = []
        for a, b, source, target in edges:
            if column.get(key, 0) < 0 and a == key:
                rows.append(software_rows[target] if b == "sw"
                            else anchor(b, seen + (key,)))
            if column.get(key, 0) > 0 and b == key:
                rows.append(software_rows[source] if a == "sw"
                            else anchor(a, seen + (key,)))
        memo[key] = min(rows) if rows else 999
        return memo[key]

    sizes = {s.key: measure(s) for s in specs}
    by_column = {}
    for key in keys:
        by_column.setdefault(column.get(key, 0), []).append(key)

    gap = 110.0
    x_of, x = {}, 0.0
    for col in range(0, depth_right + 2):
        x_of[col] = x
        x += max((sizes[k][0] for k in by_column.get(col, [])), default=0) + gap
    x = 0.0
    for col in range(-1, -depth_left - 2, -1):
        width = max((sizes[k][0] for k in by_column.get(col, [])), default=0)
        x -= width + gap
        x_of[col] = x

    software_top = 0.0
    positions = {}
    for col, members in by_column.items():
        members.sort(key=lambda k: (anchor(k), k))
        # ``None`` until the first block is placed.  Blocks not tied to a
        # software port (all of them when no software is loaded) stack from
        # the software block's top edge -- never from a sentinel coordinate,
        # which once put every block a billion pixels up and froze the view.
        y = None
        for key in members:
            if key == "sw":
                target = software_top
            else:
                row = anchor(key)
                if row < 999:
                    target = software_top + HEADER + row * ROW - 30
                else:
                    target = software_top if y is None else y
            y = target if y is None else max(y, target)
            positions[key] = (x_of[col], y)
            y += sizes[key][1] + 28
    return positions

