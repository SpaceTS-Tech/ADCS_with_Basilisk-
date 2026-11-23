# AOCS connector

A Simulink-style editor for binding the generated AOCS flight software to the
Basilisk simulator. Each side is drawn as blocks with ports; you wire them,
with conversion blocks in between, and save the wiring to a `.aocslink` file.
The simulator runs the saved file directly: **save (Ctrl+S), then run the
simulator** and the new wiring is used. Nothing in `ADCS_Software_v1_grt_rtw`
is edited or rebuilt.

## Start

Double-click **`Run_Connector.exe`**. It opens the editor with no console
window, on the last file you used, or else on
`links/ADCS_Software_v1_bridge.aocslink` — today's bridge wiring.

Requirements: Python 3 with Basilisk, and PySide6 (`pip install
PySide6-Essentials`; the app offers to install it if it is missing).

## Using the editor

| To | Do |
|---|---|
| Connect | Drag from a port to a port. The wire turns red while hovering over an incompatible port. |
| Convert | Drag a block from **Block library**, or right-click a wire → **Insert block**. |
| Edit a block | Select it; change its parameters in **Properties**. |
| See more of a message | Click its name on a simulator block (▸ / ▾). Hidden fields show as `(+n)`. |
| Add a simulator module | Tick it in **Simulator modules**. Modules with wires are always shown. |
| Delete | Select wires/blocks, press Delete. Undo with Ctrl+Z. |
| Navigate | Wheel zooms, right/middle drag pans, F fits, Ctrl+L auto-layout. |
| Change the software | **Select software DLL** (or folder). |
| Run without software | **No flight software** (toolbar / Sources). The simulator's actuator inputs stay at zero — no control torque or force. Saved as `"none": true` in the link file; wires are kept for when you switch back. |
| Pick up code changes | **Reload ports** (F5) re-reads both sides. New ports appear unconnected; removed ones become broken wires in **Problems**. |
| Save | Ctrl+S. Paths are stored relative to the file. The saved file becomes the simulator's wiring (`active_link.txt`). |

**Problems** lists everything that would make the binding wrong: shape
mismatches, unconnected block inputs, broken wires, software inputs left at
zero, simulator inputs the old bridge drove that nothing drives now, and a
software step that does not match the simulator task. Double-click an entry
to jump to it.

### Conversion blocks

| Category | Blocks |
|---|---|
| Attitude | MRP → Quaternion, Quaternion → MRP, Inertial → Body (MRP), Body → Inertial (MRP), quaternion scalar first ↔ last |
| Routing | Select (indices), Place into array, Mux |
| Math | Gain, Cast |
| Sources | Constant |
| Custom | Expression (numpy in `u1, u2…`), Python function (`module:function` from the simulator code) |

Each block is a small numpy function (`aocs_connector/conversions.py`). The
editor runs it once on zeros to find its output type, so wires are
type-checked before anything is simulated.

## Status

| Phase | What | State |
|---|---|---|
| 1 | Read the ports of both sides | done |
| 3 | Graphical editor, link file, validation | done |
| 2 | Runtime that **runs** a link file in Basilisk (`simulator/adcs_sim/flightsoftware/link_runtime.py`); the default bridge link reproduces the hand-written bridge exactly | done |
| 4 | Parameters (mission command, gains), camera-feed source block, optional C-wrapper export | later |

## How the simulator uses the link file

When the simulator starts it picks the link file in this order:
`--link FILE`, the `ADCS_LINK_FILE` environment variable, the file this editor
saved last (`active_link.txt`), then `links/ADCS_Software_v1_bridge.aocslink`.
`--no-link` (or `ADCS_LINK_FILE=none`) uses the hand-written bridge in
`controller.py` instead. The run banner prints which wiring is in use.

The flight software also comes from the link file — the selected DLL, or
**No flight software** — unless `--flight-software`, `--no-flight-software`
or `ADCS_LIBRARY` overrides it.

At start-up every wire is resolved against the built simulation and the DLL.
A broken wire, an unknown block, an unconnected block input or a loop stops
the run with the list of problems. Simulator inputs the old bridge drove but
the link leaves unconnected are held at zero.

Not wired through the link (still done by `controller.py`): the camera and
star tracker image feeds, and the parameters set at reset (mission command,
gains, `h_max`). The link file must use the same model the controller loads
(`ADCS_Software_v1`).

## How the ports are found

**Software** — from the generated headers, not a hand-kept list: `ExtU_<model>_T`
→ inputs, `ExtY_<model>_T` → outputs, with byte offsets from ctypes, so the
layout drift documented in `structs.py` cannot recur. `P` and `B` structs are
read as parameters and internal signals. The DLL is checked for its entry
points, globals and (when it exports them) struct sizes.

**Simulator** — the scenario (`adcs_sim.scenario.builder:build_simulation` by
default) is built, not run, and Basilisk's message graph is expanded into one
port per payload field. A model added under `adcs_sim/models` appears on the
next reload. The old bridge module is left out; its 9 connections are kept in
`meta["bridge_connections"]` for reference.

Basilisk's `GetMessageConnectionGraph` dedupes visited objects by `id()`,
which SWIG proxies reuse; it dropped the SPICE planet messages in about half
of all runs. `simulator_block.message_graph` works around it.

## Command line

```text
python inspect_ports.py [--software path] [--json out.json] [--parameters] [--internal]
python -m unittest discover tests -v
```

## Layout

```text
connector/
  Run_Connector.exe             double-click launcher (tools/launcher/GuiLauncher.cs)
  connector_app.py              editor entry point
  inspect_ports.py              console port listing
  links/                        saved wirings (.aocslink)
  aocs_connector/
    ports.py                    Port / Block
    c_header.py                 compiler-free C header -> ctypes layout
    software_block.py           Simulink Coder package -> software block
    simulator_block.py          Basilisk scenario -> simulator block
    conversions.py              conversion block library
    link.py                     link file model and validation
    bridge_default.py           today's bridge as a link document
    gui/                        canvas, window, auto layout
  tests/                        unittest suites
```

## Known limits

- Simulator integer fields show as `int`: SWIG hides their width.
- `Camerafeed` / `STR1` come from Vizard frames, not messages, so they cannot
  be wired yet (shown as warnings).
- GRT code layout is assumed (`<model>_U/_Y`, `<model>_step(void)`).
