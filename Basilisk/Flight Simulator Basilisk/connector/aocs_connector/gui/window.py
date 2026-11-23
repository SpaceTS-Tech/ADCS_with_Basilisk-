"""Main window of the AOCS connector editor."""

import json
import traceback
from pathlib import Path

from PySide6.QtCore import QMimeData, QSettings, Qt, QTimer
from PySide6.QtGui import QAction, QColor, QDrag, QKeySequence
from PySide6.QtWidgets import (QApplication, QDialog, QDialogButtonBox,
                               QDockWidget, QFileDialog, QFormLayout,
                               QInputDialog, QLabel, QLineEdit,
                               QListWidget, QListWidgetItem, QMainWindow,
                               QMenu, QMessageBox, QPushButton, QScrollArea,
                               QToolBar, QTreeWidget, QTreeWidgetItem,
                               QVBoxLayout, QWidget)

from aocs_connector import conversions
from aocs_connector.bridge_default import bridge_document
from aocs_connector.gui.layout import arrange
from aocs_connector.gui.canvas import (MIME_BLOCK, Canvas, CanvasView,
                                       NodeItem, NodeSpec, PortItem, Row,
                                       WireItem)
from aocs_connector.link import (SUFFIX, LinkDocument, check_types, owner,
                                 port_name, validate)
from aocs_connector.simulator_block import (DEFAULT_ENTRY,
                                            DEFAULT_FLIGHT_SOFTWARE_TAGS,
                                            DEFAULT_SIMULATOR_DIR,
                                            load_simulator_block)
from aocs_connector.software_block import build_software_block, check_library

CONNECTOR_DIR = Path(__file__).resolve().parents[2]
LINKS_DIR = CONNECTOR_DIR / "links"
DEFAULT_LINK = LINKS_DIR / "ADCS_Software_v1_bridge.aocslink"
#: The link file the simulator flies; rewritten on every save.  Read by
#: ``simulator/adcs_sim/config/paths.py:active_link_file``.
ACTIVE_LINK_POINTER = CONNECTOR_DIR / "active_link.txt"


def set_active_link(path):
    """Make ``path`` the wiring the simulator uses on its next run."""
    path = Path(path).resolve()
    try:
        text = path.relative_to(CONNECTOR_DIR.resolve()).as_posix()
    except ValueError:
        text = str(path)
    ACTIVE_LINK_POINTER.write_text(text + "\n", encoding="utf-8")
DEFAULT_SOFTWARE = CONNECTOR_DIR.parents[1] / "ADCS_Software_v1_grt_rtw"
FILE_FILTER = f"AOCS link (*{SUFFIX})"

COLORS = {
    "software": QColor("#2563a8"),
    "module": QColor("#2f7d55"),
    "Attitude": QColor("#7048b8"),
    "Routing": QColor("#1f8a8a"),
    "Math": QColor("#b7741a"),
    "Sources": QColor("#8a6d1f"),
    "Custom": QColor("#5b6675"),
}
LEVEL_ICONS = {"error": "⛔", "warning": "⚠", "info": "ℹ"}


#: Block coordinates beyond this are corrupt (an old auto-layout bug wrote
#: -1e9); a file holding any is re-laid out on open instead of drawn.
MAX_SANE_COORDINATE = 1e6


def _has_runaway_positions(doc):
    points = [n.pos for n in doc.nodes.values()] + list(doc.positions.values())
    return any(abs(c) > MAX_SANE_COORDINATE for point in points
               for c in point)


class ConnectorWindow(QMainWindow):
    def __init__(self, path=None):
        super().__init__()
        self.settings = QSettings("SpaceTS", "AOCS Connector")
        self.doc = LinkDocument()
        self.software = None
        self.simulator = None
        self.load_errors = []
        self.report = None
        self.modified = False
        self.undo_stack, self.redo_stack = [], []
        self._pending_snapshot = None
        self._rebuild_scheduled = False

        self.canvas = Canvas(self)
        self.view = CanvasView(self.canvas, self)
        self.setCentralWidget(self.view)
        self.canvas.selectionChanged.connect(self.update_properties)
        self._build_actions()
        self._build_docks()
        self.resize(1500, 900)

        start = path or self.settings.value("last_file", "")
        if start and Path(start).is_file():
            self.open_path(start)
        elif DEFAULT_LINK.is_file():
            self.open_path(DEFAULT_LINK)
        else:
            self.new_from_bridge()

    # ======================================================================
    # UI construction
    # ======================================================================

    def _action(self, text, slot, shortcut=None, tip=""):
        action = QAction(text, self)
        action.triggered.connect(slot)
        if shortcut:
            action.setShortcut(QKeySequence(shortcut))
        if tip:
            action.setToolTip(tip)
            action.setStatusTip(tip)
        return action

    def _build_actions(self):
        a = self._action
        self.actions = {
            "new_bridge": a("New from current bridge", self.new_from_bridge,
                            tip="Start from the wiring adcs_sim uses today"),
            "new_empty": a("New empty", self.new_empty,
                           tip="Start with no wires"),
            "open": a("Open…", self.open_dialog, "Ctrl+O"),
            "save": a("Save", self.save, "Ctrl+S"),
            "save_as": a("Save as…", self.save_as, "Ctrl+Shift+S"),
            "undo": a("Undo", self.undo, "Ctrl+Z"),
            "redo": a("Redo", self.redo, "Ctrl+Y"),
            "software_dll": a("Select software DLL…", self.select_software_dll,
                              tip="Pick the generated flight-software DLL"),
            "software_dir": a("Select software folder…",
                              self.select_software_folder,
                              tip="Pick the generated-code folder"),
            "no_software": a("No flight software", self.toggle_no_software,
                             tip="Run the simulator without any ADCS "
                                 "software: no control torque or force"),
            "simulator": a("Simulator settings…", self.simulator_settings),
            "reload": a("Reload ports", self.reload, "F5",
                        "Re-read software and simulator; changed ports show up"),
            "layout": a("Auto layout", self.auto_layout, "Ctrl+L"),
            "fit": a("Fit to window", self.view.fit, "F"),
            "delete": a("Delete selected", self.delete_selection),
            "rotate_cw": a("Rotate block", lambda: self.rotate_selection(90),
                           "Ctrl+R", "Rotate the selected blocks 90° "
                                     "clockwise"),
            "rotate_ccw": a("Rotate block counter-clockwise",
                            lambda: self.rotate_selection(-90),
                            "Ctrl+Shift+R", "Rotate the selected blocks 90° "
                                            "counter-clockwise"),
        }
        menu = self.menuBar()
        file_menu = menu.addMenu("&File")
        for key in ("new_bridge", "new_empty", "open", "save", "save_as"):
            file_menu.addAction(self.actions[key])
        file_menu.addSeparator()
        file_menu.addAction(self._action("Exit", self.close))
        edit_menu = menu.addMenu("&Edit")
        for key in ("undo", "redo", "delete", "rotate_cw", "rotate_ccw"):
            edit_menu.addAction(self.actions[key])
        sources = menu.addMenu("&Sources")
        self.actions["no_software"].setCheckable(True)
        for key in ("software_dll", "software_dir", "no_software",
                    "simulator", "reload"):
            sources.addAction(self.actions[key])
        view_menu = menu.addMenu("&View")
        for key in ("layout", "fit"):
            view_menu.addAction(self.actions[key])
        self.view_menu = view_menu

        bar = QToolBar("Main")
        bar.setMovable(False)
        bar.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self.addToolBar(bar)
        for key in ("open", "save", None, "undo", "redo", None,
                    "software_dll", "no_software", "simulator", "reload",
                    None,
                    "rotate_cw", "layout", "fit"):
            if key is None:
                bar.addSeparator()
            else:
                bar.addAction(self.actions[key])

        self.status = QLabel()
        self.statusBar().addPermanentWidget(self.status)

    def _build_docks(self):
        # Library of conversion blocks
        self.library = QTreeWidget()
        self.library.setHeaderHidden(True)
        self.library.setDragEnabled(True)
        categories = {}
        for block in conversions.TYPES.values():
            parent = categories.get(block.category)
            if parent is None:
                parent = QTreeWidgetItem([block.category])
                parent.setFlags(Qt.ItemIsEnabled)
                self.library.addTopLevelItem(parent)
                categories[block.category] = parent
            item = QTreeWidgetItem([block.label])
            item.setData(0, Qt.UserRole, block.key)
            item.setToolTip(0, block.help)
            parent.addChild(item)
        self.library.expandAll()
        self.library.startDrag = self._library_drag
        self.library.itemDoubleClicked.connect(self._library_double_click)
        library_dock = self._dock("Block library", self.library,
                                  Qt.LeftDockWidgetArea)
        library_hint = QLabel("Drag a block onto the diagram, or "
                              "double-click it.")
        library_hint.setWordWrap(True)

        # Simulator modules
        self.modules = QListWidget()
        self.modules.itemChanged.connect(self._module_toggled)
        self._dock("Simulator modules", self.modules, Qt.LeftDockWidgetArea)

        # Properties
        self.properties = QScrollArea()
        self.properties.setWidgetResizable(True)
        self.properties_dock = self._dock("Properties", self.properties,
                                          Qt.RightDockWidgetArea)

        # Problems
        self.problems = QTreeWidget()
        self.problems.setHeaderLabels(["", "Problem"])
        self.problems.setRootIsDecorated(False)
        self.problems.setColumnWidth(0, 30)
        self.problems.itemDoubleClicked.connect(self._problem_clicked)
        self.problems_dock = self._dock("Problems", self.problems,
                                        Qt.BottomDockWidgetArea)
        self.resizeDocks([library_dock], [230], Qt.Horizontal)
        self.resizeDocks([self.properties_dock], [320], Qt.Horizontal)
        self.resizeDocks([self.problems_dock], [150], Qt.Vertical)

    def _dock(self, title, widget, area):
        dock = QDockWidget(title, self)
        dock.setObjectName(title)
        dock.setWidget(widget)
        dock.setFeatures(QDockWidget.DockWidgetMovable |
                         QDockWidget.DockWidgetFloatable)
        self.addDockWidget(area, dock)
        return dock

    def _library_drag(self, actions):
        item = self.library.currentItem()
        key = item.data(0, Qt.UserRole) if item else None
        if not key:
            return
        mime = QMimeData()
        mime.setData(MIME_BLOCK, key.encode())
        drag = QDrag(self.library)
        drag.setMimeData(mime)
        drag.exec(Qt.CopyAction)

    def _library_double_click(self, item):
        key = item.data(0, Qt.UserRole)
        if key:
            center = self.view.mapToScene(self.view.viewport().rect().center())
            self.add_block(key, (center.x(), center.y()))

    # ======================================================================
    # Documents
    # ======================================================================

    def new_from_bridge(self):
        if not self._confirm_discard():
            return
        doc = bridge_document(DEFAULT_SOFTWARE, DEFAULT_SIMULATOR_DIR)
        self._set_document(doc, layout=True)

    def new_empty(self):
        if not self._confirm_discard():
            return
        doc = LinkDocument(
            software_path=self.doc.software_path or str(DEFAULT_SOFTWARE),
            no_software=self.doc.no_software,
            simulator=dict(self.doc.simulator) or
            {"entry": DEFAULT_ENTRY,
             "simulator_dir": str(DEFAULT_SIMULATOR_DIR)})
        self._set_document(doc, layout=True)

    def open_dialog(self):
        if not self._confirm_discard():
            return
        start = str(Path(self.doc.path).parent) if self.doc.path \
            else str(LINKS_DIR)
        path, _ = QFileDialog.getOpenFileName(self, "Open link file", start,
                                              FILE_FILTER)
        if path:
            self.open_path(path)

    def open_path(self, path):
        try:
            doc = LinkDocument.load(path)
        except Exception as error:
            QMessageBox.critical(self, "Cannot open", f"{path}\n\n{error}")
            return
        self.settings.setValue("last_file", str(Path(path).resolve()))
        self._set_document(doc, layout="sw" not in doc.positions
                           or _has_runaway_positions(doc))

    def _set_document(self, doc, layout=False):
        self.doc = doc
        self.undo_stack.clear()
        self.redo_stack.clear()
        self.load_blocks()
        self.validate_and_show()
        if layout:
            self.auto_layout(record=False)
        self.modified = False
        self.rebuild()
        QTimer.singleShot(0, self.view.fit)

    def save(self):
        if not self.doc.path:
            return self.save_as()
        return self._write(self.doc.path)

    def save_as(self):
        start = self.doc.path or str(LINKS_DIR / f"new{SUFFIX}")
        path, _ = QFileDialog.getSaveFileName(self, "Save link file", start,
                                              FILE_FILTER)
        if not path:
            return False
        if not path.endswith(SUFFIX):
            path += SUFFIX
        return self._write(path)

    def _write(self, path):
        signatures = {}
        if self.software:
            signatures["software"] = self.software.signature()
        if self.simulator:
            signatures["simulator"] = self.simulator.signature()
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self.doc.save(path, signatures)
        except Exception as error:
            QMessageBox.critical(self, "Cannot save", f"{path}\n\n{error}")
            return False
        self.settings.setValue("last_file", str(Path(path).resolve()))
        try:
            set_active_link(path)
            active = True
        except OSError:
            active = False
        self.modified = False
        self.validate_and_show()
        errors = self.report.counts()["error"] if self.report else 0
        if not active:
            note = " — could not update active_link.txt; the simulator " \
                   "may run another file"
        elif errors:
            note = (f" — {errors} problem{'s' * (errors != 1)}: the simulator "
                    f"will refuse to run this wiring until they are fixed")
        else:
            note = " — the simulator uses this wiring on its next run"
        self.statusBar().showMessage(f"Saved {path}{note}", 8000)
        return True

    def _confirm_discard(self):
        if not self.modified:
            return True
        answer = QMessageBox.question(
            self, "Unsaved changes", "Save changes to the current wiring?",
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
        if answer == QMessageBox.Save:
            return bool(self.save())
        return answer == QMessageBox.Discard

    def closeEvent(self, event):
        if self._confirm_discard():
            event.accept()
        else:
            event.ignore()

    # ======================================================================
    # Reading the two sides
    # ======================================================================

    def load_blocks(self):
        self.load_errors = []
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.statusBar().showMessage("Reading flight software…")
            QApplication.processEvents()
            self.actions["no_software"].setChecked(self.doc.no_software)
            try:
                self.software = (None if self.doc.no_software else
                                 build_software_block(self.doc.software_path))
            except Exception as error:
                self.software = None
                self.load_errors.append(
                    f"Software not loaded ({self.doc.software_path or 'none'}): "
                    f"{error}")

            self.statusBar().showMessage(
                "Reading the simulator's ports…")
            QApplication.processEvents()
            sim = self.doc.simulator
            tags = DEFAULT_FLIGHT_SOFTWARE_TAGS
            if self.software:
                tags = (self.software.name, *tags)
            try:
                # From simulator/simulator_ports.json when it is current;
                # the scenario is only built when the simulator code changed.
                self.simulator, _ = load_simulator_block(
                    entry=sim.get("entry", DEFAULT_ENTRY),
                    simulator_dir=sim.get("simulator_dir",
                                          DEFAULT_SIMULATOR_DIR),
                    kwargs=sim.get("kwargs"),
                    flight_software_tags=tags)
            except Exception as error:
                self.simulator = None
                self.load_errors.append(
                    f"Simulator not loaded: {type(error).__name__}: {error}")
                traceback.print_exc()
        finally:
            QApplication.restoreOverrideCursor()
            self.statusBar().clearMessage()

    def reload(self):
        self.load_blocks()
        self.rebuild()

    def select_software_dll(self):
        start = str(Path(self.doc.software_path or DEFAULT_SOFTWARE))
        path, _ = QFileDialog.getOpenFileName(
            self, "Select flight-software library", start,
            "Shared library (*.dll *.so *.dylib);;Model header (*.h)")
        if path:
            self._set_software(path)

    def select_software_folder(self):
        start = str(Path(self.doc.software_path or DEFAULT_SOFTWARE))
        path = QFileDialog.getExistingDirectory(
            self, "Select generated-code folder", start)
        if path:
            self._set_software(path)

    def _set_software(self, path):
        self.checkpoint()
        self.doc.software_path = str(Path(path).resolve())
        self.doc.no_software = False
        self.reload()

    def toggle_no_software(self):
        """Disconnect the flight software, or reconnect the selected one.

        With no software the simulator runs open loop: every actuator input
        stays at zero, so no control torque or force acts on the spacecraft.
        Wires to the software are kept, so switching back restores them.
        """
        self.checkpoint()
        self.doc.no_software = not self.doc.no_software
        self.reload()

    def simulator_settings(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Simulator")
        form = QFormLayout(dialog)
        sim = self.doc.simulator
        folder = QLineEdit(sim.get("simulator_dir", str(DEFAULT_SIMULATOR_DIR)))
        browse = QPushButton("Browse…")
        browse.clicked.connect(lambda: folder.setText(
            QFileDialog.getExistingDirectory(dialog, "Simulator folder",
                                             folder.text()) or folder.text()))
        entry = QLineEdit(sim.get("entry", DEFAULT_ENTRY))
        kwargs = QLineEdit(json.dumps(sim["kwargs"]) if sim.get("kwargs")
                           else "")
        kwargs.setPlaceholderText("default")
        form.addRow("Folder", folder)
        form.addRow("", browse)
        form.addRow("Entry (module:function)", entry)
        form.addRow("Arguments (JSON)", kwargs)
        note = QLabel("The entry function builds the scenario; it is not run. "
                      "It must return a SimBaseClass or an object with .sim.")
        note.setWordWrap(True)
        form.addRow(note)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok |
                                   QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec() != QDialog.Accepted:
            return
        try:
            parsed = json.loads(kwargs.text()) if kwargs.text().strip() \
                else None
        except json.JSONDecodeError as error:
            QMessageBox.warning(self, "Arguments", f"Not valid JSON: {error}")
            return
        self.checkpoint()
        self.doc.simulator = {"entry": entry.text().strip(),
                              "simulator_dir": folder.text().strip()}
        if parsed:
            self.doc.simulator["kwargs"] = parsed
        self.reload()

    # ======================================================================
    # Undo
    # ======================================================================

    def _snapshot(self):
        return json.dumps(self.doc.to_json())

    def _restore(self, snapshot):
        path = self.doc.path
        software = (self.doc.software_path, self.doc.no_software)
        simulator = dict(self.doc.simulator)
        restored = LinkDocument.from_json(json.loads(snapshot), path)
        self.doc = restored
        self.doc.path = path
        if ((restored.software_path, restored.no_software),
                restored.simulator) != (software, simulator):
            self.load_blocks()
        self.modified = True
        self.rebuild()

    def checkpoint(self):
        self.undo_stack.append(self._snapshot())
        del self.undo_stack[:-200]
        self.redo_stack.clear()
        self.modified = True

    def checkpoint_pending(self):
        self._pending_snapshot = self._snapshot()

    def nodes_moved(self):
        if self._pending_snapshot is not None:
            self.undo_stack.append(self._pending_snapshot)
            self.redo_stack.clear()
            self._pending_snapshot = None
            self.modified = True
            self._update_title()

    def undo(self):
        if self.undo_stack:
            self.redo_stack.append(self._snapshot())
            self._restore(self.undo_stack.pop())

    def redo(self):
        if self.redo_stack:
            self.undo_stack.append(self._snapshot())
            self._restore(self.redo_stack.pop())

    # ======================================================================
    # Editing (called by the canvas)
    # ======================================================================

    def connect_ports(self, source, target):
        if (source, target) in self.doc.wires:
            return
        level, message = self.check_connection(source, target)
        if level == "error":
            answer = QMessageBox.question(
                self, "Incompatible ports",
                f"{message}\n\nConnect anyway? (You can put a conversion "
                f"block in between afterwards.)")
            if answer != QMessageBox.Yes:
                return
        self.checkpoint()
        self.doc.connect(source, target)
        self.schedule_rebuild()

    def check_connection(self, source, target):
        """Level and message for a prospective wire (used while dragging)."""
        port = self._boundary_port(target)
        if port is None:
            return "ok", ""
        return check_types(self.spec_of(source), port)

    def spec_of(self, endpoint):
        port = self._boundary_port(endpoint)
        if port is not None:
            return port.dtype, tuple(port.shape)
        if self.report:
            return self.report.node_outputs.get(endpoint)
        return None

    def _boundary_port(self, endpoint):
        kind, name = owner(endpoint), port_name(endpoint)
        block = {"sw": self.software, "sim": self.simulator}.get(kind)
        if block is None:
            return None
        try:
            return block.port(name)
        except KeyError:
            return None

    def add_block(self, type_key, pos):
        self.checkpoint()
        node_id = self.doc.add_node(type_key, (pos[0] - 70, pos[1] - 20))
        self.schedule_rebuild(select=node_id)

    def delete_selection(self):
        items = self.canvas.selectedItems()
        wires = [i.wire for i in items if isinstance(i, WireItem)]
        nodes = [i.key for i in items if isinstance(i, NodeItem)
                 and i.key in self.doc.nodes]
        modules = [i.key for i in items if isinstance(i, NodeItem)
                   and i.key.startswith("sim:")]
        if not (wires or nodes or modules):
            return
        self.checkpoint()
        for wire in wires:
            self.doc.disconnect(wire)
        for key in nodes:
            self.doc.remove_node(key)
        for key in modules:
            self._hide_module(key[4:])
        self.schedule_rebuild()

    def _hide_module(self, module):
        if module in self.doc.shown_modules:
            self.doc.shown_modules.remove(module)

    def toggle_group(self, group):
        if group in self.doc.expanded_groups:
            self.doc.expanded_groups.remove(group)
        else:
            self.doc.expanded_groups.append(group)
        self.modified = True
        self.schedule_rebuild()

    def node_position_changed(self, key, pos):
        if key in self.doc.nodes:
            self.doc.nodes[key].pos = pos
        else:
            self.doc.positions[key] = pos

    def focus_properties(self, key):
        self.properties_dock.raise_()

    def insert_block_on_wire(self, wire, type_key):
        source, target = wire
        spec = conversions.TYPES[type_key]
        a, b = self.canvas.ports[source].scenePos(), \
            self.canvas.ports[target].scenePos()
        self.checkpoint()
        node_id = self.doc.add_node(type_key, ((a.x() + b.x()) / 2 - 70,
                                               (a.y() + b.y()) / 2 - 30))
        node = self.doc.nodes[node_id]
        self.doc.disconnect(wire)
        inputs = node.input_names()
        if inputs:
            self.doc.connect(source, f"{node_id}:{inputs[0]}")
        self.doc.connect(f"{node_id}:{spec.outputs[0]}", target)
        self.schedule_rebuild(select=node_id)

    def rename_node(self, key):
        node = self.doc.nodes[key]
        text, ok = QInputDialog.getText(self, "Rename block", "Label",
                                        text=node.label or node.spec().label)
        if ok:
            self.checkpoint()
            node.label = text.strip()
            self.schedule_rebuild()

    def context_menu(self, item, scene_pos, global_pos):
        menu = QMenu(self)
        if isinstance(item, PortItem):
            item = item.node
        if isinstance(item, WireItem):
            item.setSelected(True)
            insert = menu.addMenu("Insert block")
            for block in conversions.TYPES.values():
                if block.key == "constant":
                    continue
                insert.addAction(block.label).triggered.connect(
                    lambda _=False, k=block.key, w=item.wire:
                    self.insert_block_on_wire(w, k))
            menu.addAction("Delete wire").triggered.connect(
                lambda _=False, w=item.wire: self._delete_wires([w]))
        elif isinstance(item, NodeItem):
            key = item.key
            if not item.isSelected():
                self.canvas.clearSelection()
                item.setSelected(True)
            menu.addAction(self.actions["rotate_cw"])
            menu.addAction(self.actions["rotate_ccw"])
            menu.addSeparator()
            if key in self.doc.nodes:
                menu.addAction("Rename…").triggered.connect(
                    lambda _=False: self.rename_node(key))
                menu.addAction("Delete block").triggered.connect(
                    lambda _=False: self._delete_nodes([key]))
            elif key.startswith("sim:"):
                module = key[4:]
                groups = sorted({p.group for p in self.simulator.ports
                                 if p.extra["module"] == module})
                menu.addAction("Expand all messages").triggered.connect(
                    lambda _=False: self._set_groups(groups, True))
                menu.addAction("Collapse all messages").triggered.connect(
                    lambda _=False: self._set_groups(groups, False))
                if module in self.doc.shown_modules:
                    menu.addAction("Hide module").triggered.connect(
                        lambda _=False: self._toggle_module(module, False))
            elif key == "sw":
                menu.addAction(self.actions["software_dll"])
                menu.addAction(self.actions["software_dir"])
                menu.addAction(self.actions["reload"])
        else:
            add = menu.addMenu("Add block")
            for block in conversions.TYPES.values():
                add.addAction(f"{block.category}: {block.label}").triggered.\
                    connect(lambda _=False, k=block.key, p=scene_pos:
                            self.add_block(k, (p.x(), p.y())))
            menu.addSeparator()
            menu.addAction(self.actions["layout"])
            menu.addAction(self.actions["fit"])
        menu.exec(global_pos)

    def _delete_wires(self, wires):
        self.checkpoint()
        for wire in wires:
            self.doc.disconnect(wire)
        self.schedule_rebuild()

    def _delete_nodes(self, keys):
        self.checkpoint()
        for key in keys:
            self.doc.remove_node(key)
            self.doc.rotations.pop(key, None)
        self.schedule_rebuild()

    def rotate_selection(self, step):
        """Turn every selected block by ``step`` degrees (+-90)."""
        keys = [item.key for item in self.canvas.selectedItems()
                if isinstance(item, NodeItem)]
        if not keys:
            self.statusBar().showMessage("Select a block to rotate", 3000)
            return
        self.checkpoint()
        for key in keys:
            angle = (self.doc.rotations.get(key, 0) + step) % 360
            if angle:
                self.doc.rotations[key] = angle
            else:
                self.doc.rotations.pop(key, None)
        self.modified = True
        self.rebuild()

    def _set_groups(self, groups, expanded):
        for group in groups:
            if expanded and group not in self.doc.expanded_groups:
                self.doc.expanded_groups.append(group)
            if not expanded and group in self.doc.expanded_groups:
                self.doc.expanded_groups.remove(group)
        self.modified = True
        self.schedule_rebuild()

    def _module_toggled(self, item):
        if self._filling_modules:
            return
        self._toggle_module(item.data(Qt.UserRole),
                            item.checkState() == Qt.Checked)

    def _toggle_module(self, module, shown):
        self.checkpoint()
        if shown and module not in self.doc.shown_modules:
            self.doc.shown_modules.append(module)
            if f"sim:{module}" not in self.doc.positions:
                center = self.view.mapToScene(
                    self.view.viewport().rect().center())
                self.doc.positions[f"sim:{module}"] = (center.x(), center.y())
        if not shown:
            self._hide_module(module)
        self.schedule_rebuild(select=f"sim:{module}" if shown else None)

    # ======================================================================
    # Building the picture
    # ======================================================================

    def schedule_rebuild(self, select=None):
        """Rebuild after the current event: the item that triggered the
        change (a port mid-drag, a clicked group) must not be deleted while
        Qt is still delivering its event."""
        self._select_after = select
        if not self._rebuild_scheduled:
            self._rebuild_scheduled = True
            QTimer.singleShot(0, self._deferred_rebuild)

    def _deferred_rebuild(self):
        self._rebuild_scheduled = False
        self.rebuild()
        select = getattr(self, "_select_after", None)
        if select and select in self.canvas.nodes:
            self.canvas.clearSelection()
            self.canvas.nodes[select].setSelected(True)

    def validate_and_show(self):
        self.report = validate(self.doc, self.software, self.simulator)
        self.problems.clear()
        for message in self.load_errors:
            self._add_problem("error", message, None)
        for issue in self.report.issues:
            self._add_problem(issue.level, issue.message, issue.ref)
        counts = self.report.counts()
        counts["error"] += len(self.load_errors)
        self.problems_dock.setWindowTitle(
            f"Problems — {counts['error']} errors, {counts['warning']} "
            f"warnings, {counts['info']} notes")
        sw = (f"Software: {self.software.name} "
              f"({self.software.meta.get('sample_time')} s)"
              if self.software else
              "Software: none (no control torque/force)"
              if self.doc.no_software else "Software: not loaded")
        sim = (f"Simulator: {len(self.simulator.meta['modules'])} modules "
               f"({', '.join(str(v) for v in self.simulator.meta['tasks'].values())} s)"
               if self.simulator else "Simulator: not loaded")
        self.status.setText(f"{sw}    |    {sim}    |    "
                            f"{len(self.doc.wires)} wires, "
                            f"{len(self.doc.nodes)} blocks")
        self._update_title()

    def _add_problem(self, level, message, ref):
        item = QTreeWidgetItem([LEVEL_ICONS.get(level, ""), message])
        item.setData(0, Qt.UserRole, ref)
        item.setToolTip(1, message)
        color = {"error": "#c53030", "warning": "#b45309"}.get(level)
        if color:
            item.setForeground(1, QColor(color))
        self.problems.addTopLevelItem(item)

    def _update_title(self):
        name = Path(self.doc.path).name if self.doc.path else "untitled"
        self.setWindowTitle(f"{name}{' *' if self.modified else ''} — "
                            f"AOCS Connector")

    def rebuild(self):
        self.validate_and_show()
        specs = self._node_specs()
        positions = dict(self.doc.positions)
        for key, node in self.doc.nodes.items():
            positions[key] = node.pos
        self.canvas.rebuild(specs, positions, self.doc.wires,
                            self.report.wire_status, self.doc.rotations)
        self._fill_modules()
        self.update_properties()

    def _wired_endpoints(self):
        wired = set()
        for source, target in self.doc.wires:
            wired.add(source)
            wired.add(target)
        return wired

    def _visible_modules(self):
        if not self.simulator:
            return []
        wired = {self.simulator.port(port_name(e)).extra["module"]
                 for e in self._wired_endpoints()
                 if owner(e) == "sim" and self._boundary_port(e) is not None}
        return [m for m in self.simulator.meta["modules"]
                + ["python"] if m in wired or m in self.doc.shown_modules]

    def _node_specs(self):
        wired = self._wired_endpoints()
        specs = []
        if self.software:
            specs.append(self._software_spec(wired))
        for module in self._visible_modules():
            specs.append(self._module_spec(module, wired))
        for key, node in self.doc.nodes.items():
            if node.type in conversions.TYPES:
                specs.append(self._conversion_spec(key, node, wired))
        return specs

    def _port_row(self, endpoint, text, port, wired, group=""):
        description = f"\n{port.description}" if port.description and \
            port.description != port.id else ""
        wiring = port.extra.get("wiring", {})
        note = ""
        if wiring.get("status") == "flight_software":
            note = "\nDriven by the old bridge"
        elif wiring.get("read_by_flight_software"):
            note = "\nRead by the old bridge"
        return Row("port", text, endpoint=endpoint, group=group,
                   dtype=port.dtype, connected=endpoint in wired,
                   tooltip=f"{port.id}\n{port.type_string()}"
                           f"{description}{note}")

    def _software_spec(self, wired):
        block = self.software
        rows = {d: [self._port_row(f"sw:{p.id}",
                                   p.description or p.id, p, wired)
                    for p in ports]
                for d, ports in (("in", block.inputs),
                                 ("out", block.outputs))}
        return NodeSpec("sw", block.name,
                        f"Flight software · step "
                        f"{block.meta.get('sample_time')} s",
                        COLORS["software"], rows["in"], rows["out"],
                        "software", tooltip=block.source)

    def _module_spec(self, module, wired):
        groups = {}
        for port in self.simulator.ports:
            if port.extra["module"] == module:
                groups.setdefault((port.direction, port.group), []).append(port)
        sides = {"in": [], "out": []}
        for (direction, group), ports in groups.items():
            expanded = group in self.doc.expanded_groups
            message = ports[0].extra["message"]
            shown = [p for p in ports if expanded or f"sim:{p.id}" in wired]
            hidden = len(ports) - len(shown)
            label = message + (f"  (+{hidden})" if hidden else "")
            sides[direction].append(Row("group", label, group=group,
                                        expanded=expanded))
            for port in shown:
                sides[direction].append(self._port_row(
                    f"sim:{port.id}", port.extra["field"], port, wired,
                    group=group))
        title = module.split(":")[-1] if ":" in module else module
        subtitle = "Simulator" + (f" · {module.split(':')[0]}"
                                  if ":" in module else "")
        return NodeSpec(f"sim:{module}", title, subtitle, COLORS["module"],
                        sides["in"], sides["out"], "module",
                        tooltip="Click a message name to show or hide its "
                                "fields")

    def _conversion_spec(self, key, node, wired):
        block = node.spec()
        broken = any(i.ref == key and i.level == "error"
                     for i in self.report.issues)
        left = [Row("port", name, endpoint=f"{key}:{name}", dtype="float64",
                    connected=f"{key}:{name}" in wired,
                    tooltip=f"{key}:{name}")
                for name in node.input_names()]
        right = []
        for name in node.output_names():
            endpoint = f"{key}:{name}"
            spec = self.report.node_outputs.get(endpoint)
            text = f"{spec[0]}[{','.join(map(str, spec[1]))}]" \
                if spec and spec[1] else (spec[0] if spec else "?")
            right.append(Row("port", name, endpoint=endpoint,
                             dtype=spec[0] if spec else "unknown",
                             connected=endpoint in wired,
                             tooltip=f"{endpoint}\n{text}"))
        return NodeSpec(key, node.label or block.label,
                        _param_summary(node), COLORS.get(block.category,
                                                         COLORS["Custom"]),
                        left, right, "conversion", tooltip=block.help,
                        warning=broken)

    def _fill_modules(self):
        self._filling_modules = True
        self.modules.clear()
        visible = set(self._visible_modules())
        if self.simulator:
            wired = {self.simulator.port(port_name(e)).extra["module"]
                     for e in self._wired_endpoints()
                     if owner(e) == "sim" and self._boundary_port(e)}
            for module in self.simulator.meta["modules"]:
                item = QListWidgetItem(module)
                item.setData(Qt.UserRole, module)
                item.setCheckState(Qt.Checked if module in visible
                                   else Qt.Unchecked)
                if module in wired:
                    item.setFlags(item.flags() & ~Qt.ItemIsUserCheckable)
                    item.setToolTip("Shown because it has connections")
                else:
                    item.setToolTip("Tick to place this module on the diagram")
                self.modules.addItem(item)
        self._filling_modules = False

    _filling_modules = False

    # ======================================================================
    # Layout
    # ======================================================================

    def auto_layout(self, record=True):
        """Place every block in signal-flow columns (see gui/layout.py)."""
        if record:
            self.checkpoint()
        specs = self._node_specs()
        rows = {}
        if self.software:
            for index, port in enumerate(self.software.inputs):
                rows[f"sw:{port.id}"] = index
            for index, port in enumerate(self.software.outputs):
                rows[f"sw:{port.id}"] = index
        def measure(spec):
            node = NodeItem(spec, self.canvas.font, self.canvas.bold)
            return node.width, node.height

        positions = arrange(specs, self.doc.wires, self._owner_key, rows,
                            measure)
        for key, pos in positions.items():
            if key in self.doc.nodes:
                self.doc.nodes[key].pos = pos
            else:
                self.doc.positions[key] = pos
        if record:
            self.rebuild()
            self.view.fit()

    def _owner_key(self, endpoint):
        kind = owner(endpoint)
        if kind == "sw":
            return "sw"
        if kind == "sim":
            return self._module_key(endpoint)
        return kind

    def _module_of(self, endpoint):
        port = self._boundary_port(endpoint)
        return port.extra["module"] if port is not None else None

    def _module_key(self, endpoint):
        if owner(endpoint) != "sim":
            return None
        module = self._module_of(endpoint)
        return f"sim:{module}" if module else None

    # ======================================================================
    # Properties and problems
    # ======================================================================

    def update_properties(self):
        items = self.canvas.selectedItems()
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setAlignment(Qt.AlignTop)
        if len(items) == 1 and isinstance(items[0], NodeItem):
            key = items[0].key
            if key in self.doc.nodes:
                self._conversion_properties(layout, key)
            elif key == "sw":
                self._software_properties(layout)
            elif key.startswith("sim:"):
                self._module_properties(layout, key[4:])
        elif len(items) == 1 and isinstance(items[0], WireItem):
            self._wire_properties(layout, items[0])
        else:
            self._document_properties(layout)
        self.properties.setWidget(panel)

    @staticmethod
    def _heading(layout, text):
        label = QLabel(f"<b style='font-size:11pt'>{text}</b>")
        layout.addWidget(label)

    @staticmethod
    def _text(layout, text):
        label = QLabel(text)
        label.setWordWrap(True)
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(label)

    def _document_properties(self, layout):
        self._heading(layout, "Wiring")
        self._text(layout, f"File: {self.doc.path or 'not saved yet'}")
        self._text(layout, "Software: none — no control torque or force"
                   if self.doc.no_software
                   else f"Software: {self.doc.software_path}")
        self._text(layout, f"Simulator: {self.doc.simulator.get('entry')}<br>"
                           f"{self.doc.simulator.get('simulator_dir')}")
        self._text(layout, "<br><b>How to</b><br>"
                   "• Drag from a port to another port to connect.<br>"
                   "• Drag a block from the library, or right-click a wire "
                   "to insert one.<br>"
                   "• Click a message name to show all its fields.<br>"
                   "• Delete removes selected wires and blocks.<br>"
                   "• Wheel zooms; right or middle drag pans.<br>"
                   "• F5 re-reads both sides after you change them.")

    def _software_properties(self, layout):
        block = self.software
        meta = block.meta
        self._heading(layout, block.name)
        self._text(layout, f"Library: {meta.get('library')}")
        self._text(layout, f"Sample time: {meta.get('sample_time')} s")
        self._text(layout, f"{len(block.inputs)} inputs, "
                           f"{len(block.outputs)} outputs, "
                           f"{len(meta.get('parameters', []))} parameters")
        self._text(layout, f"Signature: {block.signature()}")
        for key in ("software_dll", "software_dir"):
            button = QPushButton(self.actions[key].text())
            button.clicked.connect(self.actions[key].trigger)
            layout.addWidget(button)
        self._heading(layout, "Library check")
        for level, message in check_library(block):
            icon = {"ok": "✔", "warn": "⚠", "error": "⛔"}[level]
            self._text(layout, f"{icon} {message}")

    def _module_properties(self, layout, module):
        self._heading(layout, module)
        messages = {}
        for port in self.simulator.ports:
            if port.extra["module"] == module:
                messages.setdefault((port.direction, port.extra["message"]),
                                    port)
        for (direction, message), port in sorted(messages.items()):
            wiring = port.extra["wiring"]
            note = ""
            if direction == "in":
                note = f" — {wiring['status']}"
                if wiring.get("source"):
                    note += f" from {wiring['source']}"
            elif wiring.get("read_by"):
                note = f" — read by {', '.join(wiring['read_by'])}"
            self._text(layout, f"<b>{'⇢' if direction == 'in' else '⇠'} "
                               f"{message}</b> [{port.extra['payload']}]"
                               f"<span style='color:#6b7785'>{note}</span>")
        if module in self.doc.shown_modules:
            button = QPushButton("Hide module")
            button.clicked.connect(lambda: self._toggle_module(module, False))
            layout.addWidget(button)

    def _wire_properties(self, layout, item):
        source, target = item.wire
        self._heading(layout, "Wire")
        self._text(layout, f"<b>From</b> {source}")
        self._text(layout, f"<b>To</b> {target}")
        spec = self.spec_of(source)
        if spec:
            self._text(layout, f"Carries {spec[0]}"
                               f"{list(spec[1]) if spec[1] else ''}")
        if item.message:
            self._text(layout, f"{LEVEL_ICONS.get(item.level, '')} "
                               f"{item.message}")
        button = QPushButton("Delete wire")
        button.clicked.connect(lambda: self._delete_wires([item.wire]))
        layout.addWidget(button)

    def _conversion_properties(self, layout, key):
        node = self.doc.nodes[key]
        block = node.spec()
        self._heading(layout, f"{block.label}  ({key})")
        self._text(layout, block.help)
        form = QFormLayout()
        label = QLineEdit(node.label)
        label.setPlaceholderText(block.label)
        label.editingFinished.connect(
            lambda: self._set_label(key, label.text()))
        form.addRow("Label", label)
        for param in block.params:
            edit = QLineEdit(str(node.params.get(param.name, param.default)))
            edit.setToolTip(param.help)
            edit.editingFinished.connect(
                lambda p=param, e=edit: self._set_param(key, p, e.text()))
            form.addRow(param.name, edit)
            if param.help:
                hint = QLabel(f"<span style='color:#6b7785'>{param.help}"
                              f"</span>")
                hint.setWordWrap(True)
                form.addRow("", hint)
        layout.addLayout(form)
        outputs = [f"{name}: {self.report.node_outputs.get(f'{key}:{name}')}"
                   for name in node.output_names()]
        self._text(layout, "Output: " + ", ".join(outputs))
        for issue in self.report.issues:
            if issue.ref == key:
                self._text(layout, f"{LEVEL_ICONS[issue.level]} "
                                   f"{issue.message}")
        button = QPushButton("Delete block")
        button.clicked.connect(lambda: self._delete_nodes([key]))
        layout.addWidget(button)

    def _set_label(self, key, text):
        node = self.doc.nodes.get(key)
        if node is None or node.label == text.strip():
            return
        self.checkpoint()
        node.label = text.strip()
        self.schedule_rebuild(select=key)

    def _set_param(self, key, param, text):
        node = self.doc.nodes.get(key)
        if node is None:
            return
        try:
            value = {"float": float, "int": int}.get(param.kind, str)(text)
        except ValueError:
            QMessageBox.warning(self, "Parameter",
                                f"{param.name} must be a {param.kind}")
            self.update_properties()
            return
        if node.params.get(param.name) == value:
            return
        self.checkpoint()
        node.params[param.name] = value
        # Inputs that no longer exist lose their wires.
        names = {f"{key}:{n}" for n in node.input_names()}
        self.doc.wires = [w for w in self.doc.wires
                          if owner(w[1]) != key or w[1] in names]
        self.schedule_rebuild(select=key)

    def _problem_clicked(self, item):
        ref = item.data(0, Qt.UserRole)
        target = None
        if isinstance(ref, (tuple, list)):
            for wire in self.canvas.wires:
                if tuple(wire.wire) == tuple(ref):
                    target = wire
        elif isinstance(ref, str):
            if ref in self.canvas.nodes:
                target = self.canvas.nodes[ref]
            elif ref in self.canvas.ports:
                target = self.canvas.ports[ref].node
            elif owner(ref) == "sim":
                module = self._module_of(ref)
                if module and module not in self._visible_modules():
                    self._toggle_module(module, True)
                    return
        if target is not None:
            self.canvas.clearSelection()
            target.setSelected(True)
            self.view.centerOn(target)


def _param_summary(node):
    params = node.params
    if node.type == "gain":
        return f"k = {params.get('k')}"
    if node.type == "select":
        return f"[{params.get('indices')}]"
    if node.type == "place":
        return f"into [{params.get('length')}] at {params.get('offset')}"
    if node.type == "python_function":
        return str(params.get("function", "")).split(":")[-1] + "()"
    if node.type == "expression":
        return str(params.get("expression"))
    if node.type == "constant":
        return str(params.get("value"))
    if node.type == "cast":
        return f"→ {params.get('dtype')}"
    if node.type == "quat_swap":
        return f"scalar {params.get('to')}"
    return conversions.TYPES[node.type].category
