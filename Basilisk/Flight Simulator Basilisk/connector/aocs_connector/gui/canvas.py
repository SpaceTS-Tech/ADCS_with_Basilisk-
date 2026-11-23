"""The block diagram: nodes, ports, wires, zoom/pan, drag-to-connect.

The canvas holds no state of its own.  The window rebuilds it from the link
document whenever the structure changes (a wire added, a block removed, a
group expanded); with a few dozen blocks that is instant and keeps the
picture and the document from ever disagreeing.  Only moving a block is
handled in place, by rerouting the wires attached to it.
"""

import math
from dataclasses import dataclass

from PySide6.QtCore import QLineF, QPointF, QRectF, Qt
from PySide6.QtGui import (QBrush, QColor, QFont, QFontMetricsF, QPainter,
                           QPainterPath, QPainterPathStroker, QPen)
from PySide6.QtWidgets import (QGraphicsItem,
                               QGraphicsPathItem, QGraphicsScene,
                               QGraphicsView)

ROW = 19
HEADER = 38
PAD = 10
PORT_RADIUS = 5
#: Port arrowhead: length along the signal, half-height across it [px].
ARROW_LENGTH = 10.0
ARROW_HALF = 6.0
MIME_BLOCK = "application/x-aocs-block"

GRID_MINOR = QColor("#eef1f5")
GRID_MAJOR = QColor("#dde2ea")
#: Grids denser than this on screen are not drawn [px between lines].
MIN_GRID_PIXELS = 6
BACKGROUND = QColor("#fafbfc")
TEXT = QColor("#1f2933")
MUTED = QColor("#6b7785")
SELECTED = QColor("#f0a020")

WIRE_COLORS = {
    "ok": QColor("#52606d"),
    "info": QColor("#2b7bd0"),
    "warning": QColor("#e07b16"),
    "error": QColor("#d64545"),
}


def dtype_color(dtype):
    if dtype.startswith("float"):
        return QColor("#2b7bd0")
    if dtype.startswith(("int", "uint")) or dtype == "int":
        return QColor("#2f9e6e")
    if dtype == "bool":
        return QColor("#8a4fd6")
    return QColor("#8792a2")


@dataclass
class Row:
    """One line of a block: a port, or a collapsible message group."""

    kind: str                   # "port" or "group"
    text: str
    endpoint: str = ""
    group: str = ""
    expanded: bool = False
    dtype: str = ""
    tooltip: str = ""
    connected: bool = False


@dataclass
class NodeSpec:
    key: str
    title: str
    subtitle: str
    color: QColor
    left: list
    right: list
    kind: str                   # "software", "module", "conversion"
    tooltip: str = ""
    warning: bool = False


def arrow_path(direction, pointing):
    """Arrowhead for a port, drawn around the port's wire attachment point.

    ``pointing`` is +1 when the signal flows towards +x in block axes, -1
    otherwise.  An **input** arrow sits outside the block with its tip on the
    edge (the wire arrives at its base); an **output** arrow sits outside with
    its base on the edge (the wire leaves from its tip).  Either way the arrow
    points the way the signal travels.
    """
    tip = ARROW_LENGTH if direction == "in" else 0.0
    base = 0.0 if direction == "in" else -ARROW_LENGTH
    path = QPainterPath(QPointF(pointing * base, -ARROW_HALF))
    path.lineTo(QPointF(pointing * tip, 0.0))
    path.lineTo(QPointF(pointing * base, ARROW_HALF))
    path.closeSubpath()
    return path


class PortItem(QGraphicsPathItem):
    """A port, drawn as an arrowhead in the direction of the signal.

    Its position is where the wire attaches: the base of an input arrow, the
    tip of an output arrow.  Filled when connected, hollow when not; coloured
    by data type.
    """

    def __init__(self, node, row, direction, pos, pointing=1):
        super().__init__(arrow_path(direction, pointing), node)
        self.node = node
        self.endpoint = row.endpoint
        self.direction = direction
        self.row = row
        self.setPos(pos)
        color = dtype_color(row.dtype)
        self.setBrush(QBrush(color if row.connected else QColor("white")))
        pen = QPen(color, 1.6)
        pen.setJoinStyle(Qt.MiterJoin)
        self.setPen(pen)
        self.setToolTip(row.tooltip)
        self.setAcceptHoverEvents(True)
        self.setCursor(Qt.CrossCursor)
        self.setZValue(2)

    def shape(self):
        """Generous hit area, so the small arrow is easy to grab."""
        path = QPainterPath()
        path.addRect(self.path().boundingRect().adjusted(-4, -3, 4, 3))
        return path

    def outward(self):
        """Unit vector, in scene axes, pointing out of the block at this port.

        Wires leave and enter along it, so they meet a rotated block square-on.
        """
        local = QPointF(-1.0 if self.pos().x() < self.node.width / 2 else 1.0,
                        0.0)
        transform = self.node.sceneTransform()
        vector = transform.map(local) - transform.map(QPointF(0.0, 0.0))
        length = math.hypot(vector.x(), vector.y()) or 1.0
        return QPointF(vector.x() / length, vector.y() / length)

    def hoverEnterEvent(self, event):
        self.setScale(1.5)
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self.setScale(1.0)
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.scene().start_drag(self, event.scenePos())
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        self.scene().update_drag(event.scenePos())

    def mouseReleaseEvent(self, event):
        self.scene().end_drag(event.scenePos())


class NodeItem(QGraphicsItem):
    """One block.  ``rotation`` is degrees clockwise, a multiple of 90.

    Like Simulink: at 90 the inputs face up and the outputs down, at 270 the
    reverse.  At 180 the block is mirrored rather than turned -- inputs on the
    right, outputs on the left -- so its text is never upside down.
    """

    def __init__(self, spec, font, bold, rotation=0):
        super().__init__()
        self.spec = spec
        self.orientation = int(rotation) % 360
        #: Inputs on the right, outputs on the left.
        self.mirrored = self.orientation == 180
        self.key = spec.key
        self.font, self.bold = font, bold
        self.setFlags(QGraphicsItem.ItemIsMovable |
                      QGraphicsItem.ItemIsSelectable |
                      QGraphicsItem.ItemSendsGeometryChanges)
        self.setAcceptHoverEvents(True)
        self.setToolTip(spec.tooltip)
        self.setZValue(1)

        metrics, bold_metrics = QFontMetricsF(font), QFontMetricsF(bold)
        left_w = max([self._row_width(r, metrics, bold_metrics)
                      for r in spec.left] or [0])
        right_w = max([self._row_width(r, metrics, bold_metrics)
                       for r in spec.right] or [0])
        title_w = bold_metrics.horizontalAdvance(spec.title) + 2 * PAD
        sub_w = metrics.horizontalAdvance(spec.subtitle) + 2 * PAD
        self.width = max(130.0, title_w, sub_w,
                         left_w + right_w + 3 * PAD + 20)
        self.height = HEADER + max(len(spec.left), len(spec.right), 1) * ROW \
            + PAD
        self.ports = []
        self.group_hits = []
        for side, rows in (("in", spec.left), ("out", spec.right)):
            on_left = (side == "in") != self.mirrored
            # Signal flows left-to-right in block axes, or right-to-left when
            # the block is mirrored; arrows sit just outside the edge.
            pointing = -1 if self.mirrored else 1
            for index, row in enumerate(rows):
                y = HEADER + index * ROW + ROW / 2
                edge = 0.0 if on_left else self.width
                # Wire attachment point: an input's arrow lies upstream of
                # the edge, an output's downstream of it.
                x = edge + (pointing if side == "out" else -pointing) \
                    * ARROW_LENGTH
                if row.kind == "port":
                    self.ports.append(PortItem(self, row, side, QPointF(x, y),
                                               pointing))
                else:
                    hit = QRectF(0 if on_left else self.width / 2,
                                 y - ROW / 2, self.width / 2, ROW)
                    self.group_hits.append((hit, row.group))
        self._press_pos = None
        if self.orientation in (90, 270):
            self.setTransformOriginPoint(self.width / 2, self.height / 2)
            self.setRotation(90 if self.orientation == 90 else -90)

    @staticmethod
    def _row_width(row, metrics, bold_metrics):
        if row.kind == "group":
            return bold_metrics.horizontalAdvance("▾ " + row.text)
        return metrics.horizontalAdvance(row.text) + (12 if row.group else 0)

    def boundingRect(self):
        margin = ARROW_LENGTH + 4
        return QRectF(-margin, -2, self.width + 2 * margin, self.height + 6)

    def paint(self, painter, option, widget=None):
        spec = self.spec
        body = QRectF(0, 0, self.width, self.height)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 28))
        painter.drawRoundedRect(body.translated(2, 3), 7, 7)

        painter.setBrush(QColor("white"))
        border = SELECTED if self.isSelected() else \
            (WIRE_COLORS["error"] if spec.warning else QColor("#c5ccd6"))
        painter.setPen(QPen(border, 2.2 if self.isSelected() else 1.2))
        painter.drawRoundedRect(body, 7, 7)

        header = QPainterPath()
        header.setFillRule(Qt.WindingFill)
        header.addRoundedRect(QRectF(0, 0, self.width, HEADER - 4), 7, 7)
        header.addRect(QRectF(0, HEADER - 12, self.width, 8))
        painter.setPen(Qt.NoPen)
        painter.setBrush(spec.color)
        painter.drawPath(header.simplified())

        painter.setPen(QColor("white"))
        painter.setFont(self.bold)
        painter.drawText(QRectF(PAD, 3, self.width - 2 * PAD, 18),
                         Qt.AlignLeft | Qt.AlignVCenter, spec.title)
        painter.setFont(self.font)
        painter.setPen(QColor(255, 255, 255, 215))
        painter.drawText(QRectF(PAD, 18, self.width - 2 * PAD, 15),
                         Qt.AlignLeft | Qt.AlignVCenter, spec.subtitle)

        for side, rows in (("in", spec.left), ("out", spec.right)):
            on_left = (side == "in") != self.mirrored
            for index, row in enumerate(rows):
                y = HEADER + index * ROW
                rect = QRectF(PAD + 4, y, self.width - 2 * PAD - 8, ROW)
                align = (Qt.AlignLeft if on_left else Qt.AlignRight) \
                    | Qt.AlignVCenter
                if row.kind == "group":
                    painter.setFont(self.bold)
                    painter.setPen(MUTED)
                    arrow = "▾ " if row.expanded else "▸ "
                    text = arrow + row.text if on_left else \
                        row.text + " " + arrow.strip()
                    painter.drawText(rect, align, text)
                else:
                    painter.setFont(self.font)
                    painter.setPen(TEXT if row.connected else MUTED)
                    if row.group:
                        rect = rect.adjusted(12, 0, 0, 0) if on_left \
                            else rect.adjusted(0, 0, -12, 0)
                    painter.drawText(rect, align, row.text)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged and self.scene():
            self.scene().node_moved(self)
        return super().itemChange(change, value)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            for rect, group in self.group_hits:
                if rect.contains(event.pos()):
                    self.scene().ctrl.toggle_group(group)
                    event.accept()
                    return
            self._press_pos = self.pos()
            self.scene().ctrl.checkpoint_pending()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        if self._press_pos is not None and self.pos() != self._press_pos:
            self.scene().ctrl.nodes_moved()
        self._press_pos = None

    def mouseDoubleClickEvent(self, event):
        self.scene().ctrl.focus_properties(self.key)


class WireItem(QGraphicsPathItem):
    def __init__(self, wire, status):
        super().__init__()
        self.wire = wire
        self.level, self.message = status
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setAcceptHoverEvents(True)
        self.setZValue(0)
        self.setToolTip(f"{wire[0]}\n→ {wire[1]}" +
                        (f"\n\n{self.message}" if self.message else ""))
        self._hover = False
        self._apply_pen()

    def _apply_pen(self):
        color = SELECTED if self.isSelected() else WIRE_COLORS[self.level]
        width = 3.0 if (self.isSelected() or self._hover) else 1.8
        pen = QPen(color, width)
        if self.level == "error":
            pen.setStyle(Qt.DashLine)
        self.setPen(pen)

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemSelectedHasChanged:
            self._apply_pen()
        return super().itemChange(change, value)

    def hoverEnterEvent(self, event):
        self._hover = True
        self._apply_pen()

    def hoverLeaveEvent(self, event):
        self._hover = False
        self._apply_pen()

    def route(self, source, target):
        """Draw from an output port to an input port."""
        self.setPath(wire_path(source.scenePos(), target.scenePos(),
                               source.outward(), target.outward()))

    def shape(self):
        stroker = QPainterPathStroker()
        stroker.setWidth(10)
        return stroker.createStroke(self.path())


def wire_path(start, end, start_dir=QPointF(1, 0), end_dir=QPointF(-1, 0)):
    """Curve leaving ``start`` along ``start_dir`` and entering ``end``
    from ``end_dir`` (both pointing out of their blocks)."""
    path = QPainterPath(start)
    span = end - start
    horizontal = start_dir.y() == 0 and end_dir.y() == 0
    reach = abs(span.x()) if horizontal else max(abs(span.x()),
                                                 abs(span.y()))
    d = max(50.0, reach * 0.5)
    path.cubicTo(start + start_dir * d, end + end_dir * d, end)
    return path


class Canvas(QGraphicsScene):
    """Scene holding the diagram; actions go to ``ctrl`` (the window)."""

    def __init__(self, ctrl):
        super().__init__()
        self.ctrl = ctrl
        self.nodes = {}
        self.ports = {}
        self.wires = []
        self._drag = None
        self.font = QFont("Segoe UI", 9)
        self.bold = QFont("Segoe UI", 9, QFont.DemiBold)
        self.setSceneRect(-5000, -5000, 10000, 10000)

    # -- building -----------------------------------------------------------

    def rebuild(self, specs, positions, wires, wire_status, rotations=None):
        selected = {i.key for i in self.selectedItems()
                    if isinstance(i, NodeItem)}
        selected_wires = {i.wire for i in self.selectedItems()
                          if isinstance(i, WireItem)}
        self.clear()
        self.nodes, self.ports, self.wires = {}, {}, []
        for spec in specs:
            node = NodeItem(spec, self.font, self.bold,
                            (rotations or {}).get(spec.key, 0))
            node.setPos(QPointF(*positions.get(spec.key, (0, 0))))
            self.addItem(node)
            self.nodes[spec.key] = node
            for port in node.ports:
                self.ports[port.endpoint] = port
            node.setSelected(spec.key in selected)
        for wire in wires:
            if wire[0] in self.ports and wire[1] in self.ports:
                item = WireItem(wire, wire_status.get(wire, ("ok", "")))
                self.addItem(item)
                self.wires.append(item)
                item.setSelected(wire in selected_wires)
        self.reroute()

    def reroute(self, nodes=None):
        for item in self.wires:
            source, target = self.ports[item.wire[0]], self.ports[item.wire[1]]
            if nodes is None or source.node in nodes or target.node in nodes:
                item.route(source, target)

    def node_moved(self, node):
        self.ctrl.node_position_changed(node.key, (node.pos().x(),
                                                   node.pos().y()))
        self.reroute({node})

    # -- drag to connect ----------------------------------------------------

    def start_drag(self, port, pos):
        item = QGraphicsPathItem()
        item.setPen(QPen(WIRE_COLORS["ok"], 2, Qt.DashLine))
        item.setZValue(5)
        self.addItem(item)
        self._drag = (port, item)
        self.update_drag(pos)

    def _target_at(self, pos):
        origin = self._drag[0]
        for item in self.items(pos, Qt.IntersectsItemShape,
                               Qt.DescendingOrder, self.views()[0].transform()
                               if self.views() else None):
            if isinstance(item, PortItem) and item is not origin and \
                    item.direction != origin.direction and \
                    item.node is not origin.node:
                return item
        # Forgiving hit area: nearest compatible port within 12 px.
        best, best_d = None, 12.0
        for item in self.ports.values():
            if item is origin or item.direction == origin.direction or \
                    item.node is origin.node:
                continue
            d = (item.scenePos() - pos).manhattanLength()
            if d < best_d:
                best, best_d = item, d
        return best

    def update_drag(self, pos):
        if not self._drag:
            return
        origin, item = self._drag
        target = self._target_at(pos)
        end = target.scenePos() if target else pos
        start = origin.scenePos()
        start_dir = origin.outward()
        # A loose end points back towards the port it is dragged from.
        end_dir = target.outward() if target else -start_dir
        if origin.direction == "in":
            start, end = end, start
            start_dir, end_dir = end_dir, start_dir
        item.setPath(wire_path(start, end, start_dir, end_dir))
        level = "ok"
        if target:
            source, sink = (origin, target) if origin.direction == "out" \
                else (target, origin)
            level, _ = self.ctrl.check_connection(source.endpoint,
                                                  sink.endpoint)
        pen = QPen(WIRE_COLORS[level], 2.4 if target else 2,
                   Qt.SolidLine if target else Qt.DashLine)
        item.setPen(pen)

    def end_drag(self, pos):
        if not self._drag:
            return
        origin, item = self._drag
        target = self._target_at(pos)
        self.removeItem(item)
        self._drag = None
        if target:
            source, sink = (origin, target) if origin.direction == "out" \
                else (target, origin)
            self.ctrl.connect_ports(source.endpoint, sink.endpoint)

    # -- appearance ---------------------------------------------------------

    def drawBackground(self, painter, rect):
        """Grid behind the blocks, drawn in one call per line colour.

        The scale decides which grids are worth drawing: a grid whose lines
        would be closer than ``MIN_GRID_PIXELS`` on screen is skipped (it would
        only read as a grey wash), so zooming out never multiplies the work.
        """
        painter.fillRect(rect, BACKGROUND)
        scale = painter.worldTransform().m11() or 1.0
        left, right = rect.left(), rect.right()
        top, bottom = rect.top(), rect.bottom()
        for step, color in ((20, GRID_MINOR), (100, GRID_MAJOR)):
            if step * scale < MIN_GRID_PIXELS:
                continue
            first_x = math.floor(left / step) * step
            first_y = math.floor(top / step) * step
            lines = [QLineF(x, top, x, bottom)
                     for x in range(first_x, math.ceil(right) + 1, step)]
            lines += [QLineF(left, y, right, y)
                      for y in range(first_y, math.ceil(bottom) + 1, step)]
            painter.setPen(QPen(color, 0))      # cosmetic: 1 px at any zoom
            painter.drawLines(lines)

    def item_at(self, pos):
        view = self.views()[0]
        for item in self.items(pos, Qt.IntersectsItemShape,
                               Qt.DescendingOrder, view.transform()):
            if isinstance(item, (PortItem, NodeItem, WireItem)):
                return item
        return None


class CanvasView(QGraphicsView):
    """Zoom with the wheel, pan with the middle or right mouse button."""

    def __init__(self, scene, ctrl):
        super().__init__(scene)
        self.ctrl = ctrl
        self.setRenderHints(QPainter.Antialiasing |
                            QPainter.TextAntialiasing)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setAcceptDrops(True)
        self.setContextMenuPolicy(Qt.PreventContextMenu)
        self._pan = None
        self._pan_moved = False

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        scale = self.transform().m11() * factor
        if 0.15 <= scale <= 3.0:
            self.scale(factor, factor)

    def mousePressEvent(self, event):
        if event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._pan = event.position()
            self._pan_moved = False
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pan is not None:
            delta = event.position() - self._pan
            if delta.manhattanLength() > 3:
                self._pan_moved = True
            if self._pan_moved:
                self._pan = event.position()
                self.horizontalScrollBar().setValue(
                    self.horizontalScrollBar().value() - int(delta.x()))
                self.verticalScrollBar().setValue(
                    self.verticalScrollBar().value() - int(delta.y()))
                self.viewport().setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._pan is not None and event.button() in (Qt.MiddleButton,
                                                        Qt.RightButton):
            moved = self._pan_moved
            self._pan = None
            self.viewport().unsetCursor()
            if event.button() == Qt.RightButton and not moved:
                scene_pos = self.mapToScene(event.position().toPoint())
                self.ctrl.context_menu(self.scene().item_at(scene_pos),
                                       scene_pos,
                                       event.globalPosition().toPoint())
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Delete, Qt.Key_Backspace):
            self.ctrl.delete_selection()
            return
        super().keyPressEvent(event)

    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat(MIME_BLOCK):
            event.acceptProposedAction()

    def dragMoveEvent(self, event):
        if event.mimeData().hasFormat(MIME_BLOCK):
            event.acceptProposedAction()

    def dropEvent(self, event):
        key = bytes(event.mimeData().data(MIME_BLOCK)).decode()
        pos = self.mapToScene(event.position().toPoint())
        self.ctrl.add_block(key, (pos.x(), pos.y()))
        event.acceptProposedAction()

    def fit(self):
        items = self.scene().itemsBoundingRect()
        if items.isEmpty():
            return
        self.fitInView(items.adjusted(-40, -40, 40, 40), Qt.KeepAspectRatio)
        if self.transform().m11() > 1.2:
            self.resetTransform()
            self.scale(1.2, 1.2)
            self.centerOn(items.center())
