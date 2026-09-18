#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""treport.py - perf report like tool written using textual."""
from abc import ABC, abstractmethod
from typing import Dict, Optional
import argparse
import os
import sys
import perf
from rich.segment import Segment
from rich.style import Style
from textual import events
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.color import Color
from textual.scroll_view import ScrollView
from textual.strip import Strip
from textual.widgets import Footer, Header, TabbedContent, TabPane, Tree
from textual.widgets.tree import TreeNode

# Global session.
session :Optional[perf.session] = None

def make_fixed_length_string(s: str, length: int, pad_char=' '):
    """Make the string s a fixed length.

    Increases or decreases the length of s to be length. If the length is
    increased then pad_char is inserted on the right.
    """
    return s[:length] if len(s) > length else s.ljust(length, pad_char)


class FlameVisitor(ABC):
    """Parent for visitor used by ProfileNode.flame_walk"""
    @abstractmethod
    def visit(self, node: Optional["ProfileNode"], width: int) -> None:
        """Visit a profile node width the specified flame graph width.

        Args:
            node: The `ProfileNode` for the current segment. This may be `None`
                to represent a gap or an unknown portion of the stack.
            width: The calculated width of the flame graph rectangle for this
                node, which is proportional to its sample count.
        """


class ProfileNode:
    """Represents a single node in a call stack tree.

    Generally a ProfileNode corresponds to a symbol in a call stack.
    The root is special, its children are events and the events
    children are process names. After the process name come the
    samples.

    Attributes:
        name (str): The name of the function, process or event.
        value (int): The sample count for this node including counts from its
                     children.
        parent (ProfileNode): The parent of this node, this node belongs to its
                              children.
        children (Dict[str, ProfileNode]): A dictionary of child nodes, keyed by
                                           their names.
    """
    def __init__(self, name: str, parent: "ProfileNode"):
        """Initializes a ProfileNode."""
        self.name = name
        self.value: int = 0
        self.parent = parent if parent else self
        self.children: Dict[str, ProfileNode] = {}

    def find_or_create_node(self, name: str) -> "ProfileNode":
        """Finds a child node by name or creates it if it doesn't exist."""
        if name in self.children:
            return self.children[name]
        child = ProfileNode(name, self)
        self.children[name] = child
        return child

    def depth(self) -> int:
        """The maximum depth of the call stack tree from this node down."""
        if not self.children:
            return 1
        return max(child.depth() for child in self.children.values()) + 1

    def process_event(self, sample) -> None:
        """Processes a single profiling event to update the call stack tree.

        Args:
            sample: a single profiling sample.
        """
        pid = sample.sample_pid
        try:
            assert session
            thread = session.find_thread(sample.sample_tid)
            comm = thread.comm()
        except Exception:
            comm = f"unknown ({pid})"

        period = sample.sample_period
        self.value += period

        node = self.find_or_create_node(comm)
        node.value += period

        if sample.callchain:
            for entry in reversed(sample.callchain):
                name = entry.symbol
                if not name or name == "[unknown]":
                    name = entry.dso or "unknown"
                    if entry.ip:
                        name += f" 0x{entry.ip:x}"
                node = node.find_or_create_node(name)
                node.value += period
        else:
            name = sample.symbol
            if not name or name == "[unknown]":
                name = sample.dso or "unknown"
                if sample.sample_ip:
                    name += f" 0x{sample.sample_ip:x}"
            node = node.find_or_create_node(name)
            node.value += period

    def add_to_tree(self, node: TreeNode, root_value: int) -> None:
        """Recursively adds this node and its children to a textual TreeNode.

        Args:
            node (TreeNode): The textual `TreeNode` object to which this
                             ProfileNode should be added.
            root_value (int): Value at the root of the tree.
        """
        if root_value == 0:
            root_value = self.value

        # Calculate the percentage for the node, highlighting the
        # percentage with reversed colors.
        if root_value != 0:
            percent = self.value / root_value * 100
            label = f"{self.name} [r]{percent:.3g}%[/]"
        else:
            label = self.name

        # Add a standalone leaf.
        if not self.children:
            node.add_leaf(label)
            return

        # Recursively add children.
        new_node = node.add(label)
        for pnode in sorted(self.children.values(),
                            key=lambda pnode: pnode.value, reverse=True):
            pnode.add_to_tree(new_node, root_value)

    def largest_child(self) -> "ProfileNode":
        """Finds the child with the highest value (sample count)."""
        if self.children:
            return max(self.children.values(), key=lambda node: node.value)
        return self

    def child_after(self, sought: "ProfileNode") -> "ProfileNode":
        """Finds the next sibling after the given node, sorted by value."""
        found = False
        for child in sorted(self.children.values(), key=lambda node: node.value,
                            reverse=True):
            if child == sought:
                found = True
            elif found:
                return child
        return sought

    def child_before(self, sought: "ProfileNode") -> "ProfileNode":
        """Finds the previous sibling before the given node, sorted by value."""
        last = None
        for child in sorted(self.children.values(), key=lambda node: node.value,
                            reverse=True):
            if child == sought:
                return last if last else sought
            last = child
        return sought

    def has_parent(self, parent: "ProfileNode") -> bool:
        """Checks if the parent node is an ancestor of this node."""
        p = self.parent
        while True:
            if p == parent:
                return True
            new_p = p.parent
            if new_p == p:
                break
            p = new_p
        return False

    def has_child(self, sought: "ProfileNode") -> bool:
        """Checks if the sought node is a descendant of this node."""
        return sought.has_parent(self)

    def flame_walk(self, wanted_strip: int, cur_strip: int, parent_width: int,
                   selected: "ProfileNode", visitor: FlameVisitor) -> None:
        """Recursively walks the tree to visit a single flame graph row.

        This method calculates the proportional width for each child
        based on its value (sample count) relative to its parent. It
        then invokes a `visitor` to process each segment of the flame
        graph row.

        Args:
            wanted_strip (int): The target depth (Y-axis) of the flame graph row
                                to generate.
            cur_strip (int): The current depth of the traversal.
            parent_width (int): The width of the parent of this node.
            selected (ProfileNode): The currently selected node in the UI, used
                                    to adjust rendering to highlight the
                                    selected path.
            visitor (FlameVisitor): A visitor object whose `visit` method is
                                    called for each segment of the flame graph
                                    row.
        """
        if parent_width == 0:
            return

        parent_selected = selected == self or self.has_parent(selected)
        child_selected = not parent_selected and self.has_child(selected)
        if not parent_selected and not child_selected:
            # Branches of the tree with no node selected aren't drawn.
            return

        # left_over is used to check for a gap after the children due
        # to samples being in the parent.
        left_over = parent_width
        for child in sorted(self.children.values(), key=lambda node: node.value,
                            reverse=True):
            if parent_selected:
                if self.value:
                    desired_width = int((parent_width * child.value) / self.value)
                else:
                    desired_width = parent_width // len(self.children)
                if desired_width == 0:
                    # Nothing can be drawn for this node or later smaller children.
                    break
            elif child == selected or child.has_child(selected):
                desired_width = parent_width
            else:
                # A sibling or its child are selected, but not this branch.
                continue

            # Either visit the wanted_strip or recurse to the next level.
            if wanted_strip == cur_strip:
                visitor.visit(child, desired_width)
            else:
                child.flame_walk(wanted_strip, cur_strip + 1, desired_width,
                                 selected, visitor)
            left_over -= desired_width
            if left_over == 0:
                # No space left to draw in.
                break

        # Always visit the left_over regardless of the wanted_strip as there
        # may be additional gap added to a line by a parent.
        if left_over:
            visitor.visit(None, left_over)

    def make_flame_strip(self, wanted_strip: int, parent_width: int,
                         cursor: "ProfileNode", selected: "ProfileNode",
                         theme_variables: Dict[str, str]) -> Strip:
        """Creates a renderable 'Strip' for a single row of a flame graph.

        This method orchestrates the `flame_walk` traversal with a specialized
        visitor to generate a list of segments. The segments are used by a`Strip`
        object for rendering in the terminal.

        Args:
            wanted_strip (int): The target depth (Y-axis) of the flame graph row.
            parent_width (int): The total width (in characters) of the display
                                area.
            cursor (ProfileNode): The node currently under the cursor, for
                                  highlighting.
            selected (ProfileNode): The node that is actively selected.
            theme_variables(Dict): Values of colors for the textual theme.

        Returns:
            Strip: A renderable strip of segments for the specified row.
        """
        primary = Color.parse(theme_variables["primary"])
        secondary = Color.parse(theme_variables["secondary"])
        surface = Color.parse(theme_variables["surface"])
        def luminance(color: Color) -> float:
            """Computes the luminance of a color from the rgb"""
            return color.r * 0.299 + color.g * 0.587 + color.b * 0.114

        # Set of styles for different flamegraph segments, the styles are
        # cycled through to provide contrast.
        normal_styles = []
        for x in range(0, 125, 25):
            fgcolor = secondary.blend(primary, x/100)
            if luminance(fgcolor) > luminance(surface):
                bgcolor = surface.lighten(0.05+x/500)
            else:
                bgcolor = surface.darken(0.05+x/500)
            normal_styles.append(Style(color=fgcolor.rich_color,
                                       bgcolor=bgcolor.rich_color))

        # Style for the selected flame graph node.
        accent = Color.parse(theme_variables["accent"])
        accent_muted = Color.parse(theme_variables["accent-muted"])
        cursor_style = Style(color=accent.rich_color, bgcolor=accent_muted.rich_color)

        class StripVisitor(FlameVisitor):
            """Visitor creating textual flame graph segments.

            Attributes:
                segments (list): The textual segments that will be placed in a
                                 `Strip`.
                gap_width (int): The width of any outstanding gap between the
                                 last and next node.
                ctr (int): Used to adjust the flame graph segment's color.
            """
            def __init__(self):
                self.segments = []
                self.gap_width = 0
                self.ctr = wanted_strip

            def visit(self, node: Optional[ProfileNode], width: int) -> None:
                if node:
                    if self.gap_width > 0:
                        self.segments.append(Segment(
                            make_fixed_length_string(" ", self.gap_width)))
                        self.gap_width = 0
                    style = cursor_style
                    if node != cursor:
                        style = normal_styles[self.ctr % len(normal_styles)]
                    self.segments.append(Segment(
                        make_fixed_length_string(node.name, width), style))
                else:
                    self.gap_width += width
                self.ctr += 1

        visitor = StripVisitor()
        self.flame_walk(wanted_strip, 0, parent_width, selected, visitor)
        return Strip(visitor.segments) if visitor.segments else Strip.blank(parent_width)

    def find_node(self, sought_x: int, sought_y: int, parent_width: int,
                  selected: "ProfileNode") -> "ProfileNode":
        """Finds the ProfileNode corresponding to specific X, Y coordinates.

        This translates a mouse click on a flame graph back to the
        `ProfileNode` that it represents.

        Args:
            sought_x (int): The X coordinate (character column).
            sought_y (int): The Y coordinate (row or depth).
            parent_width (int): The total width of the display area.
            selected (ProfileNode): The currently selected node, which affects
                                    layout.

        Returns:
            Optional[ProfileNode]: The node found at the coordinates, or None.

        """
        class FindVisitor(FlameVisitor):
            """Visitor locating a `ProfileNode`.

            Attributes:
                x (int): offset within line.
                found (Optional[ProfileNode]): located node
                gap_width (int): The width of any outstanding gap between the
                                 last and next node.
                ctr (int): Used to adjust the flame graph segment's color.
            """
            def __init__(self):
                self.x = 0
                self.found = None

            def visit(self, node: Optional[ProfileNode], width: int) -> None:
                if self.x <= sought_x < self.x + width:
                    self.found = node
                self.x += width

        visitor = FindVisitor()
        self.flame_walk(sought_y, 0, parent_width, selected, visitor)
        return visitor.found


class FlameGraph(ScrollView):
    """A scrollable widget to display a flame graph from a profile.

    Attributes:
        root (ProfileNode): Root of the profile tree.
        cursor (ProfileNode): Currently highlighted cursor node.
        selected (ProfileNode): The currently selected node for zooming.
    """

    # Define key bindings for navigating the flame graph.
    # Allows movement with vim-style keys (h,j,k,l) and arrow keys.
    BINDINGS = [
        Binding("j,down", "move_down", "Down", key_display="↓",
                tooltip="Move cursor down to largest child"),
        Binding("k,up", "move_up", "Up", key_display="↑",
                tooltip="Move cursor up to parent"),
        Binding("l,right", "move_right", "Right", key_display="→",
                tooltip="Move cursor to the right sibling"),
        Binding("h,left", "move_left", "Left", key_display="←",
                tooltip="Move cursor to the left sibling"),
        Binding("enter", "zoom_in", "Zoom In",
                tooltip="Expand the cursor's node to be screen width"),
        Binding("escape", "zoom_out", "Zoom Out",
                tooltip="Zoom out to initial view."),
     ]

    # Default CSS for the widget to ensure it fills its container's width.
    DEFAULT_CSS = """
    FlameGraph {
        width: 100%;
    }
    """

    def __init__(self, root: ProfileNode, *args, **kwargs):
        """Initialize the FlameGraph widget."""
        super().__init__(*args, **kwargs)
        self.root = root
        self.cursor = root
        self.selected = root

    def action_move_down(self) -> None:
        """Handle key press down."""
        self.cursor = self.cursor.largest_child()
        self.refresh()

    def action_move_up(self) -> None:
        """Handle key press up."""
        if self.cursor.parent != self.cursor.parent.parent:
            self.cursor = self.cursor.parent
            self.refresh()

    def action_move_right(self) -> None:
        """Handle key press right."""
        self.cursor = self.cursor.parent.child_after(self.cursor)
        self.refresh()

    def action_move_left(self) -> None:
        """Handle key press left."""
        self.cursor = self.cursor.parent.child_before(self.cursor)
        self.refresh()

    def action_zoom_in(self) -> None:
        """Handle key press zoom in."""
        self.selected = self.cursor
        self.refresh()

    def action_zoom_out(self) -> None:
        """Handle key press zoom out."""
        self.selected = self.root
        self.refresh()

    def render_line(self, y: int) -> Strip:
        """Render a single line (row) of the flame graph."""
        _, scroll_y = self.scroll_offset
        y += scroll_y
        return self.root.make_flame_strip(y, self.size.width, self.cursor,
                                          self.selected, self.app.theme_variables)

    def on_mount(self) -> None:
        """Set the height of the widget when it is displayed."""
        self.styles.height = self.root.depth()

    def on_click(self, click: events.Click) -> None:
        """Handles a mouse click and update the cursor position."""
        _, scroll_y = self.scroll_offset
        y = scroll_y + click.y
        clicked_node = self.root.find_node(click.x, y, self.size.width,
                                           self.selected)
        if clicked_node:
            self.cursor = clicked_node
            self.refresh()


class ReportApp(App):
    """A Textual application to display profiling data."""

    # The ^q binding is implied but having it here adds it in the Footer.
    BINDINGS = [
        Binding(key="^q", action="quit", description="Quit",
                tooltip="Quit the app"),
    ]

    def __init__(self, root: ProfileNode):
        """Initialize the application."""
        super().__init__()
        self.root = root

    def make_report_tree(self) -> Tree:
        """Make a Tree widget from the profile data."""
        tree: Tree[None] = Tree("Profile")
        # Add events to tree skipping the root.
        for pnode in sorted(self.root.children.values(),
                            key=lambda node: node.value, reverse=True):
            pnode.add_to_tree(tree.root, root_value=0)

        # Expand the root tree (shows all events) and the largest of the children
        # for each event.
        def expand_first_child(tnode: TreeNode) -> None:
            """Recursively expand the first child node"""
            if not tnode.children:
                return
            first = tnode.children[0]
            first.expand()
            expand_first_child(first)
        tree.root.expand()
        for tnode in tree.root.children:
            expand_first_child(tnode)

        # If there is only one event, expand it also.
        if len(tree.root.children) == 1:
            tree.root.children[0].expand()

        return tree

    def compose(self) -> ComposeResult:
        """Composes the user interface of the application."""
        yield Header()
        with TabbedContent(initial="report"):
            with TabPane("Report", id="report"):
                yield self.make_report_tree()
            with TabPane("Flame Graph", id="flame"):
                yield FlameGraph(self.root)
        yield Footer()


class ProfileBuilder:
    """Constructs a profile tree from a stream of events."""
    def __init__(self):
        self.root = ProfileNode("root", parent=None)

    def process_event(self, sample) -> None:
        """Called by session.process_events to update the profile tree."""
        ev_name = str(sample.evsel)[6:-1]
        ev_root = self.root.find_or_create_node(ev_name)
        ev_root.process_event(sample)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="TUI report and flame graph using perf python module.")
    parser.add_argument("-i", "--input", help="input perf.data file")
    args = parser.parse_args()

    input_file = args.input or "perf.data"
    if not os.path.exists(input_file):
        print(f"Error: {input_file} not found. (try 'perf record' first)", file=sys.stderr)
        sys.exit(1)

    profile = ProfileBuilder()
    try:
        session = perf.session(perf.data(input_file), sample=profile.process_event)
    except Exception as e:
        print(f"Error opening session: {e}", file=sys.stderr)
        sys.exit(1)

    # profile.process_event is called for each perf event to build the profile.
    session.process_events()

    # Visualize data.
    app = ReportApp(profile.root)
    app.run()
