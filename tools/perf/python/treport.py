#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""treport.py - perf report like tool written using textual."""
from typing import Dict, Optional
import argparse
import os
import sys
import perf
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Footer, Header, TabbedContent, TabPane, Tree
from textual.widgets.tree import TreeNode

# Global session.
session :Optional[perf.session] = None

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
    parser = argparse.ArgumentParser(description="TUI report using perf python module.")
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
