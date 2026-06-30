#!/usr/bin/env python3
"""Guards the geofence parameter YAMLs against silent mis-wiring.

A ROS 2 parameter file section is keyed by the node's runtime name; if the
section name does not match the node, the node silently falls back to its
declared defaults. That class of bug already shipped once (the gridmap section
was named ``geofencing_gridmap_node`` while the node is ``geofence_gridmap_node``
-> the gridmap never received ``auto_activate`` and never activated). This test
fails fast on any node-section name that is not a real launched node.
"""

from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"

# The set of node names actually launched by rises_bringup/launch/geofence.launch.py
# (ComposableNode/LifecycleNode name=...). Any params section must target one of
# these, otherwise the params are silently dropped.
KNOWN_NODE_NAMES = {
    "geofence_node",
    "geofence_gridmap_node",
    "laserscan_preprocessor_node",
    "message_translator_node",
    "fleet_interface_node",
    "validation_node",
}

PARAM_FILES = sorted(CONFIG_DIR.glob("params_*.yaml"))


def _node_sections(node):
    """Yield (name, body) for every mapping that declares ros__parameters."""
    if not isinstance(node, dict):
        return
    for key, value in node.items():
        if isinstance(value, dict) and "ros__parameters" in value:
            yield key, value
        elif isinstance(value, dict):
            # Descend through wildcard prefixes such as "/**:".
            yield from _node_sections(value)


def test_param_files_exist():
    assert PARAM_FILES, f"no params_*.yaml found under {CONFIG_DIR}"


@pytest.mark.parametrize("path", PARAM_FILES, ids=lambda p: p.name)
def test_node_section_names_match_real_nodes(path):
    data = yaml.safe_load(path.read_text())
    sections = dict(_node_sections(data))
    assert sections, f"{path.name}: no node sections with ros__parameters found"

    unknown = sorted(set(sections) - KNOWN_NODE_NAMES)
    assert not unknown, (
        f"{path.name}: parameter section(s) {unknown} do not match any launched "
        f"node {sorted(KNOWN_NODE_NAMES)}. Such params are silently ignored — "
        f"rename the section to the node's runtime name."
    )
