#!/usr/bin/env python3
"""Generate a read-only ROS 2 + Gazebo system graph.

The tool inspects the live ROS 2 graph, the live Gazebo Transport topic list,
and this repository's ros_gz_bridge YAML files. It writes a Graphviz DOT file
and, when Graphviz is installed, an SVG rendering.
"""

from __future__ import annotations

import argparse
import html
import re
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


DEFAULT_OUTPUT_NAME = "ros_gz_system_graph"
IMPORTANT_GZ_TOPICS = {
    "/lidar2",
    "/model/x3_lidar/pose",
    "/X3/gazebo/command/motor_speed",
}
PRESENTATION_ROS_TOPICS = {
    "/laser_scan",
    "/tf",
    "/X3/gazebo/command/motor_speed",
    "/flight_controller/manual_reference_delta",
    "/flight_controller/emergency_stop",
}
PRESENTATION_GZ_EXTRA_PATTERNS: tuple[re.Pattern[str], ...] = ()
ROS_INTERNAL_TOPICS = {
    "/parameter_events",
    "/rosout",
}
PARAMETER_SERVICE_SUFFIXES = (
    "/describe_parameters",
    "/get_parameter_types",
    "/get_parameters",
    "/list_parameters",
    "/set_parameters",
    "/set_parameters_atomically",
)
FLOW_STYLES = {
    "sensor data": {"color": "#2a9d8f", "penwidth": "2.0"},
    "pose feedback": {"color": "#d62828", "penwidth": "2.6"},
    "motor command": {"color": "#d97706", "penwidth": "2.6"},
    "manual reference": {"color": "#6c757d", "penwidth": "1.8"},
    "emergency stop": {"color": "#6c757d", "penwidth": "1.8"},
    "static transforms": {"color": "#6c757d", "penwidth": "1.8"},
    "visualization": {"color": "#6c757d", "penwidth": "1.5", "style": "dashed"},
}
DEBUG_FLOW_STYLES = {
    "publishes": {"color": "#1976d2", "penwidth": "1.4"},
    "subscribes": {"color": "#2d6a4f", "penwidth": "1.4"},
    "bridge": {"color": "#7b2cbf", "penwidth": "1.8"},
    "service": {"color": "#b00020", "penwidth": "1.2"},
    "action": {"color": "#9d0208", "penwidth": "1.2"},
}
SVG_HOVER_STYLE_ID = "ros-gz-system-graph-hover-style"
SVG_HOVER_STYLE = f"""<style id="{SVG_HOVER_STYLE_ID}" type="text/css"><![CDATA[
g.edge path,
g.edge polygon {{
  cursor: help;
  pointer-events: visiblePainted;
  transition: stroke 120ms ease, fill 120ms ease, stroke-width 120ms ease, filter 120ms ease;
}}

g.edge:hover path {{
  stroke: #ff6d00 !important;
  stroke-width: 4px !important;
  filter: drop-shadow(0 0 4px rgba(255, 109, 0, 0.85));
}}

g.edge:hover polygon {{
  stroke: #ff6d00 !important;
  fill: #ff6d00 !important;
  filter: drop-shadow(0 0 4px rgba(255, 109, 0, 0.85));
}}
]]></style>"""


@dataclass
class CommandResult:
    command: list[str]
    ok: bool
    stdout: str = ""
    stderr: str = ""
    warning: str = ""


@dataclass
class BridgeMapping:
    source_file: Path
    ros_topic_name: str
    gz_topic_name: str
    ros_type_name: str
    gz_type_name: str
    direction: str


@dataclass
class NodeInfo:
    name: str
    publishers: dict[str, str] = field(default_factory=dict)
    subscribers: dict[str, str] = field(default_factory=dict)
    service_servers: dict[str, str] = field(default_factory=dict)
    service_clients: dict[str, str] = field(default_factory=dict)
    action_servers: dict[str, str] = field(default_factory=dict)
    action_clients: dict[str, str] = field(default_factory=dict)


@dataclass
class GraphData:
    ros_topics: dict[str, str]
    ros_nodes: dict[str, NodeInfo]
    ros_services: dict[str, str]
    ros_actions: dict[str, str]
    gz_topics: dict[str, str]
    bridge_mappings: list[BridgeMapping]
    warnings: list[str] = field(default_factory=list)


@dataclass
class GraphOptions:
    view: str
    include_services: bool
    include_gazebo_internal: bool
    include_ros_internal: bool


@dataclass
class RenderGraph:
    ros_topics: dict[str, str]
    ros_nodes: dict[str, NodeInfo]
    ros_services: dict[str, str]
    ros_actions: dict[str, str]
    gz_topics: dict[str, str]
    bridge_mappings: list[BridgeMapping]


def run_command(args: list[str], timeout_s: float = 6.0) -> CommandResult:
    if shutil.which(args[0]) is None:
        return CommandResult(
            command=args,
            ok=False,
            warning=f"Command not found: {args[0]}",
        )

    try:
        completed = subprocess.run(
            args,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout if isinstance(exc.stdout, str) else ""
        stderr = exc.stderr if isinstance(exc.stderr, str) else ""
        return CommandResult(
            command=args,
            ok=False,
            stdout=stdout,
            stderr=stderr,
            warning=f"Command timed out after {timeout_s:.1f}s: {' '.join(args)}",
        )
    except OSError as exc:
        return CommandResult(
            command=args,
            ok=False,
            warning=f"Could not run {' '.join(args)}: {exc}",
        )

    warning = ""
    if completed.returncode != 0:
        warning = (
            f"Command returned {completed.returncode}: {' '.join(args)}"
        )
    elif not completed.stdout.strip():
        warning = f"Command returned no output: {' '.join(args)}"

    return CommandResult(
        command=args,
        ok=completed.returncode == 0,
        stdout=completed.stdout,
        stderr=completed.stderr,
        warning=warning,
    )


def summarize_stderr(stderr: str) -> str:
    text = stderr.strip()
    if not text:
        return ""
    if "PermissionError" in text:
        return (
            "Permission denied while inspecting the graph. "
            "Run this tool from a normal sourced terminal if the sandbox blocks ROS/Gazebo sockets."
        )
    if "error in getifaddrs" in text:
        return (
            "Gazebo Transport could not inspect network interfaces. "
            "This usually means Gazebo is not running or the environment blocks socket inspection."
        )

    for line in text.splitlines():
        line = line.strip()
        if line:
            return line[:240]
    return ""


def append_command_warning(warnings: list[str], result: CommandResult) -> None:
    if result.warning:
        warnings.append(result.warning)
    detail = summarize_stderr(result.stderr)
    if detail:
        warnings.append(detail)


def parse_typed_list(output: str) -> dict[str, str]:
    entries: dict[str, str] = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        match = re.match(r"^(\S+)(?:\s+\[(.+)\])?$", line)
        if match:
            entries[match.group(1)] = match.group(2) or ""
        else:
            entries[line] = ""
    return entries


def parse_ros_node_info(name: str, output: str) -> NodeInfo:
    info = NodeInfo(name=name)
    section = ""
    section_targets = {
        "Subscribers": info.subscribers,
        "Publishers": info.publishers,
        "Service Servers": info.service_servers,
        "Service Clients": info.service_clients,
        "Action Servers": info.action_servers,
        "Action Clients": info.action_clients,
    }

    for raw_line in output.splitlines():
        stripped = raw_line.strip()
        if not stripped:
            continue
        if stripped.endswith(":"):
            section = stripped[:-1]
            continue
        if section not in section_targets:
            continue

        match = re.match(r"^(.+?):\s+(.+)$", stripped)
        if match:
            section_targets[section][match.group(1)] = match.group(2)

    return info


def discover_bridge_yaml_files(repo_root: Path) -> list[Path]:
    known = [
        repo_root / "src/lidar_mapping_drone_bringup/config/bridge_lidar.yaml",
        repo_root / "src/lidar_mapping_drone_control/config/motor_bridge.yaml",
    ]
    discovered = [
        path
        for path in sorted(repo_root.glob("src/**/config/*bridge*.yaml"))
        if path.is_file()
    ]

    ordered: list[Path] = []
    for path in known + discovered:
        if path.exists() and path not in ordered:
            ordered.append(path)
    return ordered


def parse_simple_bridge_yaml(path: Path) -> list[BridgeMapping]:
    """Parse the current ros_gz_bridge list-of-dictionaries YAML format.

    Assumptions: each bridge entry is a top-level list item and each field is a
    scalar `key: value` pair. Quoted strings are unquoted; nested structures are
    intentionally not supported because the project's bridge files do not use
    them.
    """

    mappings: list[BridgeMapping] = []
    current: dict[str, str] = {}

    def store_current() -> None:
        if not current:
            return
        required = {
            "ros_topic_name",
            "gz_topic_name",
            "ros_type_name",
            "gz_type_name",
            "direction",
        }
        if required.issubset(current):
            mappings.append(
                BridgeMapping(
                    source_file=path,
                    ros_topic_name=current["ros_topic_name"],
                    gz_topic_name=current["gz_topic_name"],
                    ros_type_name=current["ros_type_name"],
                    gz_type_name=current["gz_type_name"],
                    direction=current["direction"],
                )
            )

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        content = raw_line.split("#", 1)[0].rstrip()
        if not content.strip():
            continue
        stripped = content.lstrip()

        if stripped.startswith("- "):
            store_current()
            current = {}
            stripped = stripped[2:].strip()

        if ":" not in stripped:
            continue

        key, value = stripped.split(":", 1)
        value = value.strip()
        if (
            len(value) >= 2
            and value[0] == value[-1]
            and value[0] in {"'", '"'}
        ):
            value = value[1:-1]
        current[key.strip()] = value

    store_current()
    return mappings


def inspect_ros_graph(warnings: list[str]) -> tuple[dict[str, str], dict[str, NodeInfo], dict[str, str], dict[str, str]]:
    if shutil.which("ros2") is None:
        warnings.append("Command not found: ros2. Source ROS 2 before running this tool.")
        return {}, {}, {}, {}

    topic_result = run_command(["ros2", "topic", "list", "-t"])
    append_command_warning(warnings, topic_result)
    ros_topics = parse_typed_list(topic_result.stdout) if topic_result.ok else {}

    node_result = run_command(["ros2", "node", "list"])
    append_command_warning(warnings, node_result)

    ros_nodes: dict[str, NodeInfo] = {}
    if node_result.ok:
        for node in sorted(line.strip() for line in node_result.stdout.splitlines() if line.strip()):
            info_result = run_command(["ros2", "node", "info", node])
            append_command_warning(warnings, info_result)
            if info_result.ok:
                ros_nodes[node] = parse_ros_node_info(node, info_result.stdout)
            else:
                ros_nodes[node] = NodeInfo(name=node)

    service_result = run_command(["ros2", "service", "list", "-t"])
    append_command_warning(warnings, service_result)
    ros_services = parse_typed_list(service_result.stdout) if service_result.ok else {}

    action_result = run_command(["ros2", "action", "list", "-t"])
    append_command_warning(warnings, action_result)
    ros_actions = parse_typed_list(action_result.stdout) if action_result.ok else {}

    return ros_topics, ros_nodes, ros_services, ros_actions


def inspect_gazebo_topics(warnings: list[str], bridge_mappings: list[BridgeMapping]) -> dict[str, str]:
    gz_available = shutil.which("gz") is not None
    topics: dict[str, str] = {}
    for topic in IMPORTANT_GZ_TOPICS:
        topics.setdefault(topic, "important/configured")
    for mapping in bridge_mappings:
        if topics.get(mapping.gz_topic_name) in {None, "", "important/configured"}:
            topics[mapping.gz_topic_name] = mapping.gz_type_name

    if not gz_available:
        warnings.append("Command not found: gz. Gazebo topic inspection was skipped.")
        return topics

    list_result = run_command(["gz", "topic", "-l"])
    append_command_warning(warnings, list_result)
    if not list_result.ok:
        return topics

    live_topics = [
        line.strip()
        for line in list_result.stdout.splitlines()
        if line.strip()
    ] if list_result.ok else []
    for topic in live_topics:
        topics.setdefault(topic, "")

    for topic in sorted(topics):
        info_result = run_command(["gz", "topic", "-i", "-t", topic], timeout_s=3.0)
        append_command_warning(warnings, info_result)
        if not info_result.ok:
            continue
        topic_type = parse_gz_topic_type(info_result.stdout)
        if topic_type:
            topics[topic] = topic_type

    return topics


def parse_gz_topic_type(output: str) -> str:
    patterns = [
        r"Message Type:\s*([^\n]+)",
        r"Type:\s*([^\n]+)",
        r"Msg Type:\s*([^\n]+)",
    ]
    for pattern in patterns:
        match = re.search(pattern, output, flags=re.IGNORECASE)
        if match:
            return match.group(1).strip()
    return ""


def collect_graph_data(repo_root: Path) -> GraphData:
    warnings: list[str] = []
    bridge_files = discover_bridge_yaml_files(repo_root)
    bridge_mappings: list[BridgeMapping] = []

    for bridge_file in bridge_files:
        try:
            bridge_mappings.extend(parse_simple_bridge_yaml(bridge_file))
        except OSError as exc:
            warnings.append(f"Could not read {bridge_file}: {exc}")

    ros_topics, ros_nodes, ros_services, ros_actions = inspect_ros_graph(warnings)
    gz_topics = inspect_gazebo_topics(warnings, bridge_mappings)

    for mapping in bridge_mappings:
        ros_topics.setdefault(mapping.ros_topic_name, mapping.ros_type_name)

    return GraphData(
        ros_topics=ros_topics,
        ros_nodes=ros_nodes,
        ros_services=ros_services,
        ros_actions=ros_actions,
        gz_topics=gz_topics,
        bridge_mappings=bridge_mappings,
        warnings=warnings,
    )


def make_render_graph(data: GraphData, options: GraphOptions) -> RenderGraph:
    if options.view == "debug":
        return RenderGraph(
            ros_topics=data.ros_topics,
            ros_nodes=data.ros_nodes,
            ros_services=data.ros_services if options.include_services else {},
            ros_actions=data.ros_actions if options.include_services else {},
            gz_topics=data.gz_topics,
            bridge_mappings=data.bridge_mappings,
        )

    bridge_ros_topics = {mapping.ros_topic_name for mapping in data.bridge_mappings}
    bridge_gz_topics = {mapping.gz_topic_name for mapping in data.bridge_mappings}

    visible_ros_topics = set(PRESENTATION_ROS_TOPICS) | bridge_ros_topics
    visible_gz_topics = set(IMPORTANT_GZ_TOPICS) | bridge_gz_topics

    if options.include_ros_internal:
        visible_ros_topics.update(data.ros_topics)
    else:
        for topic in data.ros_topics:
            if topic in visible_ros_topics:
                continue
            if is_ros_internal_topic(topic):
                continue
            if topic_matches_main_node_io(topic, data.ros_nodes):
                visible_ros_topics.add(topic)

    if options.include_gazebo_internal:
        visible_gz_topics.update(data.gz_topics)
    else:
        for topic in data.gz_topics:
            if topic in visible_gz_topics or is_presentation_gz_extra(topic):
                visible_gz_topics.add(topic)

    visible_ros_nodes = {
        node_name
        for node_name, node_info in data.ros_nodes.items()
        if is_presentation_ros_node(node_name, node_info, visible_ros_topics)
    }
    if options.include_ros_internal:
        visible_ros_nodes.update(data.ros_nodes)

    # Expand topic visibility once around the visible nodes so keyboard/control
    # relationships appear when those nodes are actually running.
    for node_name in visible_ros_nodes:
        node_info = data.ros_nodes[node_name]
        for topic in set(node_info.publishers) | set(node_info.subscribers):
            if options.include_ros_internal or not is_ros_internal_topic(topic):
                visible_ros_topics.add(topic)

    ros_topics = {
        topic: data.ros_topics.get(topic, "")
        for topic in sorted(visible_ros_topics)
        if topic in data.ros_topics or topic in bridge_ros_topics
    }
    for mapping in data.bridge_mappings:
        if mapping.ros_topic_name in visible_ros_topics:
            ros_topics.setdefault(mapping.ros_topic_name, mapping.ros_type_name)

    gz_topics = {
        topic: data.gz_topics.get(topic, "")
        for topic in sorted(visible_gz_topics)
        if topic in data.gz_topics or topic in bridge_gz_topics
    }
    for mapping in data.bridge_mappings:
        if mapping.gz_topic_name in visible_gz_topics:
            gz_topics.setdefault(mapping.gz_topic_name, mapping.gz_type_name)

    ros_nodes = {
        node_name: data.ros_nodes[node_name]
        for node_name in sorted(visible_ros_nodes)
        if node_name in data.ros_nodes
        and (options.include_ros_internal or not is_ros_internal_node(node_name))
    }

    ros_services = {}
    ros_actions = {}
    if options.include_services:
        ros_services = {
            service: service_type
            for service, service_type in data.ros_services.items()
            if options.include_ros_internal or not is_ros_internal_service(service)
        }
        ros_actions = data.ros_actions

    return RenderGraph(
        ros_topics=ros_topics,
        ros_nodes=ros_nodes,
        ros_services=ros_services,
        ros_actions=ros_actions,
        gz_topics=gz_topics,
        bridge_mappings=data.bridge_mappings,
    )


def topic_matches_main_node_io(topic: str, nodes: dict[str, NodeInfo]) -> bool:
    for node_name, node_info in nodes.items():
        if not is_main_project_node(node_name):
            continue
        if topic in node_info.publishers or topic in node_info.subscribers:
            return True
    return False


def is_presentation_ros_node(node_name: str, node_info: NodeInfo, visible_topics: set[str]) -> bool:
    if is_ros_internal_node(node_name):
        return False
    if is_bridge_node(node_name):
        return False
    if is_main_project_node(node_name):
        return True
    if any(topic in visible_topics for topic in node_info.publishers):
        return True
    if any(topic in visible_topics for topic in node_info.subscribers):
        return True
    return False


def is_main_project_node(node_name: str) -> bool:
    normalized = node_name.strip("/")
    lower = normalized.lower()
    return (
        "flight_controller" in lower
        or "keyboard_control" in lower
        or "rviz" in lower
        or "static_transform_publisher" in lower
        or is_bridge_node(node_name)
    )


def is_bridge_node(node_name: str) -> bool:
    lower = node_name.strip("/").lower()
    return (
        "ros_gz_bridge" in lower
        or "parameter_bridge" in lower
        or lower in {
            "lidarsensor_and_modelpose_bridge",
            "motorcommand_bridge",
        }
    )


def is_ros_internal_topic(topic: str) -> bool:
    return topic in ROS_INTERNAL_TOPICS or topic.startswith("/_")


def is_ros_internal_node(node_name: str) -> bool:
    lower = node_name.lower()
    return (
        "transform_listener_impl" in lower
        or lower.startswith("/_")
        or lower.startswith("_")
    )


def is_ros_internal_service(service: str) -> bool:
    if service.endswith(PARAMETER_SERVICE_SUFFIXES):
        return True
    return service.endswith("/get_type_description")


def parameter_service_count(node_info: NodeInfo) -> int:
    return sum(
        1
        for service in node_info.service_servers
        if is_ros_internal_service(service)
    )


def is_gazebo_internal_topic(topic: str) -> bool:
    return (
        topic.startswith("/gui/")
        or topic == "/gazebo/resource_paths"
        or re.match(r"^/world/[^/]+/(scene|stats|light_config)(/.*)?$", topic) is not None
    )


def is_presentation_gz_extra(topic: str) -> bool:
    return any(pattern.match(topic) for pattern in PRESENTATION_GZ_EXTRA_PATTERNS)


def dot_escape(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def dot_id(prefix: str, value: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_]", "_", value.strip("/"))
    safe = safe or "root"
    return f"{prefix}_{safe}"


def label_with_type(name: str, type_name: str) -> str:
    if type_name:
        return f"{name}\n{type_name}"
    return name


def topic_label(topic: str, type_name: str, options: GraphOptions, kind: str) -> str:
    if options.view != "presentation" or kind != "ros":
        return label_with_type(topic, type_name)
    if topic == "/tf":
        return label_with_subtitle(topic, "dynamic transforms", type_name)
    if topic == "/tf_static":
        return label_with_subtitle(topic, "static transforms", type_name)
    return label_with_type(topic, type_name)


def label_with_subtitle(name: str, subtitle: str, type_name: str) -> str:
    if type_name:
        return f"{name}\n{subtitle}\n{type_name}"
    return f"{name}\n{subtitle}"


def html_node_label(
    title: str,
    *,
    subtitle: str = "",
    type_name: str = "",
    signal_lines: Iterable[tuple[str, str]] = (),
) -> str:
    rows = [html_label_row(title, point_size="11", bold=True)]
    if subtitle:
        rows.append(html_label_row(subtitle, point_size="9"))
    if type_name:
        rows.append(html_label_row(type_name, point_size="9"))
    for text, color in signal_lines:
        rows.append(html_label_row(text, point_size="9", color=color))
    return (
        '<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="2">'
        f"{''.join(rows)}"
        "</TABLE>"
    )


def html_label_row(
    text: str,
    *,
    point_size: str,
    color: str = "",
    bold: bool = False,
) -> str:
    attrs = f' POINT-SIZE="{point_size}"'
    if color:
        attrs += f' COLOR="{color}"'
    content = html.escape(text)
    if bold:
        content = f"<B>{content}</B>"
    return f"<TR><TD><FONT{attrs}>{content}</FONT></TD></TR>"


def presentation_gz_topic_label(topic: str, type_name: str, render: RenderGraph) -> str:
    return html_node_label(
        topic,
        type_name=type_name,
        signal_lines=presentation_gz_topic_signal_lines(topic, render),
    )


def presentation_ros_topic_label(topic: str, type_name: str, render: RenderGraph) -> str:
    subtitle = ""
    if topic == "/tf":
        subtitle = "dynamic transforms"
    elif topic == "/tf_static":
        subtitle = "static transforms"
    return html_node_label(
        topic,
        subtitle=subtitle,
        type_name=type_name,
        signal_lines=presentation_ros_topic_signal_lines(topic, render),
    )


def presentation_ros_node_label(node_name: str, node_info: NodeInfo) -> str:
    return html_node_label(
        node_name,
        subtitle=presentation_node_role(node_name),
        signal_lines=presentation_ros_node_signal_lines(node_name, node_info),
    )


def presentation_bridge_node_label(render: RenderGraph) -> str:
    return html_node_label(
        "ROS <-> Gazebo Bridge",
        subtitle=bridge_mapping_summary(render),
        signal_lines=presentation_bridge_signal_lines(render),
    )


def debug_gz_topic_label(topic: str, type_name: str, render: RenderGraph) -> str:
    return html_node_label(
        topic,
        type_name=type_name,
        signal_lines=debug_gz_topic_signal_lines(topic, render),
    )


def debug_ros_topic_label(topic: str, type_name: str, render: RenderGraph) -> str:
    return html_node_label(
        topic,
        type_name=type_name,
        signal_lines=debug_ros_topic_signal_lines(topic, render),
    )


def debug_ros_node_label(node_name: str, node_info: NodeInfo) -> str:
    return html_node_label(
        node_name,
        signal_lines=debug_ros_node_signal_lines(node_info),
    )


def debug_bridge_node_label(render: RenderGraph) -> str:
    return html_node_label(
        "ROS <-> Gazebo Bridge",
        signal_lines=[
            (
                f"bridge mappings: {len(render.bridge_mappings)}",
                DEBUG_FLOW_STYLES["bridge"]["color"],
            )
        ],
    )


def graph_gz_topic_label(topic: str, type_name: str, render: RenderGraph, options: GraphOptions) -> str:
    if options.view == "presentation":
        return presentation_gz_topic_label(topic, type_name, render)
    return debug_gz_topic_label(topic, type_name, render)


def graph_ros_topic_label(topic: str, type_name: str, render: RenderGraph, options: GraphOptions) -> str:
    if options.view == "presentation":
        return presentation_ros_topic_label(topic, type_name, render)
    return debug_ros_topic_label(topic, type_name, render)


def graph_ros_node_label(node_name: str, node_info: NodeInfo, options: GraphOptions) -> str:
    if options.view == "presentation":
        return presentation_ros_node_label(node_name, node_info)
    return debug_ros_node_label(node_name, node_info)


def graph_bridge_node_label(render: RenderGraph, options: GraphOptions) -> str:
    if options.view == "presentation":
        return presentation_bridge_node_label(render)
    return debug_bridge_node_label(render)


def emit_node(
    lines: list[str],
    node_id: str,
    label: str,
    *,
    html_label: bool = False,
    **attrs: str,
) -> None:
    rendered_parts: list[str] = []
    if html_label:
        rendered_parts.append(f"label=<{label}>")
    else:
        rendered_parts.append(f'label="{dot_escape(label)}"')
    rendered_parts.extend(
        f'{key}="{dot_escape(value)}"' for key, value in attrs.items()
    )
    rendered_attrs = ", ".join(rendered_parts)
    lines.append(f'    "{node_id}" [{rendered_attrs}];')


def emit_edge(lines: list[str], source: str, target: str, **attrs: str) -> None:
    rendered_attrs = ""
    if attrs:
        rendered_attrs = " [" + ", ".join(
            f'{key}="{dot_escape(value)}"' for key, value in attrs.items()
        ) + "]"
    lines.append(f'  "{source}" -> "{target}"{rendered_attrs};')


def edge_tooltip_attrs(source: str, target: str, signal: str) -> dict[str, str]:
    return {
        "tooltip": f"{source} -> {target}\nSignal: {signal}",
    }


def edge_signal_for_node(node_name: str, topic: str, relation: str) -> str:
    semantic_label = presentation_node_edge_label(node_name, topic, relation)
    if semantic_label:
        return semantic_label
    return relation


def edge_signal_for_bridge(mapping: BridgeMapping) -> str:
    semantic_label = presentation_bridge_label(mapping)
    if semantic_label:
        return semantic_label
    return "bridge mapping"


def sorted_topics(topics: dict[str, str], options: GraphOptions, kind: str) -> list[tuple[str, str]]:
    return sorted(
        topics.items(),
        key=lambda item: topic_sort_key(item[0], options, kind),
    )


def topic_sort_key(topic: str, options: GraphOptions, kind: str) -> tuple[int, str]:
    if options.view != "presentation":
        return (100, topic)

    if kind == "gz":
        order = {
            "/lidar2": 10,
            "/lidar2/points": 15,
            "/model/x3_lidar/pose": 20,
            "/X3/gazebo/command/motor_speed": 90,
        }
    else:
        order = {
            "/laser_scan": 10,
            "/tf": 20,
            "/tf_static": 25,
            "/flight_controller/manual_reference_delta": 60,
            "/flight_controller/emergency_stop": 65,
            "/X3/gazebo/command/motor_speed": 90,
        }
    return (order.get(topic, 50), topic)


def node_label(node_name: str, node_info: NodeInfo, options: GraphOptions) -> str:
    if options.view == "presentation":
        role = presentation_node_role(node_name)
        if role:
            return f"{node_name}\n{role}"
        return node_name

    count = parameter_service_count(node_info)
    if count:
        return f"{node_name}\nParameter services: {count}"
    return node_name


def presentation_gz_topic_signal_lines(topic: str, render: RenderGraph) -> list[tuple[str, str]]:
    entries: list[tuple[str, str]] = []
    for mapping in render.bridge_mappings:
        if mapping.gz_topic_name != topic:
            continue
        label = presentation_bridge_label(mapping)
        if not label:
            continue
        if mapping.direction.upper() == "GZ_TO_ROS":
            entries.append((label, "out"))
        elif mapping.direction.upper() == "ROS_TO_GZ":
            entries.append((label, "in"))
        else:
            entries.extend([(label, "in"), (label, "out")])
    return presentation_signal_lines(entries)


def presentation_ros_topic_signal_lines(topic: str, render: RenderGraph) -> list[tuple[str, str]]:
    entries: list[tuple[str, str]] = []
    for mapping in render.bridge_mappings:
        if mapping.ros_topic_name != topic:
            continue
        label = presentation_bridge_label(mapping)
        if not label:
            continue
        if mapping.direction.upper() == "GZ_TO_ROS":
            entries.append((label, "in"))
        elif mapping.direction.upper() == "ROS_TO_GZ":
            entries.append((label, "out"))
        else:
            entries.extend([(label, "in"), (label, "out")])

    for node_name, node_info in render.ros_nodes.items():
        if topic in node_info.publishers:
            label = presentation_node_edge_label(node_name, topic, "publishes")
            if label:
                entries.append((label, "in"))
        if topic in node_info.subscribers:
            label = presentation_node_edge_label(node_name, topic, "subscribes")
            if label:
                entries.append((label, "out"))

    return presentation_signal_lines(entries)


def presentation_ros_node_signal_lines(node_name: str, node_info: NodeInfo) -> list[tuple[str, str]]:
    entries: list[tuple[str, str]] = []
    for topic in node_info.publishers:
        label = presentation_node_edge_label(node_name, topic, "publishes")
        if label:
            entries.append((label, "out"))
    for topic in node_info.subscribers:
        label = presentation_node_edge_label(node_name, topic, "subscribes")
        if label:
            entries.append((label, "in"))
    return presentation_signal_lines(entries)


def presentation_bridge_signal_lines(render: RenderGraph) -> list[tuple[str, str]]:
    entries: list[tuple[str, str]] = []
    for mapping in render.bridge_mappings:
        label = presentation_bridge_label(mapping)
        if not label:
            continue
        entries.extend([(label, "in"), (label, "out")])
    return presentation_signal_lines(entries)


def presentation_signal_lines(entries: Iterable[tuple[str, str]]) -> list[tuple[str, str]]:
    directions_by_label: dict[str, set[str]] = {}
    order: list[str] = []
    for label, direction in entries:
        if label not in directions_by_label:
            directions_by_label[label] = set()
            order.append(label)
        directions_by_label[label].add(direction)

    lines: list[tuple[str, str]] = []
    for label in order:
        directions = directions_by_label[label]
        if {"in", "out"}.issubset(directions):
            text = f"{label} in/out"
        elif "in" in directions:
            text = f"{label} in"
        elif "out" in directions:
            text = f"{label} out"
        else:
            text = label
        lines.append((text, FLOW_STYLES[label]["color"]))
    return lines


def debug_gz_topic_signal_lines(topic: str, render: RenderGraph) -> list[tuple[str, str]]:
    directions = debug_bridge_directions(topic, render, kind="gz")
    return debug_bridge_signal_lines(directions)


def debug_ros_topic_signal_lines(topic: str, render: RenderGraph) -> list[tuple[str, str]]:
    lines: list[tuple[str, str]] = []
    publisher_count = sum(1 for node in render.ros_nodes.values() if topic in node.publishers)
    subscriber_count = sum(1 for node in render.ros_nodes.values() if topic in node.subscribers)
    if publisher_count:
        lines.append((f"publishers: {publisher_count}", DEBUG_FLOW_STYLES["publishes"]["color"]))
    if subscriber_count:
        lines.append((f"subscribers: {subscriber_count}", DEBUG_FLOW_STYLES["subscribes"]["color"]))
    lines.extend(debug_bridge_signal_lines(debug_bridge_directions(topic, render, kind="ros")))
    return lines


def debug_ros_node_signal_lines(node_info: NodeInfo) -> list[tuple[str, str]]:
    lines: list[tuple[str, str]] = []
    if node_info.publishers:
        lines.append((f"publishes: {len(node_info.publishers)}", DEBUG_FLOW_STYLES["publishes"]["color"]))
    if node_info.subscribers:
        lines.append((f"subscribes: {len(node_info.subscribers)}", DEBUG_FLOW_STYLES["subscribes"]["color"]))
    if node_info.service_servers:
        lines.append((f"service servers: {len(node_info.service_servers)}", DEBUG_FLOW_STYLES["service"]["color"]))
    if node_info.service_clients:
        lines.append((f"service clients: {len(node_info.service_clients)}", DEBUG_FLOW_STYLES["service"]["color"]))
    action_count = len(node_info.action_servers) + len(node_info.action_clients)
    if action_count:
        lines.append((f"actions: {action_count}", DEBUG_FLOW_STYLES["action"]["color"]))
    return lines


def debug_bridge_directions(topic: str, render: RenderGraph, *, kind: str) -> set[str]:
    directions: set[str] = set()
    for mapping in render.bridge_mappings:
        mapping_topic = mapping.gz_topic_name if kind == "gz" else mapping.ros_topic_name
        if mapping_topic != topic:
            continue
        direction = mapping.direction.upper()
        if kind == "gz":
            if direction == "GZ_TO_ROS":
                directions.add("out")
            elif direction == "ROS_TO_GZ":
                directions.add("in")
            else:
                directions.update({"in", "out"})
        else:
            if direction == "GZ_TO_ROS":
                directions.add("in")
            elif direction == "ROS_TO_GZ":
                directions.add("out")
            else:
                directions.update({"in", "out"})
    return directions


def debug_bridge_signal_lines(directions: set[str]) -> list[tuple[str, str]]:
    if {"in", "out"}.issubset(directions):
        text = "bridge in/out"
    elif "in" in directions:
        text = "bridge in"
    elif "out" in directions:
        text = "bridge out"
    else:
        return []
    return [(text, DEBUG_FLOW_STYLES["bridge"]["color"])]


def presentation_node_role(node_name: str) -> str:
    lower = node_name.strip("/").lower()
    if "flight_controller" in lower:
        return "control node"
    if "keyboard_control" in lower:
        return "manual input node"
    if "rviz" in lower:
        return "visualization node"
    if "static_transform_publisher" in lower:
        return "static TF node"
    return ""


def graph_edge_attrs(options: GraphOptions, node_name: str, topic: str, relation: str) -> dict[str, str]:
    if options.view == "presentation":
        label = presentation_node_edge_label(node_name, topic, relation)
        attrs = presentation_flow_attrs(label)
        if label == "motor command":
            attrs["constraint"] = "false"
        return attrs

    if relation == "publishes":
        return dict(DEBUG_FLOW_STYLES["publishes"])
    if relation == "subscribes":
        return dict(DEBUG_FLOW_STYLES["subscribes"])
    return {"color": "#555555"}


def presentation_node_edge_label(node_name: str, topic: str, relation: str) -> str:
    lower = node_name.lower()
    if "flight_controller" in lower:
        if relation == "subscribes" and topic == "/tf":
            return "pose feedback"
        if relation == "publishes" and topic == "/X3/gazebo/command/motor_speed":
            return "motor command"
        if relation == "subscribes" and topic == "/flight_controller/manual_reference_delta":
            return "manual reference"
        if relation == "subscribes" and topic == "/flight_controller/emergency_stop":
            return "emergency stop"
    if "keyboard_control" in lower and relation == "publishes":
        if topic == "/flight_controller/manual_reference_delta":
            return "manual reference"
        if topic == "/flight_controller/emergency_stop":
            return "emergency stop"
    if "static_transform_publisher" in lower and relation == "publishes":
        if topic == "/tf_static":
            return "static transforms"
    if "rviz" in lower and relation == "subscribes":
        if topic == "/laser_scan":
            return "sensor data"
        if topic == "/tf":
            return "pose feedback"
        if topic == "/tf_static":
            return "static transforms"
    return ""


def presentation_flow_attrs(label: str, *, include_label: bool = False) -> dict[str, str]:
    attrs = dict(FLOW_STYLES.get(label, {"color": "#555555"}))
    if include_label and label:
        attrs["label"] = label
    return attrs


def bridge_edge_attrs(mapping: BridgeMapping, options: GraphOptions, segment: str) -> dict[str, str]:
    if options.view == "debug":
        return dict(DEBUG_FLOW_STYLES["bridge"])
    attrs = presentation_flow_attrs(presentation_bridge_label(mapping))
    if mapping.gz_topic_name == "/X3/gazebo/command/motor_speed" and options.view == "presentation":
        attrs["constraint"] = "false"
    return attrs


def service_action_edge_attrs(kind: str, relation: str, options: GraphOptions) -> dict[str, str]:
    style = "dashed" if relation in {"server", "action_server"} else "dotted"
    color = "#b00020" if kind == "service" else "#9d0208"
    return {
        "style": style,
        "color": color,
    }


def presentation_bridge_label(mapping: BridgeMapping) -> str:
    if mapping.gz_topic_name == "/lidar2":
        return "sensor data"
    if mapping.gz_topic_name == "/model/x3_lidar/pose":
        return "pose feedback"
    if mapping.gz_topic_name == "/X3/gazebo/command/motor_speed":
        return "motor command"
    return ""


def is_control_loop_node_edge(node_name: str, topic: str, relation: str) -> bool:
    lower = node_name.strip("/").lower()
    if "flight_controller" not in lower:
        return False
    return (
        relation == "subscribes" and topic == "/tf"
    ) or (
        relation == "publishes" and topic == "/X3/gazebo/command/motor_speed"
    )


def is_control_loop_bridge_mapping(mapping: BridgeMapping) -> bool:
    return (
        mapping.gz_topic_name == "/model/x3_lidar/pose"
        and mapping.ros_topic_name == "/tf"
    ) or (
        mapping.gz_topic_name == "/X3/gazebo/command/motor_speed"
        and mapping.ros_topic_name == "/X3/gazebo/command/motor_speed"
    )


def graph_html_label(render: RenderGraph, options: GraphOptions) -> str:
    subtitle = (
        "Gazebo + ROS 2 Presentation View"
        if options.view == "presentation"
        else "Gazebo + ROS 2 Debug View"
    )
    legend_cells = []
    if render.gz_topics:
        legend_cells.append(legend_cell("#ffd166", "GZ topic"))
    if render.bridge_mappings:
        legend_cells.append(legend_cell("#d0b3ff", "Bridge"))
    if render.ros_topics:
        legend_cells.append(legend_cell("#b7e4c7", "ROS topic"))
    if render.ros_nodes:
        legend_cells.append(legend_cell("#90caf9", "ROS node"))
    if render.ros_services:
        legend_cells.append(legend_cell("#ffccd5", "Service"))
    if render.ros_actions:
        legend_cells.append(legend_cell("#ffc8dd", "Action"))

    legend_row = ""
    if legend_cells:
        legend_row = (
            '<TR><TD>'
            '<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="5">'
            f"<TR>{''.join(legend_cells)}</TR>"
            "</TABLE>"
            "</TD></TR>"
        )

    return (
        "<<TABLE BORDER=\"0\" CELLBORDER=\"0\" CELLSPACING=\"0\">"
        "<TR><TD><FONT POINT-SIZE=\"18\"><B>LIDAR Mapping Drone Runtime Graph</B></FONT></TD></TR>"
        f"<TR><TD><FONT POINT-SIZE=\"14\">{html.escape(subtitle)}</FONT></TD></TR>"
        f"{legend_row}"
        "</TABLE>>"
    )


def legend_cell(color: str, label: str) -> str:
    return f'<TD BGCOLOR="{color}"><FONT POINT-SIZE="10">{html.escape(label)}</FONT></TD>'


def bridge_mapping_summary(render: RenderGraph) -> str:
    descriptions: list[str] = []
    for mapping in render.bridge_mappings:
        if mapping.gz_topic_name == "/lidar2":
            descriptions.append("LaserScan")
        elif mapping.gz_topic_name == "/model/x3_lidar/pose":
            descriptions.append("TF")
        elif mapping.gz_topic_name == "/X3/gazebo/command/motor_speed":
            descriptions.append("Motor Command")

    if descriptions:
        return " + ".join(unique(descriptions))
    return "No bridge mappings detected"


def quote_id(node_id: str) -> str:
    return f'"{node_id}"'


def emit_presentation_inferred_edges(lines: list[str], render: RenderGraph, options: GraphOptions) -> None:
    if options.view != "presentation":
        return

    for node_name in render.ros_nodes:
        if "rviz" not in node_name.lower():
            continue
        node_id = dot_id("ros_node", node_name)
        if "/tf" in render.ros_topics:
            emit_edge(
                lines,
                dot_id("ros_topic", "/tf"),
                node_id,
                **presentation_flow_attrs("pose feedback"),
                **edge_tooltip_attrs("/tf", node_name, "pose feedback"),
            )
        if "/tf_static" in render.ros_topics:
            emit_edge(
                lines,
                dot_id("ros_topic", "/tf_static"),
                node_id,
                **presentation_flow_attrs("static transforms"),
                **edge_tooltip_attrs("/tf_static", node_name, "static transforms"),
            )


def emit_presentation_layout_guides(lines: list[str], render: RenderGraph, options: GraphOptions) -> None:
    if options.view != "presentation":
        return

    gz_ids = [
        dot_id("gz_topic", topic)
        for topic, _ in sorted_topics(render.gz_topics, options, "gz")
    ]
    ros_topic_ids = [
        dot_id("ros_topic", topic)
        for topic, _ in sorted_topics(render.ros_topics, options, "ros")
    ]
    ros_node_ids = [
        dot_id("ros_node", node)
        for node in sorted(render.ros_nodes)
    ]

    for ids in (gz_ids, ["bridge_ros_gz_bridge"], ros_topic_ids, ros_node_ids):
        if ids:
            lines.append(f"  {{ rank=same; {'; '.join(quote_id(node_id) for node_id in ids)}; }}")

    guide_chain = [
        first_existing(
            [
                dot_id("gz_topic", "/lidar2"),
                dot_id("gz_topic", "/model/x3_lidar/pose"),
            ],
            gz_ids,
        ),
        "bridge_ros_gz_bridge",
        first_existing(
            [
                dot_id("ros_topic", "/laser_scan"),
                dot_id("ros_topic", "/tf"),
            ],
            ros_topic_ids,
        ),
        first_existing(
            [
                dot_id("ros_node", "/rviz"),
                dot_id("ros_node", "/flight_controller"),
            ],
            ros_node_ids,
        ),
    ]
    guide_chain = [node_id for node_id in guide_chain if node_id]
    for source, target in zip(guide_chain, guide_chain[1:]):
        emit_edge(lines, source, target, style="invis", weight="80")


def first_existing(candidates: list[str], existing: list[str]) -> str:
    existing_set = set(existing)
    for candidate in candidates:
        if candidate in existing_set:
            return candidate
    return existing[0] if existing else ""


def build_dot(data: GraphData, repo_root: Path, options: GraphOptions) -> str:
    render = make_render_graph(data, options)
    node_font_size = "11" if options.view == "presentation" else "10"
    edge_font_size = "9"
    ranksep = "1.15" if options.view == "presentation" else "0.85"
    nodesep = "0.65" if options.view == "presentation" else "0.45"
    graph_label = graph_html_label(render, options)
    lines: list[str] = [
        "digraph RosGzSystemGraph {",
        (
            "  graph [rankdir=LR, bgcolor=\"white\", pad=\"0.2\", "
            f"nodesep=\"{nodesep}\", ranksep=\"{ranksep}\", splines=\"ortho\", "
            f"label={graph_label}, labelloc=\"t\", labeljust=\"c\", fontsize=\"18\", fontname=\"DejaVu Sans\"];"
        ),
        f"  node [fontname=\"DejaVu Sans\", fontsize=\"{node_font_size}\", style=\"filled\"];",
        f"  edge [fontname=\"DejaVu Sans\", fontsize=\"{edge_font_size}\", color=\"#555555\", arrowsize=\"0.8\"];",
        "",
    ]

    lines.extend(
        [
            "  subgraph cluster_gazebo {",
            "    label=\"Gazebo Transport\";",
            "    color=\"#f4a261\";",
            "    style=\"rounded\";",
        ]
    )
    for topic, topic_type in sorted_topics(render.gz_topics, options, "gz"):
        emit_node(
            lines,
            dot_id("gz_topic", topic),
            graph_gz_topic_label(topic, topic_type, render, options),
            html_label=True,
            shape="ellipse",
            fillcolor="#ffd166",
            color="#cc7a00",
        )
    lines.append("  }")
    lines.append("")

    lines.extend(
        [
            "  subgraph cluster_bridge {",
            "    label=\"Bridge\";",
            "    color=\"#9d4edd\";",
            "    style=\"rounded\";",
        ]
    )
    emit_node(
        lines,
        "bridge_ros_gz_bridge",
        graph_bridge_node_label(render, options),
        html_label=True,
        shape="box",
        fillcolor="#d0b3ff",
        color="#6a00a8",
    )
    lines.append("  }")
    lines.append("")

    lines.extend(
        [
            "  subgraph cluster_ros_topics {",
            "    label=\"ROS 2 Topics\";",
            "    color=\"#2d6a4f\";",
            "    style=\"rounded\";",
        ]
    )
    for topic, topic_type in sorted_topics(render.ros_topics, options, "ros"):
        emit_node(
            lines,
            dot_id("ros_topic", topic),
            graph_ros_topic_label(topic, topic_type, render, options),
            html_label=True,
            shape="ellipse",
            fillcolor="#b7e4c7",
            color="#2d6a4f",
        )
    lines.append("  }")
    lines.append("")

    lines.extend(
        [
            "  subgraph cluster_ros_nodes {",
            "    label=\"ROS 2 Nodes\";",
            "    color=\"#1976d2\";",
            "    style=\"rounded\";",
        ]
    )
    for node, node_info in sorted(render.ros_nodes.items()):
        fill = "#90caf9"
        color = "#1565c0"
        if "bridge" in node:
            fill = "#d0b3ff"
            color = "#6a00a8"
        emit_node(
            lines,
            dot_id("ros_node", node),
            graph_ros_node_label(node, node_info, options),
            html_label=True,
            shape="box",
            fillcolor=fill,
            color=color,
        )
    lines.append("  }")
    lines.append("")

    if render.ros_services or render.ros_actions:
        lines.extend(
            [
                "  subgraph cluster_services_actions {",
                "    label=\"Services / Actions\";",
                "    color=\"#b00020\";",
                "    style=\"rounded\";",
            ]
        )
        for service, service_type in sorted(render.ros_services.items()):
            emit_node(
                lines,
                dot_id("ros_service", service),
                label_with_type(service, service_type),
                shape="diamond",
                fillcolor="#ffccd5",
                color="#b00020",
            )
        for action, action_type in sorted(render.ros_actions.items()):
            emit_node(
                lines,
                dot_id("ros_action", action),
                label_with_type(action, action_type),
                shape="octagon",
                fillcolor="#ffc8dd",
                color="#9d0208",
            )
        lines.append("  }")
        lines.append("")

    emit_presentation_layout_guides(lines, render, options)

    for mapping in render.bridge_mappings:
        gz_id = dot_id("gz_topic", mapping.gz_topic_name)
        ros_id = dot_id("ros_topic", mapping.ros_topic_name)
        if mapping.gz_topic_name not in render.gz_topics or mapping.ros_topic_name not in render.ros_topics:
            continue

        direction = mapping.direction.upper()
        bridge_signal = edge_signal_for_bridge(mapping)
        if direction == "GZ_TO_ROS":
            emit_edge(
                lines,
                gz_id,
                "bridge_ros_gz_bridge",
                **bridge_edge_attrs(mapping, options, "input"),
                **edge_tooltip_attrs(mapping.gz_topic_name, "ROS <-> Gazebo Bridge", bridge_signal),
            )
            emit_edge(
                lines,
                "bridge_ros_gz_bridge",
                ros_id,
                **bridge_edge_attrs(mapping, options, "output"),
                **edge_tooltip_attrs("ROS <-> Gazebo Bridge", mapping.ros_topic_name, bridge_signal),
            )
        elif direction == "ROS_TO_GZ":
            emit_edge(
                lines,
                ros_id,
                "bridge_ros_gz_bridge",
                **bridge_edge_attrs(mapping, options, "input"),
                **edge_tooltip_attrs(mapping.ros_topic_name, "ROS <-> Gazebo Bridge", bridge_signal),
            )
            emit_edge(
                lines,
                "bridge_ros_gz_bridge",
                gz_id,
                **bridge_edge_attrs(mapping, options, "output"),
                **edge_tooltip_attrs("ROS <-> Gazebo Bridge", mapping.gz_topic_name, bridge_signal),
            )
        else:
            attrs = bridge_edge_attrs(mapping, options, "input")
            emit_edge(
                lines,
                gz_id,
                "bridge_ros_gz_bridge",
                dir="both",
                **attrs,
                **edge_tooltip_attrs(mapping.gz_topic_name, "ROS <-> Gazebo Bridge", bridge_signal),
            )
            emit_edge(
                lines,
                "bridge_ros_gz_bridge",
                ros_id,
                dir="both",
                **attrs,
                **edge_tooltip_attrs("ROS <-> Gazebo Bridge", mapping.ros_topic_name, bridge_signal),
            )

    for node_name, node_info in sorted(render.ros_nodes.items()):
        node_id = dot_id("ros_node", node_name)
        for topic in node_info.publishers:
            if topic in render.ros_topics:
                emit_edge(
                    lines,
                    node_id,
                    dot_id("ros_topic", topic),
                    **graph_edge_attrs(options, node_name, topic, "publishes"),
                    **edge_tooltip_attrs(node_name, topic, edge_signal_for_node(node_name, topic, "publishes")),
                )
        for topic in node_info.subscribers:
            if topic in render.ros_topics:
                emit_edge(
                    lines,
                    dot_id("ros_topic", topic),
                    node_id,
                    **graph_edge_attrs(options, node_name, topic, "subscribes"),
                    **edge_tooltip_attrs(topic, node_name, edge_signal_for_node(node_name, topic, "subscribes")),
                )
        for service in node_info.service_servers:
            if service in render.ros_services:
                emit_edge(
                    lines,
                    node_id,
                    dot_id("ros_service", service),
                    **service_action_edge_attrs("service", "server", options),
                    **edge_tooltip_attrs(node_name, service, "service server"),
                )
        for service in node_info.service_clients:
            if service in render.ros_services:
                emit_edge(
                    lines,
                    node_id,
                    dot_id("ros_service", service),
                    **service_action_edge_attrs("service", "client", options),
                    **edge_tooltip_attrs(node_name, service, "service client"),
                )
        for action in node_info.action_servers:
            if action in render.ros_actions:
                emit_edge(
                    lines,
                    node_id,
                    dot_id("ros_action", action),
                    **service_action_edge_attrs("action", "action_server", options),
                    **edge_tooltip_attrs(node_name, action, "action server"),
                )
        for action in node_info.action_clients:
            if action in render.ros_actions:
                emit_edge(
                    lines,
                    node_id,
                    dot_id("ros_action", action),
                    **service_action_edge_attrs("action", "action_client", options),
                    **edge_tooltip_attrs(node_name, action, "action client"),
                )

    emit_presentation_inferred_edges(lines, render, options)

    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def add_svg_hover_styles(svg_path: Path) -> None:
    try:
        svg_text = svg_path.read_text(encoding="utf-8")
    except OSError:
        return
    if SVG_HOVER_STYLE_ID in svg_text:
        return

    match = re.search(r"<svg\b[^>]*>", svg_text)
    if not match:
        return

    svg_text = (
        svg_text[: match.end()]
        + "\n"
        + SVG_HOVER_STYLE
        + svg_text[match.end() :]
    )
    svg_path.write_text(svg_text, encoding="utf-8")


def write_outputs(dot_text: str, output_dir: Path, output_name: str) -> tuple[Path, Path, str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    dot_path = output_dir / f"{output_name}.dot"
    svg_path = output_dir / f"{output_name}.svg"
    dot_path.write_text(dot_text, encoding="utf-8")

    if shutil.which("dot") is None:
        return dot_path, svg_path, "Graphviz 'dot' was not found; SVG was not generated."

    result = run_command(["dot", "-Tsvg", str(dot_path), "-o", str(svg_path)], timeout_s=20.0)
    if not result.ok:
        detail = result.stderr.strip() or result.warning or "unknown Graphviz error"
        return dot_path, svg_path, f"Graphviz SVG generation failed: {detail}"
    add_svg_hover_styles(svg_path)
    return dot_path, svg_path, ""


def print_summary(
    data: GraphData,
    dot_path: Path,
    svg_path: Path,
    svg_warning: str,
    options: GraphOptions,
) -> None:
    print(f"Graph view: {options.view}")
    print(f"Wrote DOT: {dot_path}")
    if svg_warning:
        print(svg_warning)
    else:
        print(f"Wrote SVG: {svg_path}")

    print("\nDetected ROS 2 nodes:")
    print_list(data.ros_nodes.keys())

    print("\nDetected ROS 2 topics:")
    print_typed_list(data.ros_topics)

    print("\nDetected Gazebo topics:")
    print_typed_list(data.gz_topics)

    print("\nDetected bridge mappings:")
    if data.bridge_mappings:
        for mapping in data.bridge_mappings:
            if mapping.direction.upper() == "GZ_TO_ROS":
                summary = (
                    f"{mapping.gz_topic_name} ({mapping.gz_type_name}) "
                    f"-> {mapping.ros_topic_name} ({mapping.ros_type_name})"
                )
            elif mapping.direction.upper() == "ROS_TO_GZ":
                summary = (
                    f"{mapping.ros_topic_name} ({mapping.ros_type_name}) "
                    f"-> {mapping.gz_topic_name} ({mapping.gz_type_name})"
                )
            else:
                summary = (
                    f"{mapping.gz_topic_name} ({mapping.gz_type_name}) "
                    f"<-> {mapping.ros_topic_name} ({mapping.ros_type_name})"
                )
            print(f"  {mapping.direction}: {summary}")
    else:
        print("  none")

    if data.warnings:
        print("\nWarnings:")
        for warning in unique(data.warnings):
            print(f"  - {warning}")


def print_list(values: Iterable[str]) -> None:
    rendered = sorted(value for value in values if value)
    if not rendered:
        print("  none")
        return
    for value in rendered:
        print(f"  {value}")


def print_typed_list(values: dict[str, str]) -> None:
    if not values:
        print("  none")
        return
    for name, type_name in sorted(values.items()):
        suffix = f" [{type_name}]" if type_name else ""
        print(f"  {name}{suffix}")


def unique(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    output: list[str] = []
    for value in values:
        if value not in seen:
            seen.add(value)
            output.append(value)
    return output


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Generate a read-only ROS 2 + Gazebo system graph.",
    )
    parser.add_argument(
        "--view",
        choices=("presentation", "debug"),
        default="presentation",
        help=(
            "presentation draws the curated architecture map; "
            "debug draws the detailed live graph."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repo_root / "tools/generated",
        help="Directory for generated DOT/SVG files.",
    )
    parser.add_argument(
        "--output-name",
        default=None,
        help=(
            "Base filename for generated outputs. Defaults to "
            "ros_gz_system_graph for presentation and "
            "ros_gz_system_graph_debug for debug."
        ),
    )
    parser.add_argument(
        "--include-services",
        action="store_true",
        help="Include ROS 2 services/actions in presentation view.",
    )
    parser.add_argument(
        "--include-gazebo-internal",
        action="store_true",
        help="Include Gazebo GUI/world/internal topics in presentation view.",
    )
    parser.add_argument(
        "--include-ros-internal",
        action="store_true",
        help="Include ROS 2 internal topics/nodes/services in presentation view.",
    )
    args = parser.parse_args()
    args.repo_root = repo_root
    if args.output_name is None:
        args.output_name = (
            DEFAULT_OUTPUT_NAME
            if args.view == "presentation"
            else f"{DEFAULT_OUTPUT_NAME}_debug"
        )
    return args


def main() -> int:
    args = parse_args()
    options = GraphOptions(
        view=args.view,
        include_services=args.include_services or args.view == "debug",
        include_gazebo_internal=args.include_gazebo_internal or args.view == "debug",
        include_ros_internal=args.include_ros_internal or args.view == "debug",
    )
    data = collect_graph_data(args.repo_root)
    dot_text = build_dot(data, args.repo_root, options)
    dot_path, svg_path, svg_warning = write_outputs(
        dot_text,
        args.output_dir,
        args.output_name,
    )
    print_summary(data, dot_path, svg_path, svg_warning, options)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
