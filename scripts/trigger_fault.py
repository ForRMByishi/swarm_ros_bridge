#!/usr/bin/env python3
"""
File: trigger_fault.py

Purpose:
    Sends a remote control request to one swarm_ros_bridge node and triggers a
    simulated node-level network fault for a configured duration.

Responsibilities:
    - Build the lightweight key-value control request used by control_server.cpp.
    - Connect to the target node's ZMQ REP control endpoint.
    - Print the bridge response for lab scripts and manual tests.

Notes:
    The target bridge must enable control_enabled and expose control_bind_port
    in its private ROS/YAML parameters.
"""

import argparse
import sys

try:
    import zmq
except ImportError:  # pragma: no cover - depends on the user's ROS/Python env.
    zmq = None


def build_request(duration_sec: float, token: str, target: str) -> str:
    """
    Builds the key-value request string expected by the C++ control server.

    Parameters:
        duration_sec:
            Fault duration in seconds. It must be positive.
        token:
            Optional shared control token. Empty string means no token field is sent.
        target:
            Optional node id filter. Empty string means the receiving endpoint decides.

    Returns:
        Semicolon-separated key-value request text.

    Side Effects:
        None.
    """
    fields = [
        "command=start_node_fault",
        f"duration_sec={duration_sec}",
    ]
    if token:
        fields.append(f"token={token}")
    if target:
        fields.append(f"target={target}")
    return ";".join(fields)


def send_request(ip: str, port: int, request_text: str, timeout_ms: int) -> str:
    """
    Sends one control request to a bridge node and waits for its response.

    Parameters:
        ip:
            Target node IP address or hostname.
        port:
            Target control port.
        request_text:
            Key-value request string.
        timeout_ms:
            Send/receive timeout in milliseconds.

    Returns:
        Response text returned by the bridge node.

    Raises:
        RuntimeError:
            If pyzmq is unavailable or the request times out.

    Side Effects:
        Opens a short-lived ZMQ REQ socket.
    """
    if zmq is None:
        raise RuntimeError("pyzmq is not installed in this Python environment")

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)

    endpoint = f"tcp://{ip}:{port}"
    try:
        socket.connect(endpoint)
        socket.send_string(request_text)
        return socket.recv_string()
    except zmq.Again as exc:
        raise RuntimeError(f"request to {endpoint} timed out") from exc
    finally:
        socket.close()
        context.term()


def parse_args() -> argparse.Namespace:
    """
    Parses command-line arguments for the remote fault trigger tool.

    Parameters:
        None.

    Returns:
        Parsed argparse namespace.

    Side Effects:
        Reads sys.argv and may print argparse validation errors.
    """
    parser = argparse.ArgumentParser(
        description="Trigger a simulated node-level network fault on one swarm_ros_bridge node."
    )
    parser.add_argument("--ip", required=True, help="Target bridge control IP address.")
    parser.add_argument("--port", type=int, default=3999, help="Target bridge control port.")
    parser.add_argument("--duration", type=float, required=True, help="Fault duration in seconds.")
    parser.add_argument("--token", default="", help="Optional shared control token.")
    parser.add_argument("--target", default="", help="Optional target control_node_id.")
    parser.add_argument("--timeout-ms", type=int, default=2000, help="Request timeout in milliseconds.")
    return parser.parse_args()


def main() -> int:
    """
    CLI entry point.

    Parameters:
        None.

    Returns:
        Process exit code. 0 means a response was received; non-zero means local validation
        or communication failed.

    Side Effects:
        Sends one network control request and prints the response.
    """
    args = parse_args()
    if args.duration <= 0.0:
        print("duration must be greater than 0", file=sys.stderr)
        return 1
    if args.port <= 0 or args.port > 65535:
        print("port must be in 1..65535", file=sys.stderr)
        return 1

    request_text = build_request(args.duration, args.token, args.target)
    try:
        response_text = send_request(args.ip, args.port, request_text, args.timeout_ms)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    print(response_text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
