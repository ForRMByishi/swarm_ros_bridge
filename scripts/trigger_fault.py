#!/usr/bin/env python3
"""
File: trigger_fault.py

Purpose:
    向一个 swarm_ros_bridge 节点发送远程控制请求，触发指定时长的
    simulated node-level network fault。

Responsibilities:
    - 构造 control_server.cpp 使用的轻量 key-value 控制请求。
    - 连接目标节点的 ZMQ REP 控制端口。
    - 打印 bridge 响应，便于实验脚本和手动测试使用。

Notes:
    目标 bridge 必须在私有 ROS/YAML 参数中启用 control_enabled，并开放
    control_bind_port。
"""

import argparse
import sys

try:
    import zmq
except ImportError:  # pragma: no cover - 取决于用户 ROS/Python 环境。
    # pyzmq 是运行时依赖，缺失时在 send_request() 中给出明确错误。
    zmq = None


def build_request(duration_sec: float, token: str, target: str) -> str:
    """
    构造 C++ control server 期望的 key-value 请求字符串。

    Parameters:
        duration_sec:
            故障持续时间，单位秒，必须为正数。
        token:
            可选共享控制 token。空字符串表示不发送 token 字段。
        target:
            可选节点 ID 过滤字段。空字符串表示由接收端 IP/端口决定目标。

    Returns:
        分号分隔的 key-value 请求文本。

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
    向一个 bridge 节点发送控制请求并等待响应。

    Parameters:
        ip:
            目标节点 IP 地址或 hostname。
        port:
            目标控制端口。
        request_text:
            key-value 请求字符串。
        timeout_ms:
            发送/接收超时时间，单位毫秒。

    Returns:
        bridge 节点返回的响应文本。

    Raises:
        RuntimeError:
            pyzmq 不可用或请求超时时抛出。

    Side Effects:
        打开一个短生命周期的 ZMQ REQ socket。
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
        # REQ/REP 一次请求只发送一个字符串，便于人工抓包和脚本解析。
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
    解析远程故障触发工具的命令行参数。

    Parameters:
        None.

    Returns:
        argparse 解析后的 namespace。

    Side Effects:
        读取 sys.argv，并可能打印 argparse 校验错误。
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
    CLI 入口函数。

    Parameters:
        None.

    Returns:
        进程退出码。0 表示收到响应；非 0 表示本地校验或通信失败。

    Side Effects:
        发送一次网络控制请求并打印响应。
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
