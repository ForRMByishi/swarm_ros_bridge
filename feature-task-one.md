# 通信控制功能配置说明

本文面向使用者，说明本仓库新增通信控制配置项的含义、使用方式和可能产生的影响。阅读本文不需要了解内部实现细节，只需要知道如何配置、配置后系统会怎样表现。

## 功能概览

新增功能主要分为三类：

- 发送带宽限制：通过 `max_bitrate` 控制某个发送话题的最大发送码率。
- 模拟节点断链：通过 `fault_policy`、`node_fault_start_after`、`node_fault_duration` 模拟节点级网络中断期间的消息处理。
- 远程触发断链：通过 `control_enabled` 等配置开启控制端口，再用 `trigger_fault.py` 从外部触发指定节点断链。

这些功能默认不改变原有通信行为。只有显式配置对应字段后才会生效。

## 发送带宽限制

`max_bitrate` 是 `send_topics` 内的可选字段，用于限制某个发送话题的序列化 ROS payload 发送码率。

示例：

```yaml
send_topics:
- topic_name: /string
  msg_type: std_msgs/String
  max_freq: 1000
  max_bitrate: "100k"
  srcIP: self
  srcPort: 3003
```

配置含义：

- 不配置 `max_bitrate`：不限制带宽。
- `max_bitrate: 0`：不限制带宽。
- `max_bitrate: 100000`：限制为 100000 bps。
- `max_bitrate: "100k"`：限制为 100000 bps。
- `max_bitrate: "1m"`：限制为 1000000 bps。

单位规则：

- `1k = 1000`
- `1m = 1000k = 1000000`
- `k` 和 `m` 不区分大小写，例如 `"100K"`、`"1M"` 都可以。

行为影响：

- 当发送速率低于 `max_bitrate` 时，消息基本按原流程发送。
- 当发送速率超过 `max_bitrate` 时，消息会在发送端排队并延迟发送。
- 如果队列积压过多，系统会丢弃最早进入队列的旧消息，避免无人车控制链路收到过期数据。

建议：

- 实时控制类话题优先使用较高 `max_bitrate`，避免人为引入过大延迟。
- 状态、日志、低频感知类话题更适合做带宽限制。
- 如果只想限制消息频率，继续使用原有 `max_freq` 即可。

## 模拟节点断链

节点断链模拟用于测试类似 WiFi 切换 4G、短时网络不可用、节点通信临时中断等场景。它是节点级效果：故障期间，该 bridge 的发送侧和接收侧都会按各话题配置处理消息。

### 固定启动演示

可以通过全局参数让 bridge 启动后自动触发一次断链：

```yaml
node_fault_start_after: 5.0
node_fault_duration: 2.0
```

配置含义：

- `node_fault_start_after`：bridge 启动多少秒后进入模拟断链。
- `node_fault_duration`：模拟断链持续多少秒。

不配置 `node_fault_duration` 或配置为非正数时，不会自动触发断链。

### 话题故障策略

`fault_policy` 是 `send_topics` 和 `recv_topics` 内的可选字段，用于决定模拟断链期间该话题消息如何处理。

发送侧示例：

```yaml
send_topics:
- topic_name: /string
  msg_type: std_msgs/String
  max_freq: 1000
  fault_policy: buffer
  srcIP: self
  srcPort: 3003
```

接收侧示例：

```yaml
recv_topics:
- topic_name: /string_recv
  msg_type: std_msgs/String
  fault_policy: buffer
  srcIP: node_a
  srcPort: 3003
```

可选值：

- `drop`：故障期间直接丢弃消息。
- `buffer`：故障期间缓存消息，恢复后回放。
- `latest`：故障期间只保留最新一条消息，恢复后只发送或提交这一条。

默认值：

- 不配置 `fault_policy` 时，默认是 `drop`。

策略选择建议：

- 强实时控制话题：建议 `drop` 或 `latest`，避免恢复后执行过期控制指令。
- 状态快照话题：建议 `latest`，恢复后拿到最新状态即可。
- 日志、事件、低频业务数据：可以使用 `buffer`，尽量保留故障期间的数据。

恢复行为：

- 发送侧 `buffer` 缓存会在恢复后加速发送，当前按话题 `max_freq` 的 2 倍节奏回放。
- 接收侧 `buffer` 缓存会在恢复后加速提交给本机 ROS 上层，目标是在约半个故障时长内清空缓存。
- 恢复期间会优先释放缓存，再处理新到达消息，减少消息顺序歧义。

## 远程触发节点断链

远程触发适合多车、多节点或 Docker/WiFi 局域网测试场景。每个被控 bridge 可以开启一个轻量控制端口，实验控制机通过 IP 和端口触发指定节点断链。

### 被控节点配置

在被控节点 YAML 中配置：

```yaml
control_enabled: true
control_node_id: node_a
control_bind_ip: '*'
control_bind_port: 3999
control_token: ""
```

配置含义：

- `control_enabled`：是否开启远程控制服务。默认关闭。
- `control_node_id`：当前节点的逻辑 ID，用于匹配远程命令中的 `--target`。
- `control_bind_ip`：控制服务监听地址。通常使用 `'*'` 监听全部网卡。
- `control_bind_port`：控制服务 TCP 端口。
- `control_token`：可选共享 token。为空字符串 `""` 表示不校验 token。

注意：

- 不要写成 `control_token:` 空值形式。ROS1 参数服务器不能稳定处理 YAML null，建议写 `control_token: ""` 或直接注释掉。
- 同一台宿主机上运行多个 bridge 且共享网络命名空间时，`control_bind_port` 不能重复。
- Docker 非 host 网络下，不同容器有各自网络命名空间，容器内部端口可以相同；如果映射到宿主机，则宿主机端口不能冲突。

### 触发命令

无 token 示例：

```bash
rosrun swarm_ros_bridge trigger_fault.py \
  --ip 172.28.0.2 \
  --port 3999 \
  --duration 2.0 \
  --target node_a
```

带 token 示例：

```bash
rosrun swarm_ros_bridge trigger_fault.py \
  --ip 192.168.1.101 \
  --port 3999 \
  --duration 2.0 \
  --target node_a \
  --token test-lab-token
```

参数含义：

- `--ip`：被控节点 IP。
- `--port`：被控节点 `control_bind_port`。
- `--duration`：本次断链持续时间，单位秒。
- `--target`：目标节点逻辑 ID，应与被控节点 `control_node_id` 一致。
- `--token`：可选 token，应与被控节点 `control_token` 一致。

成功响应示例：

```text
ok=true;node_id=node_a;message=node_fault_started;duration_sec=2
```

常见失败响应：

- `target_mismatch`：`--target` 与节点 `control_node_id` 不一致。
- `unauthorized`：token 不匹配。
- `invalid_duration_sec`：`--duration` 非法或小于等于 0。
- `node_fault_busy`：节点正在故障或正在恢复回放，暂时拒绝新的故障触发。

## 配置模板建议

发送端示例：

```yaml
IP:
  self: '*'
  node_a: 172.28.0.2
  node_b: 172.28.0.3

send_topics:
- topic_name: /string
  msg_type: std_msgs/String
  max_freq: 1000
  # max_bitrate: "100k"
  # fault_policy: buffer
  srcIP: self
  srcPort: 3003

recv_topics: []
```

接收端示例：

```yaml
IP:
  self: '*'
  node_a: 172.28.0.2
  node_b: 172.28.0.3

send_topics: []

recv_topics:
- topic_name: /string_recv
  msg_type: std_msgs/String
  # fault_policy: buffer
  srcIP: node_a
  srcPort: 3003
```

更完整的 node_a/node_b 示例可参考：

- `config/bw_node_a.yaml`
- `config/bw_node_b.yaml`

## 简要实现说明

新增功能位于 bridge 的通信控制层。发送消息在真正写入 ZMQ 之前，会经过频率控制、可选带宽控制和可选故障策略处理。接收消息在从 ZMQ 收到后、发布到本机 ROS topic 前，也会经过故障策略处理。

远程控制使用一个独立的 ZMQ REP 控制端口。控制端口只负责接收触发命令和校验参数，真正的断链期间缓存、丢弃和恢复逻辑仍由通信控制模块统一执行。

因此，正常情况下不配置这些新字段时，bridge 会保持原有行为；只有开启对应配置后，才会引入限速、模拟故障或远程触发能力。
