# swarm_ros_bridge Docker 三节点一发布两订阅测试脚本

## 拓扑

```text
Docker 网络 swarmtest: 172.28.0.0/24

node_a    172.28.0.2    发布 /string
  |
  | ZMQ/TCP port 3003
  |
  +--> node_b  172.28.0.3    接收为 /string_recv
  |
  +--> node_c  172.28.0.4    接收为 /string_recv
```

`node_a` 仍然只发布 `/string`，`node_b` 仍然像之前一样订阅 `node_a:3003`，新增的 `node_c` 也订阅 `node_a:3003`，形成一发布两订阅场景。

## 使用方式

```bash
unzip swarm_ros_bridge_docker_test_scripts_3nodes.zip
cd swarm_ros_bridge_docker_test_scripts_3nodes
chmod +x *.sh
./run_all_prepare.sh
```

然后开三个终端：

```bash
./05_run_subscriber_b.sh
./05_run_subscriber_c.sh
./06_run_publisher.sh
```

默认宿主机源码目录：

```bash
/home/lixueyang/code/swarm_ros_bridge
```

默认镜像名：

```bash
swarm_ros_bridge:noetic
```

如果你使用包含 pyzmq 的镜像：

```bash
IMAGE_NAME=swarm_ros_bridge:noetic-pyzmq ./run_all_prepare.sh
```

## 验证点

两个接收端都会输出：

```text
recv_payload=... Mbps, recv_rate=... msg/s, lost_seq=..., seq_range=...
```

重点看 `node_b` 和 `node_c` 的 `seq_range` 是否大致同步，`recv_payload` 是否接近。

如果你设置了发送端总限速，例如：

```bash
MAX_BANDWIDTH_KBPS=512 ./03_write_test_files.sh
./04_start_ros_and_bridge.sh
```

那么 `node_b` 和 `node_c` 各自看到的流量应接近你的发送端实际发出的同一份发布流。注意容器网络真实总转发量会因为两个订阅者接收而接近两份 TCP 流。

## 常用命令

修改发送压力：

```bash
PUB_RATE_HZ=1000 PAYLOAD_BYTES=8192 ./06_run_publisher.sh
```

修改代码后重新编译并重启：

```bash
./08_recompile_and_restart_bridge.sh
```

查看日志：

```bash
./07_show_logs.sh
```

查看 topic：

```bash
./10_check_topics.sh
```

清理容器：

```bash
./09_cleanup.sh
```
