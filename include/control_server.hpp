/**
 * @file control_server.hpp
 * @brief 模拟通信故障的远程控制服务接口。
 *
 * Purpose:
 *   为 swarm_ros_bridge 提供基于 ZMQ 的轻量级控制面，用于远程触发节点级
 *   模拟网络故障。
 *
 * Responsibilities:
 *   - 在配置的 ZMQ REP socket 上监听简单 key-value 控制请求。
 *   - 校验目标节点标识、token 和故障持续时间。
 *   - 请求有效时触发 sim_comm 中的节点故障流程。
 *
 * Notes:
 *   请求协议有意保持无额外依赖：
 *   command=start_node_fault;duration_sec=2.0;token=...
 */

#ifndef __CONTROL_SERVER__
#define __CONTROL_SERVER__

#include <string>

struct ControlServerConfig
{
  bool enabled = false;
  std::string node_id;
  std::string bind_ip = "*";
  int bind_port = 3999;
  std::string token;
};

/**
 * 启动 ZMQ 控制服务线程。
 *
 * Parameters:
 *   config: 控制服务配置。
 *
 * Returns:
 *   服务未启用或启动成功时返回 true；配置非法时返回 false。
 *
 * Side Effects:
 *   enabled 为 true 时创建后台线程并绑定 ZMQ REP socket。
 */
bool start_control_server(const ControlServerConfig &config);

/**
 * 停止 ZMQ 控制服务线程。
 *
 * Parameters:
 *   None.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   唤醒并 join 后台控制线程。
 */
void stop_control_server();

#endif
