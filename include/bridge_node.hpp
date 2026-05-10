/**
 * @file bridge_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file of bridge_node.cpp
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __BRIDGE_NODE__
#define __BRIDGE_NODE__
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <set>
#include <unistd.h>
/*
zmqpp is the c++ wrapper around ZeroMQ
Intall zmqpp first:
    sudo apt install libzmqpp-dev
zmqpp reference link:
    https://zeromq.github.io/zmqpp/namespacezmqpp.html
*/
#include "bridge_state.hpp"
#include "ros_sub_pub.hpp"

/**
 * Checks whether a send-topic message would exceed the configured max_freq.
 *
 * Parameters:
 *   i: Index into sendTopics and frequency-control state vectors.
 *
 * Returns:
 *   true if the caller should discard this message; false if it may be forwarded.
 *
 * Side Effects:
 *   Updates frequency-control counters for the topic.
 */
bool send_freq_control(int i);

/**
 * Runs the receive loop for one configured ZMQ SUB socket.
 *
 * Parameters:
 *   i: Index into recvTopics and receive-thread state vectors.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   Receives remote payloads and submits them to the local ROS publishing path.
 */
void recv_func(int i);

// ***************** stop send/receive ******************************
/**
 * Stops one configured send path.
 *
 * Parameters:
 *   i: Index into sendTopics, topic_subs, and senders.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   Shuts down local subscription, bitrate-control thread, and ZMQ PUB socket.
 */
void stop_send(int i);

/**
 * Stops one configured receive path.
 *
 * Parameters:
 *   i: Index into recvTopics, receivers, and topic_pubs.
 *
 * Returns:
 *   None.
 *
 * Side Effects:
 *   Requests receive-loop exit, closes ZMQ SUB socket, and unadvertises ROS publisher.
 */
void stop_recv(int i);

#endif
