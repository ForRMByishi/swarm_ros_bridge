#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import String

bytes_sum = 0
msg_sum = 0
last_seq = None
lost_sum = 0

def cb(msg):
    global bytes_sum, msg_sum, last_seq, lost_sum

    data = msg.data
    bytes_sum += len(data.encode('utf-8'))
    msg_sum += 1

    try:
        seq = int(data.split(',', 1)[0])
        if last_seq is not None and seq > last_seq + 1:
            lost_sum += seq - last_seq - 1
        last_seq = seq
    except Exception:
        pass

def main():
    global bytes_sum, msg_sum, lost_sum

    rospy.init_node('bw_sub', anonymous=True)
    rospy.Subscriber('/string_recv', String, cb, queue_size=10000)

    last_t = time.time()

    while not rospy.is_shutdown():
        time.sleep(1.0)
        now = time.time()
        dt = now - last_t
        mbps = bytes_sum * 8.0 / dt / 1e6
        mps = msg_sum / dt

        print(f'recv_payload={mbps:.3f} Mbps, recv_rate={mps:.1f} msg/s, lost_seq={lost_sum}')

        bytes_sum = 0
        msg_sum = 0
        lost_sum = 0
        last_t = now

if __name__ == '__main__':
    main()
