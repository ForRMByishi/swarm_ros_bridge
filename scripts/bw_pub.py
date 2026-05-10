#!/usr/bin/env python3
import sys
import time
import rospy
from std_msgs.msg import String

def main():
    rate_hz = float(sys.argv[1]) if len(sys.argv) > 1 else 1000.0
    payload_bytes = int(sys.argv[2]) if len(sys.argv) > 2 else 2048

    rospy.init_node('bw_pub', anonymous=True)
    pub = rospy.Publisher('/string', String, queue_size=1000)

    rate = rospy.Rate(rate_hz)
    seq = 0

    # 等待 bridge 订阅 /string
    time.sleep(1.0)

    while not rospy.is_shutdown():
        prefix = f'{seq:012d},{time.time_ns()},'
        filler_len = max(0, payload_bytes - len(prefix))
        data = prefix + ('x' * filler_len)
        pub.publish(String(data=data))
        seq += 1
        rate.sleep()

if __name__ == '__main__':
    main()
