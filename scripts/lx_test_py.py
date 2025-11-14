#!/usr/bin/env python3
import lxros as lx
from std_msgs.msg import String


def cb1(msg: String):
    print("node1 heard:", msg.data)


def cb2(msg: String):
    print("node2 heard:", msg.data)


def main():
    node1 = lx.init_node("node1")
    node2 = lx.init_node("node2")

    node1.sub("chatter", String, cb1)
    pub2 = node2.pub("chatter2", String)

    node1.info("node1 started in namespace %s", node1.namespace)
    node2.info("node2 started")

    # publish something
    node1.info(f'TEST: {lx.ok()}')
    while lx.ok():
        pub2.publish(data="hello from node2")        
        lx.sleep(0.02)
        lx.spin_once(timeout=0.001)

    #lx.spin()  # spins both nodes in shared MultiThreadedExecutor


if __name__ == "__main__":
    main()
