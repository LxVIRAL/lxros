#!/usr/bin/env python3
import lxros as lx
import rclpy
from std_msgs.msg import String

a=''

def cb1(msg: String):
    global a
    lx.get_context()._nodes[0].logger.info(f"node1 heard: {msg.data}")
    rclpy.logging.get_logger('whatevs').info(f"!!node1 heard: {msg.data}")
    a=msg.data


def cb2(msg: String):
    print("node2 heard:", msg.data)


class MyClass():
    def __init__(self):
        self.counter = 0
        self.node=lx.init_node("node3")
        self.node.sub("chatter2", String, self.cb)

    def cb(self, msg: String):
        self.counter += 1
        self.node.info(f"!!!!!!node3 heard ({self.counter}): {msg.data}")

    

def main():

    
    node1 = lx.init_node("node1")
    node2 = lx.init_node("node2")

    cl=MyClass()


    node1.sub("chatter", String, cb1)
    pub2 = node2.pub("chatter2", String)

    node1.info("node1 started in namespace %s", node1.namespace)
    node2.info("node2 started")

    # publish something
    while lx.ok():
        node1.info(f"publishing {a} from main()")
        pub2.publish(data=a)        
        lx.spin_once(timeout=0.001)
        lx.sleep(0.1)
        

    #lx.spin()  # spins both nodes in shared MultiThreadedExecutor


if __name__ == "__main__":
    main()
