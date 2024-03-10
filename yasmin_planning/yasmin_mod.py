import time
import rclpy

from simple_node import Node

from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

from std_msgs.msg import String

from yasmin.blackboard import Blackboard


# define state Foo
class FooState(State):
    def __init__(self):
        super().__init__(["loop", "end"])
        self.counter = 0

    def execute(self, blackboard):
        print("Executing state FOO")
        time.sleep(3)

        if self.counter < 3:
            self.counter += 1
            blackboard.foo_str = f"Counter: {self.counter}"
            return "loop"
        else:
            return "end"


# define state Bar
class BarState(State):
    def __init__(self):
        super().__init__(outcomes=["loop"])

    def execute(self, blackboard):
        print("Executing state BAR")
        time.sleep(3)

        print(blackboard.foo_str)
        return "loop"


class BarFooState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=["loop"])
        self._pub = node.create_publisher(String, "yasmin_demo", 10)
        self._rate = node.create_rate(1)

    def execute(self, blackboard):
        print("Executing state FOOBAR")

        for i in range(4):
            self._rate.sleep()
            msg = String()
            msg.data = f"Hello {i}"
            self._pub.publish(msg)

        print(blackboard.foo_str)
        return "loop"


class DemoNode(Node):

    def __init__(self):
        super().__init__("yasmin_node")

        # create a state machine
        sm = StateMachine(outcomes=["end_machine"])

        # add states
        sm.add_state(
            "FOO", FooState(), transitions={"loop": "BAR", "end": "end_machine"}
        )
        sm.add_state("BAR", BarState(), transitions={"loop": "FOOBAR"})

        sm.add_state("FOOBAR", BarFooState(self), transitions={"loop": "FOO"})

        # pub
        YasminViewerPub(self, "YASMIN_DEMO", sm)

        blackboard = Blackboard()

        # execute
        outcome = sm(blackboard)
        print(outcome)

        print("bb:", blackboard["foo_str"])


# main
def main(args=None):

    print("yasmin_demo")
    rclpy.init(args=args)
    node = DemoNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
