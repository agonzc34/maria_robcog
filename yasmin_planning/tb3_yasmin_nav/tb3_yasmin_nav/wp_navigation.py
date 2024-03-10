import rclpy

from typing import List
from nav2_msgs.action import FollowWaypoints
from simple_node import Node
from yasmin import State, StateMachine
from geometry_msgs.msg import PoseStamped

from threading import Event
from .locked_blackboard import LBlackboard as Blackboard
import random

from tb3_yasmin_msgs.srv import Mode


class RandomState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=[Mode.Request.SEQUENTIAL, Mode.Request.IDLE])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state RANDOM")
        finish = False
        goal = FollowWaypoints.Goal()
        pose = PoseStamped()
        waypoints = blackboard['waypoints']
        client = blackboard['nav_client']

        while not finish:
            random_waypoint = random.choice(waypoints)
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()

            pose.pose.position.x = float(random_waypoint[0])
            pose.pose.position.y = float(random_waypoint[1])

            goal.poses.append(pose)

            client.wait_for_server()
            client.send_goal(goal)
            client.wait_for_result()

            if client.is_succeeded():
                print("Waypoint reached")

            elif client.is_canceled():
                return blackboard.mode


class SequentialState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=[Mode.Request.RANDOM, Mode.Request.IDLE])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state SEQUENTIAL")
        finish = False
        latest_waypoint = 0
        waypoints = blackboard['waypoints']
        goal = FollowWaypoints.Goal()
        pose = PoseStamped()
        client = blackboard['nav_client']

        while not finish:
            next_waypoint = waypoints[latest_waypoint]
            latest_waypoint = (latest_waypoint + 1) % len(waypoints)
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()

            pose.pose.position.x = float(next_waypoint[0])
            pose.pose.position.y = float(next_waypoint[1])

            goal.poses.append(pose)

            client.wait_for_server()
            client.send_goal(goal)
            client.wait_for_result()

            if client.is_succeeded():
                print("Waypoint reached")

            elif client.is_canceled():
                return blackboard.mode
            
            
class IdleState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=[Mode.Request.RANDOM, Mode.Request.SEQUENTIAL, 'end'])

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state IDLE")
        idle_event: Event = blackboard['idle_event']
        idle_event.wait()
        idle_event.clear()
        
        return blackboard['mode']


class WaypointNavigation(Node):
    def __init__(self):
        super().__init__("waypoint_navigation")

        self.blackboard = Blackboard()
        
        self.blackboard['nav_client'] = self.create_action_client(FollowWaypoints, "follow_waypoints")
        self.blackboard['idle_event'] = Event()
        self.blackboard['waypoints'] = [(-2.0, -0.5), (0.2, -2), (0.4, -0.63), (1.5, 1.35), (-0.1, 1.90)]
        self.blackboard['mode'] = Mode.Request.IDLE
        
        self.create_service(Mode, "mode", self.mode_callback)
        
        sm = StateMachine(outcomes=["end_machine"])
        
        sm.add_state("IDLE", IdleState(), transitions={Mode.Request.RANDOM: "RANDOM", Mode.Request.SEQUENTIAL: "SEQUENTIAL", 'end': "end_machine"})
        sm.add_state("RANDOM", RandomState(self), transitions={Mode.Request.SEQUENTIAL: "SEQUENTIAL", Mode.Request.IDLE: "IDLE"})
        sm.add_state("SEQUENTIAL", SequentialState(self), transitions={Mode.Request.RANDOM: "RANDOM", Mode.Request.IDLE: "IDLE"})
        
        sm.execute(self.blackboard)
        

    def mode_callback(self, request: Mode.Request, response: Mode.Response):
        if request.mode not in [
            Mode.Request.RANDOM,
            Mode.Request.SEQUENTIAL,
            Mode.Request.IDLE,
        ]:
            response.result = f"Invalid mode {request.mode}"
            return response

        curr_mode = request.mode
        if curr_mode != self.blackboard.mode:
            print(f"Service: Mode changed to {curr_mode}")
            self.blackboard['mode'] = curr_mode
            
            self.blackboard['nav_client'].cancel_goal()
            
            if curr_mode != Mode.Request.IDLE:
                self.blackboard['idle_event'].set()

        response.result = "OK"
        return response


def main():
    rclpy.init()
    node = WaypointNavigation()
    node.join_spin()
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()