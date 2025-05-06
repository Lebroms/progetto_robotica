import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from personal_interfaces.action import MoveHead

class MoveHeadClientNode(Node):
    def __init__(self):
        super().__init__("move_head_client")
        self.count_until_client_ = ActionClient(self, MoveHead, "move_head")
    
    # Wait for action server, send a goal, and register a callback for the response
    # Also register another callback for the optional feedback
    def send_goal(self, left_limit, right_limit,  step_angle):
        self.count_until_client_.wait_for_server()
        goal = MoveHead.Goal()
        goal.left_limit = left_limit
        goal.right_limit = right_limit
        goal.step_angle = step_angle
        self.count_until_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback). \
            add_done_callback(self.goal_response_callback)

    # Method to send a cancel request for the current goal
    def cancel_goal(self):
        self.get_logger().info("Send a cancel goal request")
        self.goal_handle_.cancel_goal_async()

    # Get the goal response and if accepted, register a callback for the result
    def goal_response_callback(self, future):
        self.goal_handle_ = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal got rejected")

    # Get the goal result and print it
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.success))

    # Get the goal feedback and print it
    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_angle
        self.get_logger().info("Got feedback: " + str(number))


def main(args=None):
    rclpy.init(args=args)
    node = MoveHeadClientNode()
    node.send_goal(-0.217, 0.217, 0.01)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()