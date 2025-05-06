import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from personal_interfaces.action import MoveHead

class MoveHeadServerNode(Node):
    def __init__(self):
        super().__init__("move_head_server")
        self.move_camera_server_ = ActionServer(
            self,
            MoveHead,
            "move_head",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback)
        
        self.get_logger().info("Action server has been started.")
        self.joint_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10) # creazione del publisher con parametri di riferimento

    # Every new received goal will be processed here first
    # We can decide to accept or reject the incoming goal
    def goal_callback(self, goal_request: MoveHead.Goal):
        self.get_logger().info("Received a goal")
        if goal_request.left_limit < -0.217 or goal_request.right_limit > 0.217:
            self.get_logger().warn("Rejecting the goal, target must be in limits")
            return GoalResponse.REJECT
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
    
    # Any cancel request will be processed here, we can accept or reject it
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT

    # If a goal has been accepted, it will then be executed in this callback
    # After we are done with the goal execution we set a final state and return the result
    # When executing the goal we also check if we need to cancel it
    def execute_callback(self, goal_handle: ServerGoalHandle):
        
        left = goal_handle.request.left_limit
        right = goal_handle.request.right_limit
        step = goal_handle.request.step_angle

        feedback_msg = MoveHead.Feedback()

        angle=left
        while angle <= right:
            traj = JointTrajectory()
            traj.joint_names = ['head_1_joint', 'head_2_joint']
            point = JointTrajectoryPoint()
            point.positions = [angle, -0.57]
            point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
            traj.points.append(point)
            self.joint_pub.publish(traj)

            # Feedback
            feedback_msg.current_angle = angle
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Muovo a: {angle:.3f} rad')

            # Controllo se il goal è stato cancellato
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('Goal cancellato!')
                return MoveHead.Result(success=False)

            angle += step
            time.sleep(1.0)  # tempo tra un movimento e l'altro

        # Torna alla posizione iniziale (left_limit)
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [left, -0.57]  # ritorno alla posizione iniziale
        point.time_from_start = rclpy.duration.Duration(seconds=2.0).to_msg()
        traj.points.append(point)
        self.joint_pub.publish(traj)

        self.get_logger().info(f'Ritorno alla posizione iniziale: {left:.3f} rad')
        time.sleep(2.0)  # attesa per completare il movimento

        # Fine movimento
        goal_handle.succeed()
        result = MoveHead.Result()
        result.success = True
        self.get_logger().info('Scansione completata con successo.')
        return result 

def main(args=None):
    rclpy.init(args=args)
    node = MoveHeadServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()