import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from demo_interfaces.action import OT2Job

class DemoActionClient(Node):
    
    def __init__(self):
        super().__init__('demo_action_client')
        self._action_client = ActionClient(self, OT2Job, 'OT2')
        # self.heartbeat_publisher = self.create_publisher()

    def send_goal(self, pc_path=None, rc_path=None, protocol_config="testing 1 2", robot_config="testing 3 4"):
        goal_msg = OT2Job.Goal()
        if rc_path is not None:
            goal_msg.job.rc_path = rc_path
        if pc_path is not None:
            goal_msg.job.pc_path = pc_path

        goal_msg.job.robot_config = robot_config
        goal_msg.job.protocol_config = protocol_config
        
        self._action_client.wait_for_server()

        print("sending goal")

        self.goal_future = self._action_client.send_goal_async(goal_msg) #, feedback_callback=self.goal_feedback_callback)
        self.goal_future.add_done_callback(self.goal_respond_callback)
        
        # return self._action_client.send_goal_async(goal_msg)


    def goal_feedback_callback(self,feed):
        pass
        # feedback_msg = feed.feedback
        # self.get_logger().info("Progress: {:.2f}".format(feedback_msg.total_percentage_progress))


    def goal_respond_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result:")
        self.get_logger().info("--success: {}".format(result.success))
        self.get_logger().info("--error message: {}".format(result.error_msg))
        # if not result.success:
        #     self.get_logger().info("Error Message: " + result.error_msg)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    demo_action_client = DemoActionClient()
    future = demo_action_client.send_goal()
    rclpy.spin(demo_action_client)

if __name__ == '__main__':
    main()