import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from demo_interfaces.action import SequenceCounter

class DemoActionClient(Node):
    
    def __init__(self):
        super().__init__('demo_action_client')
        self._action_client = ActionClient(self, SequenceCounter, 'sequence_counter')
        # self.heartbeat_publisher = self.create_publisher()

    def send_goal(self, int_list):
        goal_msg = SequenceCounter.Goal()
        goal_msg.input_sequence = int_list
        
        self._action_client.wait_for_server()

        self.goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.goal_feedback_callback)
        self.goal_future.add_done_callback(self.goal_respond_callback)
        
        # return self._action_client.send_goal_async(goal_msg)


    def goal_feedback_callback(self,feed):
        feedback_msg = feed.feedback
        self.get_logger().info("Progress: {:.2f}".format(feedback_msg.total_percentage_progress))


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
        self.get_logger().info("Total Count: {0}".format(result.total_count))
        self.get_logger().info("Sequence Size: {0}".format(result.sequence_size))
        if not result.success:
            self.get_logger().info("Error Message: " + result.error_msg)
        # rclpy.shutdown()


def main(args=["seq"]):
    rclpy.init(args=args)
    demo_action_client = DemoActionClient()
    print("seq param")
    # demo_action_client.get_pa
    # print(demo_action_client.get_parameter("seq"))
    # rclpy.Parameter
    seq = demo_action_client.get_parameter_or("~seq", [5,10,15])
    future = demo_action_client.send_goal(seq)
    rclpy.spin(demo_action_client)

if __name__ == '__main__':
    main()