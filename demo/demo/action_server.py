import time
import subprocess

# from argparse import Action
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from demo_interfaces.action import OT2Job

class DemoActionServer(Node):
    def __init__(self):
        super().__init__('demo_action_server')
        self._action_server = ActionServer(
            self, OT2Job, 
            'OT2', 
            self.action_callback
        )

    def action_callback(self, goal_handle):
        job = goal_handle.request.job
        pc_path = job.pc_path
        
        ## log/print the recieved Job
        self.get_logger().info("Recieved New Job Request...")

        self.get_logger().info(".. robot config path: {}".format(job.rc_path))
        self.get_logger().info(".. robot config: {}".format(job.robot_config))

        self.get_logger().info(".. protocol config path: {}".format(job.pc_path))
        self.get_logger().info(".. protocol config: {}".format(job.protocol_config))

###  KYLE COMMAND
# cmdpython3 ot2_driver_ssh.py -rc ./robot_config.yaml -pc ./document.yaml -v -s 

        # seq_str = [str(x) for x in seq_int]
        # self.get_logger().info("Received: [" +",".join(seq_str) +"]")
        # self.get_logger().info("Executing counting sequence...")

        # success = False
        # counts_total = sum(seq_int)
        # current_total = 0

        # phases_total = len(seq_int)
        # current_phase = 0 #1
        
        # feedback_msg = SequenceCounter.Feedback()
        # feedback_msg.phase_size = phases_total
    

        # for i in seq_int:
        #     phase_count_total = seq_int[current_phase]
        #     current_phase +=1
        #     current_phase_count = 0
        #     for j in range(i):
        #         current_phase_count += 1
        #         current_total += 1
                
        #         feedback_msg.total_count_progress = current_total
        #         feedback_msg.total_percentage_progress = round(float(current_total)/counts_total * 100, 3)
        #         feedback_msg.current_phase = current_phase
        #         feedback_msg.phase_count_progress = current_phase_count
        #         feedback_msg.phase_percentage_progress = round(float(current_phase_count)/phase_count_total * 100, 3)
                
        #         # self.get_logger().info('Progress: {0}, {1}'.format(feedback_msg.total_percentage_progress, feedback_msg.phase_percentage_progress))
        #         goal_handle.publish_feedback(feedback_msg)
                
        #         time.sleep(1)

        success = True
        goal_handle.succeed()

        result = OT2Job.Result()
        result.success = success
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = DemoActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()