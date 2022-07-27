import time
import subprocess
import yaml
import os


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
        
        python_file_path = "/root/ot2_driver/ot2_driver/ot2_driver_ssh.py"
        rc_document_path = "/root/config/temp/rc_document.yaml"
        pc_document_path = "/root/config/temp/pc_document.yaml"

        job = goal_handle.request.job

        ## Comment out to test hard-coded job_path 
        pc_path = job.pc_path
        rc_path = job.rc_path
        simulate = job.simulate

        ## Uncomment to test out hard_coded job_path
        # simulate = True
        # rc_path = "/root/config/test_config.yaml"
        # pc_path = "/root/ot2_driver/ot2_driver/protopiler/example_configs/basic_config.yaml"
        
        robot_config = job.robot_config
        protocol_config = job.protocol_config
        
        ## log/print the recieved Job
        self.get_logger().info("Recieved New Job Request...")
        self.get_logger().info(".. robot config path: {}".format(job.rc_path))
        self.get_logger().info(".. robot config: {}".format(job.robot_config))
        self.get_logger().info(".. protocol config path: {}".format(job.pc_path))
        self.get_logger().info(".. protocol config: {}".format(job.protocol_config))

        path = '/root/config/temp'
        if not os.path.exists(path):
            os.mkdir(path)

        if job.rc_path=='None':
            rc_path = rc_document_path
            self.get_logger().info("write to {} file".format(rc_document_path))
            with open(rc_document_path, 'w',encoding = "utf-8") as rc_file:
                rc_file.write(job.robot_config)

        if job.pc_path=='None':
            pc_path = pc_document_path
            self.get_logger().info("write to {} file".format(pc_document_path))
            with open(pc_document_path,'w',encoding = "utf-8") as pc_file:
                pc_file.write(job.protocol_config)
        
        success = False
        cmd = ["python3", python_file_path, "-rc", rc_path, "-pc", pc_path, "-v"]
        if simulate:
            cmd.append("-s")

        completed_process = subprocess.run(cmd, capture_output=True, text=True)  #check=True
        goal_handle.succeed()
        
        self.get_logger().info("Completed Process")
        self.get_logger().info("args")
        self.get_logger().info(str(completed_process.args))
        self.get_logger().info("returncode")
        self.get_logger().info(str(completed_process.returncode))
        self.get_logger().info("stderr")
        self.get_logger().info(completed_process.stderr)
        self.get_logger().info("stdout")
        self.get_logger().info(completed_process.stdout)


        result = OT2Job.Result()
        result.error_msg = completed_process.stderr

        if not completed_process.returncode:
            success = True
        result.success = success

        return result


        ### FOR FEEDBACK, ETC ###

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

        ### FOR FEEDBACK, ETC ###

def main(args=None):
    rclpy.init(args=args)
    action_server = DemoActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()