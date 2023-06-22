import threading
import rclpy

from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from ur_msgs.std_srvs import Trigger


class UR5Node():
    def __init__(self): 
        # init node
        rclpy.init()
        self.node = rclpy.create_node('flip_master_node')

        # get input sub
        self.get_io_sub = self.node.create_subscription(IOStates, 'io_and_status_controller/io_states', self.io_callback, 10)

        # set output client
        self.set_io_cli = self.node.create_client(SetIO, '/io_and_status_controller/set_io')
        print('waiting for set_io client ...')
        if self.set_io_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("Set IO client timeout")
           

        # trigger move client (starting pos)
        self.trigger_move_cli = self.node.create_client(Trigger, '/trigger_movement/move')
        print('waiting for trigger_move client')
        if self.trigger_move_cli.wait_for_service(timeout_sec=10) == False:
            raise Exception("Trigger Move client timeout")

        # trigger scan client (scan path)
        self.trigger_scan_cli = self.node.create_client(Trigger, '/trigger_movement/scan')
        print('waiting for trigger_scan client')
        if self.trigger_scan_cli.wait_for_service(timeout_sec=10) == False:
            raise Exception("Trigger Scan client timeout")

        # spin node in seperate thread
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        print("UR5 Node ready")

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def set_digital_output(self, pin, state):
        if self.set_io_cli.wait_for_service(timeout_sec=10) == False:
            raise Exception("Set IO client timeout")
        set_io_req = SetIO()

        set_io_req.fun = 1 #digital?
        set_io_req.pin = pin
        set_io_req.state = state

        print(f"Setting pin {pin}: {state}")
        set_io_res = self.set_io_cli(set_io_req)

        print(set_io_res)
        if (not set_io_res.success):
            raise Exception(f"Failed to set pin {pin} to {state}")
        
    def trigger_start_position(self):
        if self.trigger_move_cli.wait_for_service(timeout_sec=10) == False:
            raise Exception("Trigger Move client timeout")
        return self.trigger_move_cli(Trigger())
    
    def trigger_scan(self):
        if self.trigger_scan_cli.wait_for_service(timeout_sec=10) == False:
            raise Exception("Trigger Scan client timeout")
        return self.trigger_scan_cli(Trigger())
    
    def io_callback(self, msg):
        print(f"[io_callback] msg: {msg}")
        pin = msg.digital_in_states[0].pin
        print(f"[io_callback] msg.digital_in_state[0].pin: {pin}")
        