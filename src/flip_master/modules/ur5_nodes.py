import threading
import rclpy

from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger


class UR5Node():
    def __init__(self): 
        # init node
        rclpy.init()
        self.node = rclpy.create_node('flip_master_node')

        # I/O subscription w/ callback and set_io client 
        self.get_io_sub = self.node.create_subscription(IOStates, 'io_and_status_controller/io_states', self.io_callback, 10)

        self.set_io_cli = self.node.create_client(SetIO, '/io_and_status_controller/set_io')
        print('waiting for io_and_status_controller/set_io client ...')
        if self.set_io_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("IO status controller /set_io service timeout")
        
        # dashboard_client tiggers
        self.dashboard_engage_cli = self.node.create_client(Trigger, '/dashboard_client/play')
        print('waiting for dashboard_client/play service ...')
        if self.set_io_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/play service timeout")
    
        self.dashboard_run_cli = self.node.create_client(Trigger, '/dashboard_client/close_popup')
        print('waiting for dashboard_client/close_popup service ...')
        if self.set_io_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/close_popup service timeout")

        self.dashboard_disengage_cli = self.node.create_client(Trigger, '/dashboard_client/stop')
        print('waiting for dashboard_client/stop service ...')
        if self.set_io_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/stop service timeout")

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
        set_io_req = SetIO.Request()

        set_io_req.fun = 1
        set_io_req.pin = pin
        set_io_req.state = state

        print(f"Setting pin {pin}: {state}")
        set_io_res = self.set_io_cli.call_async(set_io_req)
        set_io_res.add_done_callback(self.output_callback)

        # print(set_io_res)
        # if (not set_io_res.success):
            # raise Exception(f"Failed to set pin {pin} to {state}")

    def output_callback(self, msg):
        print(f'[set_output] {msg}')
    
    def io_callback(self, msg):
        # print(f"[io_callback] msg: {msg}")
        pin0 = msg.digital_in_states[0].pin
        state0 = msg.digital_in_states[0].state

        pin1 = msg.digital_in_states[1].pin
        state1 = msg.digital_in_states[1].state

        pin2 = msg.digital_in_states[2].pin
        state2 = msg.digital_in_states[2].state

        pin3 = msg.digital_in_states[3].pin
        state3 = msg.digital_in_states[3].state

        pin4 = msg.digital_in_states[4].pin
        state4 = msg.digital_in_states[4].state

        pin5 = msg.digital_in_states[5].pin
        state5 = msg.digital_in_states[5].state

        pin6 = msg.digital_in_states[6].pin
        state6 = msg.digital_in_states[6].state

        pin7 = msg.digital_in_states[7].pin
        state7 = msg.digital_in_states[7].state

        pin8 = msg.digital_in_states[8].pin
        state8 = msg.digital_in_states[8].state

        pin9 = msg.digital_in_states[9].pin
        state9 = msg.digital_in_states[9].state

        pin10 = msg.digital_in_states[10].pin
        state10 = msg.digital_in_states[10].state

        pin11 = msg.digital_in_states[11].pin
        state11 = msg.digital_in_states[11].state

        pin12 = msg.digital_in_states[12].pin
        state12 = msg.digital_in_states[12].state

        pin13 = msg.digital_in_states[13].pin
        state13 = msg.digital_in_states[13].state

        pin14 = msg.digital_in_states[14].pin
        state14 = msg.digital_in_states[14].state

        pin15 = msg.digital_in_states[15].pin
        state15 = msg.digital_in_states[15].state

        pin16 = msg.digital_in_states[16].pin
        state16 = msg.digital_in_states[16].state

        pin17 = msg.digital_in_states[17].pin
        state17 = msg.digital_in_states[17].state

        print(f"[io_callback] Digital In States: \n\t{pin0}: {state0}\n\t{pin1}: {state1}\n\t{pin2}: {state2}\n\t{pin3}: {state3}\n\t{pin4}: {state4}\n\t{pin5}: {state5}\n\t{pin6}: {state6}\n\t{pin7}: {state7}\n\t{pin8}: {state8}\n\t{pin9}: {state9}\n\t{pin10}: {state10}\n\t{pin11}: {state11}\n\t{pin12}: {state12}\n\t{pin13}: {state13}\n\t{pin14}: {state14}\n\t{pin15}: {state15}\n\t{pin16}: {state16}\n\t{pin17}: {state17}")
        

     # # trigger move client (starting pos)
        # self.trigger_move_cli = self.node.create_client(Trigger, '/trigger_movement/move')
        # print('waiting for trigger_move client')
        # if self.trigger_move_cli.wait_for_service(timeout_sec=10) == False:
        #     raise Exception("Trigger Move client timeout")

        # # trigger scan client (scan path)
        # self.trigger_scan_cli = self.node.create_client(Trigger, '/trigger_movement/scan')
        # print('waiting for trigger_scan client')
        # if self.trigger_scan_cli.wait_for_service(timeout_sec=10) == False:
        #     raise Exception("Trigger Scan client timeout")

    # def trigger_start_position(self):
    #     if self.trigger_move_cli.wait_for_service(timeout_sec=10) == False:
    #         raise Exception("Trigger Move client timeout")
    #     return self.trigger_move_cli(Trigger())
    
    # def trigger_scan(self):
    #     if self.trigger_scan_cli.wait_for_service(timeout_sec=10) == False:
    #         raise Exception("Trigger Scan client timeout")
    #     return self.trigger_scan_cli(Trigger())