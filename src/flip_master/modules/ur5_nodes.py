import threading
import rclpy

from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load


class UR5Node():
    def __init__(self): 
        # init node
        rclpy.init()
        self.node = rclpy.create_node('flip_master_node')

        # I/O subscription w/ callback and set_io client 
        self.get_io_sub = self.node.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_callback, 10)

        self.set_io_cli = self.node.create_client(SetIO, '/io_and_status_controller/set_io')
        print('waiting for io_and_status_controller/set_io client ...')
        if self.set_io_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("IO status controller /set_io service timeout")
        
        # dashboard_client tiggers
        self.dashboard_load_program_cli = self.node.create_client(Load, '/dashboard_client/load_program')
        print('waiting for dashboard_client/load_program service ...')
        if self.dashboard_load_program_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/load_program service timeout")

        self.dashboard_engage_cli = self.node.create_client(Trigger, '/dashboard_client/play')
        print('waiting for dashboard_client/play service ...')
        if self.dashboard_engage_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/play service timeout")
    
        self.dashboard_run_cli = self.node.create_client(Trigger, '/dashboard_client/close_popup')
        print('waiting for dashboard_client/close_popup service ...')
        if self.dashboard_run_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/close_popup service timeout")

        self.dashboard_disengage_cli = self.node.create_client(Trigger, '/dashboard_client/stop')
        print('waiting for dashboard_client/stop service ...')
        if self.dashboard_disengage_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/stop service timeout")

        # spin node in seperate thread
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        print("UR5 Node ready")

        self.digital_in_states = [0] * 18

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()

    async def trigger_scan(self):
        # load
        if self.dashboard_load_program_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/load_program service timeout")
        bla = Load.Request()
        bla.filename = "visir_wp4_demo.urp"
        load_resp = await self.dashboard_load_program_cli.call_async(bla)
        print(load_resp)

        # engange
        if self.dashboard_engage_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/play service timeout")
        engage_resp = await self.dashboard_run_cli.call_async(Trigger.Request())
        print(engage_resp)

        # run
        if self.dashboard_run_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/close_popup service timeout")
        run_resp = await self.dashboard_run_cli.call_async(Trigger.Request())
        print(run_resp)

        # # disengage
        # if self.dashboard_disengage_cli.wait_for_service(timeout_sec=10) == False: 
        #     raise Exception("dashboard_client/stop service timeout")
        # disengage_resp = await self.dashboard_disengage_cli.call_async(Trigger.Request())
        # print(disengage_resp)

    def set_digital_output(self, pin, state):
        if self.set_io_cli.wait_for_service(timeout_sec=10) == False:
            raise Exception("Set IO client timeout")
        set_io_req = SetIO.Request()

        set_io_req.fun = 1
        set_io_req.pin = pin
        set_io_req.state = state

        self.set_io_cli.call_async(set_io_req)
        print(f"Set pin {pin}: {state}")

    def output_callback(self, msg):
        print(f'[set_output] {msg}')
    
    def io_callback(self, msg):
        index = 0
        change = False
        for pin in msg.digital_in_states:
            if len(self.digital_in_states) == 0:
                self.digital_in_states.append(pin.state)
            elif  pin.state != self.digital_in_states[index]:
                self.digital_in_states[index] = pin.state
                change = True
            index += 1

        if change:
            print("Digital Input change detected.")
            index = 0
            for state in self.digital_in_states:
                print(f"Pin {index}: {state}")