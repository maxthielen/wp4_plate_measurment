import threading
from time import sleep

import rclpy
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load


class UR5Node():
    def __init__(self, debug = False): 
        self.digital_inputs = [False] * 18

        self.a0 = self.digital_inputs[4]
        self.a1 = self.digital_inputs[5]
        self.b0 = self.digital_inputs[6]
        self.b1 = self.digital_inputs[7]

        self.lowering_piston = False
        self.flip_enganged = False

        self.verbose = debug

        # init node
        print("Init UR5 Node")

        rclpy.init()
        self.node_l = rclpy.create_node('flip_master_listener')
        self.node = rclpy.create_node('flip_master_node')

        # wait a bit for ur servies to become available
        sleep(2)

        # I/O subscription w/ callback and set_io client 
        self.get_io_sub = self.node_l.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_callback, 10)

        self.set_io_cli = self.node.create_client(SetIO, '/io_and_status_controller/set_io')
        print('waiting for io_and_status_controller/set_io client ...')
        if self.set_io_cli.wait_for_service(timeout_sec=5.0) == False: 
            raise Exception("IO status controller /set_io service timeout")
        
        # dashboard_client tiggers
        self.dashboard_load_program_cli = self.node.create_client(Load, '/dashboard_client/load_program')
        print('waiting for dashboard_client/load_program service ...')
        if self.dashboard_load_program_cli.wait_for_service(timeout_sec=5.0) == False: 
            raise Exception("dashboard_client/load_program service timeout")

        self.dashboard_engage_cli = self.node.create_client(Trigger, '/dashboard_client/play')
        print('waiting for dashboard_client/play service ...')
        if self.dashboard_engage_cli.wait_for_service(timeout_sec=5.0) == False: 
            raise Exception("dashboard_client/play service timeout")
    
        self.dashboard_run_cli = self.node.create_client(Trigger, '/dashboard_client/close_popup')
        print('waiting for dashboard_client/close_popup service ...')
        if self.dashboard_run_cli.wait_for_service(timeout_sec=5.0) == False: 
            raise Exception("dashboard_client/close_popup service timeout")

        self.dashboard_disengage_cli = self.node.create_client(Trigger, '/dashboard_client/stop')
        print('waiting for dashboard_client/stop service ...')
        if self.dashboard_disengage_cli.wait_for_service(timeout_sec=5.0) == False: 
            raise Exception("dashboard_client/stop service timeout")

        # spin node in seperate thread
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # # spin node in seperate thread
        self.executor_l = rclpy.executors.MultiThreadedExecutor()
        self.executor_l.add_node(self.node_l)
        self.executor_l_thread = threading.Thread(target=self.executor_l.spin, daemon=True)
        self.executor_l_thread.start()

        self.init_scan()

        print("UR5 Node ready")

    def __del__(self):
        # disengage dashboard
        if self.dashboard_disengage_cli.wait_for_service(timeout_sec=10) == False: 
            raise Exception("dashboard_client/stop service timeout")
        disengage_resp = self.dashboard_disengage_cli.call(Trigger.Request())
        print(disengage_resp)
        print('del called')

        self.node.destroy_node()
        self.node_l.destroy_node()
        rclpy.shutdown()

    def init_scan(self):
        # load
        if self.dashboard_load_program_cli.wait_for_service(timeout_sec=5.0) == False: 
            raise Exception("dashboard_client/load_program service timeout")
        req = Load.Request()
        req.filename = "visir_wp4_demo.urp"
        load_resp = self.dashboard_load_program_cli.call(req)
        print(load_resp)

        # engange
        if self.dashboard_engage_cli.wait_for_service(timeout_sec=5.0) == False: 
            raise Exception("dashboard_client/play service timeout")
        engage_resp = self.dashboard_engage_cli.call(Trigger.Request())
        print(engage_resp)

        sleep(1)

    def trigger_scan(self):
        # run
        if self.dashboard_run_cli.wait_for_service(timeout_sec=5.0) == False: 
            raise Exception("dashboard_client/close_popup service timeout")
        run_resp = self.dashboard_run_cli.call(Trigger.Request())
        print(run_resp)

    def set_digital_output(self, pin, state):
        # piston a = 4
        # piston b = 5
        if self.set_io_cli.wait_for_service(timeout_sec=5.0) == False:
            raise Exception("Set IO client timeout")
        set_io_req = SetIO.Request()

        set_io_req.fun = 1
        set_io_req.pin = pin
        set_io_req.state = state

        set_io_resp = self.set_io_cli.call(set_io_req)
        print(set_io_resp)

    async def trigger_flip(self):
        self.flip_enganged = True
        self.set_digital_output(4,1.0)
        while self.flip_enganged:
            sleep(1)

    def output_callback(self, msg):
        print(f'[set_output] {msg}')
    
    def io_callback(self, msg):
        change = False
        for i in range(0, len(msg.digital_in_states)):
            if self.digital_inputs[i] != msg.digital_in_states[i].state:
                self.digital_inputs[i] = msg.digital_in_states[i].state
                change = True

                self.a0 = self.digital_inputs[4]
                self.a1 = self.digital_inputs[5]
                self.b0 = self.digital_inputs[6]
                self.b1 = self.digital_inputs[7]

        if change:
            print("Digital Input change detected.")
            # for i in range(0, len(self.digital_inputs)):
            #     print(f"Pin {i}: {self.digital_inputs[i]}")

            print()
            print(f"A0: {self.a0}")
            print(f"A1: {self.a1}")
            print(f"B0: {self.b0}")
            print(f"B1: {self.b1}")
            print()

            if self.flip_enganged:
                if self.a0 == True and self.a1 == False and self.b0 == True and self.b1 == False and self.lowering_piston: # both down and was lowering
                    # reset
                    print(f'reset')
                    self.lowering_piston = False
                    self.flip_enganged = False
                elif self.a0 == True and self.a1 == False and self.b0 == True and self.b1 == False: # both down
                    # raise big plate (a)
                    print(f'raising a')
                    self.set_digital_output(4,1.0)
                elif self.a0 == False and self.a1 == True and self.b0 == True and self.b1 == False: # a:up b:down
                    # raise small plate (b)
                    print(f'raising b')
                    self.set_digital_output(5,1.0)
                elif self.a0 == False and self.a1 == True and self.b0 == False and self.b1 == True: #both up 
                    # lower both plates
                    print(f'lowering both')
                    self.lowering_piston = True
                    self.set_digital_output(4,0.0)
                    self.set_digital_output(5,0.0)
            