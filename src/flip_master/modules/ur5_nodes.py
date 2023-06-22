import threading
import rclpy
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates


from rclpy.node import Node
from std_msgs.msg import Bool


class UR5MoveNode(Node):
    _instance = None

    def __new__(cls):
        # Singleton class to prevent duplicate node in flipper and scanner classes
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls.instance

    def __init__(self):
        if self._instance is not None:
            return

        super().__init__('ur5_move_node')

        # Create publishers and subscribers (queue size:10)
        self.move_it_pub = self.create_publisher(Bool, 'ur_driver/move_it', 10)
        
    def __del__(self):
        self.destroy_publisher(self.move_it_pub)

    def sweep_scan(self):
        msg = Bool()
        msg.data = True
        # trigger scan
        self.move_it_pub.publish(msg)
        # wait for scan to finish
        rclpy.wait_for_message('ur_driver/sweep_scan', 'done')


class UR5IONode():
    # _instance = None

    # def __new__(cls):
    #     # Singleton class to prevent duplicate node in flipper and scanner classes
    #     if not cls._instance:
    #         cls._instance = super().__new__(cls)
    #     return cls.instance

    def __init__(self, callback=None):
        # if self._instance is not None:
        #     # todo:: enable user to subscribe to input callback without initializing
        #     if callback is not None:
        #         self.callbacks.append(callback)
        #     return
        
        rclpy.init()
        self.node = rclpy.create_node('flip_io_node')

        # self.set_io = self.node.create_client(SetIO, '/io_and_status_controller/set_io')
        # print('created set_io')
        # self.set_io.wait_for_service()
        # print('service answered')

        self.get_io = self.node.create_subscription(IOStates, 'io_and_status_controller/io_states', self.io_callback, 10)

        # self.callbacks = [callback]

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

       

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()

    # def set_digital_output(self, pin, state):
    #     self.set_io.wait_for_service()
    #     set_io_req = SetIO()

    #     set_io_req.fun = 1 #digital?
    #     set_io_req.pin = pin
    #     set_io_req.state = state

    #     print(f"Setting pin {pin}: {state}")
    #     set_io_res = self.set_io(set_io_req)

    #     print(set_io_res)
    #     if (not set_io_res.success):
    #         raise Exception(f"Failed to set pin {pin} to {state}")
    
    def io_callback(self, msg):
        print(msg)
        pin = msg.digital_in_states[0].pin
        # if not self.prev_digital_in_state:
        #     if msg.digital_in_states[0].state:
        #         if not self.pauze_state:
        #             self.node.get_logger().info('Demonstrator set to pauze mode..')
        #             self.pauze_state = True
        #             self.start_stop_ext_control(False)
        #         else:
        #             self.node.get_logger().info('Demonstrator is going to resume..')
        #             self.pauze_state = False
        #             self.start_stop_ext_control(True)
        # self.prev_digital_in_state = msg.digital_in_states[0].state
        #self.node.get_logger().info('digital pin %i state: %i' % (pin, self.pauze_state))


    # def digital_input_callback(self, msg):
    #     value = msg.data
    #     self.get_logger().info('Digital input value: {}'.format(value))

    #     for callback in self.callbacks:
    #         callback()