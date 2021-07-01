from rclpy.node import Node
import json
from splash_interfaces.msg import ModeChange
import time

from std_msgs.msg import String
from std_srvs.srv import Empty

class ModeManager(Node):
    def __init__(self, context):
        super().__init__("splash_mode_manager", context=context)
        self.configuration_map = {}
        self.status_map = {}
        self.subscription_modechange = self.create_subscription(ModeChange, 'splash_modechange', self.mode_change_callback, 10)
        self.handler = None

    def set_modechange_configuration(self, factory, modechange_configuration):
        self.configuration_map[factory] = modechange_configuration
        self.status_map[factory] = self.configuration_map[factory]["initial_mode"]

    def set_handler(self, handler):
        self.handler = handler

    def mode_change_callback(self, msg):
        start = time.time()
        cur_mode = next((item for item in self.configuration_map[msg.factory]["mode_list"] if item["name"] == self.status_map[msg.factory]), None)
        if cur_mode:
            cur_event = next((item for item in cur_mode["events"] if item["name"] == msg.event), None)
            if cur_event:
                next_mode = cur_event["next_mode"]
                self.status_map[msg.factory] = next_mode
                self.handler(factory=msg.factory, next_mode=self.status_map[msg.factory])
        print(f"modechange execution time: {(time.time() - start) * 1000:.2f}ms")
        