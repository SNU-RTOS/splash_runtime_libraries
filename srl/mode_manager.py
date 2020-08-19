from rclpy.node import Node
import json
from test_pkg_interfaces.srv import RegisterMode, RequestModeChange
from std_msgs.msg import String

class ModeManager(Node):
    def __init__(self, context):
        super().__init__("splash_mode_manager", context=context)
        self.mode_conf_map = {}
        self.mode_map = {}
        self.service_mode_change = self.create_service(RequestModeChange, 'request_splash_mode_change', self._request_mode_change_callback)
        self.service_register_mode = self.create_service(RegisterMode, 'register_splash_mode', self._register_mode_callback)
        self.publisher_map = {}
        
    def _request_mode_change_callback(self, request, response):
        response.ok = False
        cur_mode = next((item for item in self.mode_conf_map[request.factory]["mode_list"] if item["name"] == self.mode_map["factory"]), None)
        if cur_mode:
            cur_event = next((item for item in cur_mode["events"] if item["name"] == request.event), None)
            if cur_event:
                next_mode = cur_event["next_mode"]
                response.ok = True
                self.mode_map[request.factory] = next_mode
                msg = ModeChange()
                msg.mode = next_mode
                self.publisher_map[request.factory].publish(msg)

        return response

    def _register_mode_callback(self, request, response):
        response.ok = True

        self.mode_conf_map[request.factory] = json.loads(request.mode_configuration)
        self.mode_map[request.factory] = mode_conf_map[request.factory]["initial_mode"]
        self.publisher_map[request.factory] = self.create_publisher(String, "splash_mode_change_{}".format(request.factory), 1)

        return response