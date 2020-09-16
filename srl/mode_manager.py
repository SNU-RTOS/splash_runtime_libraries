from rclpy.node import Node
import json
from splash_interfaces.srv import RegisterMode, UnregisterMode, RequestModeChange
from std_msgs.msg import String
from std_srvs.srv import Empty
class ModeManager(Node):
    def __init__(self, context):
        super().__init__("splash_mode_manager", context=context)
        self.mode_conf_map = {}
        self.mode_map = {}
        self.service_mode_change = self.create_service(RequestModeChange, 'request_splash_mode_change', self._request_mode_change_callback)
        self.service_register_mode = self.create_service(RegisterMode, 'register_splash_mode', self._register_mode_callback)
        self.service_unregister_mode = self.create_service(UnregisterMode, 'unregister_splash_mode', self._unregister_mode_callback)
        self.service_check_alive = self.create_service(Empty, 'check_alive_mode_manager', self._check_alive_mode_manager_callback)
        self.publisher_map = {}
        
    def _request_mode_change_callback(self, request, response):
        response.ok = False
        cur_mode = next((item for item in self.mode_conf_map[request.factory]["mode_list"] if item["name"] == self.mode_map[request.factory]), None)
        if cur_mode:
            cur_event = next((item for item in cur_mode["events"] if item["name"] == request.event), None)
            if cur_event:
                next_mode = cur_event["next_mode"]
                response.ok = True
                self.mode_map[request.factory] = next_mode
                msg = String()
                msg.data = next_mode
                for publisher in self.publisher_map[request.factory]:
                    publisher.publish(msg)
            else:
                response.ok = False
        return response

    def _register_mode_callback(self, request, response):
        print("Mode registration: ", request.factory)
        try:
            response.ok = True
            self.mode_conf_map[request.factory] = json.loads(request.mode_configuration)
            self.mode_map[request.factory] = self.mode_conf_map[request.factory]["initial_mode"]
            topic = "{}/splash_mode_change_{}".format(request.name_space, request.factory)
            if not request.factory in self.publisher_map.keys():
                self.publisher_map[request.factory] = []
            self.publisher_map[request.factory].append(self.create_publisher(String, topic, 1))
        except:
            response.ok = False
        
        return response 
    
    def _unregister_mode_callback(self, request, response):
        print("Mode unregistration: ", request.factory)
        try:
            del(self.mode_conf_map[request.factory])
            del(self.mode_map[request.factory])
            del(self.publisher_map[request.factory])
            response.ok = True
        except:
            response.ok = False
        
        return response
    
    def _check_alive_mode_manager_callback(self, request, response):
        return response