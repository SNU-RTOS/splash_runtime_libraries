from splash_interfaces.msg import SplashMessage
from rclpy.time import Time
from std_msgs.msg import Header

import pickle
from array import *

class FusionRule:
    def __init__(self, mandatory_ports, optional_ports, optional_ports_threshold, correlation_constraint):
        self.mandatory_ports = mandatory_ports
        self.optional_ports = optional_ports
        self.optional_ports_threshold = optional_ports_threshold
        self.correlation_constraint = correlation_constraint
    
class SensorFusion:
    def __init__(self, fusion_operator):
        self.fusion_operator = fusion_operator

    def fusion_callback(self, channel, msg):
        self.fusion_operator.queues_for_input_ports[channel].append(msg)
        valid_input_data = self._find_valid_input_data()
        oldest_birthmark = None
        if valid_input_data:
            for c, data in valid_input_data.items():
                if oldest_birthmark == None or Time.from_msg(data.header.stamp) < Time.from_msg(oldest_birthmark):
                    oldest_birthmark = data.header.stamp
                    freshness_constraint = data.freshness_constraint
                valid_input_data[c] = pickle.loads(data.body)
            splash_message = SplashMessage()
            header = Header()
            header.stamp = oldest_birthmark
            splash_message.header = header
            splash_message.body = array('B', pickle.dumps(valid_input_data))
            splash_message.freshness_constraint = freshness_constraint
            self.fusion_operator.stream_output_ports[self.fusion_operator.output_channel].write(splash_message)
        else:
            pass
    def _find_valid_input_data(self):
        index_list = [None] * len(self.fusion_operator.queues_for_input_ports.keys())
        i = 0
        optional_ports_count = 0

        for mandatory_port in self.fusion_operator.fusion_rule.mandatory_ports:
            queue = next((item for key, item in self.fusion_operator.queues_for_input_ports.items() if mandatory_port.channel == key), False)
            if not queue or len(queue) == 0:
                return None
        
        for channel, queue in self.fusion_operator.queues_for_input_ports.items():
            if len(queue) > 0:
                index_list[i] = 0
                if next((item for item in self.fusion_operator.fusion_rule.optional_ports if item.channel == channel), False):
                    optional_ports_count = optional_ports_count + 1
            i = i + 1
        
        if optional_ports_count < self.fusion_operator.fusion_rule.optional_ports_threshold:
            return None
        
        flag = True
        while flag:
            if self._is_valid_data(index_list):
                return self._build_data(index_list)
            k, is_last = self._get_earlist_index(index_list)

            if is_last:
                index_list[k] = None
            else:
                index_list[k] = index_list[k] + 1
            flag = False
            for index in index_list:
                if index is not None:
                    flag = True
                    break
        return None

    def _is_valid_data(self, index_list):
        i = 0
        cur_data_list = []
        for queue in self.fusion_operator.queues_for_input_ports.values():
            if index_list[i] is not None:
                cur_data_list.append(queue[index_list[i]])
            i = i + 1
        if len(cur_data_list) < 2:
            return False
        
        for data in cur_data_list:
            for data2 in cur_data_list:
                if data == data2: continue
                time_diff_ms = abs(Time.from_msg(data.header.stamp).nanoseconds - Time.from_msg(data2.header.stamp).nanoseconds) / 1000000
                if time_diff_ms > self.fusion_operator.fusion_rule.correlation_constraint:
                    return False
            return True
    
    def _build_data(self, index_list):
        i = 0
        data = {}
        for key, queue in self.fusion_operator.queues_for_input_ports.items():
            data[key] = None
            if index_list[i] is not None:
                data[key] = queue[index_list[i]]
                if len(queue) == index_list[i] + 1:
                    queue = []
                else:
                    queue = queue[index_list[i]+1:]
                self.fusion_operator.queues_for_input_ports[key] = queue
            i = i + 1

        return data
    
    def _get_earlist_index(self, index_list):
        is_last = False
        earlist_index = -1
        index = 0
        queue_list = []
        for queue in self.fusion_operator.queues_for_input_ports.values():
            queue_list.append(queue)
            index = index + 1
        index = 0
        for queue in queue_list:
            if index_list[index] is not None:
                if earlist_index < 0 or (len(queue) > 0 and Time.from_msg(queue[index_list[index]].header.stamp) < Time.from_msg(queue_list[earlist_index][index_list[earlist_index]].header.stamp)):
                    earlist_index = index
                index = index + 1
            if len(queue_list[earlist_index]) == index_list[earlist_index] + 1:
                is_last = True

            return earlist_index, is_last
