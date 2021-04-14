from rclpy.time import Time

class RateController():
    def __init__(self, stream_output_port):
        self._queue = []
        self._stream_output_port = stream_output_port
        self._publisher = self._stream_output_port.get_publisher()
        self._last_sent_item = None
        self._initialized = False
        self.exception = Exception

    def _create_timer(self):
        component = self._stream_output_port.component
        return component.create_timer(1.0/self._stream_output_port.rate, self._task)

    def _task(self):
        if len(self._queue) > 0:
            msg = self._queue.pop(0)
            time_diff_ms = (self._stream_output_port.component.get_clock().now().nanoseconds - Time.from_msg(msg.header.stamp).nanoseconds) / 1000000
            if msg.freshness > 0 and msg.freshness < time_diff_ms:
                raise self.exception('{}ms exceeded(constraint: {}ms, cur: {}ms'.format(time_diff_ms - msg.freshness, msg.freshness, time_diff_ms))
                self._task()
                return 
        elif self._last_sent_item:
            msg = self._last_sent_item
        else:
            return
        self._last_sent_item = msg
        self._publisher.publish(msg)
    
    def push(self, msg):
        if not self._initialized:
            self._timer = self._create_timer()
            self._initialized = True
        self._queue.append(msg)
