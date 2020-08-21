

class RateController():
    def __init__(self, stream_output_port):
        self._queue = []
        self._stream_output_port = stream_output_port
        self._publisher = self._stream_output_port.get_publisher()
        self._last_sent_item = None
        self._create_timer()
    def _create_timer(self):
        component = self._stream_output_port.parent
        component.create_timer(self._stream_output_port.get_rate_constraint(), self._task)

    def _task(self):
        if len(self._queue) > 0:
            msg = self._queue.pop(0)
        elif self._last_sent_item:
            msg = self._last_sent_item
        else:
            return
        self._last_sent_item = msg
        self._publisher.publish(msg)
    
    def push(self, msg):
        self._queue.append(msg)
