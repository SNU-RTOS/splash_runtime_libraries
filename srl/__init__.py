import rclpy
from rclpy.executors import SingleThreadedExecutor
from .mode_manager import ModeManager
from .rate_controller import RateController

def run():
    context = rclpy.init()
    __executor = SingleThreadedExecutor()
    __modeManager = ModeManager(context=context)
    __executor.add_node(__modeManager)
    __executor.spin()
    rclpy.shutdown()