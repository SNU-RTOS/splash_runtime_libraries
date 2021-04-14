import rclpy
from rclpy.executors import SingleThreadedExecutor
from .mode_manager_new import ModeManager
from .rate_controller import RateController

def init():
    context = rclpy.init()
    __executor = SingleThreadedExecutor()
    __modeManager = ModeManager(context=context)
    __executor.add_node(__modeManager)
    __executor.spin()
    rclpy.shutdown()