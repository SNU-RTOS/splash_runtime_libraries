from multiprocessing.managers import SyncManager
from multiprocessing import Manager
from .mode_manager import ModeManager
from .rate_controller import RateController
from .timing_behavior_monitor import TimingBehaviorMonitor

def make_server_manager(port, authkey):
    """ Create a manager for the server, listening on the given port.
        Return a manager object with get_job_q and get_result_q methods.
    """
    modeManager = ModeManager()
    

    # This is based on the examples in the official docs of multiprocessing.
    # get_{job|result}_q return synchronized proxies for the actual Queue
    # objects.
    class JobQueueManager(SyncManager):
        pass
    
    mm = Manager()
    d = mm.dict()
    JobQueueManager.register('get_mode', callable=lambda: d)

    manager = JobQueueManager(address=('', port), authkey=authkey)
    manager.start()
    print('Server started at port %s' % port)
    return manager

def runserver(port, authkey):
    # Start a shared manager server and access its queues
    manager = make_server_manager(port, authkey)
    shared_mode = manager.get_mode()

    
    time.sleep(2)
    manager.shutdown()

def make_client_manager(ip, port, authkey):
    """ Create a manager for a client. This manager connects to a server on the
        given address and exposes the get_job_q and get_result_q methods for
        accessing the shared queues from the server.
        Return a manager object.
    """
    class ServerQueueManager(SyncManager):
        pass

    ServerQueueManager.register('get_mode')

    manager = ServerQueueManager(address=(ip, port), authkey=authkey)
    manager.connect()

    print('Client connected to %s:%s' % (ip, port))
    return manager

def start(target, mode):
    p = multiprocessing.Process(target=target, args=(mode))
    p.start()
    p.join()