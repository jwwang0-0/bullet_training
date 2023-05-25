import compas
import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.backends.pybullet import const
from compas_fab.backends.pybullet.planner import PyBulletPlanner
from compas_fab.backends.pybullet.utils import redirect_stdout
from compas_fab.utilities import LazyLoader
import tempfile


from compas_fab.robots import Robot
from compas.robots import RobotModel

pybullet = LazyLoader('pybullet', globals(), 'pybullet')

class CompasClient(PyBulletClient):
    """Create a compas_pybullet for inverse kinematics calculation
    """
    def __init__(self, connection_type='gui', verbose=False):
        super(CompasClient, self).__init__(connection_type)
        self.planner = PyBulletPlanner(self)
        self.verbose = verbose
        self.collision_objects = {}
        self.attached_collision_objects = {}
        self.disabled_collisions = set()
        self._cache_dir = None
        self.__enter__()

    def connect(self, shadows=True, color=None, width=None, height=None):
        """Connect from the PyBullet server.

        Parameters
        ----------
        shadows : :obj:`bool`
            Display shadows in the GUI. Defaults to ``True``.
        color : :obj:`tuple` of :obj:`float`
            Set the background color of the GUI. Defaults to ``None``.
        width : :obj:`int`
            Set the width in pixels of the GUI. Defaults to ``None``.
        height : :obj:`int`
            Set the height in pixels of GUI. Defaults to ``None``.

        Returns
        -------
        ``None``
        """
        # Shared Memory: execute the physics simulation and rendering in a separate process
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/vrminitaur.py#L7
        self._detect_display()
        options = self._compose_options(color, width, height)
        #with redirect_stdout():
        self.client_id = pybullet.connect(const.CONNECTION_TYPE[self.connection_type], options=options)
        if self.client_id < 0:
            raise Exception('Error in establishing connection with PyBullet.')
        if self.connection_type == 'gui':
            self._configure_debug_visualizer(shadows)       

    def __enter__(self):
        self._cache_dir = tempfile.TemporaryDirectory()
        self.connect()
        return self

    def __exit__(self, *args):
        self._cache_dir.cleanup()
        self.disconnect()

    def load_ur5(self, load_geometry=False, concavity=False):
        """"Load a UR5 robot to PyBullet.

        Parameters
        ----------
        load_geometry : :obj:`bool`, optional
            Indicate whether to load the geometry of the robot or not.
        concavity : :obj:`bool`, optional
            When ``False`` (the default), the mesh will be loaded as its
            convex hull for collision checking purposes.  When ``True``,
            a non-static mesh will be decomposed into convex parts using v-HACD.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            A robot instance.
        """
        robot_model = RobotModel.ur5(load_geometry)
        robot = Robot(robot_model, client=self)
        robot.attributes['pybullet'] = {}
        if load_geometry:
            self.cache_robot(robot, concavity)
        else:
            robot.attributes['pybullet']['cached_robot'] = robot.model
            robot.attributes['pybullet']['cached_robot_filepath'] = compas.get('ur_description/urdf/ur5.urdf')

        urdf_fp = robot.attributes['pybullet']['cached_robot_filepath']

        self._load_robot_to_pybullet(urdf_fp, robot)

        srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
        self.load_semantics(robot, srdf_filename)

        return robot
    
    def _load_robot_to_pybullet(self, urdf_file, robot):
        cached_robot = self.get_cached_robot(robot)
        with redirect_stdout(enabled=not self.verbose):
            pybullet_uid = pybullet.loadURDF(urdf_file, useFixedBase=True,
                                                physicsClientId=self.client_id,
                                                flags=pybullet.URDF_USE_SELF_COLLISION)
            cached_robot.attr['uid'] = pybullet_uid

        self._add_ids_to_robot_joints(cached_robot)
        self._add_ids_to_robot_links(cached_robot)

    def resetSimulation(self):
        return pybullet.resetSimulation(physicsClientId=self.client_id)        

    def loadURDF(self,path,pos):
        return pybullet.loadURDF(path,pos,physicsClientId=self.client_id)     

    def setGravity(self,x,y,z):
        return pybullet.setGravity(x,y,z,physicsClientId=self.client_id)   

    def setTimeStep(self,timeStep):
        return pybullet.setTimeStep(timeStep,physicsClientId=self.client_id)

    def setRealTimeSimulation(self,num):
        return pybullet.setRealTimeSimulation(num,physicsClientId=self.client_id)
        
    def removeBody(self, object_id):
        return pybullet.removeBody(object_id,physicsClientId=self.client_id)    

    def getBasePositionAndOrientation(self,object_id):
        return pybullet.getBasePositionAndOrientation(object_id,physicsClientId=self.client_id)    

    def disconnect(self):
        """Disconnect from the PyBullet server.""" 
        # with redirect_stdout():      
        return pybullet.disconnect(physicsClientId=self.client_id)         