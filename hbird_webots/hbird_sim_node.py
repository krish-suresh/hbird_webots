
# IMPORTS
import rclpy
from hbird_msgs.msg import State, Waypoint
from rcl_interfaces.msg import ParameterDescriptor

import yaml
import pathlib
import math
import numpy as np

from .scripts.controller import Controller3D
from .scripts.utils import HBParams


FF_VERTICAL_THRUST = 55


# HBIRD DRIVER CLASS
class HbirdDriver:

    def init(self, webots_node, properties):
        
        # initialize Webots robot
        self._robot = webots_node.robot

        # initialize Webots sensors
        self._gps = self._robot.getDevice('gps')
        self._gyro = self._robot.getDevice('gyro')
        self._imu = self._robot.getDevice('inertial_unit')

        # initialize Webots propellers
        self._propellers = [
            self._robot.getDevice('prop_1_motor'),
            self._robot.getDevice('prop_2_motor'),
            self._robot.getDevice('prop_3_motor'),
            self._robot.getDevice('prop_4_motor')
        ]
        for propeller in self._propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)
        

        # initialize ROS2 node
        rclpy.init(args=None)
        self._node = rclpy.create_node('hbird_sim_node')
        self._node.get_logger().info('Starting up the Crazyflie Simulation Node...')

        # TODO: Add the parameter getter and the parameter declaration
        param_descriptor = ParameterDescriptor(description='Defines agent ID.')
        self._node.declare_parameter('agent_id', 'HB1', param_descriptor)
        self._agent_id = self._node.get_parameter('agent_id')._value
        
        # define ROS2 topics
        vehicle_state_topic = '/'+self._agent_id+'/agent_state'
        pos_setpoint_topic = '/'+self._agent_id+'/position_setpoint'

        # initialize subscriber and publisher
            # subscriber
        self._pos_setpoint_subscriber = self._node.create_subscription(Waypoint, pos_setpoint_topic,
				            self.pos_setpoint_callback, 10)
            # publisher
        self._state_publisher = self._node.create_publisher(State, vehicle_state_topic, 10)


        # get pid_gains from yaml file
        config_path = str(str(pathlib.Path(__file__).parent))+"/config/"
        config_file = 'pid_gains.yaml'
        with open(config_path+config_file, 'r') as file:
            pid_gains = yaml.load(file, Loader=yaml.FullLoader)

        # get crazyflie params
        hbparams = HBParams()

        # initialize controller
        self._controller = Controller3D(hbparams, pid_gains)

        # initialize variables
        self.pos_setpoint = Waypoint()
        self.agent_state = State()
        


    def step(self):
        """main Webots controller step function. It is run once every simulation step"""
        rclpy.spin_once(self._node, timeout_sec=0)

        # get agent state
        self.agent_state = self.get_agent_states()

        # compute force/torque commands [u1,...,u4]
        U = self._controller.compute_commands(self.pos_setpoint, self.agent_state)


        # apply mixing algorithm to compute prop velocities
        prop_vel = self.motor_mixing_matrix(U)


        # set propeller velocities
        self.set_prop_velocities(prop_vel)




    def motor_mixing_matrix(self, U):
        prop_vel = np.array([0.,0.,0.,0.])

        prop_vel[0] = FF_VERTICAL_THRUST + U[0] + U[3] + U[2] + U[1]
        prop_vel[1] = FF_VERTICAL_THRUST + U[0] - U[3] - U[2] + U[1]
        prop_vel[2] = FF_VERTICAL_THRUST + U[0] - U[3] + U[2] - U[1]
        prop_vel[3] = FF_VERTICAL_THRUST + U[0] + U[3] - U[2] - U[1]

        return prop_vel
    

    def set_prop_velocities(self, prop_vel):
        """ sets the velocity of each propeller
            prop_vel is a list [m1, m2, m3, m4]
        """
        self._propellers[0].setVelocity(-prop_vel[0])
        self._propellers[1].setVelocity(prop_vel[1])
        self._propellers[2].setVelocity(prop_vel[2])
        self._propellers[3].setVelocity(-prop_vel[3])

    


    def get_agent_states(self):
        current_state = State()

        current_state.position.x = self._gps.getValues()[0]
        current_state.position.y = self._gps.getValues()[1]
        current_state.position.z = self._gps.getValues()[2]

        current_state.velocity.x = self._gps.getSpeedVector()[0]
        current_state.velocity.y = self._gps.getSpeedVector()[1]
        current_state.velocity.z = self._gps.getSpeedVector()[2]

        current_state.orientation.x = self._imu.getRollPitchYaw()[0]
        current_state.orientation.y = self._imu.getRollPitchYaw()[1]
        current_state.orientation.z = self._imu.getRollPitchYaw()[2]

        current_state.angular_velocity.x = self._gyro.getValues()[0]
        current_state.angular_velocity.y = self._gyro.getValues()[1]
        current_state.angular_velocity.z = self._gyro.getValues()[2]

        # publish the state
        self._state_publisher.publish(current_state)

        return current_state



    def pos_setpoint_callback(self, msg):
        self.pos_setpoint = msg


    


