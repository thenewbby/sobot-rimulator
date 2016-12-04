# Sobot Rimulator - A Robot Programming Tool
# Copyright (C) 2013-2014 Nicholas S. D. McCrea
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Email mccrea.engineering@gmail.com for questions, comments, or to report bugs.





from math import *
from differential_drive_dynamics import *
from polygon import *
from pose import *
from proximity_sensor import *
from robot_supervisor_interface import *
from supervisor import *
from wheel_encoder import *

# Roomba create 2 Properties
RC2_WHEEL_RADIUS = 0.072         # meters
RC2_WHEEL_BASE_LENGTH = 0.235   # meters
RC2_WHEEL_TICKS_PER_REV = 508.8
RC2_MAX_WHEEL_DRIVE_RATE = 5.0  # rad/s

SCALING_FACTOR = 3
# Roomba create 2 Dimensions
RC2_BOTTOM_PLATE = [[ 0.336,  0.09 ],
                   [  0.246,  0.246 ],
                   [  0.090,  0.336 ],
                   [  -0.085, 0.319 ],
                   [  -0.233, 0.233 ],
                   [  -0.318, 0.085 ],
                   [  -0.318, -0.085 ],
                   [ -0.233, -0.233 ],
                   [ -0.085, -0.319 ],
                   [ 0.090, -0.336 ],
                   [ 0.246,  -0.246 ],
                   [ 0.336,  -0.09 ]]
for i in RC2_BOTTOM_PLATE:
    i[0] /= SCALING_FACTOR
    i[1] /= SCALING_FACTOR

RC2_SENSOR_MIN_RANGE = 0.05
RC2_SENSOR_MAX_RANGE = 0.2
RC2_SENSOR_POSES = [#[ -0.203,  0.260,  128 ],
				    [  0.147,  0.315,  65  ], # x, y, theta_degrees
                    [  0.301,  0.174,  30  ],
                    [  0.346,  0.036,  6   ],
                    [  0.346, -0.036, -6   ],
                    [  0.301, -0.174, -30  ],
                    [  0.147, -0.315, -65  ]]
                    #[ -0.203, -0.260, -128 ],
                    #[ -0.330,  0.000,  180 ]]

for i in RC2_SENSOR_POSES:
    i[0] /= SCALING_FACTOR
    i[1] /= SCALING_FACTOR

class Robot: # Khepera III robot

  def __init__( self ):
    # geometry
    self.geometry = Polygon( RC2_BOTTOM_PLATE )
    self.global_geometry = Polygon( RC2_BOTTOM_PLATE ) # actual geometry in world space

    # wheel arrangement
    self.wheel_radius = RC2_WHEEL_RADIUS             # meters
    self.wheel_base_length = RC2_WHEEL_BASE_LENGTH   # meters

    # pose
    self.pose = Pose( 0.0, 0.0, 0.0 )

    # wheel encoders
    self.left_wheel_encoder = WheelEncoder( RC2_WHEEL_TICKS_PER_REV )
    self.right_wheel_encoder = WheelEncoder( RC2_WHEEL_TICKS_PER_REV )
    self.wheel_encoders = [ self.left_wheel_encoder, self.right_wheel_encoder ]

    # IR sensors
    self.ir_sensors = []
    for _pose in RC2_SENSOR_POSES:
      ir_pose = Pose( _pose[0], _pose[1], radians( _pose[2] ) )
      self.ir_sensors.append(
          ProximitySensor( self, ir_pose, RC2_SENSOR_MIN_RANGE, RC2_SENSOR_MAX_RANGE, radians( 20 ) ) )

    # dynamics
    self.dynamics = DifferentialDriveDynamics( self.wheel_radius, self.wheel_base_length )

    # supervisor
    self.supervisor = Supervisor( RobotSupervisorInterface( self ),
                                  RC2_WHEEL_RADIUS, RC2_WHEEL_BASE_LENGTH, RC2_WHEEL_TICKS_PER_REV, RC2_SENSOR_POSES, RC2_SENSOR_MAX_RANGE )

    ## initialize state
    # set wheel drive rates (rad/s)
    self.left_wheel_drive_rate = 0.0
    self.right_wheel_drive_rate = 0.0

  # simulate the robot's motion over the given time interval
  def step_motion( self, dt ):
    v_l = self.left_wheel_drive_rate
    v_r = self.right_wheel_drive_rate

    # apply the robot dynamics to moving parts
    self.dynamics.apply_dynamics( v_l, v_r, dt,
                                  self.pose, self.wheel_encoders )

    # update global geometry
    self.global_geometry = self.geometry.get_transformation_to_pose( self.pose )

    # update all of the sensors
    for ir_sensor in self.ir_sensors:
      ir_sensor.update_position()

  # set the drive rates (angular velocities) for this robot's wheels in rad/s
  def set_wheel_drive_rates( self, v_l, v_r ):
    # simulate physical limit on drive motors
    v_l = min( RC2_MAX_WHEEL_DRIVE_RATE, v_l )
    v_r = min( RC2_MAX_WHEEL_DRIVE_RATE, v_r )

    # set drive rates
    self.left_wheel_drive_rate = v_l
    self.right_wheel_drive_rate = v_r
