import math
import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rclpy.utilities import remove_ros_args
from std_msgs import msg
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class fab_position_control_uts(Node):

    def __init__(self):
        super().__init__('fab_position_control_uts')

        # See this code for Original Code: https://github.com/FabrickDev/ri_position_control/blob/main/ri_position_control/fab_position_control.py

        # Enable this for More Debug Info (Wheel Speed, Linear Speed, Angular Speed, DT, PosX, PosY, Tetha)
        self.ShowMoreDebugInfo = False

        # Script Kurangi 0.2 tambah 0.4
        self.TargetsPosition = [
            [0.9, 2.51],
            [2.5, 2.51],
            [2.5, 3.21],
            [3.8, 3.21],
            [3.8, 0.81],
            [8.43, 0.81],
            [8.41, 6.71],
            [7.6, 7.71],
            [7.0, 8.01],
            [6.2, 8.01],
            [5.6, 7.51],
            [5.1, 6.71],
            [5.1, 4.76],
            [4.2, 4.76],
            [4.2, 7.41],
            [1.7, 8.31],
            [0.3, 8.31],
            [0.3, 5.01]
        ]
        self.RunningOrder = 0
        self.IsFinished = False

        # Constants Proportional:
        self.KTetha = 1
        self.KVel = 1.2

        # Error Tolarance
        self.ErrorTol = 0.02
        self.AlignTol = 0.02      # selesai berputar
        self.RealignTol = 0.2     # putar lagi jika melenceng sebesar ini
        self.WasRotate = False

        self.wheel_radius = 0.03
        self.wheel_separation = 0.17

        # Declaration DeltaTime Variables
        self.DT = 0.0
        self.CurrentTime = 0.0
        self.PrevTime = 0.0

        # Declaration Robin Transforms Variable
        self.PosX = 0.0
        self.PosY = 0.0
        self.Tetha = 0.0

        # Wheel Velocity Variables
        self.Vel_WL = Float64()
        self.Vel_WR = Float64()

        # Target Position Variables
        # self.TargetPosX = TargetX
        # self.TargetPosY = TargetY
        self.TargetPosX = self.TargetsPosition[self.RunningOrder][0]
        self.TargetPosY = self.TargetsPosition[self.RunningOrder][1]

        # Last Vel
        self.LastVelR = 0.0
        self.LastVelL = 0.0

        # Gathering Data From input_ik and Wheels
        self.PubCmdVel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.create_subscription(
            Float64MultiArray,
            '/wheel/speed',
            self.WheelSpeedCallback,
            10
        )

        self.create_timer(0.01, self.fab_position_control_func)
        #self.create_timer(0.1, self.fab_test_angular)

        self.get_logger().info(
            f'ri_position_control - position_control_uts is running...'
        )

    def WheelSpeedCallback(self, msg):
        self.Vel_WL.data = msg.data[0]
        self.Vel_WR.data = msg.data[1]
        # if self.ShowMoreDebugInfo:
        #     self.get_logger().info(
        #         f'Left Wheel Velocity: {self.Vel_WL.data} | Right Wheel Velocity: {self.Vel_WR.data}'
        #     )
    def fab_test_angular(self):
        _msg = Twist()
        _msg.linear.x = 0.0
        _msg.angular.z = 0.1
        self.PubCmdVel.publish(_msg)

    def fab_position_control_func(self):

        self.PrevTime = self.CurrentTime
        self.CurrentTime = self.get_clock().now().nanoseconds * 1e-9
        LastDT = self.DT
        self.DT = self.CurrentTime - self.PrevTime

        if self.CurrentTime == 0 or self.DT > 1000.0:  # First Time or DT too big, reset
            _msg = Twist()
            _msg.linear.x = 0.0
            _msg.angular.z = 0.0
            self.PubCmdVel.publish(_msg)
            self.DT = LastDT
            return

        # Speed Calculations
        ThisLinearSpeed = (self.Vel_WL.data + self.Vel_WR.data) * (self.wheel_radius / 2)
        ThisAngularSpeed = (self.Vel_WR.data - self.Vel_WL.data) * (self.wheel_radius / self.wheel_separation)

        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'[LOGGER] Linear Speed: {ThisLinearSpeed} m/s'
            )

        # Global Position Estimator Calculation
        ThisXDot = ThisLinearSpeed * math.cos(self.Tetha)
        ThisYDot = ThisLinearSpeed * math.sin(self.Tetha)

        self.PosX += ThisXDot * self.DT
        self.PosY += ThisYDot * self.DT
        self.Tetha += ThisAngularSpeed * self.DT
        self.Tetha = math.atan2(math.sin(self.Tetha), math.cos(self.Tetha)) # Normalize

        formatDT = "{:.2f}".format(self.DT)
        formatVelX = "{:.2f}".format(self.PosX)
        formatVelY = "{:.2f}".format(self.PosY)
        formatAngularSpeed = "{:.2f}".format(ThisAngularSpeed)

        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'[LOGGER] DeltaTime: {formatDT}: '
                f'PosX: {formatVelX} m, PosY: {formatVelY} m, '
                f'Angular Speed: {formatAngularSpeed} rad/s'
            )

        # Position Control Calculation
        _DeltaPosX = self.TargetPosX - self.PosX
        _DeltaPosY = self.TargetPosY - self.PosY
        _d = math.hypot(_DeltaPosX, _DeltaPosY)
        _Tethad = math.atan2(_DeltaPosY, _DeltaPosX)
        _ErrorTetha = math.atan2(math.sin(_Tethad - self.Tetha), math.cos(_Tethad - self.Tetha))
        _VelB = self.KVel * _d
        _OmegaB = self.KTetha * _ErrorTetha

        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'DeltaPosX: {_DeltaPosX}'
                f'DeltaPosY: {_DeltaPosY}'
            )

        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'Error Tetha: {_ErrorTetha} rad/s '
                f'Error DIstance: {_d} m'
            )

        # Lil Script to Change Rotation First then Change Position
        _msg = Twist()
        if _d < self.ErrorTol:
            self.WasRotate = False   # stop
            self.IsFinished = True
        else:
            if not self.WasRotate:
                if abs(_ErrorTetha) > self.AlignTol:
                    _msg.angular.z = _OmegaB
                else:
                    self.WasRotate = True
            else:
                if abs(_ErrorTetha) > self.RealignTol:
                    self.WasRotate = False
                else:
                    _msg.linear.x = _VelB
                    _msg.angular.z = _OmegaB

        if self.RunningOrder < len(self.TargetsPosition) and self.IsFinished:
            self.RunningOrder += 1
            if self.RunningOrder < len(self.TargetsPosition):
                self._logger.info(
                    f'Next Target Position: {self.TargetsPosition[self.RunningOrder]}'
                )
                self.TargetPosX = self.TargetsPosition[self.RunningOrder][0]
                self.TargetPosY = self.TargetsPosition[self.RunningOrder][1]
                self.IsFinished = False
            else:
                _msg.linear.x = 0.0
                _msg.angular.z = 0.0
                self.get_logger().info(
                    f'All Targets Position Finished'
                )

        self.PubCmdVel.publish(_msg)

def main(args=None):
    rclpy.init(args=args)

    node = fab_position_control_uts()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
