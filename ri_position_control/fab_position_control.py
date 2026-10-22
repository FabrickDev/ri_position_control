import math
import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rclpy.utilities import remove_ros_args
from std_msgs import msg
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class fab_position_control(Node):

    def __init__(self, TargetX = 0.0, TargetY = 0.0):
        super().__init__('fab_position_control')

        # Enable this for More Debug Info (Wheel Speed, Linear Speed, Angular Speed, DT, PosX, PosY, Tetha)
        self.ShowMoreDebugInfo = True

        # Constants Proportional:
        self.KTetha = 3.0
        self.KVel = 3.0

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
        self.TargetPosX = TargetX
        self.TargetPosY = TargetY

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

        self.create_timer(0.1, self.fab_position_control_func)

        self.get_logger().info(
            f'ri_position_control/fab_position_control.py is running...'
        )

    def WheelSpeedCallback(self, msg):
        self.Vel_WL.data = msg.data[0]
        self.Vel_WR.data = msg.data[1]
        # if self.ShowMoreDebugInfo:
        #     self.get_logger().info(
        #         f'Left Wheel Velocity: {self.Vel_WL.data} | Right Wheel Velocity: {self.Vel_WR.data}'
        #     )

    def fab_position_control_func(self):

        self.PrevTime = self.CurrentTime
        self.CurrentTime = self.get_clock().now().nanoseconds * 1e-9
        self.DT = self.CurrentTime - self.PrevTime

        if self.CurrentTime == 0:
            _msg = Twist()
            _msg.linear.x = 0.0
            _msg.angular.z = 0.0
            self.PubCmdVel.publish(_msg)
            return

        # Speed Calculations
        ThisLinearSpeed = (self.Vel_WL.data + self.Vel_WR.data) * (self.wheel_radius / 2)
        ThisAngularSpeed = (self.Vel_WR.data - self.Vel_WL.data) * (self.wheel_radius / self.wheel_separation)

        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'[LOGGER] Linear Speed: {ThisLinearSpeed} m/s'
            )

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

        _DeltaPosX = self.TargetPosX - self.PosX
        _DeltaPosY = self.TargetPosY - self.PosY
        _d = math.sqrt(_DeltaPosX**2 + _DeltaPosY**2)
        _Tethad = math.atan2(_DeltaPosY, _DeltaPosX)
        _ErrorTetha = math.atan2(math.sin(_Tethad - self.Tetha), math.cos(_Tethad - self.Tetha))
        _VelB = self.KVel * _d
        _OmegaB = self.KTetha * _ErrorTetha

        if self.ShowMoreDebugInfo:
            self.get_logger().info(
                f'Error Tetha: {_ErrorTetha} rad/s '
                f'Error DIstance: {_d} m'
            )

        # Wut to publish
        # msg.linear.x = 0.5   # Kecepatan linear (m/s)
        # msg.angular.z = 0.0  # Kecepatan angular (radian/s)
        _msg = Twist()

        if _d < 0.05:  # sudah sampai, berhenti
            _msg.linear.x = 0.0
            _msg.angular.z = 0.0
        else:
            _msg.linear.x = _VelB
            _msg.angular.z = _OmegaB

        self.PubCmdVel.publish(_msg)

def main(args=None):
    rclpy.init(args=args)

    user_args = remove_ros_args(args=sys.argv)[1:]

    TargetPosX = 0.0
    TargetPosY = 0.0
    try:
        if len(user_args) >= 1:
            TargetPosX = float(user_args[0])
        if len(user_args) >= 2:
            TargetPosY = float(user_args[1])
    except ValueError:
        print('Argumen must be Floating Number Mate!!')
        print('For Example:ros2 run ir-fk-ik ir-fkik 0.2 0.5')
        rclpy.shutdown()
        return

    node = fab_position_control(TargetPosX, TargetPosY)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
