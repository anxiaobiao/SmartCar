import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from sensor_msgs.msg import Imu, MagneticField

from Rosmaster_Lib import Rosmaster

class DriveCar(Node):
    def __init__(self) -> None:
        super().__init__("drive_car")
        self.bot = Rosmaster()
        self.bot.create_receive_threading()
        self.get_logger().info("启动小车底层驱动程序节点")

        # 订阅移动话题
        self.motion_sub = self.create_subscription(Twist, "/cmd_vel", self.motion, 10)
        # 订阅蜂鸣器话题
        self.sub_beep = self.create_subscription(UInt8, "/beep_time", self.play_beep, 10)

        # 发布IMU(加速度计，陀螺仪)数据话题
        self.imu_publish = self.create_publisher(Imu, "imu/data_raw", 10)
        # 发布IMU(磁力计)数据话题
        self.mag_publish = self.create_publisher(MagneticField, "imu/mag", 10)
        # 发布车辆自身运动状态的数据话题
        self.motion_pub = self.create_publisher(Twist, "/cmd_self_vel", 10)

        # 0.1秒为间隔发布数据话题
        self.timer = self.create_timer(0.1, self.pub_data)

    def motion(self, twist):
        # 增强鲁棒性
        if not isinstance(twist, Twist): return

        vx = twist.linear.x / 2
        if vx > 1.0:
            vx = 1.0
        elif vx < -1.0:
            vx = -1.0

        vy = twist.linear.y / 2
        if vy > 1.0:
            vy = 1.0
        elif vy < -1.0:
            vy = -1.0

        vz = twist.angular.z / 2
        if vz > 5.0:
            vz = 5.0
        elif vz < -5.0:
            vz = -5.0

        self.bot.set_car_motion(vx, vy, vz)

    def play_beep(self, beep_time):
        if not isinstance(beep_time, UInt8): return
        time = beep_time.data * 1000
        self.bot.set_beep(time)

    def pub_data(self):
        ax, ay, az = self.bot.get_accelerometer_data()  # 加速度计
        gx, gy, gz = self.bot.get_gyroscope_data()      # 陀螺仪
        mx, my, mz = self.bot.get_magnetometer_data()   # 磁力计
        vx, vy, vz = self.bot.get_motion_data()         # 获得速度方向等信息

        # print("加速计：",ax, ay, az)
        # print("陀螺仪：",gx, gy, gz)
        # print("磁力计：",mx, my, mz)
        # print("速度信息：",vx, vy, vz)

        imu_data = Imu()
        imu_data.header.stamp = self.get_clock().now().to_msg()
        imu_data.header.frame_id = "imu_link"
        imu_data.linear_acceleration.x = ax*1.0
        imu_data.linear_acceleration.y = ay*1.0
        imu_data.linear_acceleration.z = az*1.0
        imu_data.angular_velocity.x = gx*1.0
        imu_data.angular_velocity.y = gy*1.0
        imu_data.angular_velocity.z = gz*1.0

        mag_data = MagneticField()
        mag_data.header.stamp = self.get_clock().now().to_msg()
        mag_data.header.frame_id = "imu_link"
        mag_data.magnetic_field.x = mx*1.0
        mag_data.magnetic_field.y = my*1.0
        mag_data.magnetic_field.z = mz*1.0

        # 发布IMU数据
        self.imu_publish.publish(imu_data)
        self.mag_publish.publish(mag_data)

        # 发布自身速度数据
        pub = Twist()
        pub.linear.x = vx*1.0
        pub.linear.y = vy*1.0
        pub.angular.z = vz*1.0

        self.motion_pub.publish(pub)

def main(args=None):
    rclpy.init(args=args)
    drive_car = DriveCar()
    rclpy.spin(drive_car)
    drive_car.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
