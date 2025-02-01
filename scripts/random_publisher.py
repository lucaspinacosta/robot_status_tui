# !/usr/bin python3
import rospy
import random

# Import required message types
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from geographic_msgs.msg import GeoPoint, GeoPose
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import Header


def random_odometry():
    msg = Odometry()
    # Set header information
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"

    # Populate pose (position and orientation)
    msg.pose.pose.position = Point(
        random.uniform(-10, 10),
        random.uniform(-10, 10),
        random.uniform(0, 2)
    )
    msg.pose.pose.orientation = Quaternion(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(0, 1)
    )
    # 6x6 covariance matrix (36 elements)
    msg.pose.covariance = [random.random() for _ in range(36)]

    # Populate twist (linear and angular velocities)
    msg.twist.twist.linear = Vector3(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    msg.twist.twist.angular = Vector3(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    msg.twist.covariance = [random.random() for _ in range(36)]
    return msg


def random_imu():
    msg = Imu()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "imu_link"

    # Orientation as a quaternion
    msg.orientation = Quaternion(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(0, 1)
    )
    msg.orientation_covariance = [random.random() for _ in range(9)]

    # Angular velocity (radians/sec)
    msg.angular_velocity = Vector3(
        random.uniform(-5, 5),
        random.uniform(-5, 5),
        random.uniform(-5, 5)
    )
    msg.angular_velocity_covariance = [random.random() for _ in range(9)]

    # Linear acceleration (m/s^2)
    msg.linear_acceleration = Vector3(
        random.uniform(-10, 10),
        random.uniform(-10, 10),
        random.uniform(-10, 10)
    )
    msg.linear_acceleration_covariance = [random.random() for _ in range(9)]
    return msg


def random_battery_state():
    msg = BatteryState()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.voltage = random.uniform(10, 15)             # Volts
    # Amperes (could be negative when discharging)
    msg.current = random.uniform(-5, 5)
    msg.charge = random.uniform(0, 100)                # Current charge in Ah
    msg.capacity = random.uniform(50, 200)             # Battery capacity in Ah
    msg.design_capacity = random.uniform(50, 200)      # Design capacity in Ah
    # Percentage (0.0 to 1.0)
    msg.percentage = random.uniform(0, 1)
    # Example: 0=Unknown, 1=Charging, 2=Discharging, etc.
    msg.power_supply_status = random.randint(0, 4)
    # Example: 0=Unknown, 1=Good, etc.
    msg.power_supply_health = random.randint(0, 4)
    # Example: 0=Unknown, 1=Li-ion, etc.
    msg.power_supply_technology = random.randint(0, 4)
    msg.present = True

    # Simulate a battery with, e.g., 4 cells
    msg.cell_voltage = [random.uniform(3.5, 4.2) for _ in range(4)]
    msg.cell_temperature = [random.uniform(20, 40) for _ in range(4)]

    msg.location = "Battery compartment"
    msg.serial_number = "SN" + str(random.randint(1000, 9999))
    return msg


def random_geo_point():
    msg = GeoPoint()
    msg.latitude = random.uniform(-90, 90)
    msg.longitude = random.uniform(-180, 180)
    msg.altitude = random.uniform(0, 5000)  # altitude in meters
    return msg


def random_geo_pose():
    msg = GeoPose()
    # Position is a GeoPoint
    msg.position = random_geo_point()
    # Orientation as a quaternion
    msg.orientation = Quaternion(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(0, 1)
    )
    return msg


def main():
    rospy.init_node('random_publisher', anonymous=True)

    # Create publishers for each topic
    pub_odometry = rospy.Publisher('/odometry', Odometry, queue_size=10)
    pub_imu = rospy.Publisher('/imu_data', Imu, queue_size=10)
    pub_battery = rospy.Publisher(
        '/battery_status', BatteryState, queue_size=10)
    pub_geo_point = rospy.Publisher(
        '/geographic_position', GeoPoint, queue_size=10)
    pub_geo_pose = rospy.Publisher('/geographic_pose', GeoPose, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz publishing rate
    rospy.loginfo("Random publisher node started. Publishing on topics...")

    while not rospy.is_shutdown():
        # Publish random messages on each topic
        pub_odometry.publish(random_odometry())
        pub_imu.publish(random_imu())
        pub_battery.publish(random_battery_state())
        pub_geo_point.publish(random_geo_point())
        pub_geo_pose.publish(random_geo_pose())

        rospy.loginfo("Published random messages on all topics.")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
