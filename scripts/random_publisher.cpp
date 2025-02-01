#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <cstdlib>
#include <ctime>
#include <sstream>
#include <string>

// Helper function to return a random double between min and max.
double randomDouble(double min, double max)
{
    double r = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    return min + r * (max - min);
}

// Helper function to return a random integer between min and max (inclusive).
int randomInt(int min, int max)
{
    return min + rand() % (max - min + 1);
}

// Generate a random nav_msgs::Odometry message.
nav_msgs::Odometry randomOdometry()
{
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    // Randomize pose position.
    msg.pose.pose.position.x = randomDouble(-10, 10);
    msg.pose.pose.position.y = randomDouble(-10, 10);
    msg.pose.pose.position.z = randomDouble(0, 2);

    // Randomize pose orientation.
    msg.pose.pose.orientation.x = randomDouble(-1, 1);
    msg.pose.pose.orientation.y = randomDouble(-1, 1);
    msg.pose.pose.orientation.z = randomDouble(-1, 1);
    msg.pose.pose.orientation.w = randomDouble(0, 1);

    // Fill the 6x6 covariance matrix with random values.
    for (size_t i = 0; i < msg.pose.covariance.size(); ++i)
    {
        msg.pose.covariance[i] = randomDouble(0, 1);
    }

    // Randomize twist (linear and angular velocities).
    msg.twist.twist.linear.x = randomDouble(-1, 1);
    msg.twist.twist.linear.y = randomDouble(-1, 1);
    msg.twist.twist.linear.z = randomDouble(-1, 1);

    msg.twist.twist.angular.x = randomDouble(-1, 1);
    msg.twist.twist.angular.y = randomDouble(-1, 1);
    msg.twist.twist.angular.z = randomDouble(-1, 1);

    // Fill the 6x6 covariance matrix for twist.
    for (size_t i = 0; i < msg.twist.covariance.size(); ++i)
    {
        msg.twist.covariance[i] = randomDouble(0, 1);
    }

    return msg;
}

// Generate a random sensor_msgs::Imu message.
sensor_msgs::Imu randomImu()
{
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "imu_link";

    // Randomize orientation.
    msg.orientation.x = randomDouble(-1, 1);
    msg.orientation.y = randomDouble(-1, 1);
    msg.orientation.z = randomDouble(-1, 1);
    msg.orientation.w = randomDouble(0, 1);

    for (size_t i = 0; i < msg.orientation_covariance.size(); ++i)
    {
        msg.orientation_covariance[i] = randomDouble(0, 1);
    }

    // Randomize angular velocity.
    msg.angular_velocity.x = randomDouble(-5, 5);
    msg.angular_velocity.y = randomDouble(-5, 5);
    msg.angular_velocity.z = randomDouble(-5, 5);

    for (size_t i = 0; i < msg.angular_velocity_covariance.size(); ++i)
    {
        msg.angular_velocity_covariance[i] = randomDouble(0, 1);
    }

    // Randomize linear acceleration.
    msg.linear_acceleration.x = randomDouble(-10, 10);
    msg.linear_acceleration.y = randomDouble(-10, 10);
    msg.linear_acceleration.z = randomDouble(-10, 10);

    for (size_t i = 0; i < msg.linear_acceleration_covariance.size(); ++i)
    {
        msg.linear_acceleration_covariance[i] = randomDouble(0, 1);
    }

    return msg;
}

// Generate a random sensor_msgs::BatteryState message.
sensor_msgs::BatteryState randomBatteryState()
{
    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.voltage = randomDouble(10, 15);           // Volts.
    msg.current = randomDouble(-5, 5);              // Amperes.
    msg.charge = randomDouble(0, 100);              // Current charge in Ah.
    msg.capacity = randomDouble(50, 200);           // Battery capacity in Ah.
    msg.design_capacity = randomDouble(50, 200);    // Design capacity in Ah.
    msg.percentage = randomDouble(0, 1);            // Percentage (0.0 to 1.0).
    msg.power_supply_status = randomInt(0, 4);      // e.g., 0=Unknown, 1=Charging, etc.
    msg.power_supply_health = randomInt(0, 4);      // e.g., 0=Unknown, 1=Good, etc.
    msg.power_supply_technology = randomInt(0, 4);  // e.g., 0=Unknown, 1=Li-ion, etc.
    msg.present = true;

    // Simulate a battery with 4 cells.
    msg.cell_voltage.clear();
    msg.cell_temperature.clear();
    for (int i = 0; i < 4; ++i)
    {
        msg.cell_voltage.push_back(randomDouble(3.5, 4.2));
        msg.cell_temperature.push_back(randomDouble(20, 40));
    }

    msg.location = "Battery compartment";
    int serial = randomInt(1000, 9999);
    std::stringstream ss;
    ss << "SN" << serial;
    msg.serial_number = ss.str();

    return msg;
}

// Generate a random geographic_msgs::GeoPoint message.
geographic_msgs::GeoPoint randomGeoPoint()
{
    geographic_msgs::GeoPoint msg;
    msg.latitude  = randomDouble(-90, 90);
    msg.longitude = randomDouble(-180, 180);
    msg.altitude  = randomDouble(0, 5000);  // Altitude in meters.
    return msg;
}

// Generate a random geographic_msgs::GeoPose message.
geographic_msgs::GeoPose randomGeoPose()
{
    geographic_msgs::GeoPose msg;
    // The position is a GeoPoint.
    msg.position = randomGeoPoint();

    // Randomize orientation.
    msg.orientation.x = randomDouble(-1, 1);
    msg.orientation.y = randomDouble(-1, 1);
    msg.orientation.z = randomDouble(-1, 1);
    msg.orientation.w = randomDouble(0, 1);
    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_publisher");
    ros::NodeHandle nh;

    // Seed the random number generator.
    srand(static_cast<unsigned int>(time(NULL)));

    // Create publishers for each topic.
    ros::Publisher pub_odometry = nh.advertise<nav_msgs::Odometry>("/odometry", 10);
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/imu_data", 10);
    ros::Publisher pub_battery = nh.advertise<sensor_msgs::BatteryState>("/battery_status", 10);
    ros::Publisher pub_geo_point = nh.advertise<geographic_msgs::GeoPoint>("/geographic_position", 10);
    ros::Publisher pub_geo_pose = nh.advertise<geographic_msgs::GeoPose>("/geographic_pose", 10);

    ros::Rate rate(1);  // Publish at 1 Hz.
    ROS_INFO("Random publisher node started. Publishing on topics...");

    while (ros::ok())
    {
        // Publish random messages on each topic.
        pub_odometry.publish(randomOdometry());
        pub_imu.publish(randomImu());
        pub_battery.publish(randomBatteryState());
        pub_geo_point.publish(randomGeoPoint());
        pub_geo_pose.publish(randomGeoPose());

        ROS_INFO("Published random messages on all topics.");

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
