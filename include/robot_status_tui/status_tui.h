#ifndef STATUS_TUI_H
#define STATUS_TUI_H


#include <ros/ros.h>
#include <ncurses.h>
#include <topic_tools/shape_shifter.h>
#include <ros/serialization.h>
#include <XmlRpcValue.h>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <boost/bind.hpp>
#include <cmath> 

// Include any additional message headers you need:
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2_msgs/TFMessage.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPath.h>
#include <geographic_msgs/RouteNetwork.h>

// Structure for topic displays.
struct TopicDisplay {
    std::string name;
    std::string type;
    int width;
    int height;
    int pos_x;
    int pos_y;
    WINDOW *win;
    std::string content;
};

// Structure for hardware displays.
struct HardwareDisplay {
    std::string name;
    std::string device;
    int width;
    int height;
    int pos_x;
    int pos_y;
    WINDOW *win;
    std::string content;
};

enum DisplayMode {
    TOPICS = 1,
    HARDWARE = 2
};

class StatusTUI {
public:
    // Constructor and destructor.
    StatusTUI(ros::NodeHandle &nh);
    ~StatusTUI();

    // Functions to update displays.
    void updateDisplay();
    void updateHardwareDisplay();

    // New function to run the main loop and handle input.
    void run();

private:
    // Callback for topic messages.
    void topicCallback(const topic_tools::ShapeShifter::ConstPtr &msg, const std::string &topic_name);
    void formatMessage(const topic_tools::ShapeShifter &msg, std::stringstream &ss);
    void updateLayout();

    // Containers to hold the display objects.
    std::vector<TopicDisplay> topic_displays_;
    std::vector<HardwareDisplay> hardware_displays_;

    // Container to store topic subscribers.
    std::map<std::string, ros::Subscriber> subscribers_;

    // Member to track the current display mode.
    DisplayMode current_mode_;
};

#endif // STATUS_TUI_H
