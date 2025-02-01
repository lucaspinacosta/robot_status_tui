#include "robot_status_tui/status_tui.h"

#include <cmath>
#include <sstream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <sys/statvfs.h>
#include <ncurses.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/serialization.h>


//-----------------------------------------------------------------
// StatusTUI constructor: load topics and hardware info from parameters
StatusTUI::StatusTUI(ros::NodeHandle &nh)
{
    // Load monitored topics from parameter server
    XmlRpc::XmlRpcValue topics;
    if (!nh.getParam("/robot_status_node/monitored_topics", topics)) {
        ROS_ERROR("Parameter 'monitored_topics' not found on parameter server!");
        throw std::runtime_error("Missing monitored_topics parameter");
    }
    if (topics.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter 'monitored_topics' is not an array!");
        throw std::runtime_error("monitored_topics must be an array");
    }

    // Initially create windows with any size/position;
    // these values will be adjusted on the first updateDisplay() call.
    if(nh.getParam("/robot_status_node/monitored_topics", topics))
    {
        for (int i = 0; i < topics.size(); ++i)
        {
            TopicDisplay display;
            display.name = static_cast<std::string>(topics[i]["name"]);
            display.type = static_cast<std::string>(topics[i]["type"]);
            // Read preferred sizes from parameters (they may act as a minimum size)
            display.width = static_cast<int>(20);
            display.height = static_cast<int>(10);
            // Initial positions set to zero; they will be updated later.
            display.pos_x = 0;
            display.pos_y = 0;

            // Create new curses window with initial size/position.
            display.win = newwin(display.height, display.width,
                                display.pos_y, display.pos_x);

            topic_displays_.push_back(display);

            // Create a subscriber for this topic, using ShapeShifter.
            subscribers_[display.name] = nh.subscribe<topic_tools::ShapeShifter>(
                display.name,
                10,
                boost::bind(&StatusTUI::topicCallback, this, _1, display.name));
        }
    }

    
    // Load hardware items from parameter server
    XmlRpc::XmlRpcValue hardware;
    if (!nh.getParam("/robot_status_node/monitored_hardware", hardware)) {
        ROS_ERROR("Parameter 'monitored_hardware' not found on parameter server!");
        throw std::runtime_error("Missing monitored_hardware parameter");
    }
    if (hardware.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter 'monitored_hardware' is not an array!");
        throw std::runtime_error("monitored_hardware must be an array");
    }

    if (nh.getParam("/robot_status_node/monitored_hardware", hardware))
    {
        for (int i = 0; i < hardware.size(); ++i)
        {
            HardwareDisplay hd;
            hd.name   = static_cast<std::string>(hardware[i]["name"]);
            hd.device = static_cast<std::string>(hardware[i]["device"]);
            hd.width  = static_cast<int>(20);
            hd.height = static_cast<int>(10);
            hd.pos_x  = 0;
            hd.pos_y  = 0;

            // Create new curses window.
            hd.win = newwin(hd.height, hd.width, hd.pos_y, hd.pos_x);

            hardware_displays_.push_back(hd);
        }
    }
}

//-----------------------------------------------------------------
// Destructor: clean up curses windows
StatusTUI::~StatusTUI()
{
    for (auto &display : topic_displays_)
    {
        if (display.win)
        {
            delwin(display.win);
        }
    }
    for (auto &hd : hardware_displays_)
    {
        if (hd.win)
        {
            delwin(hd.win);
        }
    }
}

//-----------------------------------------------------------------
// Topic callback: update display content for a given topic
void StatusTUI::topicCallback(const topic_tools::ShapeShifter::ConstPtr &msg,
                              const std::string &topic_name)
{
    for (auto &display : topic_displays_)
    {
        if (display.name == topic_name)
        {
            std::stringstream ss;
            formatMessage(*msg, ss);
            display.content = ss.str();
            break;
        }
    }
}

//-----------------------------------------------------------------
// Format a ShapeShifter message based on its data type.
void StatusTUI::formatMessage(const topic_tools::ShapeShifter &msg, std::stringstream &ss)
{
    // 1. Create a buffer to hold the serialized data
    uint32_t serial_size = msg.size();
    std::vector<uint8_t> buffer(serial_size);

    // 2. Write from ShapeShifter into our buffer via an OStream
    ros::serialization::OStream ostream(buffer.data(), serial_size);
    msg.write(ostream);

    if (msg.getDataType() == "sensor_msgs/Imu")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        sensor_msgs::Imu imu_msg;
        ros::serialization::deserialize(istream, imu_msg);

        ss << "Orientation:\n";
        ss << "  x: " << imu_msg.orientation.x
           << ", y: " << imu_msg.orientation.y
           << ", z: " << imu_msg.orientation.z
           << ", w: " << imu_msg.orientation.w << "\n";

        ss << "Angular Velocity:\n";
        ss << "  x: " << imu_msg.angular_velocity.x
           << ", y: " << imu_msg.angular_velocity.y
           << ", z: " << imu_msg.angular_velocity.z << "\n";

        ss << "Linear Acceleration:\n";
        ss << "  x: " << imu_msg.linear_acceleration.x
           << ", y: " << imu_msg.linear_acceleration.y
           << ", z: " << imu_msg.linear_acceleration.z << "\n";
    }
    else if (msg.getDataType() == "nav_msgs/Odometry")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        nav_msgs::Odometry odom_msg;
        ros::serialization::deserialize(istream, odom_msg);

        ss << "Pose:\n";
        ss << "  Position:\n";
        ss << "    x: " << odom_msg.pose.pose.position.x
           << ", y: " << odom_msg.pose.pose.position.y
           << ", z: " << odom_msg.pose.pose.position.z << "\n";
        ss << "  Orientation:\n";
        ss << "    x: " << odom_msg.pose.pose.orientation.x
           << ", y: " << odom_msg.pose.pose.orientation.y
           << ", z: " << odom_msg.pose.pose.orientation.z
           << ", w: " << odom_msg.pose.pose.orientation.w << "\n";
        
        ss << "Twist:\n";
        ss << "  Linear:\n";
        ss << "    x: " << odom_msg.twist.twist.linear.x
           << ", y: " << odom_msg.twist.twist.linear.y
           << ", z: " << odom_msg.twist.twist.linear.z << "\n";
        ss << "  Angular:\n";
        ss << "    x: " << odom_msg.twist.twist.angular.x
           << ", y: " << odom_msg.twist.twist.angular.y
           << ", z: " << odom_msg.twist.twist.angular.z << "\n";

        ss << "Child Frame ID: " << odom_msg.child_frame_id << "\n";
    }
    else if (msg.getDataType() == "sensor_msgs/NavSatFix")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        sensor_msgs::NavSatFix navsat_msg;
        ros::serialization::deserialize(istream, navsat_msg);

        ss << "Latitude: " << navsat_msg.latitude << "\n";
        ss << "Longitude: " << navsat_msg.longitude << "\n";
        ss << "Altitude: " << navsat_msg.altitude << "\n";
    }
    else if (msg.getDataType() == "geographic_msgs/GeoPoint")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        geographic_msgs::GeoPoint geopoint_msg;
        ros::serialization::deserialize(istream, geopoint_msg);

        ss << "Latitude: " << geopoint_msg.latitude << "\n";
        ss << "Longitude: " << geopoint_msg.longitude << "\n";
        ss << "Altitude: " << geopoint_msg.altitude << "\n";
    }
    else if (msg.getDataType() == "geographic_msgs/GeoPose")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        geographic_msgs::GeoPose geopose_msg;
        ros::serialization::deserialize(istream, geopose_msg);

        ss << "Position:\n";
        ss << "  Latitude: " << geopose_msg.position.latitude << "\n";
        ss << "  Longitude: " << geopose_msg.position.longitude << "\n";
        ss << "  Altitude: " << geopose_msg.position.altitude << "\n";

        ss << "Orientation:\n";
        ss << "  x: " << geopose_msg.orientation.x
           << ", y: " << geopose_msg.orientation.y
           << ", z: " << geopose_msg.orientation.z
           << ", w: " << geopose_msg.orientation.w << "\n";
    }
    else if (msg.getDataType() == "sensor_msgs/PointCloud")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        sensor_msgs::PointCloud pc_msg;
        ros::serialization::deserialize(istream, pc_msg);

        ss << "Points:\n";
        for (size_t i = 0; i < pc_msg.points.size(); ++i)
        {
            ss << "  " << i << ": x=" << pc_msg.points[i].x
               << ", y=" << pc_msg.points[i].y
               << ", z=" << pc_msg.points[i].z << "\n";
        }
    }
    else if (msg.getDataType() == "sensor_msgs/PointCloud2")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        sensor_msgs::PointCloud2 pc2_msg;
        ros::serialization::deserialize(istream, pc2_msg);

        ss << "Width: " << pc2_msg.width << "\n";
        ss << "Height: " << pc2_msg.height << "\n";
        ss << "Point Step: " << pc2_msg.point_step << "\n";
        ss << "Row Step: " << pc2_msg.row_step << "\n";
        ss << "Data Size: " << pc2_msg.data.size() << " bytes\n";
        ss << "Fields:\n";
        for (size_t i = 0; i < pc2_msg.fields.size(); ++i)
        {
            ss << "  " << i << ": " << pc2_msg.fields[i].name
               << " (" << pc2_msg.fields[i].offset
               << ", " << pc2_msg.fields[i].datatype
               << ", " << pc2_msg.fields[i].count << ")\n";
        }
    }
    else if (msg.getDataType() == "sensor_msgs/Range")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        sensor_msgs::Range range_msg;
        ros::serialization::deserialize(istream, range_msg);

        ss << "Range: " << range_msg.range << "\n";
        ss << "Min Range: " << range_msg.min_range << "\n";
        ss << "Max Range: " << range_msg.max_range << "\n";
    }
    else if (msg.getDataType() == "sensor_msgs/LaserScan")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        sensor_msgs::LaserScan scan_msg;
        ros::serialization::deserialize(istream, scan_msg);

        ss << "Angle Min: " << scan_msg.angle_min << "\n";
        ss << "Angle Max: " << scan_msg.angle_max << "\n";
        ss << "Angle Increment: " << scan_msg.angle_increment << "\n";
        ss << "Time Increment: " << scan_msg.time_increment << "\n";
        ss << "Scan Time: " << scan_msg.scan_time << "\n";
        ss << "Range Min: " << scan_msg.range_min << "\n";
        ss << "Range Max: " << scan_msg.range_max << "\n";
        ss << "Ranges:\n";
        for (size_t i = 0; i < scan_msg.ranges.size(); ++i)
        {
            ss << "  " << i << ": " << scan_msg.ranges[i] << "\n";
        }
    }
    else if (msg.getDataType() == "sensor_msgs/CompressedImage")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        sensor_msgs::CompressedImage img_msg;
        ros::serialization::deserialize(istream, img_msg);

        ss << "Format: " << img_msg.format << "\n";
        ss << "Data Size: " << img_msg.data.size() << " bytes\n";
        ss << "Data: (first 16 bytes)\n  ";
        for (size_t i = 0; i < std::min<size_t>(16, img_msg.data.size()); ++i)
        {
            ss << std::hex << (static_cast<unsigned int>(img_msg.data[i]) & 0xFF) << " ";
        }
        ss << std::dec;
    }
    else if (msg.getDataType() == "sensor_msgs/BatteryState")
    {
        ros::serialization::IStream istream(buffer.data(), serial_size);
        sensor_msgs::BatteryState bat_msg;
        ros::serialization::deserialize(istream, bat_msg);

        ss << "Voltage: " << bat_msg.voltage << "\n";
        ss << "Current: " << bat_msg.current << "\n";
        ss << "Charge: " << bat_msg.charge << "\n";
        ss << "Capacity: " << bat_msg.capacity << "\n";
        ss << "Design Capacity: " << bat_msg.design_capacity << "\n";
        ss << "Percentage: " << bat_msg.percentage << "\n";
        ss << "Power Supply Status: " << bat_msg.power_supply_status << "\n";
        ss << "Power Supply Health: " << bat_msg.power_supply_health << "\n";

        ss << "Present: " << (bat_msg.present ? "true" : "false") << "\n";
        ss << std::dec;
    }
    else
    {
        // Unknown (or unhandled) data type -> print raw bytes in hex
        ss << "Raw data (hex):\n  ";
        for (uint32_t i = 0; i < serial_size; ++i)
        {
            ss << std::hex << (static_cast<unsigned int>(buffer[i]) & 0xFF) << " ";
            if ((i + 1) % 16 == 0) ss << "\n  ";
        }
        ss << std::dec;
    }
}

//-----------------------------------------------------------------
// Get Network Information by reading /proc/net/dev.
// Searches for the given device (e.g., "eth0") and returns its RX/TX bytes.
std::string getNetworkInfo(const std::string &device)
{
    std::ifstream infile("/proc/net/dev");
    if (!infile.is_open())
    {
        return "Error: cannot open /proc/net/dev";
    }

    std::string line;
    // Skip the header lines.
    std::getline(infile, line);
    std::getline(infile, line);
    while (std::getline(infile, line))
    {
        // Find the device name followed by a colon.
        size_t pos = line.find(device + ":");
        if (pos != std::string::npos)
        {
            size_t colonPos = line.find(":");
            if (colonPos != std::string::npos)
            {
                std::istringstream iss(line.substr(colonPos + 1));
                unsigned long rx_bytes, dummy;
                iss >> rx_bytes;
                // Skip the next 7 numbers (packets, errs, etc.).
                for (int i = 0; i < 7; ++i)
                    iss >> dummy;
                unsigned long tx_bytes;
                iss >> tx_bytes;

                std::stringstream ss;
                ss << "Device: " << device << "\n";
                ss << "RX: " << rx_bytes << " bytes\n";
                ss << "TX: " << tx_bytes << " bytes";
                return ss.str();
            }
        }
    }
    return "Device " + device + " not found";
}

//-----------------------------------------------------------------
// Get CPU information (load average) from /proc/loadavg.
std::string getCpuInfo()
{
    std::ifstream infile("/proc/loadavg");
    if (!infile.is_open())
    {
        return "Error: cannot open /proc/loadavg";
    }

    std::string load_line;
    std::getline(infile, load_line);
    std::istringstream iss(load_line);
    double load1, load5, load15;
    iss >> load1 >> load5 >> load15;

    std::stringstream ss;
    ss << "CPU Load Avg:\n";
    ss << load1 << " (1m), " << load5 << " (5m), " << load15 << " (15m)";
    return ss.str();
}

//-----------------------------------------------------------------
// Get GPU information using nvidia-smi (if available).
std::string getGpuInfo()
{
    // Adjust the query string to match your needs.
    FILE *pipe = popen("nvidia-smi --query-gpu=utilization.gpu,temperature.gpu --format=csv,noheader,nounits", "r");
    if (!pipe)
    {
        return "Error: cannot run nvidia-smi";
    }
    char buffer[128];
    std::string result;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        result += buffer;
    }
    pclose(pipe);
    if (result.empty())
    {
        return "No GPU data";
    }
    std::stringstream ss;
    ss << "GPU Info:\n" << result;
    return ss.str();
}

//-----------------------------------------------------------------
// Get RAM information from /proc/meminfo.
std::string getRamInfo()
{
    std::ifstream infile("/proc/meminfo");
    if (!infile.is_open())
    {
        return "Error: cannot open /proc/meminfo";
    }

    std::string line;
    unsigned long memTotal = 0, memFree = 0, buffers = 0, cached = 0;
    while (std::getline(infile, line))
    {
        if (line.find("MemTotal:") == 0)
        {
            std::istringstream iss(line);
            std::string key;
            iss >> key >> memTotal;
        }
        else if (line.find("MemFree:") == 0)
        {
            std::istringstream iss(line);
            std::string key;
            iss >> key >> memFree;
        }
        else if (line.find("Buffers:") == 0)
        {
            std::istringstream iss(line);
            std::string key;
            iss >> key >> buffers;
        }
        else if (line.find("Cached:") == 0)
        {
            std::istringstream iss(line);
            std::string key;
            iss >> key >> cached;
        }
    }
    unsigned long used = memTotal - memFree - buffers - cached;
    std::stringstream ss;
    ss << "RAM Info:\n";
    ss << "Total: " << memTotal / 1024 << " MB\n";
    ss << "Used: " << used / 1024 << " MB\n";
    ss << "Free: " << memFree / 1024 << " MB";
    return ss.str();
}

//-----------------------------------------------------------------
// Get Disk information (for a given mount point) using statvfs.
std::string getDiskInfo(const std::string &mount_point)
{
    struct statvfs stat;
    if (statvfs(mount_point.c_str(), &stat) != 0)
    {
        return "Error: cannot access " + mount_point;
    }
    unsigned long total = stat.f_blocks * stat.f_frsize;
    unsigned long free = stat.f_bfree * stat.f_frsize;
    unsigned long used = total - free;
    std::stringstream ss;
    ss << "Disk (" << mount_point << "):\n";
    ss << "Total: " << total / (1024 * 1024) << " MB\n";
    ss << "Used: " << used / (1024 * 1024) << " MB\n";
    ss << "Free: " << free / (1024 * 1024) << " MB";
    return ss.str();
}

//-----------------------------------------------------------------
// Wraps a single line of text into multiple lines that do not exceed 'width' characters.
// It attempts to break at whitespace if possible.
std::vector<std::string> wrapText(const std::string &text, size_t width)
{
    std::vector<std::string> wrapped;
    size_t start = 0;
    while (start < text.size())
    {
        // Determine the maximum length we can take from here.
        size_t len = std::min(width, text.size() - start);
        
        // If the substring fits to the end of the text, just add it.
        if (start + len == text.size())
        {
            wrapped.push_back(text.substr(start, len));
            break;
        }
        
        // Otherwise, try to break at the last whitespace within the allowed width.
        size_t breakPoint = text.rfind(' ', start + len);
        if (breakPoint == std::string::npos || breakPoint < start)
        {
            // No whitespace found or it's before the current start, so force break.
            wrapped.push_back(text.substr(start, len));
            start += len;
        }
        else
        {
            // Break at the whitespace.
            size_t segmentLength = breakPoint - start;
            wrapped.push_back(text.substr(start, segmentLength));
            // Skip the whitespace (if any).
            start = breakPoint + 1;
        }
    }
    return wrapped;
}

//-----------------------------------------------------------------
/// This function automatically re-calculates the layout of all topic windows
/// based on the current terminal size and the number of topics.
void StatusTUI::updateLayout()
{
    // Get the current terminal size.
    int term_rows, term_cols;
    getmaxyx(stdscr, term_rows, term_cols);

    // Determine the grid layout:
    // We'll arrange the displays in a grid whose number of columns is the ceiling
    // of the square root of the number of displays, and the number of rows is the
    // ceiling of (number of displays / columns).
    int count = topic_displays_.size();
    int cols = static_cast<int>(std::ceil(std::sqrt(count)));
    int rows = static_cast<int>(std::ceil(static_cast<double>(count) / cols));

    // Compute the new width and height for each window so that they evenly fill the screen.
    int new_width = term_cols / cols;
    int new_height = term_rows / rows;

    // Update each display's size and position, then adjust its ncurses window.
    for (int i = 0; i < count; ++i)
    {
        TopicDisplay &display = topic_displays_[i];
        display.width  = new_width;
        display.height = new_height;
        display.pos_x  = (i % cols) * new_width;
        display.pos_y  = (i / cols) * new_height;

        // Resize the window and move it to the new position.
        wresize(display.win, new_height, new_width);
        mvwin(display.win, display.pos_y, display.pos_x);
    }
}

//-----------------------------------------------------------------
// Update the hardware display windows with current hardware information.
void StatusTUI::updateHardwareDisplay()
{
    for (auto &hd : hardware_displays_)
    {
        if (hd.name == "Network")
        {
            hd.content = getNetworkInfo(hd.device);
        }
        else if (hd.name == "CPU")
        {
            hd.content = getCpuInfo();
        }
        else if (hd.name == "GPU")
        {
            hd.content = getGpuInfo();
        }
        else if (hd.name == "RAM")
        {
            hd.content = getRamInfo();
        }
        else if (hd.name == "Disk")
        {
            // For disk, you might use the device field as a mount point (e.g., "/").
            hd.content = getDiskInfo(hd.device);
        }
        else
        {
            hd.content = "Unknown hardware type";
        }

        // Now update the curses window (similar to updateDisplay())
        werase(hd.win);
        box(hd.win, 0, 0);
        mvwprintw(hd.win, 1, 1, "%s", hd.name.c_str());

        // Wrap and print the content.
        std::istringstream iss(hd.content);
        std::string line;
        int line_num = 3;
        while (std::getline(iss, line) && (line_num < hd.height - 1))
        {
            std::vector<std::string> wrappedLines = wrapText(line, hd.width - 2);
            for (const auto &wrappedLine : wrappedLines)
            {
                if (line_num >= hd.height - 1)
                    break;
                mvwprintw(hd.win, line_num++, 1, "%s", wrappedLine.c_str());
            }
        }
        wrefresh(hd.win);
    }
}

//-----------------------------------------------------------------
/// This function is called frequently (e.g., in your main loop) to update the
/// layout and redraw each topic window.
void StatusTUI::updateDisplay()
{
    // Recalculate layout each update so windows adjust to terminal size changes.
    updateLayout();

    // Update the content in each window.
    for (auto &display : topic_displays_)
    {
        werase(display.win);
        box(display.win, 0, 0);
        mvwprintw(display.win, 1, 1, "%s", display.name.c_str());
        mvwprintw(display.win, 2, 1, "Type: %s", display.type.c_str());

        // Start printing content from row 4.
        int line_num = 4;
        std::istringstream iss(display.content);
        std::string line;
        while (std::getline(iss, line) && (line_num < display.height - 2))
        {
            std::vector<std::string> wrappedLines = wrapText(line, display.width - 2);
            for (const auto &wrappedLine : wrappedLines)
            {
                if (line_num >= display.height - 2)
                    break;
                mvwprintw(display.win, line_num++, 1, "%s", wrappedLine.c_str());
            }
        }
        wrefresh(display.win);
    }

}

void StatusTUI::run() {
    current_mode_ = TOPICS;
    nodelay(stdscr, TRUE);
    int ch;
    while (ros::ok()) {
        ch = getch();
        if (ch != ERR) {
            if (ch == '1') {
                current_mode_ = TOPICS;
            } else if (ch == '2') {
                current_mode_ = HARDWARE;
            } else if (ch == 'q' || ch == 'Q') {
                break;
            }
        }
        
        // Clear stdscr and refresh it first.
        clear();
        refresh();
        
        // Then update and refresh your subwindows.
        if (current_mode_ == TOPICS) {
            updateDisplay();
        } else if (current_mode_ == HARDWARE) {
            updateHardwareDisplay();
        }
        
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}

