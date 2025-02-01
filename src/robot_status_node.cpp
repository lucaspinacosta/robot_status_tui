#include <ros/ros.h>
#include <ncurses.h>
#include "robot_status_tui/status_tui.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_status_node");
    ros::NodeHandle nh("~");

    // Initialize ncurses
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    start_color();

    // Create the TUI object
    StatusTUI tui(nh);

    // Main loop
    while (ros::ok())
    {
        tui.updateDisplay();
        ros::spinOnce();
        // Sleep for 50ms
        napms(50);
    }

    // Shutdown ncurses
    endwin();
    return 0;
}
