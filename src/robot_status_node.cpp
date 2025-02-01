#include "robot_status_tui/status_tui.h"
#include <ros/ros.h>
#include <ncurses.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_status_tui");
    ros::NodeHandle nh;

    // Initialize curses.
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);

    // Create your TUI instance.
    StatusTUI statusTUI(nh);

    // Run the TUI main loop.
    statusTUI.run();

    // Clean up curses.
    endwin();

    return 0;
}
