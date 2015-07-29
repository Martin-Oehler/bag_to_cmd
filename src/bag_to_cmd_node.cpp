#include <bag_to_cmd/bag_to_cmd.h>
#include <termios.h>
#include <thread>
#include <mutex>

#include <boost/shared_ptr.hpp>

boost::shared_ptr<bag_tools::BagToCmd> bag_to_cmd_ptr;
std::mutex mtx;
bool exiting = false;

struct termios initial_settings;

void set_terminal_buffer(bool on) {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t); //get the current terminal I/O structure
    if (on) {
        t = initial_settings;
    } else {
        initial_settings = t;
        t.c_iflag &= ~(IXOFF);
        t.c_lflag &= ~(ECHO | ICANON);
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &t); //Apply the new settings
}

void update() {
    ros::Rate rate(125);
    while (ros::ok() && !exiting) {
        mtx.lock();
        bag_to_cmd_ptr->update(0.008);
        mtx.unlock();
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_to_cmd_node");
    ros::NodeHandle controller_manager_nh("/thor_mang/controller_manager");

    ros::NodeHandle nh("bag_to_cmd_node");

    std::string bag_path = "";
    if (!nh.getParam("bag_path", bag_path)) {
        ROS_ERROR_STREAM("Coudln't find key bag_path in namespace " << nh.getNamespace());
        return 0;
    }
    bag_to_cmd_ptr.reset(new bag_tools::BagToCmd());
    if (!bag_to_cmd_ptr->init(bag_path, controller_manager_nh)) {
        ROS_ERROR_STREAM("Initialisation of bag_to_cmd failed.");
        return 0;
    }

    set_terminal_buffer(false);
    ROS_INFO_STREAM("[s] Start | [r] Reset | [SPACE] Pause/Resume | [z] Send zero | [e] Exit");
    std::thread update_loop(update);

    int c;
    while (ros::ok() && !exiting) {
        c = std::getchar();
        switch (c) {
            case ' ':
                mtx.lock();
                bag_to_cmd_ptr->toggle_pause();
                mtx.unlock();
                break;
            case 's':
                mtx.lock();
                bag_to_cmd_ptr->start();
                mtx.unlock();
                break;
            case 'r':
                mtx.lock();
                bag_to_cmd_ptr->stop();
                mtx.unlock();
                break;
            case 'z':
                mtx.lock();
                bag_to_cmd_ptr->send_zero();
                mtx.unlock();
                break;
            case 'e':
                exiting = true;
        }
    }

    update_loop.join();
    bag_to_cmd_ptr->stop();
    bag_to_cmd_ptr->deinit();

    set_terminal_buffer(true);

    ROS_INFO("bag_to_cmd_node finished.");
    return 0;
}
