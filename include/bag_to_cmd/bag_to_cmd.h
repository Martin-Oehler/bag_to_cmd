#ifndef BAG_TO_CMD_H
#define BAG_TO_CMD_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager_msgs/ListControllers.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/shared_ptr.hpp>

namespace bag_tools {

enum ControllerState {INITIALIZED, RUNNING, STOPPED, PAUSED, UNINITIALIZED};
static const char* StateNames[] = {"Stopped", "Running", "Stopped", "Paused", "Not initialized"};

class BagToCmd {
public:
    BagToCmd();
    ~BagToCmd();
    bool init(std::string bag_path, ros::NodeHandle &controller_manager_nh);
    void start();
    bool update(double period);
    void stop();
    void deinit();

    void toggle_pause();
    void send_zero();

    ControllerState get_state();

private:
    typedef controller_manager_msgs::ListControllers::ResponseType::_controller_type vec_controller_type ;
    typedef vec_controller_type::value_type controller_type ;

    void send_cmd(controller_type& controller, const std::vector<double>& positions);
    void send_cmd(controller_type& controller, const std::vector<double>& positions, double duration);

    ros::NodeHandle controller_manager_nh_;
    vec_controller_type controllers_;
    std::map<std::string, ros::Publisher> controller_publishers_;
    std::map<std::string, unsigned int> joint_to_ndx_;

    rosbag::View::iterator bag_it_;
    boost::shared_ptr<rosbag::View> view_ptr_;
    rosbag::Bag bag_;

    unsigned int msg_counter_;

    ControllerState state_;

};
}

/*
TODO:
1. Add configuration (namespaces, frequency, topic name)
2. More exception handling (map access)
*/


#endif

