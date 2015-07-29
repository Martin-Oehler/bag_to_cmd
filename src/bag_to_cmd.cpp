#include <bag_to_cmd/bag_to_cmd.h>

namespace bag_tools {

BagToCmd::BagToCmd() :
    state_(UNINITIALIZED){

}

BagToCmd::~BagToCmd() {

}

bool BagToCmd::init(std::string bag_path, ros::NodeHandle &controller_manager_nh) {
    if (state_ != UNINITIALIZED) {
        ROS_ERROR("BagToCmd has already been initialized.");
        return false;
    }
    ROS_INFO("[BagToCmd] Init");
    controller_manager_nh_ = controller_manager_nh;
    ros::ServiceClient client = controller_manager_nh.serviceClient<controller_manager_msgs::ListControllers>("list_controllers");
    controller_manager_msgs::ListControllers list_controllers;

    if (client.call(list_controllers)) {
        for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
            controller_manager_msgs::ListControllers::ResponseType::_controller_type::value_type controller = list_controllers.response.controller[i];
            //ROS_INFO_STREAM("Found controller:" << controller.name << " " << controller.state << " " << controller.type);
            if (controller.state == "running" && (controller.type.find("JointTrajectoryController") != std::string::npos)) {
                controllers_.push_back(controller);
                ros::NodeHandle nh("/thor_mang/" + controller.name);
                controller_publishers_.insert(std::pair<std::string, ros::Publisher>(controller.name, nh.advertise<trajectory_msgs::JointTrajectory>("command", 1000)));
            }
        }
    } else {
        ROS_ERROR_STREAM("No response from list controllers service call.");
        return false;
    }

    ROS_INFO("Controller list retrieved.");


    bag_.open(bag_path, rosbag::bagmode::Read);
    view_ptr_.reset(new rosbag::View(bag_, rosbag::TopicQuery("/joint_states")));


    sensor_msgs::JointStateConstPtr joint_state_ptr;
    if (view_ptr_->size() != 0) {
        joint_state_ptr = view_ptr_->begin()->instantiate<sensor_msgs::JointState>();
    } else {
        ROS_ERROR("View is empty.");
        return false;
    }

    std::vector<std::string> joints;
    if (joint_state_ptr != NULL) {
        joints = joint_state_ptr->name;
    } else {
        ROS_ERROR_STREAM("First joint state is null.");
        return false;
    }

    ROS_INFO("Joint list retrieved.");

    for (unsigned int i = 0; i < joints.size(); i++) {
        joint_to_ndx_.insert(std::pair<std::string, unsigned int>(joints[i], i));
    }

    std::stringstream debug;
    debug << "Active controllers: ";
    for (unsigned int i = 0; i < controllers_.size(); i++) {
        debug << controllers_[i].name;
        if (i != controllers_.size() -1) debug << ", ";
        else debug << std::endl << std::endl;
    }
    debug << "Joints in state: " << std::endl;
    for (unsigned int i = 0; i < joints.size(); i++) {
        debug << joints[i];
        if (i != joints.size() -1) debug << ", ";
        else debug << std::endl << std::endl;
    }

    ROS_INFO_STREAM(debug.str());
    state_ = INITIALIZED;
    return true;
}

void BagToCmd::start() {
    if (!(state_ == INITIALIZED || state_ == STOPPED)) {
        return;
    }
    bag_it_ = view_ptr_->begin();
    msg_counter_ = 0;

    sensor_msgs::JointStateConstPtr joint_state_ptr = bag_it_->instantiate<sensor_msgs::JointState>();
    for (unsigned int i = 0; i < controllers_.size(); i++) {
        send_cmd(controllers_[i], joint_state_ptr->position, 2);
    }
    ros::Duration(2.0).sleep();
    state_ = RUNNING;
}

bool BagToCmd::update(double period) {
    std::cout << "\rReplaying bag file [" << msg_counter_ << "|" << view_ptr_->size() << "] -- [" << StateNames[state_] << "]                   " << std::flush << "\r";
    if (state_ != RUNNING) {
        return true;
    }
    if (bag_it_ != view_ptr_->end()) {
        sensor_msgs::JointStateConstPtr joint_state_ptr = bag_it_->instantiate<sensor_msgs::JointState>();
        if (joint_state_ptr != NULL) {
            //ROS_INFO_STREAM("Next timestamp: " << joint_state_ptr->header.stamp.toNSec());
            for (unsigned int i = 0; i < controllers_.size(); i++) {
                send_cmd(controllers_[i], joint_state_ptr->position);
            }
        }
        bag_it_++;
        msg_counter_++;
        return true;
    } else {
        //std::cout << std::endl;
        //ROS_WARN_THROTTLE(1, "Bag file ended.");
        return false;
    }
}

void BagToCmd::stop() {
    if (!(state_ == RUNNING || state_ == PAUSED)) {
        return;
    }
    bag_it_ = view_ptr_->begin();
    msg_counter_ = 0;
    state_ = STOPPED;
}

void BagToCmd::deinit() {
    bag_.close();
    state_ = UNINITIALIZED;
}

void BagToCmd::send_cmd(controller_type &controller, const std::vector<double>& positions, double duration) {
    trajectory_msgs::JointTrajectory joint_traj;
    //joint_traj.header.stamp = ros::Time::now();

    joint_traj.joint_names = controller.resources;
    trajectory_msgs::JointTrajectoryPoint point;
    std::vector<double> msg_positions;
    for (unsigned int i = 0; i < controller.resources.size(); i++) {
        msg_positions.push_back(positions[joint_to_ndx_[controller.resources[i]]]);
    }
    std::stringstream debug;
    for (unsigned int i = 0; i < msg_positions.size(); i++) {
        debug << msg_positions[i] << ", ";
    }
    //ROS_INFO_STREAM("Sending command to " << controller.name << std::endl << "Positions: " << debug.str());
    point.positions = msg_positions;
    point.time_from_start = ros::Duration(duration);
    joint_traj.points.push_back(point);

    controller_publishers_[controller.name].publish(joint_traj);
}

void BagToCmd::send_cmd(controller_type &controller, const std::vector<double>& positions) {
    send_cmd(controller, positions, 0.1);
}

void BagToCmd::send_zero() {
    std::vector<double> zeroes(joint_to_ndx_.size(), 0.0);
    for (unsigned int i = 0; i < controllers_.size(); i++) {
        send_cmd(controllers_[i], zeroes, 1);
    }
}

void BagToCmd::toggle_pause() {
    if (state_ == RUNNING) {
        state_ = PAUSED;
    } else {
        if (state_ == PAUSED) {
            state_ = RUNNING;
        }
    }
}

ControllerState BagToCmd::get_state() {
    return state_;
}

}
