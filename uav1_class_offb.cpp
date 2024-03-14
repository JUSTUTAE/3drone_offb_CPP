#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath> //ㅂ

class OffboardController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Publisher local_pos_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::Subscriber pose_sub_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }

public:
    OffboardController() : nh_(""), current_pose_(), current_state_() {
        state_sub_ = nh_.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, &OffboardController::stateCallback, this);
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &OffboardController::poseCallback, this);
    }

    void run() {
        ros::Rate rate(20.0);

        // Wait for FCU connection
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;

        // Send a few setpoints before starting
        for (int i = 100; ros::ok() && i > 0; --i) {
            local_pos_pub_.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();


        while (ros::ok()) {
            if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("uav1 Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("uav1 armed");
                    }
                    last_request = ros::Time::now();
                }
                pose.pose.position.x = (current_pose_.pose.position.x)-2.5;;
                pose.pose.position.y = (current_pose_.pose.position.y)-2;
                pose.pose.position.z = 2;
            }

            local_pos_pub_.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav1_offb_node");

    OffboardController offboard_controller;
    offboard_controller.run();

    return 0;
}


