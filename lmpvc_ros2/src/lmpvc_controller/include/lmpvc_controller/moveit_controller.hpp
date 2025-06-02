#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <mutex>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#ifndef LMPVC_MOVEIT_CONTROLLER
#define LMPVC_MOVEIT_CONTROLLER
/*
 * A controller built on to of the MoveIt2 move_group interface. 
 * 
 * Builds a move_group on top of the provided node and implements a variety of member
 * functions to facilitate a simple robot controller. 
 */
class MoveitController {

    public:
        /*
         * Params: node, group_name
         *
         * Creates a move group with the provided node pointer and group name. Make sure
         * to use a group which actually exists and works. Currently, the default end
         * effector is used.
         */
        MoveitController(std::shared_ptr<rclcpp::Node>& node, std::shared_ptr<rclcpp::Node>& move_group_node, std::string group_name);

        /*
         * Params:
         * Returns: pose
         * 
         * Returns the current pose of the end effector
         */
        geometry_msgs::msg::Pose get_pose();

        /*
         * Params:
         * Returns: joint_group_positions
         * 
         * Returns the current joint positions in the move group
         */
        std::vector<double> get_joint_group_positions();

        /*
         * Params: waypoints
         * Returns: success
         * 
         * Attempts to compute a cartesian trajectory through the eef pose waypoints
         */
        bool plan_cartesian(const std::vector<geometry_msgs::msg::Pose>& waypoints, std::string plan_owner = "");
        
        /*
         * Params: pose
         * Returns: success
         * 
         * Attempts to compute a non-carthesian trajectory to a single eef pose 
         */
        bool plan_pose_goal(geometry_msgs::msg::Pose& pose, std::string plan_owner = "");

        /*
         * Params: joint_group_positions
         * Returns: success
         * 
         * Attempts to compute a non-carthesian trajectory to a single eef pose 
         */
        bool plan_joint_goal(std::vector<double> joint_group_positions, std::string plan_owner = "");

        bool reset_joints();

        /*
         * Params: wait
         * Returns: success
         * 
         * If wait==true, attempts to move the robot with a blocking call and returns after.
         * If wait==false, tells the robot to move asynchronously and returns immediately.
         * In the asynchronous case, success only indicates that the movement was *started* successfully
         */
        bool move(bool wait, std::string plan_owner = "");
        
        /*
         * Params: max_speed
         * Returns:
         * 
         * Sets a maximum cartesian speed. Unit is m/s, but may be inaccurate for now.
         */
        void limit_cartesian_speed(double max_speed);

        /*
         * Params:
         * Returns:
         * 
         * Stops any ongoing movement.
         */
        void stop();

        bool start_servo();

    private:
        bool legacy_cartesian_planner(const std::vector<geometry_msgs::msg::Pose>& waypoints);

        bool pilz_industrial_motion_planner(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                    std::vector<std::string> ids = {}, double blend_radius = 0.0);

        void twist_cb(const geometry_msgs::msg::TwistStamped& msg) const;

        std::shared_ptr<rclcpp::Node> node_;

        // Variables related to the MoveGroup Interface
        std::string group_name_;
        moveit::planning_interface::MoveGroupInterface move_group_;
        std::mutex plan_mutex_; // Mutex to safely handle plan
        std::string plan_owner_;
        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        double jump_threshold_;
        double eef_step_;
        std::mutex max_speed_mutex_; // Mutex to safely handle updating max speed
        double max_speed_ = 0.15; // Max cartesian speed

        // Clients to interact with a MoveIt Servo node
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_servo_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr unpause_servo_client_;

        // Publisher and subscriber for MoveIt Servo twist commands
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

        std::vector<double> default_joint_values_;
};

#endif