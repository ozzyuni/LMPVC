#ifndef LMPVC_MOVEIT_SERVICES
#define LMPVC_MOVEIT_SERVICES

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "lmpvc_controller/moveit_controller.hpp"
#include "lmpvc_interfaces/srv/controller_exec.hpp"
#include "lmpvc_interfaces/srv/controller_get_pose.hpp"
#include "lmpvc_interfaces/srv/controller_plan.hpp"
#include "lmpvc_interfaces/srv/controller_stop.hpp"
#include "lmpvc_interfaces/srv/controller_set_speed.hpp"
#include <std_srvs/srv/trigger.hpp>

/*
 * A service wrapper for the RCM Moveit Controller
 *
 * Adds services to a node object, with callbacks using a MoveItController object.
 */
class MoveitServices {
    public:
        /*
         * Params: node, controller
         *
         * Adds services through the node pointer, the callbacks of which use the controller pointer
         * to implement the responses.
         */
        MoveitServices(std::shared_ptr<rclcpp::Node>& node, std::shared_ptr<MoveitController>& controller);

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveitController> controller_;
        
        rclcpp::Service<lmpvc_interfaces::srv::ControllerExec>::SharedPtr execute_service_;
        rclcpp::Service<lmpvc_interfaces::srv::ControllerGetPose>::SharedPtr get_pose_service_;
        rclcpp::Service<lmpvc_interfaces::srv::ControllerPlan>::SharedPtr plan_cartesian_service_;
        rclcpp::Service<lmpvc_interfaces::srv::ControllerSetSpeed>::SharedPtr set_speed_service_;
        rclcpp::Service<lmpvc_interfaces::srv::ControllerStop>::SharedPtr stop_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr joint_reset_service_;

        void execute_cb(const std::shared_ptr<lmpvc_interfaces::srv::ControllerExec::Request> request,
                        std::shared_ptr<lmpvc_interfaces::srv::ControllerExec::Response> response);

        void get_pose_cb(const std::shared_ptr<lmpvc_interfaces::srv::ControllerGetPose::Request> request,
                        std::shared_ptr<lmpvc_interfaces::srv::ControllerGetPose::Response> response);

        void plan_cartesian_cb(const std::shared_ptr<lmpvc_interfaces::srv::ControllerPlan::Request> request,
                        std::shared_ptr<lmpvc_interfaces::srv::ControllerPlan::Response> response);

        void set_speed_cb(const std::shared_ptr<lmpvc_interfaces::srv::ControllerSetSpeed::Request> request,
                        std::shared_ptr<lmpvc_interfaces::srv::ControllerSetSpeed::Response> response);

        void stop_cb(const std::shared_ptr<lmpvc_interfaces::srv::ControllerStop::Request> request,
                        std::shared_ptr<lmpvc_interfaces::srv::ControllerStop::Response> response);
        
        void joint_reset_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};

#endif