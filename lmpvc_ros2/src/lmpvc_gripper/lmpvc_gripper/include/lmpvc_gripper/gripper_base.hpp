#ifndef RCM_GRIPPER_GRIPPER_BASE_HPP
#define RCM_GRIPPER_GRIPPER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include "lmpvc_interfaces/srv/controller_close_hand.hpp"
#include "lmpvc_interfaces/srv/controller_open_hand.hpp"

namespace lmpvc_gripper
{
  class BasicGripper 
  {
    public:
      virtual void init(std::shared_ptr<rclcpp::Node> node) = 0;
      virtual bool open() = 0;
      virtual bool close() = 0;
      virtual bool set_force(double force) = 0;
      virtual ~BasicGripper(){}
      
      void init_default_services();

    protected:
      BasicGripper(){}

      void close_hand_cb(const std::shared_ptr<lmpvc_interfaces::srv::ControllerCloseHand::Request> request,
        std::shared_ptr<lmpvc_interfaces::srv::ControllerCloseHand::Response> response);

      void open_hand_cb(const std::shared_ptr<lmpvc_interfaces::srv::ControllerOpenHand::Request> request,
        std::shared_ptr<lmpvc_interfaces::srv::ControllerOpenHand::Response> response);
      
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Service<lmpvc_interfaces::srv::ControllerCloseHand>::SharedPtr close_service_;
      rclcpp::Service<lmpvc_interfaces::srv::ControllerOpenHand>::SharedPtr open_service_;
  };
} // namespace lmpvc_gripper

#endif