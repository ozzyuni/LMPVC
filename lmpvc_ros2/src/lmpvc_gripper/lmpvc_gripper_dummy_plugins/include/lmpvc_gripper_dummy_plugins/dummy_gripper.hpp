#ifndef LMPVC_GRIPPER_PLUGINS_DUMMY_GRIPPER_HPP
#define LMPVC_GRIPPER_PLUGINS_DUMMY_GRIPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include "lmpvc_gripper/gripper_base.hpp"

namespace lmpvc_gripper_dummy_plugins
{
  class DummyGripper : public lmpvc_gripper::BasicGripper
  {
    public:
      void init(std::shared_ptr<rclcpp::Node> node) override;
      bool open() override;
      bool close() override;
      bool set_force(double force) override;

    protected:
      double force_;
  };
} // namespace lmpvc_gripper_plugins

#endif
