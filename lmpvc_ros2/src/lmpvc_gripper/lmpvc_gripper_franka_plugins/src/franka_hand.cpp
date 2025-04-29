#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/logging.hpp"
#include "lmpvc_gripper_franka_plugins/franka_hand.hpp"

using namespace std::chrono_literals;

namespace lmpvc_gripper_franka_plugins
{
  void FrankaHand::init(std::shared_ptr<rclcpp::Node> node){

    // Setup
    this->node_ = node;
    this->force_ = 0;
    this->max_width_ = 0;

    auto logger = node_->get_logger();

    init_cli();

    // Homing procedure
    if (!homing_client_->wait_for_action_server()) {
      RCLCPP_ERROR(logger, "Franka homing server not available after waiting");
      rclcpp::shutdown();
    }

    this->homing_ready_ = std::promise<bool>();
    auto future = homing_ready_.get_future(); 

    auto homing_msg = Homing::Goal();
    homing_client_->async_send_goal(homing_msg, this->homing_options_);

    if(future.wait_for(30s) == std::future_status::timeout) { // Action callbacks set the result using homing_ready_
      RCLCPP_ERROR(logger, "No response from homing withing 30s");
      rclcpp::shutdown();
    }
    else if(!future.get()) {
      RCLCPP_ERROR(logger, "Homing failed");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(logger, "Franka Hand Plugin Initialized");
  }

  bool FrankaHand::open(){
    auto logger = node_->get_logger();

    if(!stop()){
      RCLCPP_ERROR(logger, "Franka stop service call failed");
      return false;
    }

    if (!move_client_->wait_for_action_server()) {
      RCLCPP_ERROR(logger, "Franka move server not available after waiting");
      return false;
    }

    move_ready_ = std::promise<bool>();
    auto future = move_ready_.get_future(); 

    auto move_msg = Move::Goal();
    move_msg.width = 0.98*max_width_; // Leaving a bit of margin to avoid errors
    move_msg.speed = speed_;

    move_client_->async_send_goal(move_msg, move_options_);

    if(future.wait_for(30s) == std::future_status::timeout) { // Action callbacks set the result using grasp_ready_
      RCLCPP_ERROR(logger, "No response from move within 30s");
      return false;
    }
    else if(!future.get()) {
      RCLCPP_ERROR(logger, "Move failed");
      return false;
    }

    RCLCPP_INFO(logger, "Move completed");
    return true;
  }

  bool FrankaHand::close(){
    auto logger = node_->get_logger();

    if(!stop()){
      RCLCPP_ERROR(logger, "Franka stop service call failed");
      return false;
    }

    if (!grasp_client_->wait_for_action_server()) {
      RCLCPP_ERROR(logger, "Franka grasp server not available after waiting");
      return false;
    }

    grasp_ready_ = std::promise<bool>();
    auto future = grasp_ready_.get_future(); 

    auto grasp_msg = Grasp::Goal();
    grasp_msg.width = 0;
    grasp_msg.epsilon.outer = max_width_; // Grasp regardless of width
    grasp_msg.speed = speed_;
    grasp_msg.force = force_;

    grasp_client_->async_send_goal(grasp_msg, grasp_options_);

    if(future.wait_for(30s) == std::future_status::timeout) { // Action callbacks set the result using grasp_ready_
      RCLCPP_ERROR(logger, "No response from grasp within 30s");
      return false;
    }
    else if(!future.get()) {
      RCLCPP_ERROR(logger, "Grasp failed");
      return false;
    }

    RCLCPP_INFO(logger, "Grasp completed");
    return true;
  }

  bool FrankaHand::set_force(double force){
    auto logger = node_->get_logger();
    force_ = force;
    RCLCPP_INFO(logger, "New force (%f) will be used on future grasps!", force_);
    return true;
  }

  bool FrankaHand::stop(){
    auto logger = node_->get_logger();

    while (!stop_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger, "Interrupted while waiting for the Stop service. Exiting.");
        return false;
      }
      RCLCPP_INFO(logger, "Stop service not available, waiting again...");
    }

    bool success = false;
    std::future_status status;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = stop_client_->async_send_request(request);

    switch (status = result.wait_for(5s); status){
      case std::future_status::deferred:
        RCLCPP_INFO(node_->get_logger(), "Did not receive a response from Stop service");
        success = false;
        break;

      case std::future_status::timeout:
        success = false;
        RCLCPP_INFO(node_->get_logger(), "Did not receive a response from Stop service");
        break;

      case std::future_status::ready:
        success = result.get()->success;
        if(success){
          RCLCPP_INFO_STREAM(node_->get_logger(), "Stop call successful, message:" << result.get()->message);
        }
        else{
          RCLCPP_INFO_STREAM(node_->get_logger(), "Stop call failed, message:" <<  result.get()->message);
        }
        break;
    }
  
    return success;
  }

  void FrankaHand::init_cli(){
    using namespace std::placeholders;

    // Stop service
    this->stop_client_ =
    this->node_->create_client<std_srvs::srv::Trigger>("/fr3_gripper/stop");

    // Homing action
    this->homing_client_ = rclcpp_action::create_client<Homing>(
      this->node_,
      "/fr3_gripper/homing");
    
    this->homing_options_ = rclcpp_action::Client<Homing>::SendGoalOptions();
    
    homing_options_.goal_response_callback =
      std::bind(&FrankaHand::homing_response_callback, this, _1);
    homing_options_.feedback_callback =
      std::bind(&FrankaHand::homing_feedback_callback, this, _1, _2);
    homing_options_.result_callback =
      std::bind(&FrankaHand::homing_result_callback, this, _1);

    // Grasp action
    this->grasp_client_ = rclcpp_action::create_client<Grasp>(
      this->node_,
      "/fr3_gripper/grasp");
    
    this->grasp_options_ = rclcpp_action::Client<Grasp>::SendGoalOptions();
    
    grasp_options_.goal_response_callback =
      std::bind(&FrankaHand::grasp_response_callback, this, _1);
    grasp_options_.feedback_callback =
      std::bind(&FrankaHand::grasp_feedback_callback, this, _1, _2);
    grasp_options_.result_callback =
      std::bind(&FrankaHand::grasp_result_callback, this, _1);

    // Move action
    this->move_client_ = rclcpp_action::create_client<Move>(
      this->node_,
      "/fr3_gripper/move");
    
    this->move_options_ = rclcpp_action::Client<Move>::SendGoalOptions();
    
    move_options_.goal_response_callback =
      std::bind(&FrankaHand::move_response_callback, this, _1);
    move_options_.feedback_callback =
      std::bind(&FrankaHand::move_feedback_callback, this, _1, _2);
    move_options_.result_callback =
      std::bind(&FrankaHand::move_result_callback, this, _1);
  }

  void FrankaHand::homing_response_callback(const HomingGoalHandle::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR(logger, "Homing goal was rejected by Franka Hand");
      homing_ready_.set_value(false);
    } else {
      RCLCPP_INFO(logger, "Homing goal accepted by Franka Hand, waiting for result");
    }
  }
  
  void FrankaHand::homing_feedback_callback(HomingGoalHandle::SharedPtr,
    const std::shared_ptr<const Homing::Feedback> feedback){
    max_width_ = (max_width_ < feedback->current_width) ? feedback->current_width : max_width_;
  }
  
  void FrankaHand::homing_result_callback(const HomingGoalHandle::WrappedResult & result){
    auto logger = node_->get_logger();
    bool action_completed = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        action_completed = true;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(logger, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(logger, "Unknown result code");
        break;
    }

    if(!action_completed) {
      homing_ready_.set_value(false);
    }
    else if(result.result->success) {
      RCLCPP_INFO(logger, "Homing succesful!");
      homing_ready_.set_value(true);
    }
    else {
      RCLCPP_ERROR(logger, "Homing failed: %s", result.result->error.c_str());
      homing_ready_.set_value(false);
    }
  }
   
  void FrankaHand::grasp_response_callback(const GraspGoalHandle::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR(logger, "Grasp goal was rejected by Franka Hand");
      grasp_ready_.set_value(false);
    } else {
      RCLCPP_INFO(logger, "Grasp goal accepted by Franka Hand, waiting for result");
    }
  }
      
  void FrankaHand::grasp_feedback_callback(GraspGoalHandle::SharedPtr,
    const std::shared_ptr<const Grasp::Feedback> feedback){
    (void)feedback;
  }

  void FrankaHand::grasp_result_callback(const GraspGoalHandle::WrappedResult & result){
    auto logger = node_->get_logger();
    bool action_completed = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        action_completed = true;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(logger, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(logger, "Unknown result code");
        break;
    }

    if(!action_completed) {
      grasp_ready_.set_value(false);
    }
    else if(result.result->success) {
      RCLCPP_INFO(logger, "Grasp succesful!");
      grasp_ready_.set_value(true);
    }
    else {
      RCLCPP_ERROR(logger, "Grasp failed: %s", result.result->error.c_str());
      grasp_ready_.set_value(false);
    }
  }

  void FrankaHand::move_response_callback(const MoveGoalHandle::SharedPtr & goal_handle){
    auto logger = node_->get_logger();
    if (!goal_handle) {
      RCLCPP_ERROR(logger, "Move goal was rejected by Franka Hand");
      move_ready_.set_value(false);
    } else {
      RCLCPP_INFO(logger, "Move goal accepted by Franka Hand, waiting for result");
    }
  }

  void FrankaHand::move_feedback_callback(MoveGoalHandle::SharedPtr,
    const std::shared_ptr<const Move::Feedback> feedback){
    (void)feedback;
  }

  void FrankaHand::move_result_callback(const MoveGoalHandle::WrappedResult & result){
    auto logger = node_->get_logger();
    bool action_completed = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        action_completed = true;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger, "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(logger, "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(logger, "Unknown result code");
        break;
    }

    if(!action_completed) {
      move_ready_.set_value(false);
    }
    else if(result.result->success) {
      RCLCPP_INFO(logger, "Homing succesful!");
      move_ready_.set_value(true);
    }
    else {
      RCLCPP_ERROR(logger, "Homing failed: %s", result.result->error.c_str());
      move_ready_.set_value(false);
    }
  }

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(lmpvc_gripper_franka_plugins::FrankaHand, lmpvc_gripper::BasicGripper)