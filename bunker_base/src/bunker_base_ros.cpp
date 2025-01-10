/*
 * bunker_base_ros.cpp
 *
 * Created on: 3 2, 2022 16:41
 * Description:
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#include "bunker_base/bunker_base_ros.hpp"

#include "bunker_base/bunker_messenger.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

namespace westonrobot {

BunkerBaseRos::BunkerBaseRos(std::string node_name)
    : rclcpp::Node(node_name), keep_running_(false) {
  this->declare_parameter("port_name", "can0");

  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("odom_topic_name", "odom");

  this->declare_parameter("simulated_robot", false);
  this->declare_parameter("bunker_variant",
                          static_cast<int>(BunkerRobot::Variant::kBunkerV2));
  this->declare_parameter("control_rate", 50);

  LoadParameters();
}

void BunkerBaseRos::LoadParameters() {
  this->get_parameter_or<std::string>("port_name", port_name_, "can0");
  this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  this->get_parameter_or<std::string>("odom_topic_name", odom_topic_name_,
                                      "odom");
  this->get_parameter_or<bool>("simulated_robot", simulated_robot_, false);
  this->get_parameter_or<int>(
      "bunker_variant", bunker_variant_,
      static_cast<int>(BunkerRobot::Variant::kBunkerV2));
  this->get_parameter_or<int>("control_rate", sim_control_rate_, 50);

  std::cout << "Loading parameters: " << std::endl;
  std::cout << "- port name: " << port_name_ << std::endl;
  std::cout << "- odom frame name: " << odom_frame_ << std::endl;
  std::cout << "- base frame name: " << base_frame_ << std::endl;
  std::cout << "- odom topic name: " << odom_topic_name_ << std::endl;

  std::cout << "- bunker variant: ";

  if (bunker_variant_ == static_cast<int>(BunkerRobot::Variant::kBunkerV1)) {
    std::cout << "Bunker V1" << std::endl;
  } else if (bunker_variant_ ==
             static_cast<int>(BunkerRobot::Variant::kBunkerV2)) {
    std::cout << "Bunker V2" << std::endl;
  } else if (bunker_variant_ ==
             static_cast<int>(BunkerRobot::Variant::kBunkerPro)) {
    std::cout << "Bunker Pro" << std::endl;
  } else if (bunker_variant_ ==
             static_cast<int>(BunkerRobot::Variant::kBunkerMini)) {
    std::cout << "Bunker Mini" << std::endl;
  }
  else{
    std::cout << "Unknown" << std::endl;
  }

  std::cout << "- simulated robot: " << std::boolalpha << simulated_robot_
            << std::endl;
  std::cout << "- sim control rate: " << sim_control_rate_ << std::endl;
  std::cout << "----------------------------" << std::endl;
}

bool BunkerBaseRos::Initialize() {
  if (bunker_variant_ == static_cast<int>(BunkerRobot::Variant::kBunkerV1)) {
    std::cout << "Robot base: Bunker V1" << std::endl;
  } else if (bunker_variant_ ==
             static_cast<int>(BunkerRobot::Variant::kBunkerV2)) {
    std::cout << "Robot base: Bunker V2" << std::endl;
  } else if (bunker_variant_ ==
             static_cast<int>(BunkerRobot::Variant::kBunkerPro)) {
    std::cout << "Robot base: Bunker Pro" << std::endl;
  } else if (bunker_variant_ ==
             static_cast<int>(BunkerRobot::Variant::kBunkerMini)) {
    std::cout << "Robot base: Bunker Mini" << std::endl;
  } else {
    std::cout << "Unknown" << std::endl;
  }

  ProtocolDetector detector;
  if (detector.Connect(port_name_)) {
    if (bunker_variant_ == static_cast<int>(BunkerRobot::Variant::kBunkerV1)) {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      robot_ = std::unique_ptr<BunkerRobot>(
          new BunkerRobot(BunkerRobot::Variant::kBunkerV1));
    } else if (bunker_variant_ ==
               static_cast<int>(BunkerRobot::Variant::kBunkerV2)) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      robot_ = std::unique_ptr<BunkerRobot>(
          new BunkerRobot(BunkerRobot::Variant::kBunkerV2));
    } else if (bunker_variant_ ==
               static_cast<int>(BunkerRobot::Variant::kBunkerPro)) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      robot_ = std::unique_ptr<BunkerRobot>(
          new BunkerRobot(BunkerRobot::Variant::kBunkerPro));
    } else if (bunker_variant_ ==
               static_cast<int>(BunkerRobot::Variant::kBunkerMini)) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      robot_ = std::unique_ptr<BunkerRobot>(
          new BunkerRobot(BunkerRobot::Variant::kBunkerMini));
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return false;
    }
  } else {
    return false;
  }

  return true;
}

void BunkerBaseRos::Stop() { keep_running_ = false; }

void BunkerBaseRos::Run() {
  std::unique_ptr<BunkerMessenger<BunkerRobot>> messenger =
      std::unique_ptr<BunkerMessenger<BunkerRobot>>(
          new BunkerMessenger<BunkerRobot>(robot_, this));

  messenger->SetOdometryFrame(odom_frame_);
  messenger->SetBaseFrame(base_frame_);
  messenger->SetOdometryTopicName(odom_topic_name_);
  if (simulated_robot_) messenger->SetSimulationMode(sim_control_rate_);

  // connect to robot and setup ROS subscription
  if (port_name_.find("can") != std::string::npos) {
    if (robot_->Connect(port_name_)) {
      robot_->EnableCommandedMode();
      std::cout << "Using CAN bus to talk with the robot" << std::endl;
    } else {
      std::cout << "Failed to connect to the robot CAN bus" << std::endl;
      return;
    }
  } else {
    std::cout << "Please check the specified port name is a CAN port"
              << std::endl;
    return;
  }

  // publish robot state at 50Hz while listening to twist commands
  messenger->SetupSubscription();
  keep_running_ = true;
  rclcpp::Rate rate(50);
  while (keep_running_) {
    messenger->PublishStateToROS();
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}
}  // namespace westonrobot
