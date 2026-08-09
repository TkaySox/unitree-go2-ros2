/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Ported from the Gazebo Classic transport API to Gazebo Harmonic (gz-sim).
 * Gazebo Harmonic has no gazebo::transport C++ client library, so instead of
 * subscribing directly to the simulator, this node subscribes to a ROS topic
 * that ros_gz_bridge has already translated from gz.msgs.Contacts into
 * ros_gz_interfaces/msg/Contacts. See champ_gazebo/config/contact_bridge.yaml
 * for the bridge configuration that feeds this node.
 */
#include <rclcpp/rclcpp.hpp>
#include <champ/utils/urdf_loader.h>
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <boost/algorithm/string.hpp>

class ContactSensor : public rclcpp::Node
{
  bool foot_contacts_[4];
  std::vector<std::string> foot_links_;
  rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contacts_subscriber_;

public:
  ContactSensor()
  : Node(
      "contacts_sensor", rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)),
    foot_contacts_ {false, false, false, false}
  {
    std::vector<std::string> joint_names;

    joint_names = champ::URDF::getLinkNames(this->get_node_parameters_interface());
    foot_links_.push_back(joint_names[2]);
    foot_links_.push_back(joint_names[6]);
    foot_links_.push_back(joint_names[10]);
    foot_links_.push_back(joint_names[14]);

    contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>(
      "foot_contacts", 10);

    // Populated by ros_gz_bridge from the gz-sim "contact" sensor plugin,
    // see champ_gazebo/config/contact_bridge.yaml.
    contacts_subscriber_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "gz/contacts", rclcpp::SensorDataQoS(),
      std::bind(&ContactSensor::gzContactsCallback_, this, std::placeholders::_1));
  }

  void gzContactsCallback_(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    for (size_t i = 0; i < 4; i++) {
      foot_contacts_[i] = false;
    }

    for (const auto & contact : msg->contacts) {
      std::vector<std::string> results;
      boost::split(results, contact.collision1.name, [](char c) {return c == ':';});

      if (results.empty()) {
        continue;
      }

      for (size_t j = 0; j < 4; j++) {
        if (foot_links_[j] == results.back()) {
          foot_contacts_[j] = true;
          break;
        }
      }
    }
  }

  void publishContacts()
  {
    champ_msgs::msg::ContactsStamped contacts_msg;
    contacts_msg.header.stamp = this->get_clock()->now();
    contacts_msg.contacts.resize(4);

    for (size_t i = 0; i < 4; i++) {
      contacts_msg.contacts[i] = foot_contacts_[i];
    }

    contacts_publisher_->publish(contacts_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContactSensor>();
  rclcpp::Rate loop_rate(50);

  while (rclcpp::ok()) {
    node->publishContacts();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
