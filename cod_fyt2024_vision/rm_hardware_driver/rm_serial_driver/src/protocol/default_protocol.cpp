

#include "rm_serial_driver/protocol/default_protocol.hpp"

namespace fyt::serial_driver::protocol {

DefaultProtocol::DefaultProtocol(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<16>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> DefaultProtocol::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  return {node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); })

  };
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> DefaultProtocol::getClients(
  rclcpp::Node::SharedPtr node) const {
  return {node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                           rmw_qos_profile_services_default)};
}

void DefaultProtocol::send(const rm_interfaces::msg::GimbalCmd &data) {
  FixedPacket<16> packet;
  packet.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  packet.loadData<float>(static_cast<float>(data.pitch), 2);
  packet.loadData<float>(static_cast<float>(data.yaw), 6);
  packet.loadData<float>(static_cast<float>(data.distance), 10);
  packet_tool_->sendPacket(packet);
}

bool DefaultProtocol::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<16> packet;
  if (packet_tool_->recvPacket(packet)) {//找到小黑子了，就是你在偷偷传数据
    packet.unloadData(data.mode, 1);
    packet.unloadData(data.roll, 2);
    packet.unloadData(data.pitch, 6);
    packet.unloadData(data.yaw, 10);
    return true;
  } else {
    return false;
  }
}

}  // namespace fyt::serial_driver::protocol
