#ifndef LEKIWI_BASE_H
#define LEKIWI_BASE_H

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <termios.h>
#include <map>

#include <sensor_msgs/msg/joint_state.hpp>
#include <SCServo_Linux/SCServo.h>
#include "std_srvs/srv/trigger.hpp"
#include <yaml-cpp/yaml.h>


namespace lekiwi_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using ReturnType = hardware_interface::return_type;



class LekiwiBase : public hardware_interface::SystemInterface
{ 
private:
    std::string NodeName{"LeKiwiInterface"};

    bool use_serial_;
    std::string serial_port_;
    int serial_baud_;
    bool torque_enabled_{ true };

    std::vector<double> velocity_comads_;
    std::vector<double> velocity_states_;

    SMS_STS st3215_;


    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;

    int start_sevro_id_{ 7 };
    std::vector<int> servo_directions_{ 1, 1, 1, 1, 1, 1, 1, 1, 1 };  // Direction multipliers: arm normal, wheels
                                                                       // flipped Motor IDs: 1-6 (arm, normal), 7-9
                                                                       // (wheels, flipped)

public:
    LekiwiBase();
    virtual ~LekiwiBase() ;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ReturnType read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    ReturnType write(const rclcpp::Time& time, const rclcpp::Duration& period) override;


};

}




#endif // LEKIWI_BASE_H


