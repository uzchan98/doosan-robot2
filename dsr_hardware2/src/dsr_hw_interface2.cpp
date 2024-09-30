// /*
//  *  Inferfaces for doosan robot controllor
//   * Author: Minsoo Song(minsoo.song@doosan.com)
//  *
//  * Copyright (c) 2024 Doosan Robotics
//  * Use of this source code is governed by the BSD, see LICENSE
// */

#include "dsr_hardware2/dsr_hw_interface2.h"

#include <Eigen/Dense>

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "DRFC.h"
#include "DRFS.h"

// #include "dsr_hardware2/dsr_connection_node2.h"
#include <fstream>
#include <math.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_interfaces/get_node_base_interface.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pinocchio/multibody/fwd.hpp"

using dsr_hardware2::DRHWInterface;
using namespace DRAFramework;


using Vec6 = Eigen::Vector<double, 6>;

DRHWInterface::CallbackReturn
DRHWInterface::on_init(const hardware_interface::HardwareInfo& info) {
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "on_init() call");
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    const std::string rob_desc = info.original_xml;
    const std::string filepath = "/home/user/robot.urdf";
    std::ofstream     file(filepath);
    file << info.original_xml;
    file.close();
    pinocchio::urdf::buildModel(filepath, robot_mdl_);

    robot_data_ = pinocchio::Data(robot_mdl_);
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Prepared pinocchio model");


    sleep(2);
    control_mode_ = UNKNOWN;

    // robot has 6 joints and 2 interfaces
    joint_position_.assign(6, 0);
    joint_velocities_.assign(6, 0);
    joint_efforts_.assign(6, 0);
    joint_position_command_.assign(6, 0);
    joint_velocities_command_.assign(6, 0);
    joint_efforts_command_.assign(6, 0);

    for (const auto& joint : info_.joints) {
        for (const auto& interface : joint.state_interfaces) {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Connecting real-time control");
    if (!drfl.connect_rt_control("192.168.127.100", 12347)) {
        RCLCPP_ERROR(
                rclcpp::get_logger("dsr_hw_interface2"),
                "Failed to connect to RT control"
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Connected to RT control");

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Setting up control output");
    if (!drfl.set_rt_control_output("v1.0", 0.001, 4)) {
        RCLCPP_ERROR(
                rclcpp::get_logger("dsr_hw_interface2"),
                "Failed to configure to RT control"
        );
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Starting RT control");
    drfl.start_rt_control();
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Started!");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DRHWInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"]) {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"]) {
        state_interfaces.emplace_back(
                joint_name, "velocity", &joint_velocities_[ind++]
        );
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"]) {
        state_interfaces.emplace_back(joint_name, "effort", &joint_efforts_[ind++]);
    }
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Exported state interfaces");

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DRHWInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"]) {
        command_interfaces.emplace_back(
                joint_name, "position", &joint_position_command_[ind++]
        );
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"]) {
        command_interfaces.emplace_back(
                joint_name, "velocity", &joint_velocities_command_[ind++]
        );
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"]) {
        command_interfaces.emplace_back(
                joint_name, "effort", &joint_efforts_command_[ind++]
        );
    }

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Exported command interfaces");
    return command_interfaces;
}

hardware_interface::return_type
DRHWInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& /* stop_interfaces */
) {
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Called command mode switch");
    if (start_interfaces.empty()) {
        RCLCPP_DEBUG(
                rclcpp::get_logger("dsr_hw_interface2"),
                "No control_mode is specified, leaving unchanged!"
        );
        return return_type::OK;
    }

    const std::string req_cmd_interface = start_interfaces[0];
    const std::string mode = req_cmd_interface.substr(req_cmd_interface.find('/') + 1);

    if (mode == hardware_interface::HW_IF_POSITION) {
        RCLCPP_INFO(
                rclcpp::get_logger("dsr_hw_interface2"), "Switching to position control"
        );
        control_mode_ = POSITION;
    } else if (mode == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO(
                rclcpp::get_logger("dsr_hw_interface2"), "Switching to velocity control"
        );
        control_mode_ = VELOCITY;
    } else if (mode == hardware_interface::HW_IF_EFFORT) {
        RCLCPP_INFO(
                rclcpp::get_logger("dsr_hw_interface2"), "Switching to torque control"
        );
        control_mode_ = TORQUE;
    } else {
        RCLCPP_ERROR(
                rclcpp::get_logger("dsr_hw_interface2"),
                "Unknown control mode %s",
                mode.data()
        );
        return hardware_interface::return_type::ERROR;
    }
    return return_type::OK;
}

// #define LOG_STATE_MSG

DRHWInterface::return_type
DRHWInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    double now_sec = std::chrono::duration_cast<std::chrono::seconds>(
                             std::chrono::system_clock::now().time_since_epoch()
    )
                             .count();
    long int        now_ns;
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    now_ns = spec.tv_nsec;

    Eigen::VectorXd q(6);
    Eigen::VectorXd qd(6);
    Eigen::VectorXd tau(6);

    auto deg2rad = [](const double angle) -> double { return angle * M_PI / 180.0; };

    for (long i = 0; i < 6; i++) {
        joint_position_[i] = deg2rad(
                static_cast<double>(drfl.read_data_rt()->actual_joint_position[i])
        );
        joint_velocities_[i] = deg2rad(
                static_cast<double>(drfl.read_data_rt()->actual_joint_velocity[i])
        );
        joint_efforts_[i] =
                static_cast<double>(drfl.read_data_rt()->raw_joint_torque[i]);
        q[i]   = joint_position_[i];
        qd[i]  = joint_velocities_[i];
        tau[i] = joint_efforts_[i];
    }

    pinocchio::computeAllTerms(robot_mdl_, robot_data_, q, qd);
    return return_type::OK;
}

DRHWInterface::return_type
DRHWInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    float positions[6]     = {-10000, -10000, -10000, -10000, -10000, -10000};
    float velocities[6]    = {-10000, -10000, -10000, -10000, -10000, -10000};
    float accelerations[6] = {
            -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0};
    float torques[6];
    float max_velocities[6]    = {70.0, 70.0, 70.0, 70.0, 70.0, 70.0};
    float max_accelerations[6] = {500.0, 500.0, 500.0, 500.0, 500.0, 500.0};

    static int id_msgs = 0;

    static bool is_initialiased = false;


    switch (control_mode_) {
        case POSITION:
            return return_type::ERROR;
            break;

        case VELOCITY:
            return return_type::ERROR;
            break;

        case TORQUE:
            if (!is_initialiased) {
                for (int i = 0; i < 6; i++)
                    torque_state[i] = drfl.read_data_rt()->external_joint_torque[i];
                // torque_state[i] = static_cast<float>(joint_efforts_command_[i]);
                drfl.set_velj_rt(max_velocities);
                drfl.set_accj_rt(max_accelerations);
                is_initialiased = true;
            }

            float grav_torques[6];
            for (long i = 0; i < 6; i++)
                grav_torques[i] = drfl.read_data_rt()->gravity_torque[i];
            // for(long i = 0; i < 6; i++)
            //     grav_torques[i] = robot_data_.g(i);
            // grav_torques[1] = 0.0; // second axis on H-series robot is automatically
            // compensated

            for (int i = 0; i < 6; ++i)
                torque_state[i] += 1e-3
                                   * (static_cast<float>(joint_efforts_command_[i])
                                      - torque_state[i]);
            for (long i = 0; i < 6; i++)
                torques[i] =
                        static_cast<float>(joint_efforts_command_[i]) + grav_torques[i];
            for (long i = 0; i < 6; i++) torques[i] = torque_state[i] + grav_torques[i];

            for (long i = 0; i < 6; i++)
                drfl.read_data_rt()->target_joint_position[i] =
                        drfl.read_data_rt()->actual_joint_position[i];

            // Drfl.servoj_rt(
            //         Drfl.read_data_rt()->actual_joint_position,
            //         Drfl.read_data_rt()->actual_joint_velocity,
            //         accelerations,
            //         00.0
            //         );

            if (!drfl.torque_rt(torques, 0.0)) return return_type::ERROR;
#if 0
            if (unitn_log) {
                // std::cout << "torque: " << joint_efforts_command_[0] << ", " <<
                // joint_efforts_command_[1] << ", " << joint_efforts_command_[2] <<
                // std::endl;
                std::cout << "Gravity torque: ";
                for (long j = 0; j < 6; ++j) std::cout << grav_torques[j] << ", ";
                std::cout << std::endl;

                std::cout << "Commanded torque: ";
                for (long j = 0; j < 6; ++j)
                    std::cout << joint_efforts_command_[j] << ", ";
                std::cout << std::endl;

                std::cout << "Actual joint torque: ";
                for (long j = 0; j < 6; ++j)
                    std::cout << Drfl.read_data_rt()->actual_joint_torque[j] << ", ";
                std::cout << std::endl;

                std::cout << "Raw joint torque: ";
                for (long j = 0; j < 6; ++j)
                    std::cout << Drfl.read_data_rt()->raw_joint_torque[j] << ", ";
                std::cout << std::endl;

                std::cout << "External joint torque: ";
                for (long j = 0; j < 6; ++j)
                    std::cout << Drfl.read_data_rt()->external_joint_torque[j] << ", ";
                std::cout << std::endl;
            }
#endif
            break;

        case UNKNOWN:
            break;
    }

    std_msgs::msg::Float64MultiArray msg_cmd_torque;
    std::transform(
            torque_state,
            torque_state + 6,
            std::back_inserter(msg_cmd_torque.data),
            [](const float& data) -> double { return static_cast<double>(data); }
    );
    pub_cmd_torque->publish(msg_cmd_torque);

    std_msgs::msg::Float64MultiArray msg_act_torque;
    std::transform(
            drfl.read_data_rt()->actual_joint_torque,
            drfl.read_data_rt()->actual_joint_torque + 6,
            std::back_inserter(msg_act_torque.data),
            [](const float& data) -> double { return static_cast<double>(data); }
    );
    pub_act_joint_torque->publish(msg_act_torque);

    std_msgs::msg::Float64MultiArray msg_raw_torque;
    std::transform(
            drfl.read_data_rt()->raw_joint_torque,
            drfl.read_data_rt()->raw_joint_torque + 6,
            std::back_inserter(msg_raw_torque.data),
            [](const float& data) -> double { return static_cast<double>(data); }
    );
    pub_raw_joint_torque->publish(msg_raw_torque);

    std_msgs::msg::Float64MultiArray msg_ext_torque;
    std::transform(
            drfl.read_data_rt()->external_joint_torque,
            drfl.read_data_rt()->external_joint_torque + 6,
            std::back_inserter(msg_ext_torque.data),
            [](const float& data) -> double { return static_cast<double>(data); }
    );
    pub_ext_joint_torque->publish(msg_ext_torque);


    return return_type::OK;
}

DRHWInterface::~DRHWInterface() {
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Destructor called");
    if (!drfl.stop_rt_control()) {
        RCLCPP_WARN(
                rclcpp::get_logger("dsr_hw_interface2"), "Error in stop_rt_control()"
        );
    }
    if (!drfl.disconnect_rt_control()) {
        RCLCPP_WARN(
                rclcpp::get_logger("dsr_hw_interface2"),
                "Error in disconnect_rt_control()"
        );
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(

        dsr_hardware2::DRHWInterface, hardware_interface::SystemInterface
)
