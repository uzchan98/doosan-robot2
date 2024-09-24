// /*
//  *  Inferfaces for doosan robot controllor 
//   * Author: Minsoo Song(minsoo.song@doosan.com)
//  *
//  * Copyright (c) 2024 Doosan Robotics
//  * Use of this source code is governed by the BSD, see LICENSE
// */

#include "dsr_hardware2/dsr_hw_interface2.h"
#include "DRFC.h"
#include "DRFS.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/Dense>

// #include "dsr_hardware2/dsr_connection_node2.h"
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node_interfaces/get_node_base_interface.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <unistd.h>     
#include <math.h>
#include "../../common2/include/DRFLEx.h"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pinocchio/multibody/fwd.hpp"
using namespace DRAFramework;


using Vec6 = Eigen::Vector<double, 6>;

rclcpp::Node::SharedPtr s_node_ = nullptr;
rclcpp::Node::SharedPtr m_node_ = nullptr; //ROS2
CDRFLEx Drfl;
//TODO Serial_comm ser_comm;
sensor_msgs::msg::JointState msg;
bool g_bIsEmulatorMode = FALSE;
bool g_bHasControlAuthority = FALSE;
bool g_bTpInitailizingComplted = FALSE;
bool g_bHommingCompleted = FALSE;

ROBOT_JOINT_DATA g_joints[NUM_JOINT];

DR_STATE    g_stDrState;
DR_ERROR    g_stDrError;

int g_nAnalogOutputModeCh1;
int g_nAnalogOutputModeCh2;
int m_nVersionDRCF;

bool unitn_log = false;

int nDelay = 5000;
#define STABLE_BAND_JNT     0.05
#define DSR_CTL_PUB_RATE    100  //[hz] 10ms <----- 퍼블리싱 주기, but OnMonitoringDataCB() 은 100ms 마다 불려짐을 유의!   
void* get_drfl(){
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[DRFL address] %p", &Drfl);
    return &Drfl;
}
void* get_s_node_(){
    return &s_node_;
}

bool init_check = true;

void threadFunction() {
    s_node_ = rclcpp::Node::make_shared("dsr_hw_interface2");
    
    std::string param_name = std::string(s_node_->get_namespace()) + "_parameters.yaml";
    std::string package_directory = ament_index_cpp::get_package_share_directory("dsr_hardware2");
    std::string yaml_file_path = package_directory + "/config" + param_name;

    std::ifstream fin(yaml_file_path);
    if (!fin) {
        RCLCPP_ERROR(s_node_->get_logger(), "Failed to open YAML file: %s", yaml_file_path.c_str());
        return;
    }

    // YAML 파일 파싱
    YAML::Node yaml_node = YAML::Load(fin);
    fin.close();
    
    // 파싱된 YAML 노드에서 파라미터 읽기
    if (yaml_node["name"]) {
        m_name = yaml_node["name"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "name: %s", m_name.c_str());
    }
    if (yaml_node["rate"]) {
        m_rate = yaml_node["rate"].as<int>();
        RCLCPP_INFO(s_node_->get_logger(), "rate: %d", m_rate);
    }
    if (yaml_node["standby"]) {
        m_standby = yaml_node["standby"].as<int>();
        RCLCPP_INFO(s_node_->get_logger(), "standby: %d", m_standby);
    }
    if (yaml_node["command"]) {
        m_command = yaml_node["command"].as<bool>();
        RCLCPP_INFO(s_node_->get_logger(), "command: %s", m_command ? "true" : "false");
    }
    if (yaml_node["host"]) {
        m_host = yaml_node["host"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "host: %s", m_host.c_str());
    }
    if (yaml_node["port"]) {
        m_port = yaml_node["port"].as<int>();
        RCLCPP_INFO(s_node_->get_logger(), "port: %d", m_port);
    }
    if (yaml_node["mode"]) {
        m_mode = yaml_node["mode"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "mode: %s", m_mode.c_str());
    }
    if (yaml_node["model"]) {
        m_model = yaml_node["model"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "model: %s", m_model.c_str());
    }
    if (yaml_node["gripper"]) {
        m_gripper = yaml_node["gripper"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "gripper: %s", m_gripper.c_str());
    }
    if (yaml_node["mobile"]) {
        m_mobile = yaml_node["mobile"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "mobile: %s", m_mobile.c_str());
    }
}


namespace dsr_hardware2{


CallbackReturn DRHWInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    const std::string rob_desc = info.original_xml;
    const std::string filepath ="/home/user/robot.urdf";
    std::ofstream file(filepath);
    file << info.original_xml;
    file.close();
    std::cerr << "building model" << std::endl;
    pinocchio::urdf::buildModel(filepath, robot_mdl_);
    std::cerr << "model built" << std::endl;

    robot_data_ = pinocchio::Data(robot_mdl_);
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Prepared pinocchio model"); 


    sleep(4);
    control_mode_ = UNKNOWN;

    // robot has 6 joints and 2 interfaces
    joint_position_.assign(6, 0);
    joint_velocities_.assign(6, 0);
    joint_efforts_.assign(6, 0);
    joint_position_command_.assign(6, 0);
    joint_velocities_command_.assign(6, 0);
    joint_efforts_command_.assign(6, 0);

    for (const auto & joint : info_.joints)
    {
        for (const auto & interface : joint.state_interfaces)
        {
        joint_interfaces[interface.name].push_back(joint.name);
        }
    }
    
    auto joint_names = {
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    };

    // size_t i = 0;
    // for (auto & joint_name : joint_names)
    // {
    //     // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"joint_name = %s", joint_name);
    //     ++i;
    // }
    std::thread t(threadFunction);
    t.join(); // need to make sure termination of the thread.

//-----------------------------------------------------------------------------------------------------
    

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------"); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    INITAILIZE");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------"); 
    //--- doosan API's call-back fuctions : Only work within 50msec in call-back functions
#ifdef USE_FULL_LIB
    Drfl.set_on_tp_initializing_completed(DSRInterface::OnTpInitializingCompletedCB);
    Drfl.set_on_homming_completed(DSRInterface::OnHommingCompletedCB);
    Drfl.set_on_program_stopped(DSRInterface::OnProgramStoppedCB);
    Drfl.set_on_monitoring_modbus(DSRInterface::OnMonitoringModbusCB);
    Drfl.set_on_monitoring_data(DSRInterface::OnMonitoringDataCB);           // Callback function in M2.4 and earlier
    Drfl.set_on_monitoring_ctrl_io(DSRInterface::OnMonitoringCtrlIOCB);       // Callback function in M2.4 and earlier
    Drfl.set_on_monitoring_state(DSRInterface::OnMonitoringStateCB);
    Drfl.set_on_monitoring_access_control(DSRInterface::OnMonitoringAccessControlCB);
    Drfl.set_on_log_alarm(DSRInterface::OnLogAlarm);
#endif
    
    m_node_ = rclcpp::Node::make_shared("dsr_hw_interface_update");
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    m_joint_state_pub_ = m_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

    pub_cmd_torque = m_node_->create_publisher<std_msgs::msg::Float64MultiArray>("dbg_commanded_torque", rclcpp::QoS(10));
    pub_act_joint_torque = m_node_->create_publisher<std_msgs::msg::Float64MultiArray>("dbg_actual_torque", rclcpp::QoS(10));
    pub_raw_joint_torque = m_node_->create_publisher<std_msgs::msg::Float64MultiArray>("dbg_raw_torque", rclcpp::QoS(10));
    pub_ext_joint_torque = m_node_->create_publisher<std_msgs::msg::Float64MultiArray>("dbg_external_torque", rclcpp::QoS(10));
    //------------------------------------------------------------------------------
    // await for values from ros parameters
    while(m_host == "")
    {
        usleep(nDelay);
    }
    if(Drfl.open_connection(m_host, m_port))
    {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------"); 
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    OPEN CONNECTION");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------");   

        //--- connect Emulator ? ------------------------------    
        if(m_host == "127.0.0.1") g_bIsEmulatorMode = true; 
        else                    g_bIsEmulatorMode = false;

        //--- Get version -------------------------------------            
        SYSTEM_VERSION tSysVerion = {'\0', };
        assert(Drfl.get_system_version(&tSysVerion));

        //--- Get DRCF version & convert to integer  ----------            
        m_nVersionDRCF = 0; 
        int k=0;
        for(int i=strlen(tSysVerion._szController); i>0; i--)
            if(tSysVerion._szController[i]>='0' && tSysVerion._szController[i]<='9')
                m_nVersionDRCF += (tSysVerion._szController[i]-'0')*pow(10.0,k++);
        if(m_nVersionDRCF < 100000) m_nVersionDRCF += 100000; 

        if(g_bIsEmulatorMode) RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Emulator Mode");
        else                  RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Real Robot Mode");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRCF version = %s",tSysVerion._szController);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRFL version = %s",Drfl.get_library_version());
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    m_nVersionDRCF = %d", m_nVersionDRCF);  //ex> M2.40 = 120400, M2.50 = 120500  
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------");   
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"Set UNKNOWN control mode"); 

        if(m_nVersionDRCF >= 120500)    //M2.5 or later        
        {
            Drfl.set_on_monitoring_data_ex(DSRInterface::OnMonitoringDataExCB);      //Callback function in version 2.5 and higher
            Drfl.set_on_monitoring_ctrl_io_ex(DSRInterface::OnMonitoringCtrlIOExCB);  //Callback function in version 2.5 and higher                     
            Drfl.setup_monitoring_version(1);                        //Enabling extended monitoring functions 
        }

        //--- Check Robot State : STATE_STANDBY ---               
        while ((Drfl.GetRobotState() != STATE_STANDBY)){
            usleep(nDelay);
        }

        //--- Set Robot mode : MANUAL or AUTO
        assert(Drfl.SetRobotMode(ROBOT_MODE_AUTONOMOUS));

        //--- Set Robot mode : virual or real 
        ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
        if(m_mode == "real") eTargetSystem = ROBOT_SYSTEM_REAL;
        assert(Drfl.SetRobotSystem(eTargetSystem));

        // to compare with g_joints[].cmd
        for(int i = 0; i < NUM_JOINT; i++){
            RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    [init]::read %d-pos: %7.3f", i, g_joints[i].cmd);
            m_fCmd_[i] = g_joints[i].cmd;
        }

        if (!Drfl.connect_rt_control(m_host, 12347)) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Failed to connect to RT control");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Connected to RT control");

        if (!Drfl.set_rt_control_output("v1.0", 0.001, 4)) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Failed to configure to RT control");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "RT control configured");

        if (!Drfl.start_rt_control()) {
            RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Failed to start RT control");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"Started RT control");
        // float vel[6] = {100, 100, 100, 100, 100, 100};
        // Drfl.set_velj_rt(vel);

        Drfl.change_collision_sensitivity(0.0);
        Drfl.set_robot_speed_mode(SPEED_NORMAL_MODE);

        return CallbackReturn::SUCCESS;
    }
    RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"    DSRInterface::init() DRCF connecting ERROR!!!");

  return CallbackReturn::ERROR;
// return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DRHWInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    cout << "[callback StateInterface] StateInterface state_interfaces: " << joint_position_[ind] << endl;

    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["effort"])
  {
    state_interfaces.emplace_back(joint_name, "effort", &joint_efforts_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DRHWInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    cout << "[callback CommandInterface] CommandInterface joint_position_command_: " << joint_position_command_[ind] << endl;
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["effort"])
  {
    command_interfaces.emplace_back(joint_name, "effort", &joint_efforts_command_[ind++]);
  }

  return command_interfaces;
}


hardware_interface::return_type
DRHWInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces, const std::vector<std::string>& /* stop_interfaces */ )
{
    if (start_interfaces.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"), "No control_mode is specified, leaving unchanged!");
        return return_type::OK;
    }
    
    const std::string req_cmd_interface = start_interfaces[0];
    const std::string mode = req_cmd_interface.substr(req_cmd_interface.find('/') + 1);

    if (mode == hardware_interface::HW_IF_POSITION) {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Switching to position control");
        control_mode_ = POSITION;
    } else if (mode == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Switching to velocity control");
        control_mode_ = VELOCITY;
    } else if (mode == hardware_interface::HW_IF_EFFORT) {
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "Switching to torque control");
        control_mode_ = TORQUE;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "Unknown control mode %s", mode.data());
        return hardware_interface::return_type::ERROR;
    }
    return return_type::OK;
}

// #define LOG_STATE_MSG

return_type DRHWInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    static long unitn_i = 0;
    unitn_log = (unitn_i % 3000) == 0;
    unitn_log = false;
    unitn_i++;
    double now_sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    long int now_ns;
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    now_ns = spec.tv_nsec;
    msg.header.stamp.sec = (long)now_sec;
    msg.header.stamp.nanosec = now_ns;

    // LPROBOT_POSE joint_pos = Drfl.get_current_posj();
    // LPROBOT_VEL joint_vel = Drfl.get_current_velj();
    // LPROBOT_FORCE joint_tau = Drfl.get_joint_torque();

    msg.name.push_back("joint_1");
    msg.name.push_back("joint_2");
    msg.name.push_back("joint_3");
    msg.name.push_back("joint_4");
    msg.name.push_back("joint_5");
    msg.name.push_back("joint_6");

    Eigen::VectorXd  q(6);
    Eigen::VectorXd  qd(6);
    Eigen::VectorXd  tau(6);
    
    for(long i = 0; i < 6; i++) {
        // joint_position_[i] = static_cast<double>(Drfl.read_data_rt()->actual_joint_position_abs[i]);
        joint_position_[i] = deg2rad(static_cast<double>(Drfl.read_data_rt()->actual_joint_position[i]));
        joint_velocities_[i] = deg2rad(static_cast<double>(Drfl.read_data_rt()->actual_joint_velocity[i]));
        joint_efforts_[i] = static_cast<double>(Drfl.read_data_rt()->raw_joint_torque[i]);
        q[i] = joint_position_[i];
        qd[i] = joint_velocities_[i];
        tau[i] = joint_efforts_[i];

        msg.position.push_back(joint_position_[i]);
        msg.velocity.push_back(joint_velocities_[i]);
        msg.effort.push_back(joint_efforts_[i]);
    }

    pinocchio::computeAllTerms(robot_mdl_, robot_data_, q, qd);
#if 0
    if (unitn_log) {
        Eigen::VectorXd doosan_grav_torque(6);
        for(int j = 0; j < 6; j++)
            doosan_grav_torque(j) = Drfl.read_data_rt()->gravity_torque[j];
        std::cout << "Doosan grav torque: " << doosan_grav_torque.transpose()  << std::endl;
        std::cout << "Pinocchio   torque: " << robot_data_.g.transpose()  << std::endl;

    }
#endif

#if 0
    if (unitn_log) {
        std::cout << "Set torque: ";
        for (long i = 0; i < 6; ++i)
            std::cout << Drfl.read_data_rt()->target_motor_torque[i] << ", ";
        std::cout << std::endl;
        std::cout << "Meas torque: ";
        for (long i = 0; i < 6; ++i)
            std::cout << Drfl.read_data_rt()->raw_joint_torque[i] << ", ";
        std::cout << std::endl;
    }
#endif
#if 0
    if (unitn_log) {
        std::cout << "----\n";
        std::cout << "Des. pos.: ";
        for (long i = 0; i < 6; ++i)
            std::cout << Drfl.read_data_rt()->target_joint_position[i] << ", ";
        std::cout << std::endl;
        std::cout << "Des. vel.: ";
        for (long i = 0; i < 6; ++i)
            std::cout << Drfl.read_data_rt()->target_joint_velocity[i] << ", ";
        std::cout << std::endl;
        // std::cout << "Meas torque: ";
        // for (long i = 0; i < 6; ++i)
        //     std::cout << Drfl.read_data_rt()->raw_joint_torque[i] << ", ";
        // std::cout << std::endl;
    }
#endif

    m_joint_state_pub_->publish(msg);
    msg.position.clear();
    msg.velocity.clear();
    msg.effort.clear();
    msg.name.clear();
  return return_type::OK;
}

return_type DRHWInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    float positions[6] = {-10000, -10000, -10000, -10000, -10000, -10000};
    float velocities[6] = {-10000, -10000, -10000, -10000, -10000, -10000};
    float accelerations[6] = {-10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0};
    float torques[6];
    float max_velocities[6] = {70.0, 70.0, 70.0, 70.0, 70.0, 70.0};
    float max_accelerations[6] = {500.0, 500.0, 500.0, 500.0, 500.0, 500.0};

    static int id_msgs = 0;

    static bool is_initialiased = false;


    switch(control_mode_){
        case POSITION:
            for(long i = 0; i < 6; i++)
                positions[i] = rad2deg(static_cast<float>(joint_position_command_[i]));
            positions[5] = Drfl.read_data_rt()->actual_joint_position[5];
            if(!Drfl.servoj_rt(positions, velocities, accelerations, 10.0)) return return_type::ERROR;
            break;

        case VELOCITY:
            for(long i = 0; i < 6; i++)
                velocities[i] = rad2deg(static_cast<float>(joint_velocities_command_[i]));
            if(!Drfl.speedj_rt(velocities, accelerations, 0.0)) return return_type::ERROR;
            break;
            
        case TORQUE:
            if(!is_initialiased) {
                for (int i = 0; i < 6; i++) 
                    torque_state[i] =Drfl.read_data_rt()->external_joint_torque[i];
                    // torque_state[i] = static_cast<float>(joint_efforts_command_[i]);
                Drfl.set_velj_rt(max_velocities);
                Drfl.set_accj_rt(max_accelerations);
                is_initialiased = true;
            }
        
            float grav_torques[6];
            for(long i = 0; i < 6; i++)
                grav_torques[i] = Drfl.read_data_rt()->gravity_torque[i];
            // for(long i = 0; i < 6; i++)
            //     grav_torques[i] = robot_data_.g(i);
            // grav_torques[1] = 0.0; // second axis on H-series robot is automatically compensated

            for (int i = 0; i < 6; ++i) 
                torque_state[i] += 1e-3*(static_cast<float>(joint_efforts_command_[i]) - torque_state[i]);
            for(long i = 0; i < 6; i++)
                torques[i] = static_cast<float>(joint_efforts_command_[i]) + grav_torques[i];
            for(long i = 0; i < 6; i++)
                torques[i] = torque_state[i] + grav_torques[i];

            for(long i = 0; i < 6; i++)
                Drfl.read_data_rt()->target_joint_position[i] = Drfl.read_data_rt()->actual_joint_position[i];

            // Drfl.servoj_rt(
            //         Drfl.read_data_rt()->actual_joint_position,
            //         Drfl.read_data_rt()->actual_joint_velocity,
            //         accelerations, 
            //         00.0
            //         );

            if(!Drfl.torque_rt(torques, 0.0)) return return_type::ERROR;
            #if 1
            if(unitn_log) {
                // std::cout << "torque: " << joint_efforts_command_[0] << ", " << joint_efforts_command_[1] << ", " << joint_efforts_command_[2] << std::endl;
                std::cout << "Gravity torque: ";
                for (long j =0; j < 6; ++j) std::cout << grav_torques[j] << ", ";
                std::cout << std::endl;

                std::cout << "Commanded torque: ";
                for (long j =0; j < 6; ++j) std::cout << joint_efforts_command_[j] << ", ";
                std::cout << std::endl;

                std::cout << "Actual joint torque: ";
                for (long j =0; j < 6; ++j) std::cout << Drfl.read_data_rt()->actual_joint_torque[j] << ", ";
                std::cout << std::endl;

                std::cout << "Raw joint torque: ";
                for (long j =0; j < 6; ++j) std::cout << Drfl.read_data_rt()->raw_joint_torque[j] << ", ";
                std::cout << std::endl;

                std::cout << "External joint torque: ";
                for (long j =0; j < 6; ++j) std::cout << Drfl.read_data_rt()->external_joint_torque[j] << ", ";
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
            torque_state+6,
            std::back_inserter(msg_cmd_torque.data),
            [](const float& data) -> double {
                return static_cast<double>(data);
            });
    pub_cmd_torque->publish(msg_cmd_torque);

    std_msgs::msg::Float64MultiArray msg_act_torque;
    std::transform(
            Drfl.read_data_rt()->actual_joint_torque,
            Drfl.read_data_rt()->actual_joint_torque+6,
            std::back_inserter(msg_act_torque.data),
            [](const float& data) -> double {
                return static_cast<double>(data);
            });
    pub_act_joint_torque->publish(msg_act_torque);

    std_msgs::msg::Float64MultiArray msg_raw_torque;
    std::transform(
            Drfl.read_data_rt()->raw_joint_torque,
            Drfl.read_data_rt()->raw_joint_torque+6,
            std::back_inserter(msg_raw_torque.data),
            [](const float& data) -> double {
                return static_cast<double>(data);
            });
    pub_raw_joint_torque->publish(msg_raw_torque);

    std_msgs::msg::Float64MultiArray msg_ext_torque;
    std::transform(
            Drfl.read_data_rt()->external_joint_torque,
            Drfl.read_data_rt()->external_joint_torque+6,
            std::back_inserter(msg_ext_torque.data),
            [](const float& data) -> double {
                return static_cast<double>(data);
            });
    pub_ext_joint_torque->publish(msg_ext_torque);


    return return_type::OK;
}

DRHWInterface::~DRHWInterface()
{
    if(!Drfl.stop_rt_control()){
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),"Error in stop_rt_control()"); 
}
    if(!Drfl.disconnect_rt_control()){
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),"Error in disconnect_rt_control()"); 
    }
    Drfl.close_connection();

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------"); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    CONNECTION IS CLOSED");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------"); 
}

}  


#ifdef USE_FULL_LIB
const char* GetRobotStateString(int nState)
{
    switch(nState)
    {
    case STATE_INITIALIZING:    return "(0) INITIALIZING";
    case STATE_STANDBY:         return "(1) STANDBY";
    case STATE_MOVING:          return "(2) MOVING";
    case STATE_SAFE_OFF:        return "(3) SAFE_OFF";
    case STATE_TEACHING:        return "(4) TEACHING";
    case STATE_SAFE_STOP:       return "(5) SAFE_STOP";
    case STATE_EMERGENCY_STOP:  return "(6) EMERGENCY_STOP";
    case STATE_HOMMING:         return "(7) HOMMING";
    case STATE_RECOVERY:        return "(8) RECOVERY";
    case STATE_SAFE_STOP2:      return "(9) SAFE_STOP2";
    case STATE_SAFE_OFF2:       return "(10) SAFE_OFF2";
    case STATE_RESERVED1:       return "(11) RESERVED1";
    case STATE_RESERVED2:       return "(12) RESERVED2";
    case STATE_RESERVED3:       return "(13) RESERVED3";
    case STATE_RESERVED4:       return "(14) RESERVED4";
    case STATE_NOT_READY:       return "(15) NOT_READY";

    default:                  return "UNKNOWN";
    }
    return "UNKNOWN";
}

int IsInposition(double dCurPosDeg[], double dCmdPosDeg[])
{
    int cnt=0;
    double dError[NUM_JOINT] ={0.0, };

    for(int i=0;i<NUM_JOINT;i++)
    {
        dError[i] = dCurPosDeg[i] - dCmdPosDeg[i];
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    <inpos> %f = %f -%f",dError[i], dCurPosDeg[i], dCmdPosDeg[i]);
        if(fabs(dError[i]) < STABLE_BAND_JNT)
            cnt++;
    }
    if(NUM_JOINT == cnt)
        return true;
    else 
        return false;
}

//----- register the call-back functions ----------------------------------------
void DSRInterface::OnTpInitializingCompletedCB()
{
    // request control authority after TP initialized
    cout << "[callback OnTpInitializingCompletedCB] tp initializing completed" << endl;
    g_bTpInitailizingComplted = TRUE;
    //Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST);
    Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

    g_stDrState.bTpInitialized = TRUE;
}


void DSRInterface::OnHommingCompletedCB()
{
    g_bHommingCompleted = TRUE;
    // Only work within 50msec
    cout << "[callback OnHommingCompletedCB] homming completed" << endl;

    g_stDrState.bHommingCompleted = TRUE;
}

void DSRInterface::OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause)
{
    cout << "[callback OnProgramStoppedCB] Program Stop: " << (int)iStopCause << endl;
    g_stDrState.bDrlStopped = TRUE;
}
// M2.4 or lower
void DSRInterface::OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO)
{
    for (int i = 0; i < NUM_DIGITAL; i++){
        if(pCtrlIO){  
            g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
            g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
        }
    }
}
// M2.5 or higher
void DSRInterface::OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO) 
{
    //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DSRInterface::OnMonitoringCtrlIOExCB");

    for (int i = 0; i < NUM_DIGITAL; i++){
        if(pCtrlIO){  
            g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
            g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
        }
    }

    //----- In M2.5 version or higher The following variables were added -----
    for (int i = 0; i < 3; i++)
        g_stDrState.bActualSW[i] = pCtrlIO->_tInput._iActualSW[i];

    for (int i = 0; i < 2; i++){
        g_stDrState.bActualSI[i] = pCtrlIO->_tInput._iActualSI[i];
        g_stDrState.fActualAI[i] = pCtrlIO->_tInput._fActualAI[i];
        g_stDrState.iActualAT[i] = pCtrlIO->_tInput._iActualAT[i];
        g_stDrState.fTargetAO[i] = pCtrlIO->_tOutput._fTargetAO[i];
        g_stDrState.iTargetAT[i] = pCtrlIO->_tOutput._iTargetAT[i];
        g_stDrState.bActualES[i] = pCtrlIO->_tEncoder._iActualES[i];
        g_stDrState.iActualED[i] = pCtrlIO->_tEncoder._iActualED[i];
        g_stDrState.bActualER[i] = pCtrlIO->_tEncoder._iActualER[i];
    }  
    //-------------------------------------------------------------------------
}

// M2.4 or lower
void DSRInterface::OnMonitoringDataCB(const LPMONITORING_DATA pData)
{
    // This function is called every 100 msec
    // Only work within 50msec
    //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DSRInterface::OnMonitoringDataCB");

    g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
    g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

    for (int i = 0; i < NUM_JOINT; i++){
        if(pData){  
            // joint         
            g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
            g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
            g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
            g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
            g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
            g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
            // task
            g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
            g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
            g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
            g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
            // Torque
            g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
            g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
            g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
            g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

            g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
            g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
            g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
        }
    }
    g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
    g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

    for (int i = 5; i < NUM_BUTTON; i++){
        if(pData){
            g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(pData){
                g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
            }
        }
    }

    for (int i = 0; i < NUM_FLANGE_IO; i++){
        if(pData){
            g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
            g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
        }
    }
}

// M2.5 or higher    
void DSRInterface::OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
{
    // This function is called every 100 msec
    // Only work within 50msec
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DSRInterface::OnMonitoringDataExCB");

    g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
    g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

    for (int i = 0; i < NUM_JOINT; i++){
        if(pData){  
            // joint         
            g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
            g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
            g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
            g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
            g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
            g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
            // task
            g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
            g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
            g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
            g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
            // Torque
            g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
            g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
            g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
            g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

            g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
            g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
            g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
        }
    }
    g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
    g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

    for (int i = 5; i < NUM_BUTTON; i++){
        if(pData){
            g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(pData){
                g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
            }
        }
    }

    for (int i = 0; i < NUM_FLANGE_IO; i++){
        if(pData){
            g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
            g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
        }
    }

    //----- In M2.5 version or higher The following variables were added -----
    for (int i = 0; i < NUM_JOINT; i++){
        g_stDrState.fActualW2B[i] = pData->_tCtrl._tWorld._fActualW2B[i];
        g_stDrState.fCurrentVelW[i] = pData->_tCtrl._tWorld._fActualVel[i];
        g_stDrState.fWorldETT[i] = pData->_tCtrl._tWorld._fActualETT[i];
        g_stDrState.fTargetPosW[i] = pData->_tCtrl._tWorld._fTargetPos[i];
        g_stDrState.fTargetVelW[i] = pData->_tCtrl._tWorld._fTargetVel[i];
        g_stDrState.fCurrentVelU[i] = pData->_tCtrl._tWorld._fActualVel[i];
        g_stDrState.fUserETT[i] = pData->_tCtrl._tUser._fActualETT[i];
        g_stDrState.fTargetPosU[i] = pData->_tCtrl._tUser._fTargetPos[i];
        g_stDrState.fTargetVelU[i] = pData->_tCtrl._tUser._fTargetVel[i];
    }    

    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 6; j++){
            g_stDrState.fCurrentPosW[i][j] = pData->_tCtrl._tWorld._fActualPos[i][j];
            g_stDrState.fCurrentPosU[i][j] = pData->_tCtrl._tUser._fActualPos[i][j];
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            g_stDrState.fRotationMatrixWorld[j][i] = pData->_tCtrl._tWorld._fRotationMatrix[j][i];
            g_stDrState.fRotationMatrixUser[j][i] = pData->_tCtrl._tUser._fRotationMatrix[j][i];
        }
    }

    g_stDrState.iActualUCN = pData->_tCtrl._tUser._iActualUCN;
    g_stDrState.iParent    = pData->_tCtrl._tUser._iParent;
    //-------------------------------------------------------------------------
}

void DSRInterface::OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus)
{
    g_stDrState.nRegCount = pModbus->_iRegCount;
    for (int i = 0; i < pModbus->_iRegCount; i++){
        cout << "[callback OnMonitoringModbusCB] " << pModbus->_tRegister[i]._szSymbol <<": " << pModbus->_tRegister[i]._iValue<< endl;
        g_stDrState.strModbusSymbol[i] = pModbus->_tRegister[i]._szSymbol;
        g_stDrState.nModbusValue[i]    = pModbus->_tRegister[i]._iValue;
    }
}

void DSRInterface::OnMonitoringStateCB(const ROBOT_STATE eState)
{
    //This function is called when the state changes.
    //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DSRInterface::OnMonitoringStateCB");    
    // Only work within 50msec
    
    switch((unsigned char)eState)
    {
#if 0 // TP initializing logic, Don't use in API level. (If you want to operate without TP, use this logic)       
    case eSTATE_NOT_READY:
    if (g_bHasControlAuthority) Drfl.set_robot_control(CONTROL_INIT_CONFIG);
        break;
    case eSTATE_INITIALIZING:
        // add initalizing logic
        if (g_bHasControlAuthority) Drfl.set_robot_control(CONTROL_ENABLE_OPERATION);
        break;
#endif      
    case STATE_EMERGENCY_STOP:
        // popup
        break;
    case STATE_STANDBY:
    case STATE_MOVING:
    case STATE_TEACHING:
        break;
    case STATE_SAFE_STOP:
        if (g_bHasControlAuthority) {
            Drfl.set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE_DEFAULT);
            Drfl.set_robot_control(CONTROL_RESET_SAFET_STOP);
        }
        break;
    case STATE_SAFE_OFF:
        if (g_bHasControlAuthority){
            Drfl.set_robot_control(CONTROL_SERVO_ON);
            Drfl.set_robot_mode(ROBOT_MODE_MANUAL);   //Idle Servo Off 후 servo on 하는 상황 발생 시 set_robot_mode 명령을 전송해 manual 로 전환. add 2020/04/28
        } 
        break;
    case STATE_SAFE_STOP2:
        if (g_bHasControlAuthority) Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_STOP);
        break;
    case STATE_SAFE_OFF2:
        if (g_bHasControlAuthority) {
            Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_OFF);
        }
        break;
    case STATE_RECOVERY:
        Drfl.set_robot_control(CONTROL_RESET_RECOVERY);
        break;
    default:
        break;
    }

    cout << "[callback OnMonitoringStateCB] current state: " << GetRobotStateString((int)eState) << endl;
    g_stDrState.nRobotState = (int)eState;
    strncpy(g_stDrState.strRobotState, GetRobotStateString((int)eState), MAX_SYMBOL_SIZE); 
}

void DSRInterface::OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl)
{
    // Only work within 50msec

    cout << "[callback OnMonitoringAccessControlCB] eAccCtrl: " << eAccCtrl << endl;
    switch(eAccCtrl)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
        //Drfl.TransitControlAuth(MANaGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------");   
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Access control granted ");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------");   
        g_bHasControlAuthority = TRUE;
        OnMonitoringStateCB(Drfl.GetRobotState());
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------");   
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Access control deny ");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"-----------------------------------------------");   
        break;
    case MONITORING_ACCESS_CONTROL_LOSS:
        g_bHasControlAuthority = FALSE;
        if (g_bTpInitailizingComplted) {
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST);
        }
        break;
    default:
        break;
    }
    g_stDrState.nAccessControl = (int)eAccCtrl;
}

void DSRInterface::OnLogAlarm(LPLOG_ALARM pLogAlarm)
{
    //This function is called when an error occurs.
    auto PubRobotError = s_node_->create_publisher<dsr_msgs2::msg::RobotError>("error", 100);
    dsr_msgs2::msg::RobotError msg;

    switch(pLogAlarm->_iLevel)
    {
    case LOG_LEVEL_SYSINFO:
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
        break;
    case LOG_LEVEL_SYSWARN:
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
        break;
    case LOG_LEVEL_SYSERROR:
    default:
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
        break;
    }

    g_stDrError.nLevel=(unsigned int)pLogAlarm->_iLevel;
    g_stDrError.nGroup=(unsigned int)pLogAlarm->_iGroup;
    g_stDrError.nCode=pLogAlarm->_iIndex;
    strncpy(g_stDrError.strMsg1, pLogAlarm->_szParam[0], MAX_STRING_SIZE);
    strncpy(g_stDrError.strMsg2, pLogAlarm->_szParam[1], MAX_STRING_SIZE);
    strncpy(g_stDrError.strMsg3, pLogAlarm->_szParam[2], MAX_STRING_SIZE);

    msg.level=g_stDrError.nLevel;
    msg.group=g_stDrError.nGroup;
    msg.code=g_stDrError.nCode;
    msg.msg1=g_stDrError.strMsg1;
    msg.msg2=g_stDrError.strMsg2;
    msg.msg3=g_stDrError.strMsg3;

    PubRobotError->publish(msg);
}
#endif



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dsr_hardware2::DRHWInterface, hardware_interface::SystemInterface)
      

