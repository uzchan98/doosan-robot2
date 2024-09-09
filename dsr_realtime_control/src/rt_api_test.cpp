#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dsr_realtime_control/rt_api_test.hpp"

#include <pthread.h>
#include <string>

RT_STATE g_stRTState;
std::mutex mtx;
std::atomic_bool first_get(false);

using namespace DRAFramework;
CDRFLEx Drfl;

SetOnRtMonitoringDataNode::SetOnRtMonitoringDataNode() : Node("SetOnRtMonitoringData")
{
    Drfl.set_on_rt_monitoring_data(SetOnRtMonitoringDataNode::OnRtMonitoringData);
    Drfl.connect_rt_control("192.168.137.100",12347);
    Drfl.set_rt_control_output("v1.0",0.001,4);
    Drfl.start_rt_control();
}

ReadDataRtNode::ReadDataRtNode() : Node("ReadDataRt")
{
    timer_      = this->create_wall_timer(std::chrono::microseconds(333),std::bind(&ReadDataRtNode::ReadDataRtAPI,this));
    
    // auto timer_callback = [this]() -> void 
    // {
    //     auto context_switches = context_switches_counter.get();
    //     if (context_switches > 0L) 
    //     {
    //       RCLCPP_WARN(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
    //     } 
    //     else 
    //     {
    //       RCLCPP_INFO(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
    //     }
    // };
    // context_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
}

TorqueRtNode::TorqueRtNode() : Node("TorqueRt")
{
    publisher_  = this->create_publisher<dsr_msgs2::msg::TorqueRtStream>("/dsr01/torque_rt_stream",10);
    timer_      = this->create_wall_timer(std::chrono::microseconds(1000),std::bind(&TorqueRtNode::TorqueRtStreamPublisher,this));

    // auto timer_callback = [this]() -> void 
    // {
    //     auto context_switches = context_switches_counter.get();
    //     if (context_switches > 0L) 
    //     {
    //       RCLCPP_WARN(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
    //     } 
    //     else 
    //     {
    //       RCLCPP_INFO(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
    //     }
    // };
    // context_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
}

SetOnRtMonitoringDataNode::~SetOnRtMonitoringDataNode()
{
    Drfl.stop_rt_control();
    Drfl.disconnect_rt_control();
    RCLCPP_INFO(this->get_logger(), "SetOnRtMonitoringData shut down");
}
ReadDataRtNode::~ReadDataRtNode()
{
    RCLCPP_INFO(this->get_logger(), "ReadDataRt caller shut down");
}
TorqueRtNode::~TorqueRtNode()
{
    RCLCPP_INFO(this->get_logger(), "TorqueRt publisher shut down");
}

void SetOnRtMonitoringDataNode::OnRtMonitoringData(const LPRT_OUTPUT_DATA_LIST tData)
{
    for(int i=0; i<6; i++)
    {
        g_stRTState.actual_joint_position[i] = tData->actual_joint_position[i];
        g_stRTState.actual_joint_velocity[i] = tData->actual_joint_velocity[i];
        g_stRTState.gravity_torque[i] = tData->gravity_torque[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("OnMonitoringData"),"time_stamp:%f",tData->time_stamp);
}

void ReadDataRtNode::ReadDataRtAPI()
{
    LPRT_OUTPUT_DATA_LIST temp = Drfl.read_data_rt();
    // RCLCPP_INFO(this->get_logger(), "call ReadDataRt API");
    if(!first_get)
    {
        first_get=true;
    }
    g_stRTState.time_stamp=temp->time_stamp;
    for(int i=0; i<6; i++)
    {
        g_stRTState.actual_joint_position[i] = temp->actual_joint_position[i];
        g_stRTState.actual_joint_velocity[i] = temp->actual_joint_velocity[i];
        g_stRTState.gravity_torque[i] = temp->gravity_torque[i];
        g_stRTState.target_joint_position[i] = temp->target_joint_position[i];
        // q[i]    =temp->actual_joint_position[i];
        // q_dot[i]=temp->actual_joint_velocity[i];
        // trq_g[i]=temp->gravity_torque[i];
    }
    RCLCPP_INFO(this->get_logger(), "g_stRTState updated about %f",temp->time_stamp);
}

void TorqueRtNode::TorqueRtStreamPublisher()
{
    for(int i=0; i<6; i++)
    {
        mtx.lock();
        q[i]        =   g_stRTState.actual_joint_velocity[i];
        q_dot[i]    =   g_stRTState.actual_joint_velocity[i];
        trq_g[i]    =   g_stRTState.gravity_torque[i];
        mtx.unlock(); 
    }
    for(int i=0; i<6; i++)
    {
        trq_d[i]    =   trq_g[i]+kp[i]*(q_d[i]-q[i])+kd[i]*(q_dot_d[i]-q_dot[i]);  
    }
    if(first_get)
    {
        Drfl.torque_rt(trq_d,0);
        RCLCPP_INFO(this->get_logger(), "TorqueRtStream Published");

    } 
}

int main(int argc, char **argv)
{
    // --------------------cpu affinity set-------------------- //

    // Pin the main thread to CPU 3
    // int cpu_id = std::thread::hardware_concurrency()-1;
    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // CPU_SET(cpu_id, &cpuset);

    // Pin the main thread to CPUs 2 and 3
    uint32_t cpu_bit_mask = 0b1100;
    cpu_set_t cpuset;
    uint32_t cpu_cnt = 0U;
    CPU_ZERO(&cpuset);
    while (cpu_bit_mask > 0U) 
    {
        if ((cpu_bit_mask & 0x1U) > 0) 
        {
        CPU_SET(cpu_cnt, &cpuset);
        }
        cpu_bit_mask = (cpu_bit_mask >> 1U);
        cpu_cnt++;
    }
    auto ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret>0)
    {
        std::cerr << "Couldn't set CPU affinity. Error code" << strerror(errno) << std::endl;
        return EXIT_FAILURE;
    }
    ret = pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret<0)
    {
        std::cerr << "Coudln't get CPU affinity. Error code" << strerror(errno) << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Pinned CPUs:"<< std::endl;
    for (int i=0; i < CPU_SETSIZE; i++)
    {
        if(CPU_ISSET(i,&cpuset))
        {
            std::cout << "  CPU" << std::to_string(i) << std::endl;
        }
    }
    // -------------------- cpu affinity set    -------------------- //
    // -------------------- process scheduling  -------------------- //
    auto options_reader = SchedOptionsReader();
    if (!options_reader.read_options(argc, argv)) 
    {
        options_reader.print_usage();
        return 0;
    }
    auto options = options_reader.get_options();
    // -------------------- process scheduling  -------------------- //
    // $ ros2 run dsr_realtime_control realtime_control --sched SCHED_FIFO --priority 80
    // $ ros2 run dsr_realtime_control realtime_control --sched SCHED_RR --priority 80
    // $ ps -C realtime_control -L -o tid,comm,rtprio,cls,psr
    // -------------------- middleware thread scheduling -------------------- //
    // set_thread_scheduling(pthread_self(), options.policy, options.priority);
    // rclcpp::init(argc,argv);
    // auto node1= std::make_shared<ReadDataRtNode>();
    // auto node2= std::make_shared<TorqueRtNode>();
    // // auto node3= std::make_shared<ServojRtNode>();
    // // auto node4= std::make_shared<ServolRtNode>();
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node1);
    // executor.add_node(node2);
    // // executor.add_node(node3);
    // // executor.add_node(node4);
    // executor.spin();
    // rclcpp::shutdown();
    // return 0;
    // -------------------- middleware thread scheduling -------------------- //
    
    // -------------------- main thread scheduling -------------------- //
    // rclcpp::init(argc,argv);
    // auto node1= std::make_shared<ReadDataRtNode>();
    // auto node2= std::make_shared<TorqueRtNode>();
    // auto node3= std::make_shared<ServojRtNode>();
    // auto node4= std::make_shared<ServolRtNode>();
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node1);
    // executor.add_node(node2);
    // // executor.add_node(node3);
    // // executor.add_node(node4);
    // set_thread_scheduling(pthread_self(), options.policy, options.priority);
    // executor.spin();
    // rclcpp::shutdown();
    // return 0;
    // -------------------- main thread scheduling -------------------- //
    // -------------------- realtime executor scheduling -------------------- //
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SetOnRtMonitoringDataNode>();
    // auto node1= std::make_shared<ReadDataRtNode>();
    auto node2= std::make_shared<TorqueRtNode>();
    // auto node3= std::make_shared<ServojRtNode>();
    // auto node4= std::make_shared<ServolRtNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::executors::SingleThreadedExecutor executor1;
    rclcpp::executors::SingleThreadedExecutor executor2;
    // rclcpp::executors::SingleThreadedExecutor executor3;
    // rclcpp::executors::SingleThreadedExecutor executor4;
    executor.add_node(node);
    // executor1.add_node(node1);
    executor2.add_node(node2);
    // executor3.add_node(node3);
    // executor4.add_node(node4);
    auto executor_thread  = std::thread([&](){executor.spin();});
    // auto executor1_thread = std::thread([&](){executor1.spin();});
    auto executor2_thread = std::thread([&](){executor2.spin();});

    set_thread_scheduling(executor_thread.native_handle(), options.policy, options.priority);
    // set_thread_scheduling(executor1_thread.native_handle(), options.policy, options.priority);
    set_thread_scheduling(executor2_thread.native_handle(), options.policy, options.priority);
    
    executor_thread.join();
    // executor1_thread.join();
    executor2_thread.join();
    rclcpp::shutdown();
    return 0;
    // -------------------- realtime executor scheduling -------------------- //
}
            