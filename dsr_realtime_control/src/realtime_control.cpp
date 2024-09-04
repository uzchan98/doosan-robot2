#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dsr_realtime_control/realtime_control.hpp"

#include <pthread.h>
#include <string>

RT_STATE g_stRTState;
std::mutex mtx;
std::atomic_bool first_get(false);

using namespace DRAFramework;
CDRFLEx Drfl;

ReadDataRtNode::ReadDataRtNode() : Node("ReadDataRt")
{
    client_ = this->create_client<dsr_msgs2::srv::ReadDataRt>("/dsr01/realtime/read_data_rt");
    client_thread_ = std::thread(std::bind(&ReadDataRtNode::ReadDataRtClient, this));

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

ServojRtNode::ServojRtNode() : Node("ServojRt")
{
    publisher_  = this->create_publisher<dsr_msgs2::msg::ServojRtStream>("/dsr01/servoj_rt_stream",10);
    timer_      = this->create_wall_timer(std::chrono::microseconds(1000),std::bind(&ServojRtNode::ServojRtStreamPublisher,this));

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

ServolRtNode::ServolRtNode() : Node("ServolRt")
{
    publisher_  = this->create_publisher<dsr_msgs2::msg::ServolRtStream>("/dsr01/servol_rt_stream",10);
    timer_      = this->create_wall_timer(std::chrono::microseconds(1000),std::bind(&ServolRtNode::ServolRtStreamPublisher,this));

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

ReadDataRtNode::~ReadDataRtNode()
{
    if(client_thread_.joinable())
    {
        client_thread_.join();
        RCLCPP_INFO(this->get_logger(), "client_thread_.joined");
    }
    RCLCPP_INFO(this->get_logger(), "ReadDataRt client shut down");
}
TorqueRtNode::~TorqueRtNode()
{
    RCLCPP_INFO(this->get_logger(), "TorqueRt publisher shut down");
}
ServojRtNode::~ServojRtNode()
{
    RCLCPP_INFO(this->get_logger(), "ServojRt publisher shut down");
}
ServolRtNode::~ServolRtNode()
{
    RCLCPP_INFO(this->get_logger(), "ServolRt publisher shut down");
}

void ReadDataRtNode::ReadDataRtClient()
{
    rclcpp::Rate rate(3000);
    while(rclcpp::ok())
    {
        rate.sleep();
        // std::this_thread::sleep_for(std::chrono::microseconds(50));
        if (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            continue;
        }
        auto request = std::make_shared<dsr_msgs2::srv::ReadDataRt::Request>();
        auto future = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "ReadDataRt Service Request");
        try
        {
            auto response = future.get();
            if(!first_get)
            {
                first_get=true;
            }
            RCLCPP_INFO(this->get_logger(), "ReadDataRt Service Response");
            for(int i=0; i<6; i++)
            {
                g_stRTState.actual_joint_position[i] = response->data.actual_joint_position[i];
                g_stRTState.actual_joint_velocity[i] = response->data.actual_joint_velocity[i];
                g_stRTState.gravity_torque[i] = response->data.gravity_torque[i];
                g_stRTState.target_joint_position[i] = response->data.target_joint_position[i];
                // q[i]    =response->data.actual_joint_position[i];
                // q_dot[i]=response->data.actual_joint_velocity[i];
                // trq_g[i]=response->data.gravity_torque[i];
            }
            // RCLCPP_INFO(this->get_logger(), "g_stRTState updated");
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
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
    auto message = dsr_msgs2::msg::TorqueRtStream(); 
    message.tor={trq_d[0],trq_d[1],trq_d[2],trq_d[3],trq_d[4],trq_d[5]};
    message.time=0.0;

    if(first_get)
    {
        this->publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "TorqueRtStream Published");
    } 
}

void ServojRtNode::ServojRtStreamPublisher()
{
}

void ServolRtNode::ServolRtStreamPublisher()
{
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
    set_thread_scheduling(pthread_self(), options.policy, options.priority);
    rclcpp::init(argc,argv);
    auto node1= std::make_shared<ReadDataRtNode>();
    auto node2= std::make_shared<TorqueRtNode>();
    // auto node3= std::make_shared<ServojRtNode>();
    // auto node4= std::make_shared<ServolRtNode>();

    rclcpp::executors::SingleThreadedExecutor executor1;
    rclcpp::executors::SingleThreadedExecutor executor2;
    // rclcpp::executors::SingleThreadedExecutor executor3;
    // rclcpp::executors::SingleThreadedExecutor executor4;
    executor1.add_node(node1);
    executor2.add_node(node2);
    // executor3.add_node(node3);
    // executor4.add_node(node4);
    auto executor1_thread = std::thread([&](){executor1.spin();});
    auto executor2_thread = std::thread([&](){executor2.spin();});

    // set_thread_scheduling(executor1_thread.native_handle(), options.policy, options.priority);
    // set_thread_scheduling(executor2_thread.native_handle(), options.policy, options.priority);
    executor1_thread.join();
    executor2_thread.join();
    rclcpp::shutdown();
    return 0;
    // -------------------- realtime executor scheduling -------------------- //
}



            