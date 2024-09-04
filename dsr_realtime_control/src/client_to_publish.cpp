#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dsr_realtime_control/client_to_publish.hpp"

#include <pthread.h>
#include <string>

RT_STATE g_stRTState;
std::mutex mtx;
std::atomic_bool first_get(false);

using namespace DRAFramework;
CDRFLEx Drfl;

ClientToPublishNode::ClientToPublishNode() : Node("ClientToPublish")
{
    client_ = this->create_client<dsr_msgs2::srv::ReadDataRt>("/dsr01/realtime/read_data_rt");
    client_thread_ = std::thread(std::bind(&ClientToPublishNode::ReadDataRtClient, this));
    publisher_ = this->create_publisher<dsr_msgs2::msg::TorqueRtStream>("/dsr01/torque_rt_stream",10);
    publisher_thread_ = std::thread(std::bind(&ClientToPublishNode::TorqueRtStreamPublisher, this));
    // publisher2_ = this->create_publisher<dsr_msgs2::msg::ServolRtStream>("/dsr01/servol_rt_stream",10);
    // publisher2_thread_ = std::thread(std::bind(&ClientToPublishNode::ServolRtStreamPublisher, this));
    // publisher3_ = this->create_publisher<dsr_msgs2::msg::ServojRtStream>("/dsr01/servoj_rt_stream",10);
    // publisher3_thread_ = std::thread(std::bind(&ClientToPublishNode::ServojRtStreamPublisher, this));

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
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
}

ClientToPublishNode::~ClientToPublishNode()
{
    if(client_thread_.joinable())
    {
        client_thread_.join();
        RCLCPP_INFO(this->get_logger(), "client_thread_.joined");
    }
    if(publisher_thread_.joinable())
    {
        publisher_thread_.join();
        RCLCPP_INFO(this->get_logger(), "publisher_thread_.joined");
    }
}

void ClientToPublishNode::ReadDataRtClient()
{
    rclcpp::Rate rate(1000);
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
        // RCLCPP_INFO(this->get_logger(), "Service Request");
        try
        {
            auto response = future.get();
            if(!first_get)
            {
                first_get=true;
            }
            // RCLCPP_INFO(this->get_logger(), "Service Response");
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
            RCLCPP_INFO(this->get_logger(), "g_stRTState updated");
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
}

void ClientToPublishNode::TorqueRtStreamPublisher()
{
    rclcpp::Rate rate(1000);
    Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE);
    while(rclcpp::ok())
    {
        rate.sleep();
        for(int i=0; i<6; i++)
        {
            // mtx.lock();
            q[i]        =   g_stRTState.actual_joint_velocity[i];
            q_dot[i]    =   g_stRTState.actual_joint_velocity[i];
            trq_g[i]    =   g_stRTState.gravity_torque[i];
            // mtx.unlock();
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
}

void ClientToPublishNode::ServolRtStreamPublisher()
{
    rclcpp::Rate rate(1000);
    while(rclcpp::ok())
    {
        rate.sleep();
        auto message = dsr_msgs2::msg::ServolRtStream(); 
        message.pos={fTargetPos[0],fTargetPos[1],fTargetPos[2],fTargetPos[3],fTargetPos[4],fTargetPos[5]};
        message.vel={fTargetVel[0],fTargetVel[1],fTargetVel[2],fTargetVel[3],fTargetVel[4],fTargetVel[5]};
        message.acc={fTargetAcc[0],fTargetAcc[1],fTargetAcc[2],fTargetAcc[3],fTargetAcc[4],fTargetAcc[5]};
        message.time=fTargetTime;

        if(first_get)
        {
            this->publisher2_->publish(message);
            RCLCPP_INFO(this->get_logger(), "ServolRtStream Published");
        } 
    }
}

void ClientToPublishNode::ServojRtStreamPublisher()
{
    rclcpp::Rate rate(1000);
    const float st=0.001;
    const float ratio=1;
    float count=0;
    float time=0;
    float target_time=4.0;
    float target_pos[6]={0,0,90,0,90,0};

    TraParam tra;
    PlanParam plan1;
    plan1.time  = target_time;
    for(int i=0;i<6;i++)
    {
        plan1.ps[i] = g_stRTState.target_joint_position[i];
        plan1.pf[i] = target_pos[i];
    }
    
    plan1.vs[0]=0;plan1.vs[1]=0;plan1.vs[2]=0;plan1.vs[3]=0;plan1.vs[4]=0;plan1.vs[5]=0;
    plan1.vf[0]=0;plan1.vf[1]=0;plan1.vf[2]=0;plan1.vf[3]=0;plan1.vf[4]=0;plan1.vf[5]=0;
    plan1.as[0]=0;plan1.as[1]=0;plan1.as[2]=0;plan1.as[3]=0;plan1.as[4]=0;plan1.as[5]=0;
    plan1.af[0]=0;plan1.af[1]=0;plan1.af[2]=0;plan1.af[3]=0;plan1.af[4]=0;plan1.af[5]=0;
    TrajectoryPlan(&plan1);
    RCLCPP_INFO(this->get_logger(), "Started servoj motion");
    while(rclcpp::ok())
    {
        rate.sleep();
        time=(++count)*st;
        tra.time=time;

        TrajectoryGenerator(&plan1,&tra);
        auto message = dsr_msgs2::msg::ServojRtStream();
        message.pos = {tra.pos[0],tra.pos[1],tra.pos[2],tra.pos[3],tra.pos[4],tra.pos[5]};
        message.vel = {tra.vel[0],tra.vel[1],tra.vel[2],tra.vel[3],tra.vel[4],tra.vel[5]};
        message.acc = {tra.acc[0],tra.acc[1],tra.acc[2],tra.acc[3],tra.acc[4],tra.acc[5]};
        message.time = st*ratio;
        if(first_get)
        {
            this->publisher3_->publish(message);
            RCLCPP_INFO(this->get_logger(), "ServolRtStream Published");
        } 

        if(time > plan1.time)
        {
            time=0;
            printf("Finish movej with servoj");
            return;
        }
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

    // -------------------- middleware thread scheduling -------------------- //
    set_thread_scheduling(pthread_self(), options.policy, options.priority);
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ClientToPublishNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    // -------------------- middleware thread scheduling -------------------- //
    
    // -------------------- main thread scheduling -------------------- //
    // rclcpp::init(argc,argv);
    // auto node = std::make_shared<ClientToPublishNode>();
    // set_thread_scheduling(pthread_self(), options.policy, options.priority);
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    // return 0;
    // -------------------- main thread scheduling -------------------- //
}



            