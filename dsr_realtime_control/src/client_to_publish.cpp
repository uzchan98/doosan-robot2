#include "rclcpp/rclcpp.hpp"
#include "dsr_realtime_control/client_to_publish.hpp"

RT_STATE g_stRTState;
std::mutex mtx;
std::atomic_bool first_get(false);

ClientToPublishNode::ClientToPublishNode() : Node("ClientToPublish")
{
    client_ = this->create_client<dsr_msgs2::srv::ReadDataRt>("/dsr01/realtime/read_data_rt");
    client_thread_ = std::thread(std::bind(&ClientToPublishNode::ReadDataRtClient, this));
    publisher_ = this->create_publisher<dsr_msgs2::msg::TorqueRtStream>("/dsr01/torque_rt_stream",10);
    publisher_thread_ = std::thread(std::bind(&ClientToPublishNode::TorqueRtStreamPublisher, this));
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
    rclcpp::Rate rate(2000);
    while(rclcpp::ok())
    {
        rate.sleep();
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
                // g_stRTState.actual_joint_position[i] = response->data.actual_joint_position[i];
                // g_stRTState.actual_joint_velocity[i] = response->data.actual_joint_velocity[i];
                // g_stRTState.gravity_torque[i] = response->data.gravity_torque[i];
                q[i]    =response->data.actual_joint_position[i];
                q_dot[i]=response->data.actual_joint_velocity[i];
                trq_g[i]=response->data.gravity_torque[i];
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
    rclcpp::Rate rate(500);
    while(rclcpp::ok())
    {
        rate.sleep();
        // mtx.lock();
        for(int i=0; i<6; i++)
        {
            // q[i]        =   g_stRTState.actual_joint_velocity[i];
            // q_dot[i]    =   g_stRTState.actual_joint_velocity[i];
            // trq_g[i]    =   g_stRTState.gravity_torque[i];
            trq_d[i]    =   trq_g[i]+kp[i]*(q_d[i]-q[i])+kd[i]*(q_dot_d[i]-q_dot[i]);  
        }
        // mtx.unlock();
        auto message = dsr_msgs2::msg::TorqueRtStream(); 
        message.tor={trq_d[0],trq_d[1],trq_d[2],trq_d[3],trq_d[4],trq_d[5]};
        message.time=0.002;
        if(first_get)
        {
            this->publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "TorqueRTStream Published");
        } 
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ClientToPublishNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



            