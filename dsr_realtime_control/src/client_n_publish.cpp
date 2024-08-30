#include "rclcpp/rclcpp.hpp"
#include "dsr_realtime_control/client_n_publish.hpp"

ClientNPublisher::ClientNPublisher() : Node("ClientNPublisher")
{
    client_ = this->create_client<dsr_msgs2::srv::ReadDataRt>("/dsr01/realtime/read_data_rt");
    thread_ = std::thread(std::bind(&ClientNPublisher::client_n_publish, this));
    publisher_ = this->create_publisher<dsr_msgs2::msg::TorqueRtStream>("/dsr01/torque_rt_stream",10);
}
ClientNPublisher::~ClientNPublisher()
{
    if(thread_.joinable())
    {
        thread_.join();
        RCLCPP_INFO(this->get_logger(), "thread_.joined");
    }
}

void ClientNPublisher::client_n_publish()
{
    rclcpp::Rate rate(1000);
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
        RCLCPP_INFO(this->get_logger(), "Service Request");
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service Response");
            for(int i=0; i<6; i++)
            {
                q[i]        =   response->data.actual_joint_position[i];
                q_dot[i]    =   response->data.actual_joint_velocity[i];
                trq_g[i]    =   response->data.gravity_torque[i];
                trq_d[i]=trq_g[i]+kp[i]*(q_d[i]-q[i])+kd[i]*(q_dot_d[i]-q_dot[i]);
            }
            auto message = dsr_msgs2::msg::TorqueRtStream(); 
            message.tor={trq_d[0],trq_d[1],trq_d[2],trq_d[3],trq_d[4],trq_d[5]};
            message.time=0;
            this->publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Message Published");
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ClientNPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}