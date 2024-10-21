#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
// #include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_control_msgs/msg/control.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <scout_msgs/msg/ScoutStatus.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

class autoscout_interface : public rclcpp::Node
{
public:
    
private:
    
    // from autoware 订阅 autoware的控制指令
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;// (阿克曼)的控制指令。
    
    // from vehicle 发布的车辆信息
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;//车辆情况：档位。
    rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_status_pub_;//电池状况
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;//车辆控制模式
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;//速度报告
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steerang_pub_;//“转向角”报告
    
    // from vehicle to ros 订阅来自底盘的消息
    rclcpp::Subscription<scout_msgs::msg::ScoutStatus>::SharedPtr scout_status_sub_;//小车底盘状况
    
    // from autoware to ros 发布来自autoware的消息
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    
    // autoware command messages定义一些存放的信息
    autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_ptr_;//阿克曼的控制指令
    
    // callbacks(和上面的信息是联合的)
    void callback_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);//阿克曼控制指令的回调函数
    void callback_scout_status(const scout_msgs::msg::ScoutStatus::ConstSharedPtr msg);//订阅底盘的信息
    
    //给移动设备发送指令
    void to_vehicle();
    
    //更新移动设备状态
    void from_vehicle();
    
    //从 移动设备 订阅 指令 发给 autoware
    // 速度反馈
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    
    //状态反馈
    int vehicle_state = 0;
    int control_mode = 0;
    int error_code = 0;
    double battery_voltage = 0.0;
    // int actuator_state = 0;

    //灯光反馈
    // bool light_ctrl_enabled = False;
    // int front_light_state = 0;
    // int rear_light_state = 0;
    
}
