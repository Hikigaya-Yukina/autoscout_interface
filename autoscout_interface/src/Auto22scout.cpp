#include "Auto22scout.hpp"
#include <math.h>

//参数定义
float WHEEL_TREAD = 0.116;//scout的
float WHEEL_BASE = 0.498;

//这个地方是构造函数 设置了Node
autoscout_interface::autoscout_interface()
: Node("autoscout_interface")
{
  //占位符。不是一个真正的值，而是一个占位符，当你调用函数时，它会被替换成实际的值。要传递一个函数模板参数但不确定这个参数的具体类型。
  using std::placeholders::_1;
  
  // from autoware 订阅autoware的消息
  control_cmd_sub_ = create_subscription<autoware_control_msgs::msg::Control>("/control/command/control_cmd", 1, std::bind(&autoscout_interface::callback_control_cmd, this, _1));//订阅ctrl_cmd 来自autoware的指示
  
  // from vehicle 发布消息
  gear_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS{1});//发布档位消息。Qos{1}就是reliable的意思。
  battery_status_pub_ = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", rclcpp::QoS{1});//发布电池信息
  control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS{1});//发布控制模式信息

  velocity_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_Status", rclcpp::QoS{10});//发布速度信息(线速度角速度)
  steerang_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_Status", rclcpp::QoS{10});//发布角度信息

  // from vehicle to ros2 订阅底盘发给ros2的消息
  scout_status_sub_ = create_subscription<scout_msgs::msg::ScoutStatus>("/scout/status", 1, std::bind(&autoscout_interface::callback_scout_status, this, _1));//订阅底盘发来的信息
  
  // from autoware to ros2 发布autoware给ros2的消息
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd/vel", rclcpp::QoS{10});
}

void autoscout_interface::callback_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)//阿克曼的消息传递给一个的同类型常指针
{
  control_cmd_ptr_ = msg;
}


//发布scout的状态 
void autoscout_interface::callback_scout_status(const scout_msgs::msg::ScoutStatus::ConstSharedPtr msg)
{
  linear_velocity = msg.linear_velocity;
  angular_velocity = msg.angular_velocity;
  vehicle_state = msg.vehicle_state;
  control_mode = msg.control_mode;
  battery_voltage = msg.battery_voltage;
  error_code = msg.error_code;
  // actuator_state = msg.actuator_state;
  // light_ctrl_enabled = msg.light_ctrl_enabled;
  // front_light_state = msg.front_light_state;
  // rear_light_state = msg.rear_light_state;
}

// 将控制命令转化为小车命令 要发布到小车
void control_command_to_vehicle(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
    //要把转向角解成角速度
    geometry_msgs::msg::Twist twister;
    twister.linear.x = msg->longitudinal.velocity;
    
    double linear_spd = 0;
    linear_spd = twister.linear.x;
    double steer_angle = msg->lateral.steering_tire_angle;
    // steer_angle_velocity = msg->lateral.steering_tire_rotation_rate
    double tan_angle = tan(steer_angle);
    
    twister.angular.z = tan_angle * linear_spd / WHEEL_BASE;
    twist_pub_->publish(twister);
    
}

void autoscout_interface::to_vehicle()
{
  // 给小车的命令
  control_command_to_vehicle(control_cmd_ptr_);
}

void convert_gear_status_to_autoware_msg(autoware_vehicle_msgs::msg::GearReport msg)//SCOUT没有档位
{
  msg.report = 2;//随便设置的。scout没有档位
}

void convert_battery_status_to_autoware_msg(tier4_vehicle_msgs::msg::BatteryStatus msg)//发布电池信息
{
  msg.energy_level = battery_voltage;
}

void convert_control_mode_to_autoware_msg(autoware_vehicle_msgs::msg::ControlModeReport msg)
{
  int switched_mode = 0;
  switch (control_mode)
  {
  case 0:
    switched_mode = 0;//待机
    break;
  
  case 1:
    switched_mdoe = 1;//CAN指令控制————AUTONOMOUS

  case 2:
    switched_mode = 2;//串口指令控制————AUTONOMOUS ONLY STEERING
    
  case 3:
    switched_mode = 4;//遥控————MANUAL
  default:
    switched_mode = 0;
    break;
  }
  msg.mode = switched_mode;

}

void convert_steering_to_autoware_msg(autoware_vehicle_msgs::msg::VelocityReport msg)
{
  msg.longitudinal_velocity = linear_velocity;
}

void convert_velocity_to_autoware_msg(autoware_vehicle_msgs::msg::VelocityReport msg)
{
  msg.lateral_velocity = angular_velocity;
  msg.header.frame_id = "base_link";//不明白为什么这里要设定为base_link
}

//发送一个转换后的转向角报告出去
void convert_steerang_to_autoware_msg(autoware_vehicle_msgs::msg::SteeringReport msg)
{
  double R_v =linear_velocity / angular_velocity;
  double tan_strang = WHEEL_BASE / R_v;
  msg.steering_tire_angle = atan(a/b);

}

void autoscout_interface::to_autoware()
{

  // 给autoware的命令
  autoware_vehicle_msgs::msg::GearReport gear_report_msg;
  convert_gear_status_to_autoware_msg(gear_report_msg);
  gear_status_pub_->publish(gear_report_msg);
  
  //发布电池状况
  tier4_vehicle_msgs::msg::BatteryStatus battery_status_msg;
  convert_battery_status_to_autoware_msg(battery_status_msg);
  battery_status_pub_->publish(battery_status_msg);
  
  //发布控制模式
  autoware_vehicle_msgs::msg::ControlModeReport control_mode_report_msg;
  convert_control_mode_to_autoware_msg(control_mode_report_msg);
  control_mode_pub_->publish(control_mode_report_msg);
  
  //发布转向报告. 这里报告的是车轮的转向角，实际上SCOUT只会发布角速度
  autoware_vehicle_msgs::msg::VelocityReport velocity_report_msg;
  convert_steering_to_autoware_msg(velocity_report_msg);
  
  //发布速度报告 将AKM转换为差速底盘需要的
  convert_velocity_to_autoware_msg(velocity_report_msg);
  velocity_pub_->publish(velocity_report_msg);
  
  //发布转向角报告。
  autoware_vehicle_msgs::msg::SteeringReport steerang_report_msg;
  convert_steerang_to_autoware_msg(steerang_report_msg);
  steerang_pub_->publish(steerang_report_msg);
}


