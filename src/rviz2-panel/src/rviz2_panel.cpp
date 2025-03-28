

#include "rviz2-panel/rviz2_panel.hpp"

namespace custom_panel
{
  SurgeonUI::SurgeonUI(QWidget* parent)
    : Panel{ parent }
    , ui_(std::make_unique<Ui::gui>())
    , node_{ nullptr }
  {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    // Init rclcpp node
    auto options = rclcpp::NodeOptions().arguments(
      { "--ros-args", "--remap", "__node:=rviz_push_button_node", "--" });
    node_ = std::make_shared<rclcpp::Node>("_", options);


    button0_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/hard_reset_host", 1);
    button1_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/trigger_host_ui", 1);
    button2_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/trigger_reg", 1);
    button3_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/trigger_reg", 1);

    button0_2_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/lbr/plan_flag", 1);
    button1_2_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/lbr/plan_flag", 1);
    button2_2_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/lbr/plan_flag", 1);
    button3_2_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/lbr/plan_flag", 1);

    button0_3_pub_ = node_->create_publisher<std_msgs::msg::String>("/drill_commands", 1);
    button1_3_pub_ = node_->create_publisher<std_msgs::msg::String>("/drill_commands", 1);

    
    pose_index_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/lbr/drill_pose_index", 1);
  }

  SurgeonUI::~SurgeonUI()
  {
  }
  void SurgeonUI::load(const rviz_common::Config& config)
  {
    Panel::load(config);
  }

  void SurgeonUI::save(rviz_common::Config config) const
  {
    Panel::save(config);
    rviz_common::Config push_button_config = config.mapMakeChild({ "PushButton" });
  }
  void SurgeonUI::on_pushButton0_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Reset Parasight");
    button0_pub_->publish(parasight_flag_);
  }

  void SurgeonUI::on_pushButton1_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Start Parasight");
    button1_pub_->publish(parasight_flag_);
  }

  void SurgeonUI::on_pushButton2_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Re-register");
    reg_flag_.data = 0;
    button2_pub_->publish(reg_flag_);
  }

  void SurgeonUI::on_pushButton3_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Annotate");
    reg_flag_.data = 1;
    button3_pub_->publish(reg_flag_);
  }

  void SurgeonUI::on_pushButton0_2_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Track Flag");
    plan_flag_.data = 1;
    button0_2_pub_->publish(plan_flag_);
  }

  void SurgeonUI::on_pushButton1_2_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Drill Flag");
    plan_flag_.data = 2;
    button1_2_pub_->publish(plan_flag_);
    button0_pub_->publish(parasight_flag_);
  }

  void SurgeonUI::on_pushButton2_2_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Reset Flag");
    plan_flag_.data = 0;
    button2_2_pub_->publish(plan_flag_);
  }

  void SurgeonUI::on_pushButton3_2_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Stop Flag");
    plan_flag_.data = 3;
    button3_2_pub_->publish(plan_flag_);
  }

  void SurgeonUI::on_pushButton1_3_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Start Drill");
    drill_flag_.data = "d";
    button0_3_pub_->publish(drill_flag_);
  }

  void SurgeonUI::on_pushButton0_3_clicked()
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Stop Drill");
    drill_flag_.data = "s";
    button1_3_pub_->publish(drill_flag_);
  }

  void SurgeonUI::on_pushButton0_4_clicked()
  {
    ui_->pushButton0_4->setStyleSheet("background-color: rgb(230, 97, 0);");
    ui_->pushButton1_4->setStyleSheet("background-color: white;");
    ui_->pushButton2_4->setStyleSheet("background-color: white;");
    ui_->pushButton3_4->setStyleSheet("background-color: white;");
    ui_->pushButton4_4->setStyleSheet("background-color: white;");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Drill Pose 0");
    pose_index_.data = 0;
    pose_index_pub_->publish(pose_index_);
  }

  void SurgeonUI::on_pushButton1_4_clicked()
  {
    ui_->pushButton1_4->setStyleSheet("background-color: rgb(230, 97, 0);");
    ui_->pushButton0_4->setStyleSheet("background-color: white;");
    ui_->pushButton2_4->setStyleSheet("background-color: white;");
    ui_->pushButton3_4->setStyleSheet("background-color: white;");
    ui_->pushButton4_4->setStyleSheet("background-color: white;");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Drill Pose 1");
    pose_index_.data = 1;
    pose_index_pub_->publish(pose_index_);
  }

  void SurgeonUI::on_pushButton2_4_clicked()
  {
    ui_->pushButton2_4->setStyleSheet("background-color: rgb(230, 97, 0);");
    ui_->pushButton0_4->setStyleSheet("background-color: white;");
    ui_->pushButton1_4->setStyleSheet("background-color: white;");
    ui_->pushButton3_4->setStyleSheet("background-color: white;");
    ui_->pushButton4_4->setStyleSheet("background-color: white;");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Drill Pose 2");
    pose_index_.data = 2;
    pose_index_pub_->publish(pose_index_);
  } 

  void SurgeonUI::on_pushButton3_4_clicked()
  {
    ui_->pushButton3_4->setStyleSheet("background-color: rgb(98, 160, 234);");
    ui_->pushButton0_4->setStyleSheet("background-color: white;");
    ui_->pushButton1_4->setStyleSheet("background-color: white;");
    ui_->pushButton2_4->setStyleSheet("background-color: white;");
    ui_->pushButton4_4->setStyleSheet("background-color: white;");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Drill Pose 3");
    pose_index_.data = 3;
    pose_index_pub_->publish(pose_index_);
  }

  void SurgeonUI::on_pushButton4_4_clicked()
  {
    ui_->pushButton4_4->setStyleSheet("background-color: rgb(98, 160, 234);");
    ui_->pushButton0_4->setStyleSheet("background-color: white;");
    ui_->pushButton1_4->setStyleSheet("background-color: white;");
    ui_->pushButton2_4->setStyleSheet("background-color: white;");
    ui_->pushButton3_4->setStyleSheet("background-color: white;");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Drill Pose 4");
    pose_index_.data = 4;
    pose_index_pub_->publish(pose_index_);
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_panel::SurgeonUI, rviz_common::Panel)
