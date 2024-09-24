#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
// add include for srd_msgs Int32
#include <std_msgs/msg/int32.hpp>
// RVIZ2
#include <rviz_common/panel.hpp>
// Qt
#include <QtWidgets>
// STL
#include <memory>
/**
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_push_button.h>

namespace custom_panel
{
  class SurgeonUI : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit SurgeonUI(QWidget* parent = nullptr);
    ~SurgeonUI();

    /// Load and save configuration data
    virtual void load(const rviz_common::Config& config) override;
    virtual void save(rviz_common::Config config) const override;

  private Q_SLOTS:
    void on_pushButton0_clicked();
    void on_pushButton1_clicked();
    void on_pushButton2_clicked();
    void on_pushButton3_clicked();

  private:
    std::unique_ptr<Ui::gui> ui_;
    rclcpp::Node::SharedPtr node_;
    // uint16_t count_button_1_, count_button_2_;

  protected:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button0_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button1_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button2_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button3_pub_;
    // std_msgs::msg::Bool msg_;
    std_msgs::msg::Int32 plan_flag_;
    std_msgs::msg::Int32 registration_flag_;
  };
} // custom_panel
