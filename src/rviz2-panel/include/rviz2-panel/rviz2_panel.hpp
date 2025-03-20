#pragma once


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#include <rviz_common/panel.hpp>
#include <QtWidgets>
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
    void on_pushButton0_2_clicked();
    void on_pushButton1_2_clicked();
    void on_pushButton2_2_clicked();
    void on_pushButton3_2_clicked();
    void on_pushButton0_3_clicked();
    void on_pushButton1_3_clicked();
    void on_pushButton0_4_clicked();
    void on_pushButton1_4_clicked();
    void on_pushButton2_4_clicked();
    void on_pushButton3_4_clicked();
    void on_pushButton4_4_clicked();

  

  private:
    std::unique_ptr<Ui::gui> ui_;
    rclcpp::Node::SharedPtr node_;

  protected:

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr button0_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr button1_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button2_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button3_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button0_2_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button1_2_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button2_2_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr button3_2_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button0_3_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button1_3_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pose_index_pub_;

    std_msgs::msg::Int32 plan_flag_;
    std_msgs::msg::String drill_flag_;
    std_msgs::msg::Empty parasight_flag_;
    std_msgs::msg::Int32 reg_flag_;
    std_msgs::msg::Int32 pose_index_;
  };
} // custom_panel
