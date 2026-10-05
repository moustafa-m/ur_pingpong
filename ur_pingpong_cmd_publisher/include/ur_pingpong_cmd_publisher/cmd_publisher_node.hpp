#pragma once

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ruckig/ruckig.hpp>

#include <mutex>

class CmdPublisherNode : public rclcpp::Node
{
public:
    CmdPublisherNode();
    ~CmdPublisherNode();

private:
    void TimerCallback();
    void HittingPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::CallbackGroup::SharedPtr m_sub_group;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_hit_pos_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_traj_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_cmd_pub;

    rclcpp::TimerBase::SharedPtr m_timer;

    KDL::Chain m_chain;
    std::shared_ptr<TRAC_IK::TRAC_IK> m_ik_solver;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> m_fk_solver;

    KDL::JntArray m_curr_joints;
    std::mutex m_curr_joints_mutex;

    ruckig::Ruckig<3> m_otg{0.025};
    ruckig::InputParameter<3> m_ruckig_input;
    ruckig::OutputParameter<3> m_ruckig_output;

    std::mutex m_target_mutex; 
    std::atomic<bool> m_is_active{false};

};
