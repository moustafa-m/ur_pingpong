#include "ur_pingpong_cmd_publisher/cmd_publisher_node.hpp"

CmdPublisherNode::CmdPublisherNode()
    : rclcpp::Node("cmd_publisher_node")
{
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
        this,
        "robot_state_publisher"
    );
    
    using namespace std::literals::chrono_literals;
    while (rclcpp::ok() && !param_client->wait_for_service(3s))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2,
            "Waiting for robot_state_publisher parameter service"
        );
    }

    std::string urdf = param_client->get_parameter<std::string>("robot_description");
    m_ik_solver = std::make_shared<TRAC_IK::TRAC_IK>("base_link", "tool0", urdf, 0.005, 1e-5, TRAC_IK::Distance);
    
    m_ik_solver->getKDLChain(m_chain);
    m_fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(m_chain);
    m_curr_joints.resize(m_chain.getNrOfJoints());
    
    m_sub_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = m_sub_group;

    m_ruckig_input.max_velocity = {10.0, 10.0, 10.0}; 
    m_ruckig_input.max_acceleration = {3.0, 3.0, 3.0};
    m_ruckig_input.max_jerk = {5.0, 5.0, 5.0};
    m_ruckig_input.synchronization = ruckig::Synchronization::None;
    
    m_hit_pos_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/ball_tracker/hit_position",
        rclcpp::QoS(1),
        std::bind(&CmdPublisherNode::HittingPointCallback, this, std::placeholders::_1),
        sub_options
    );
    
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(1),
        std::bind(&CmdPublisherNode::JointStateCallback, this, std::placeholders::_1),
        sub_options
    );
    
    m_traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory",
        rclcpp::QoS(10)
    );
    
    m_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_position_controller/commands",
        rclcpp::QoS(1)
    );

    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CmdPublisherNode::TimerCallback, this),
        nullptr
    );
    
    RCLCPP_INFO(this->get_logger(), "Node started");
}

CmdPublisherNode::~CmdPublisherNode()
{

}

void CmdPublisherNode::TimerCallback()
{
    if (!m_is_active) return;

    ruckig::Result result;
    {
        std::lock_guard<std::mutex> lock(m_target_mutex);
        result = m_otg.update(m_ruckig_input, m_ruckig_output);
    }
    
    if (result >= ruckig::Result::Working) 
    {
        KDL::Frame target_frame;
        target_frame.M = KDL::Rotation::Quaternion(0, 0.67638, 0.73657, 0);
        target_frame.p = KDL::Vector(m_ruckig_output.new_position[0],
                                    m_ruckig_output.new_position[1],
                                    m_ruckig_output.new_position[2]);

        KDL::JntArray q_out(6);
        if (m_ik_solver->CartToJnt(m_curr_joints, target_frame, q_out) >= 0)
        {
            std_msgs::msg::Float64MultiArray cmd_msg;
            for (size_t i = 0; i < 6; i++)
            {
                cmd_msg.data.push_back(q_out(i));
            }
            m_cmd_pub->publish(cmd_msg);
        }
        m_ruckig_output.pass_to_input(m_ruckig_input);
    }

    if (result == ruckig::Result::Finished)
    {
        m_is_active = false;
    }
}

void CmdPublisherNode::HittingPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_target_mutex);
    
    m_ruckig_input.target_position = {msg->point.x, msg->point.y, msg->point.z};
    
    if (!m_is_active)
    {
        KDL::Frame curr_pose;
        m_fk_solver->JntToCart(m_curr_joints, curr_pose, m_chain.getNrOfSegments());
        m_ruckig_input.current_position = {curr_pose.p[0], curr_pose.p[1], curr_pose.p[2]};
        m_is_active = true; 
    }
}

void CmdPublisherNode::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock_guard(m_curr_joints_mutex);

    if (msg->position.size() != m_chain.getNrOfJoints())
    {
        RCLCPP_ERROR(this->get_logger(), "joint state message joint number does not match robot's joint number");
        return;
    }

    for (size_t i = 0; i < msg->position.size(); i++)
    {
        m_curr_joints(i) = msg->position[i];
    }
}
