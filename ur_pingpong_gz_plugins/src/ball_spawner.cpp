#include <gz/plugin/Register.hh>

#include "ur_pingpong_gz_plugins/ball_spawner.hpp"

using namespace ball_spawner;

IGNITION_ADD_PLUGIN(
    BallSpawner,
    gz::sim::System,
    BallSpawner::ISystemPostUpdate,
    BallSpawner::ISystemConfigure
)

void BallSpawner::PostUpdate(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{

}

void BallSpawner::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& _eventMgr
)
{
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    
    m_wrench_pub = m_node.Advertise<ignition::msgs::EntityWrench>("/world/ur_pingpong_world/wrench");
    m_ros_node = std::make_shared<rclcpp::Node>("ball_spawner_plugin");
    m_spawn_srv = m_ros_node->create_service<std_srvs::srv::Empty>(
        "/gz/spawn_ball",
        std::bind(&BallSpawner::SpawnSrvCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    m_ros_thread = std::thread([this]() {
        rclcpp::spin(m_ros_node);
    });
    m_ros_thread.detach();

}

void BallSpawner::SpawnSrvCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    const std_srvs::srv::Empty::Response::SharedPtr response
)
{
    ignition::msgs::Entity remove_req;
    remove_req.set_name("ball");
    remove_req.set_type(ignition::msgs::Entity::MODEL);
    
    ignition::msgs::Boolean reply;
    bool result = false;
    int timeout = 2000;
    m_node.Request("/world/ur_pingpong_world/remove", remove_req, timeout, reply, result);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ignition::msgs::EntityFactory factory;
    factory.set_sdf(m_ball_sdf);
    factory.set_allow_renaming(true);

    std::random_device device;
    std::mt19937 pos_generator(device());
    std::normal_distribution<double> x_distrib(0.0, 0.1);

    ignition::msgs::Pose* pose = factory.mutable_pose();
    double x_position = x_distrib(pos_generator);
    pose->mutable_position()->set_x(x_position);
    pose->mutable_position()->set_y(4.35);
    pose->mutable_position()->set_z(0.6);

    bool executed = m_node.Request("/world/ur_pingpong_world/create", factory, timeout, reply, result);

    if (!result)
    {
        ignerr << "[BallSpawner] failed to spawn ball" << std::endl;
        return;
    }

    ignition::msgs::EntityWrench wrench_msg;
    wrench_msg.mutable_entity()->set_name("ball");
    wrench_msg.mutable_entity()->set_type(ignition::msgs::Entity::MODEL);
    
    std::mt19937 force_generator(device());
    std::uniform_real_distribution<double> x_force_distrib(-10, 10);
    std::uniform_real_distribution<double> y_force_distrib(600, 700);
    std::uniform_real_distribution<double> z_force_distrib(-10, 0);
    
    ignition::msgs::Vector3d* force = wrench_msg.mutable_wrench()->mutable_force();
    force->set_x(x_force_distrib(force_generator));
    force->set_y(-1 * y_force_distrib(force_generator));
    force->set_z(z_force_distrib(force_generator));
    // auto wrench_pub = m_node.Advertise<ignition::msgs::EntityWrench>("/world/ur_pingpong_world/wrench");
    // wrench_pub.Publish(wrench_msg);
    m_wrench_pub.Publish(wrench_msg);

    ignmsg << "[BallSpawner] spawned ball with position: [" << pose->position().x() << ", "
        << pose->position().y() << ", " << pose->position().z() << "] and applied force of: ["
        << force->x() << ", " << force->y() << ", " << force->z() << "]" << std::endl;
}
