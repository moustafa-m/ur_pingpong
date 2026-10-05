#pragma once

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/transport/Node.hh>
#include <ignition/common/Console.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <thread>
#include <random>

namespace ball_spawner
{

class BallSpawner :
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemConfigure
{
public:
    void PostUpdate(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm
    ) override;
    void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _eventMgr
    ) override;

private:
    void SpawnSrvCallback(
        const std_srvs::srv::Empty::Request::SharedPtr request,
        const std_srvs::srv::Empty::Response::SharedPtr response
    );

    std::shared_ptr<rclcpp::Node> m_ros_node;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_spawn_srv;
    std::thread m_ros_thread;

    ignition::transport::Node m_node;
    ignition::transport::Node::Publisher m_wrench_pub;

    std::string m_ball_sdf = R"(
        <sdf version='1.7'>
            <model name='ball'>
                <link name='link'>
                    <inertial>
                        <mass>0.057</mass>
                        <inertia>
                            <ixx>0.0000608</ixx> <ixy>0</ixy> <ixz>0</ixz>
                            <iyy>0.0000608</iyy> <iyz>0</iyz>
                            <izz>0.0000608</izz>
                        </inertia>
                    </inertial>
                    <collision name='collision'>
                        <geometry><sphere><radius>0.025</radius></sphere></geometry>
                        <surface>
                            <friction>
                                <ode>
                                    <mu>0.9</mu>
                                    <mu2>0.9</mu2>
                                </ode>
                            </friction>
                            <bounce>
                                <restitution_coefficient>0.90</restitution_coefficient>
                                <threshold>0.01</threshold>
                            </bounce>
                            <contact>
                                <ode>
                                    <max_vel>100.0</max_vel>
                                    <kp>1000000.0</kp>
                                    <kd>10.0</kd>
                                </ode>
                            </contact>
                        </surface>
                    </collision>
                    <visual name='visual'>
                        <geometry><sphere><radius>0.025</radius></sphere></geometry>
                        <material>
                            <ambient>1 0.65 0 1</ambient>
                            <diffuse>1 0.65 0 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>)";
};

} // namespace ball_spawner
