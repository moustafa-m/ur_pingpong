#pragma once

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2/convert.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>

#include "ball_tracker.hpp"

class BallTrackerNode : public rclcpp::Node
{
public:
    BallTrackerNode();
    ~BallTrackerNode();

private:
    void SynchronisedCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud
    );

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> ApproxSyncPolicy;

    std::unique_ptr<BallTracker> m_ball_tracker;

    std::shared_ptr<ApproxSyncPolicy> m_sync_policy;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> m_synchroniser;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> m_img_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> m_pc_sub;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_pos_pub;

    geometry_msgs::msg::TransformStamped m_camera_to_world_tf;

    cv::Vec3d m_latest_prediction;
    double m_table_height;

};