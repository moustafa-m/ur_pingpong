#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ball_tracker/ball_tracker_node.hpp"

BallTrackerNode::BallTrackerNode()
    : rclcpp::Node("ball_tracker_node"), m_latest_prediction(0, 0, 0), m_table_height(0.4)
{
    m_ball_tracker = std::make_unique<BallTracker>(m_table_height);

    m_img_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, "/camera/image"
    );

    m_pc_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, "/camera/points"
    );

    m_synchroniser = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(10), *m_img_sub, *m_pc_sub
    );

    m_synchroniser->registerCallback(
        &BallTrackerNode::SynchronisedCallback, this
    );

    m_pos_pub = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/ball_tracker/hit_position",
        rclcpp::QoS(1)
    );

    auto tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);
    try
    {
        m_camera_to_world_tf = tf2_buffer->lookupTransform(
            "world",
            "camera_link",
            tf2::TimePointZero,
            tf2::Duration(std::chrono::nanoseconds((int64_t)3e9))
        );
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Node started");
}

BallTrackerNode::~BallTrackerNode()
{

}

void BallTrackerNode::SynchronisedCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud
)
{
    cv::Mat cv_image;
    try
    {
        cv_image = cv_bridge::toCvShare(img, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    auto data = m_ball_tracker->ProcessFrame(cv_image);
    if (data.center.x < 0) return;
    
    sensor_msgs::PointCloud2ConstIterator<float> x_iter(*point_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iter(*point_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iter(*point_cloud, "z");
    
    int index = data.center.y * img->width + data.center.x;
    x_iter += index;
    y_iter += index;
    z_iter += index;

    float x = *x_iter;
    float y = *y_iter;
    float z = *z_iter;

    geometry_msgs::msg::PointStamped sample;
    sample.header.stamp = this->get_clock()->now();
    sample.header.frame_id = "camera_link";
    sample.point.x = x;
    sample.point.y = y;
    sample.point.z = z;

    geometry_msgs::msg::PointStamped transformed_sample;
    tf2::doTransform(sample, transformed_sample, m_camera_to_world_tf);

    if (transformed_sample.point.z <= 1.1*m_table_height || transformed_sample.point.y < 2.5)
    {
        m_ball_tracker->ClearSamples();
        return;
    }
    
    // std::cout << "index: " << index << " Points: " << transformed_sample.point.x << " "
    //     << transformed_sample.point.y << " " << transformed_sample.point.z << std::endl;
    
    // use time in ms
    double stamp = (img->header.stamp.sec*1e3) + (img->header.stamp.nanosec*1e-6);
    m_ball_tracker->AddSample(transformed_sample.point.x, transformed_sample.point.y, transformed_sample.point.z, stamp);
    
    if (m_ball_tracker->PredictHitPosition(0.4, m_latest_prediction))
    {
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "world";
        msg.point.x = m_latest_prediction[0];
        msg.point.y = m_latest_prediction[1];
        msg.point.z = m_latest_prediction[2];
        m_pos_pub->publish(msg);
    }
}
