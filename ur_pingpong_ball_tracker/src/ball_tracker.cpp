#include <eigen3/Eigen/Dense>
#include "ball_tracker/ball_tracker.hpp"

BallTracker::BallTracker(double table_height)
    : m_table_height(table_height)
{
    m_samples.reserve(5);
}

BallTracker::~BallTracker()
{

}

BallTracker::DetectionData BallTracker::ProcessFrame(cv::Mat& img)
{
    cv::Mat colour_mask = this->SegmentColour(img);
    DetectionData data;
    this->FilterBall(colour_mask, data);
    // cv::imshow("mask", colour_mask);
    // cv::imshow("img", img);
    // cv::waitKey(1);
    return data;
}

cv::Mat BallTracker::SegmentColour(const cv::Mat& img)
{
    cv::Mat converted_img;
    cv::GaussianBlur(img, converted_img, cv::Size(5,5), 0, 0);
    cv::cvtColor(converted_img, converted_img, cv::COLOR_BGR2HSV);

    cv::Scalar orange_lower = {10, 127, 20};
    cv::Scalar orange_upper = {40, 255, 255};
    cv::Mat mask;
    cv::inRange(converted_img, orange_lower, orange_upper, mask);
    return mask;
}

void BallTracker::FilterBall(const cv::Mat& img, DetectionData& data)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point2f center = {-1, -1};
    float radius = -1;
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::minEnclosingCircle(contours[i], center, radius);

        cv::Rect bbox = cv::boundingRect(contours[i]);
        double aspect_ratio = ((double)bbox.width)/bbox.height;

        double contour_area = cv::contourArea(contours[i]);
        double area_extent = contour_area/bbox.area();

        if (contour_area >= 50 && contour_area <= 500 &&
            area_extent >= 0.5 && area_extent <= 1 &&
            aspect_ratio >= 1/1.4f && aspect_ratio <= 1.4f
        )
        {
            data.center = center;
            data.area = contour_area;
            data.radius = radius;
            data.perimeter = cv::arcLength(contours[i], true);
            data.bbox = std::move(bbox);
            return;
        }
    }
}

void BallTracker::AddSample(double x, double y, double z, double timestamp)
{
    if (m_samples.size() > 0 && y > m_samples.front().point[1])
    {
        this->ClearSamples();
    }
    m_samples.emplace_back(x, y, z, timestamp);
}

bool BallTracker::PredictHitPosition(double target_y, cv::Vec3d& prediction)
{
    auto pre_bounce_coefficients = this->PredictTrajectory();
    if (pre_bounce_coefficients.empty()) 
    {
        return false;
    }
    auto coefficients = this->PredictBounceTrajectory(pre_bounce_coefficients);
    if (coefficients.empty())
    {
        return false;
    }

    double a_y = coefficients[1](0);
    double b_y = coefficients[1](1);
    double c_y = coefficients[1](2) - target_y;

    double discriminant = (b_y * b_y) - (4 * a_y * c_y);
    double hitting_time = -1;
    if (discriminant >= 0)
    {
        double t1 = (-b_y + std::sqrt(discriminant)) / (2 * a_y);
        double t2 = (-b_y - std::sqrt(discriminant)) / (2 * a_y);
        if (t1 > 0 && t2 > 0)
        {
            hitting_time = std::min(t1, t2);
        }
        else if (t1 > 0)
        {
            hitting_time = t1; 
        }
        else
        {
            hitting_time = t2;
        }
    }
    
    if (hitting_time < 0 || std::isnan(hitting_time) || std::isinf(hitting_time)) return false;

    double x_pos = coefficients[0][0] * hitting_time * hitting_time +
                coefficients[0][1] * hitting_time +
                coefficients[0][2];
    
    double z_pos = coefficients[2][0] * hitting_time*hitting_time +
                coefficients[2][1] * hitting_time +
                coefficients[2][2];
    
    // std::cout << "hitting time: " << hitting_time << "\n";
    // std::cout << x_pos << " " << target_y << " " << z_pos << std::endl;
    prediction[0] = x_pos;
    prediction[1] = target_y;
    prediction[2] = z_pos;
    return true;
}

std::vector<cv::Vec3d> BallTracker::PredictTrajectory()
{
    if (m_samples.size() < 5) return std::vector<cv::Vec3d>();

    size_t sample_size = m_samples.size();

    Eigen::MatrixXd A(sample_size, 3);
    Eigen::VectorXd b(sample_size);

    double first_stamp = m_samples.front().stamp;
    std::vector<cv::Vec3d> coefficients;
    for (int axis = 0; axis < 3; axis++)
    {
        // solving Ax = b for each axis
        for (size_t i = 0; i < sample_size; i++)
        {
            // shifting time stamp to start from zero in case of large vaues that will overflow
            double stamp = m_samples[i].stamp - first_stamp;
            A(i, 0) = stamp * stamp;
            A(i, 1) = stamp;
            A(i, 2) = 1;
            b(i) = m_samples[i].point[axis];
        }
        Eigen::Vector3d ans = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        coefficients.emplace_back(ans.x(), ans.y(), ans.z());
    }
    return coefficients;
}

std::vector<cv::Vec3d> BallTracker::PredictBounceTrajectory(const std::vector<cv::Vec3d>& pre_bounce_coefficients)
{
    double a_z = pre_bounce_coefficients[2](0);
    double b_z = pre_bounce_coefficients[2](1);
    double c_z = pre_bounce_coefficients[2](2) - m_table_height;

    // std::cout << pre_bounce_coefficients[2](0) << " " << pre_bounce_coefficients[2](1) << " " << pre_bounce_coefficients[2](2) << "\n";
    double discriminant = (b_z * b_z) - (4 * a_z * c_z);
    double bounce_time = -1;
    if (discriminant >= 0)
    {
        double t1 = (-b_z + std::sqrt(discriminant)) / (2 * a_z);
        double t2 = (-b_z - std::sqrt(discriminant)) / (2 * a_z);
        bounce_time = (t1 > 0) ? t1 : t2; 
    }

    // std::cout << a_y << " " << b_y << " " << c_y << "\n";
    // std::cout << "bounce time: " << bounce_time << "\n";
    // std::cout << "samples:\n";
    // for (auto& sample: m_samples)
    // {
    //     std::cout << sample.point[0] << " " << sample.point[1] << " " << sample.point[2] << "\n";
    // }

    std::vector<cv::Vec3d> bounce_coefficients;
    bounce_coefficients.reserve(3);
    std::vector<cv::Vec3d> bounce_losses = {
        cv::Vec3d(0.90, 0.3),
        cv::Vec3d(0.90, 0.3),
        cv::Vec3d(-0.9, 0.9)
    };

    // quadratic after bouncing, using q as the coefficient name
    for (int axis = 0; axis < 3; axis++)
    {
        double q_1 = bounce_losses[axis][1]*pre_bounce_coefficients[axis](0);
        
        double q_2 = bounce_losses[axis][0] *
                    ((2*pre_bounce_coefficients[axis](0) * bounce_time) + pre_bounce_coefficients[axis](1)) -
                    (2*q_1*bounce_time);

        double bounce_position = (pre_bounce_coefficients[axis](0) * bounce_time * bounce_time) +
                                (pre_bounce_coefficients[axis](1) * bounce_time) +
                                pre_bounce_coefficients[axis](2);
        
        double q_3 = bounce_position -
                    (q_1 * bounce_time * bounce_time) -
                    (q_2 * bounce_time);
        bounce_coefficients.emplace_back(q_1, q_2, q_3);
    }

    return bounce_coefficients;
}
