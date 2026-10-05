#pragma once

#include <opencv4/opencv2/opencv.hpp>

class BallTracker
{
public:
    struct DetectionData
    {
        cv::Point center = {-1, -1};
        double area = 0;
        double radius = 0;
        size_t perimeter = 0;
        cv::Rect bbox;
        std::vector<cv::Vec3d> trajectory;
    };
    
    BallTracker(double table_height);
    ~BallTracker();
    
    DetectionData ProcessFrame(cv::Mat& img);
    void AddSample(double x, double y, double z, double timestamp);
    inline void ClearSamples() { m_samples.clear(); }

    bool PredictHitPosition(double target_y, cv::Vec3d& prediction);
    
private:
    struct PointSample
    {
        PointSample()
        { }
        PointSample(double x, double y, double z, double timestamp)
            : point(cv::Vec3d(x, y, z)), stamp(timestamp)
        { }
        cv::Vec3d point = cv::Vec3d(0, 0, 0);
        double stamp = 0;
    };
    
    cv::Mat SegmentColour(const cv::Mat& img);
    void FilterBall(const cv::Mat& img, DetectionData& data);
    std::vector<cv::Vec3d> PredictTrajectory();
    std::vector<cv::Vec3d> PredictBounceTrajectory(const std::vector<cv::Vec3d>& pre_bounce_coefficients);

    double m_table_height;
    std::vector<PointSample> m_samples;
    
};
