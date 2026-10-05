#include "ball_tracker/ball_tracker.hpp"
#include <filesystem>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Input file as argument\n";
        return 0;
    }
    
    std::string filename = argv[1];
    std::cout << "Opening: " << filename << "\n";
    if (!std::filesystem::exists(filename))
    {
        std::cerr << "video file does not exist!\n";
        return 0;
    }

    cv::VideoCapture video(filename);
    if (!video.isOpened())
    {
        std::cout << "Failed to open video file\n";
        return 0;
    }

    cv::Mat frame;
    BallTracker tracker(0.4);
    while (true)
    {
        video >> frame;
        if (frame.empty()) break;

        BallTracker::DetectionData data = tracker.ProcessFrame(frame);
        if (data.center.x > 0)
        {
            cv::circle(frame, data.center, data.radius, cv::Scalar(255, 0, 0), 1);
            cv::putText(frame, "BALL", data.center, cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(255, 255, 0));
        }

        cv::imshow("detection", frame);
        cv::waitKey(1.0f/15 * 1000); // 15 fps
    }
    cv::destroyAllWindows();
    return 0;
}
