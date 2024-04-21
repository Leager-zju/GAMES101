#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    if (control_points.size() == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> new_control_points;
    for (int i = 0; i < control_points.size()-1; i++) {
        new_control_points.emplace_back(t * control_points[i] + (1-t) * control_points[i+1]);
    }
    return recursive_bezier(new_control_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        cv::Point2f point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

void bezier_antialiasing(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        cv::Point2f point = recursive_bezier(control_points, t);

        float x = point.x;
        float y = point.y;
        float u = x - (int)x + 0.5;
        float v = y - (int)y + 0.5;
        if (u > 1) {
            u = u-1;
        }
        if (v > 1) {
            v = v-1;
        }

        float d00 = pow(u, 2) + pow(v, 2);
        float d01 = pow(1-u, 2) + pow(v, 2);
        float d10 = pow(u, 2) + pow(1-v, 2);
        float d11 = pow(1-u, 2) + pow(1-v, 2);

        window.at<cv::Vec3b>(y, x)[1] =
            fmin(255, window.at<cv::Vec3b>(y, x)[1] + 255 * (1 - sqrt(d00 / 2)));
        window.at<cv::Vec3b>(y, x + 1)[1] =
            fmin(255, window.at<cv::Vec3b>(y, x + 1)[1] + 255.0 * (1 - sqrt(d01 / 2)));
        window.at<cv::Vec3b>(y + 1, x)[1] =
            fmin(255, window.at<cv::Vec3b>(y + 1, x)[1] + 255.0 * (1 - sqrt(d10 / 2)));
        window.at<cv::Vec3b>(y + 1, x + 1)[1] =
            fmin(255, window.at<cv::Vec3b>(y + 1, x + 1)[1] + 255.0 * 1 - sqrt(d11 / 2));
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::Mat window_antialiasing = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::cvtColor(window_antialiasing, window_antialiasing, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
            cv::circle(window_antialiasing, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);
            bezier_antialiasing(control_points, window_antialiasing);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            cv::imwrite("my_bezier_curve_anti_aliasing.png", window_antialiasing);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
