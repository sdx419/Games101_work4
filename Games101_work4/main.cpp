#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
std::map<int, int> colorBuffer;
bool antiAliasing = false;

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
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> points = control_points;
    int length = points.size();
    while (length >= 2)
    {
        for (int j = 0; j < length - 1; j++)
        {
            points[j].x = (1 - t) * points[j].x + t * points[j + 1].x;
            points[j].y = (1 - t) * points[j].y + t * points[j + 1].y;
        }
        length--;
    }

    return points[0];

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        int rowIndex = point.y;
        int colIndex = point.x;
        if (!antiAliasing)
        {
            window.at<cv::Vec3b>(rowIndex, colIndex)[1] = 255;
        }
        else
        {
            colorBuffer.insert({ rowIndex * 700 + colIndex, 255 });
        }
    }

    if (!antiAliasing)
        return;

    std::map<int, int>::iterator iter;
    for (iter = colorBuffer.begin(); iter != colorBuffer.end(); iter++)
    {
        int index = iter->first;
        int rowIndex = index / 700;
        int colIndex = index % 700;
        int leftUpIndex = (rowIndex + 1) * 700 + colIndex - 1;
        int leftIndex = rowIndex * 700 + colIndex - 1;
        int upIndex = (rowIndex + 1) * 700 + colIndex;
        int leftUpColor = colorBuffer.find(leftUpIndex) == colorBuffer.end() ? 0 : colorBuffer[leftUpIndex];
        int leftColor = colorBuffer.find(leftIndex) == colorBuffer.end() ? 0 : colorBuffer[leftIndex];
        int upColor = colorBuffer.find(upIndex) == colorBuffer.end() ? 0 : colorBuffer[upIndex];
        int finalColor = 0.25f * (iter->second + leftUpColor + leftColor + upColor);
        window.at<cv::Vec3b>(rowIndex, colIndex)[1] = finalColor;

    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            //antiAliasing = true;
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
