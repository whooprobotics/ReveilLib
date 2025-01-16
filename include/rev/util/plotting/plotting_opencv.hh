#pragma once
#ifdef OFF_ROBOT_TESTS

#include <tuple>
#include <vector>
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/util/math/point_vector.hh"
#include "rev/util/math/pose.hh"
#include <iostream>

#include <opencv2/opencv.hpp>

using namespace rev;

cv::Point top_left(23, 26);
cv::Point top_right(471, 26);
cv::Point bottom_left(23, 475);
cv::Point bottom_right(471, 475);
double field_width_pixels = 448; //471-23 and 475-26
QLength field_width = 12_ft;

cv::Point convert_to_cv(PointVector point){
    double x_pixels = point.x.convert(inch) / field_width.convert(inch) * field_width_pixels;
    double y_pixels = point.y.convert(inch) / field_width.convert(inch) * field_width_pixels;
    return cv::Point(x_pixels, y_pixels) + top_left;
}

int display_robor_path(std::vector<PointVector> points){
    cv::Mat image = cv::imread("include/rev/util/plotting/field.png");

    if (image.empty()) {
        std::cout << "Could not read the image: " << std::endl;
        return 1;
    }

    std::vector<cv::Point> cv_points;
    for (PointVector point : points){
        cv_points.push_back(convert_to_cv(point));
    }

    cv::polylines(image, cv_points, false, cv::Scalar(0, 0, 255), 2);

    cv::imshow("Display window", image);
    cv::waitKey(0);

    return 0;
}

int test(){
    cv::Mat image = cv::imread("include/rev/util/plotting/field.png");

    if (image.empty()) {
        std::cout << "Could not read the image: " << std::endl;
        return 1;
    }

    cv::Point pt1(23, 26);
    cv::Point pt2(471, 475);

    cv::Scalar color(255, 0, 0);

    int thickness = 2;

    cv::line(image, pt1, pt2, color, thickness);

    cv::imshow("Display window", image);
    cv::waitKey(0);

    return 0;
}

#endif
