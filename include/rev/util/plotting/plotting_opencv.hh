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

int test(){
    cv::Mat image = cv::imread("field.png", cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Could not read the image: " << std::endl;
        return 1;
    }
    return 0;
}

#endif
