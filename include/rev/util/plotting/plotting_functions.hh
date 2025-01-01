#pragma once

#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/util/math/point_vector.hh"
#include "rev/util/math/pose.hh"
#include <tuple>
#include <vector>
#include <sciplot/sciplot.hpp>

using sciplot::Vec, sciplot::Plot2D, sciplot::Figure, sciplot::Canvas;
using namespace rev;

std::tuple<std::vector<double>, std::vector<double>> decompose_points(const std::vector<PointVector> points){
    std::vector<double> xs;
    std::vector<double> ys;

    xs.reserve(points.size());
    ys.reserve(points.size());

    for (PointVector point : points) {
        xs.push_back(point.x.convert(inch));
        ys.push_back(point.y.convert(inch));
    }

    return std::make_tuple(xs, ys);
}

int generate_plot(const std::vector<PointVector> points){
    auto [xs, ys] = decompose_points(points);

    // Create a Plot
    Plot2D plot;

    // Draw the first set of points
    plot.drawCurve(xs, ys)
        .label("Path 1: y = x")
        .lineColor("blue")  
        .pointType(7)       
        .lineWidth(2)       
        .pointSize(2);

    Figure fig = {{plot}};
    // Create canvas to hold figure
    Canvas canvas = {{fig}};

    // Show the plot in a pop-up window
    canvas.show();      
    
    return 0;
}

int generate_plot(const std::vector<PointVector> ideal_path, const std::vector<PointVector> actual_path){
    auto [xs1, ys1] = decompose_points(ideal_path);
    auto [xs2, ys2] = decompose_points(actual_path);

    // Create a Plot
    Plot2D plot;

    // Draw the first set of points
    plot.drawCurve(xs1, ys1)
        .label("Ideal path")
        .lineColor("blue")  
        .pointType(7)       
        .lineWidth(2)       
        .pointSize(2);
    
    // Draw the second set of points
    plot.drawCurve(xs2, ys2)
        .label("Actual path")
        .lineColor("red")  
        .pointType(7)       
        .lineWidth(2)       
        .pointSize(2);

    Figure fig = {{plot}};
    // Create canvas to hold figure
    Canvas canvas = {{fig}};

    // Show the plot in a pop-up window
    canvas.show();      
    
    return 0;

}