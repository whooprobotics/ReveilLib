#pragma once

#include <vector>
#include <algorithm>

#include "rev/api/motionplanning/planning/rrt/rrt.hpp"
#include "eigen/Dense"
#include "eigen/QR"

class SplineArcLengthTable {
    std::vector<double> tSamples;
    std::vector<double> sSamples;

public:
    double maxS;

    void generate(const std::vector<double>& tSamples, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points) {
        this->tSamples = tSamples;
        sSamples.resize(points.size());
        sSamples[0] = 0.0;
        for (size_t i = 1; i < points.size(); i++) {
            auto dS = (points[i] - points[i-1]).norm();
            sSamples[i] = sSamples[i-1] + dS;
        }
        maxS = sSamples.back();
    }

    double getT(double s) const {
        if (s < 0) return 0;
        if (s > sSamples.back()) return sSamples.back();
        auto lower = std::lower_bound(sSamples.begin(), sSamples.end(), s);
        size_t i = std::distance(sSamples.begin(), lower) - 1;

        double t0 = tSamples[i];
        double t1 = tSamples[i+1];
        double s0 = sSamples[i];
        double s1 = sSamples[i+1];

        double t = t0 + (s - s0) / (s1 - s0) * (t1 - t0);

        return t;
    }
};

class Spline {
private:
    void parameterize();
    void calculateKnots();
    void calculateControlPoints(double smoothingFactor);
    void calculateDerivativeControlPoints();
    void calculateSecondDerivativeControlPoints();
    void generateArcLengthMapping(int numSamples);

    Eigen::Vector2d calculateAtT(double t) const;
    Eigen::Vector2d calculateDerivativeAtT(double t) const;
    Eigen::Vector2d calculateSecondDerivativeAtT(double t) const;

    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampleSplineT(int numSamples) const;

    std::vector<double> t; // t
    std::vector<double> knots; // knots
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> controlPoints;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> derivativeControlPoints;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> secondDerivativeControlPoints;
    double N(int i, double t, unsigned int p) const;

    SplineArcLengthTable arcLengthTable;

public:
    std::vector<RRT::Node*> nodes;
    unsigned int degree;

    Spline() = default;
    Spline(std::vector<RRT::Node*>& nodes, unsigned int degree)
        :  nodes(nodes), degree(degree), knots(nodes.size() + degree + 1)
    {
    }

    void generate(const double smoothingFactor) {
        parameterize();
        calculateKnots();
        calculateControlPoints(smoothingFactor);
        calculateDerivativeControlPoints();
        calculateSecondDerivativeControlPoints();
        generateArcLengthMapping(100);
    }

    Eigen::Vector2d calculateAt(double s) const { return calculateAtT(arcLengthTable.getT(s)); }
    Eigen::Vector2d calculateDerivativeAt(double s) const { return calculateDerivativeAtT(arcLengthTable.getT(s)); }
    Eigen::Vector2d calculateSecondDerivativeAt(double s) const { return calculateSecondDerivativeAtT(arcLengthTable.getT(s)); }
    
    double nearestS(Eigen::Vector2d point, int numSamples) const;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampleSpline(int numSamples) const;

    int numT() const { return t.size(); }
    int numKnots() const { return knots.size(); }
    int numControlPoints() const { return controlPoints.size(); }
};
