#pragma once

#include <cmath>

class Vec2d {
private:
    double x_;
    double y_;
    mutable double theta_;
    mutable double magnitude_;
    mutable bool dirty_;

public:
    Vec2d() = default;

    Vec2d(double x, double y)
        : x_(x), y_(y), theta_(0.0), magnitude_(0.0), dirty_(true)
    {}

    double x() const { return x_; }
    double y() const { return y_; }

    double magnitude() const {
        if (dirty_) updateCache();
        return magnitude_;
    }

    double theta() const {
        if (dirty_) updateCache();
        return theta_;
    }

    void updateCache() const {
        theta_ = std::atan2(y_, x_);
        magnitude_ = std::sqrt(y_*y_ + x_*x_);
    }

    void setX(const double x) {
        x_ = x;
        dirty_ = true;
    }

    void setY(const double y) {
        y_ = y;
        dirty_ = true;
    } 
    
    Vec2d normalized() const {
        double mag  = magnitude();
        return Vec2d(x_/mag, y_/mag);
    }

    Vec2d steerToward(const Vec2d& origin, double distance) const {
        Vec2d u = (*this - origin).normalized();
        return origin + u * distance;
    }

    double dot(const Vec2d& other) {
        return x() * other.x() + y() * other.y();
    }

    bool operator==(const Vec2d& other) const {
        return (x() == other.x() && y() == other.y());
    }

    Vec2d operator+(const Vec2d& other) const {
        return Vec2d(x() + other.x(), y() + other.y());
    }

    Vec2d operator-(const Vec2d& other) const {
        return Vec2d(x() - other.x(), y() - other.y());
    }

    Vec2d operator*(const double scalar) const {
        return Vec2d(x() * scalar, y() * scalar);
    }
};