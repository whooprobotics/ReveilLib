#pragma once
#include <cmath>

struct QLength {
    double inches = 0.0;

    constexpr QLength() = default;
    constexpr explicit QLength(double inches) : inches(inches) {}

    constexpr QLength operator+(QLength rhs) const { return QLength(inches + rhs.inches); }
    constexpr QLength operator-(QLength rhs) const { return QLength(inches - rhs.inches); }
    constexpr QLength operator*(double scalar) const { return QLength(inches * scalar); }
    constexpr QLength operator/(double scalar) const { return QLength(inches / scalar); }
    constexpr bool operator<(QLength rhs) const { return inches < rhs.inches; }
    constexpr bool operator>(QLength rhs) const { return inches > rhs.inches; }
    QLength& operator+=(QLength rhs) { inches += rhs.inches; return *this; }
    QLength& operator-=(QLength rhs) { inches -= rhs.inches; return *this; }

    constexpr double convert(QLength unit) const { return inches / unit.inches; }
    constexpr double internal() const { return inches; }
};

inline constexpr QLength inch  {1.0};
inline constexpr QLength foot  {12.0};
inline constexpr QLength meter {39.3701};
inline constexpr QLength cm    {0.393701};

inline constexpr QLength operator""_in (long double v) { return QLength(static_cast<double>(v)); }
inline constexpr QLength operator""_ft (long double v) { return QLength(static_cast<double>(v) * 12.0); }
inline constexpr QLength operator""_m  (long double v) { return QLength(static_cast<double>(v) * 39.3701); }
inline constexpr QLength operator""_cm (long double v) { return QLength(static_cast<double>(v) * 0.393701); }

inline constexpr QLength operator""_in (unsigned long long v) { return QLength(static_cast<double>(v)); }
inline constexpr QLength operator""_ft (unsigned long long v) { return QLength(static_cast<double>(v) * 12.0); }
inline constexpr QLength operator""_m  (unsigned long long v) { return QLength(static_cast<double>(v) * 39.3701); }
inline constexpr QLength operator""_cm (unsigned long long v) { return QLength(static_cast<double>(v) * 0.393701); }

struct QAngle {
    double degrees = 0.0;

    constexpr QAngle() = default;
    constexpr explicit QAngle(double degrees) : degrees(degrees) {}

    constexpr QAngle operator+(QAngle rhs) const { return QAngle(degrees + rhs.degrees); }
    constexpr QAngle operator-(QAngle rhs) const { return QAngle(degrees - rhs.degrees); }
    constexpr QAngle operator*(double scalar) const { return QAngle(degrees * scalar); }
    constexpr QAngle operator/(double scalar) const { return QAngle(degrees / scalar); }
    constexpr bool operator<(QAngle rhs) const { return degrees < rhs.degrees; }
    constexpr bool operator>(QAngle rhs) const { return degrees > rhs.degrees; }
    QAngle& operator+=(QAngle rhs) { degrees += rhs.degrees; return *this; }
    QAngle& operator-=(QAngle rhs) { degrees -= rhs.degrees; return *this; }

    constexpr double convert(QAngle unit) const { return degrees / unit.degrees; }
    constexpr double internal() const { return degrees; }
};

inline constexpr QAngle degree {1.0};
inline constexpr QAngle radian {180.0 / M_PI};

inline constexpr QAngle operator""_deg(long double v) { return QAngle(static_cast<double>(v)); }
inline constexpr QAngle operator""_rad(long double v) { return QAngle(static_cast<double>(v) * (180.0 / M_PI)); }
inline constexpr QAngle operator""_deg(unsigned long long v) { return QAngle(static_cast<double>(v)); }
inline constexpr QAngle operator""_rad(unsigned long long v) { return QAngle(static_cast<double>(v) * (180.0 / M_PI)); }

struct QTime {
    double ms = 0.0;

    constexpr QTime() = default;
    constexpr explicit QTime(double ms) : ms(ms) {}

    constexpr QTime operator+(QTime rhs) const { return QTime(ms + rhs.ms); }
    constexpr QTime operator-(QTime rhs) const { return QTime(ms - rhs.ms); }
    constexpr QTime operator*(double scalar) const { return QTime(ms * scalar); }
    constexpr QTime operator/(double scalar) const { return QTime(ms / scalar); }
    constexpr bool operator<(QTime rhs) const { return ms < rhs.ms; }
    constexpr bool operator>(QTime rhs) const { return ms > rhs.ms; }
    QTime& operator+=(QTime rhs) { ms += rhs.ms; return *this; }
    QTime& operator+=(double rhs) { ms += rhs; return *this; }
    QTime& operator-=(QTime rhs) { ms -= rhs.ms; return *this; }

    constexpr double convert(QTime unit) const { return ms / unit.ms; }
    constexpr double internal() const { return ms; }
};

inline constexpr QTime millisecond {1.0};
inline constexpr QTime second {1000.0};
inline constexpr QTime minute {60000.0};

inline constexpr QTime operator""_ms (long double v) { return QTime(static_cast<double>(v)); }
inline constexpr QTime operator""_sec(long double v) { return QTime(static_cast<double>(v) * 1000.0); }
inline constexpr QTime operator""_min(long double v) { return QTime(static_cast<double>(v) * 60000.0); }
inline constexpr QTime operator""_ms (unsigned long long v) { return QTime(static_cast<double>(v)); }
inline constexpr QTime operator""_sec(unsigned long long v) { return QTime(static_cast<double>(v) * 1000.0); }
inline constexpr QTime operator""_min(unsigned long long v) { return QTime(static_cast<double>(v) * 60000.0); }
