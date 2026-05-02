#pragma once
#include <cmath>

struct center {
    double x{0.0}, y{0.0};
};

struct HeatZone {
    center _center;
    double _peak_temperature;
    double _sigma;

    double ContributionAt(double x, double y) const {
        const double dx = x - _center.x;
        const double dy = y - _center.y;
        const double dist2 = dx * dx + dy * dy;
        return _peak_temperature * std::exp(-dist2 / (2.0 * _sigma * _sigma));
    }
};
