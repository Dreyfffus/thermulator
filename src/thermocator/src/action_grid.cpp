#include "thermocator/action_grid.hpp"

#include <algorithm>
#include <cmath>

namespace thermocator {

void ActionGrid::Initialize(const nav_msgs::msg::OccupancyGrid &thermal_map) {
    _info = thermal_map.info;
    _width = thermal_map.info.width;
    _height = thermal_map.info.height;
    _resolution = thermal_map.info.resolution;
    _origin_x = thermal_map.info.origin.position.x;
    _origin_y = thermal_map.info.origin.position.y;

    const size_t size = static_cast<size_t>(_width) * _height;
    _values.assign(size, -1.0);

    _initialized = true;
}

void ActionGrid::WriteZone(
    double world_x, double world_y,
    double strength, double base_sigma) {
    if (!_initialized)
        return;

    // Sigma scales with zone strength -- stronger zones spread wider
    const double sigma = base_sigma * (strength / 100.0 + 0.5);
    const double two_sigma_sq = 2.0 * sigma * sigma;

    // Scan radius in cells -- 3 sigma covers 99.7% of the Gaussian
    const int radius = static_cast<int>(std::ceil(3.0 * sigma / _resolution));

    // Center cell
    const int center_col = static_cast<int>(
        std::floor((world_x - _origin_x) / _resolution));
    const int center_row = static_cast<int>(
        std::floor((world_y - _origin_y) / _resolution));

    for (int dr = -radius; dr <= radius; ++dr) {
        for (int dc = -radius; dc <= radius; ++dc) {
            const int row = center_row + dr;
            const int col = center_col + dc;

            if (row < 0 || row >= static_cast<int>(_height) ||
                col < 0 || col >= static_cast<int>(_width))
                continue;

            const double dist_sq =
                static_cast<double>(dr * dr + dc * dc) *
                _resolution * _resolution;

            const double gaussian =
                strength * std::exp(-dist_sq / two_sigma_sq);

            const size_t idx =
                static_cast<size_t>(row) * _width + col;

            // Take the max of existing and new contribution
            // so overlapping zones don't cancel each other out
            _values[idx] = std::max(_values[idx], gaussian);
        }
    }
}

std::vector<int8_t> ActionGrid::ToOccupancyData() const {
    const size_t size = static_cast<size_t>(_width) * _height;
    std::vector<int8_t> data(size, -1);

    // Find the max written value for normalization
    double max_val = 0.0;
    for (const auto &v : _values) {
        if (v >= 0.0)
            max_val = std::max(max_val, v);
    }

    if (max_val < 1e-6)
        return data;

    for (size_t i = 0; i < size; ++i) {
        if (_values[i] < 0.0)
            continue;

        const double normalized = _values[i] / max_val;
        const int8_t cell = static_cast<int8_t>(
            std::round(std::clamp(normalized * 100.0, 0.0, 100.0)));
        data[i] = cell;
    }

    return data;
}

} // namespace thermocator
