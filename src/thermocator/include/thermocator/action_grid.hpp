#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace thermocator {

class ActionGrid {
  public:
    ActionGrid() = default;

    void Initialize(const nav_msgs::msg::OccupancyGrid &thermal_map);

    bool IsInitialized() const { return _initialized; }

    void WriteZone(
        double world_x, double world_y,
        double strength, double base_sigma);

    std::vector<int8_t> ToOccupancyData() const;

    nav_msgs::msg::MapMetaData getInfo() const { return _info; }

  private:
    nav_msgs::msg::MapMetaData _info;

    uint32_t _width = 0;
    uint32_t _height = 0;
    double _resolution = 0.05;
    double _origin_x = 0.0;
    double _origin_y = 0.0;

    std::vector<double> _values;

    bool _initialized = false;
};

} // namespace thermocator
