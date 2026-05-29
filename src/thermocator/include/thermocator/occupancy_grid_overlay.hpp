#pragma once

#include <cstdint>
#include <string>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTexture.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

namespace thermocator {

// ----------------------------------------------------------------------------
// OccupancyGridOverlay
//
// Generic RViz2 display plugin for any OccupancyGrid topic.
//
// Interpolates all four RGBA channels between a cold and hot RGBA value
// based on the occupancy value (0-100):
//
//   pixel = cold_rgba + (value / 100.0) * (hot_rgba - cold_rgba)
//
// This lets subclasses define two distinct behaviors through defaults alone:
//
//   Thermal: cold=(blue, a=1.0) hot=(red, a=1.0)
//            -> color changes, always fully opaque
//
//   Action:  cold=(green, a=0.0) hot=(green, a=1.0)
//            -> color fixed, alpha fades in with value (Gaussian gradient)
//
// Unknown cells (-1) are always fully transparent regardless of settings.
// ----------------------------------------------------------------------------

class OccupancyGridOverlay
    : public rviz_common::MessageFilterDisplay<nav_msgs::msg::OccupancyGrid> {
    Q_OBJECT

  public:
    struct RgbaDefault {
        QColor color;
        float alpha;
    };

    explicit OccupancyGridOverlay(
        std::string default_topic = "",
        RgbaDefault cold = {QColor(0, 0, 0), 1.0f},
        RgbaDefault hot = {QColor(0, 0, 0), 1.0f});

    ~OccupancyGridOverlay() override;

  protected:
    void onInitialize() override;
    void reset() override;

  private Q_SLOTS:
    void onPropertyChanged();

  private:
    void processMessage(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) override;

    void updateTexture(const nav_msgs::msg::OccupancyGrid &grid);
    void updatePlane(const nav_msgs::msg::OccupancyGrid &grid);

    std::string _default_topic;
    RgbaDefault _default_cold;
    RgbaDefault _default_hot;

    // Ogre resources
    Ogre::SceneNode *_child_node = nullptr;
    Ogre::ManualObject *_manual_object = nullptr;
    Ogre::TexturePtr _texture;
    Ogre::MaterialPtr _material;

    uint32_t _last_width = 0;
    uint32_t _last_height = 0;

    // Cold endpoint -- color and alpha at occupancy value 0
    rviz_common::properties::ColorProperty *_cold_color_property = nullptr;
    rviz_common::properties::FloatProperty *_cold_alpha_property = nullptr;

    // Hot endpoint -- color and alpha at occupancy value 100
    rviz_common::properties::ColorProperty *_hot_color_property = nullptr;
    rviz_common::properties::FloatProperty *_hot_alpha_property = nullptr;

    std::string _resource_group_name;
    std::string _base_name;
    rclcpp::Logger _logger = rclcpp::get_logger("OccupancyGridOverlay");
};

// ----------------------------------------------------------------------------
// Registered plugin variants
// Same interpolation logic, different defaults.
// ----------------------------------------------------------------------------

// Thermal: blue (opaque) -> red (opaque) -- pure color gradient
class ThermalOverlay : public OccupancyGridOverlay {
    Q_OBJECT
  public:
    ThermalOverlay()
        : OccupancyGridOverlay(
              "/thermal_map",
              {QColor(0, 0, 255), 1.0f},
              {QColor(255, 0, 0), 1.0f}) {}
};

// Action: green (transparent) -> green (opaque) -- pure alpha gradient
class ActionOverlay : public OccupancyGridOverlay {
    Q_OBJECT
  public:
    ActionOverlay()
        : OccupancyGridOverlay(
              "/action_map",
              {QColor(0, 220, 80), 0.0f},
              {QColor(0, 220, 80), 1.0f}) {}
};

} // namespace thermocator
