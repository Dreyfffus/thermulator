#pragma once

#include <cstdint>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTexture.h>

namespace thermocator {

class ThermalDisplay : public rviz_common::MessageFilterDisplay<nav_msgs::msg::OccupancyGrid> {
    Q_OBJECT

  public:
    ThermalDisplay();
    ~ThermalDisplay() override;

  protected:
    void onInitialize() override;
    void reset() override;

  private Q_SLOTS:
    // Called by property system when user changes values in the RViz2 panel
    void onAlphaChanged();
    void onColorsChanged();

  private:
    void processMessage(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) override;

    // Fills the Ogre texture with colors derived from grid data
    void updateTexture(const nav_msgs::msg::OccupancyGrid &grid);

    // Rebuilds the quad geometry to match grid dimensions
    void updatePlane(const nav_msgs::msg::OccupancyGrid &grid);

    // ── Ogre objects ────────────────────────────────────────────────────────
    Ogre::SceneNode *child_node_ = nullptr;
    Ogre::ManualObject *manual_object_ = nullptr;
    Ogre::TexturePtr texture_;
    Ogre::MaterialPtr material_;

    // Track last dimensions so we only rebuild geometry when needed
    uint32_t last_width_ = 0;
    uint32_t last_height_ = 0;

    // ── RViz2 properties (owned by property system, not by us) ───────────────
    rviz_common::properties::FloatProperty *alpha_property_;
    rviz_common::properties::ColorProperty *cold_color_property_;
    rviz_common::properties::ColorProperty *hot_color_property_;
};

} // namespace thermocator
