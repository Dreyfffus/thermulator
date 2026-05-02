#include "thermocator/thermal_display.hpp"

#include <algorithm>
#include <string>

#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureManager.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_rendering/render_system.hpp>

#include <pluginlib/class_list_macros.hpp>
namespace thermocator {
ThermalDisplay::ThermalDisplay() {
    // Properties are parented to `this` — the property panel in RViz2 owns them
    // SLOT macros fire when the user edits the value in the panel
    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 0.7f,
        "Transparency of the thermal overlay",
        this, SLOT(onAlphaChanged()));
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    cold_color_property_ = new rviz_common::properties::ColorProperty(
        "Cold Color", QColor(0, 0, 255),
        "Color mapped to minimum temperature (value = 0)",
        this, SLOT(onColorsChanged()));

    hot_color_property_ = new rviz_common::properties::ColorProperty(
        "Hot Color", QColor(255, 0, 0),
        "Color mapped to maximum temperature (value = 100)",
        this, SLOT(onColorsChanged()));
}

ThermalDisplay::~ThermalDisplay() {
    if (initialized()) {
        scene_manager_->destroyManualObject(manual_object_);
        child_node_->getParentSceneNode()->removeAndDestroyChild(child_node_);

        if (!texture_) {
            Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
        }
        if (!material_) {
            Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::onInitialize() {
    // Must call base first — sets up scene_node_, scene_manager_, context_
    MessageFilterDisplay::onInitialize();

    // Create a child scene node so we can move it independently
    child_node_ = scene_node_->createChildSceneNode();

    // Unique name per instance — multiple displays can coexist in one RViz2 session
    static int instance_count = 0;
    const std::string base_name =
        "ThermalDisplay_" + std::to_string(instance_count++);

    // ── Ogre ManualObject — will hold a single textured quad ─────────────────
    manual_object_ = scene_manager_->createManualObject(base_name + "_obj");
    manual_object_->setDynamic(true);
    child_node_->attachObject(manual_object_);

    // ── Material ──────────────────────────────────────────────────────────────
    material_ = Ogre::MaterialManager::getSingleton().create(
        base_name + "_mat",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    material_->setReceiveShadows(false);

    auto *technique = material_->getTechnique(0);
    technique->setLightingEnabled(false);

    auto *pass = technique->getPass(0);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);

    // Texture unit — will be bound to the texture when first message arrives
    pass->createTextureUnitState();
    pass->getTextureUnitState(0)->setTextureFiltering(Ogre::TFO_NONE);
}

void ThermalDisplay::reset() {
    MessageFilterDisplay::reset();
    manual_object_->clear();
    last_width_ = 0;
    last_height_ = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Property Slots
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::onAlphaChanged() {
    // Alpha change only affects pixel colors — no geometry rebuild needed
    // Will take effect on next message. If you want immediate update,
    // you would need to cache the last message and re-call updateTexture()
}

void ThermalDisplay::onColorsChanged() {
    // Same as above — next message picks up the new color scheme
}

// ─────────────────────────────────────────────────────────────────────────────
// Message Processing
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::processMessage(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    // ── Transform grid origin into the RViz2 fixed frame ─────────────────────
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    if (!context_->getFrameManager()->transform(
            msg->header,
            msg->info.origin,
            position,
            orientation)) {
        RCLCPP_WARN(rclcpp::get_logger("ThermalDisplay"),
                    "Could not transform from [%s] to fixed frame — skipping message",
                    msg->header.frame_id.c_str());
        return;
    }

    // ── Rebuild geometry only if grid dimensions changed ─────────────────────
    const bool dims_changed =
        msg->info.width != last_width_ ||
        msg->info.height != last_height_;

    if (dims_changed) {
        updatePlane(*msg);
        last_width_ = msg->info.width;
        last_height_ = msg->info.height;
    }

    // ── Always update texture — new data every message ────────────────────────
    updateTexture(*msg);

    // ── Place geometry at the grid origin in the scene ────────────────────────
    child_node_->setPosition(position);
    child_node_->setOrientation(orientation);
}

// ─────────────────────────────────────────────────────────────────────────────
// Texture Update
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::updateTexture(const nav_msgs::msg::OccupancyGrid &grid) {
    const uint32_t w = grid.info.width;
    const uint32_t h = grid.info.height;

    // ── Create or recreate texture when dimensions change ────────────────────
    if (texture_ ||
        texture_->getWidth() != w ||
        texture_->getHeight() != h) {
        if (!texture_) {
            Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
        }

        texture_ = Ogre::TextureManager::getSingleton().createManual(
            "ThermalTexture_" + std::to_string(w) + "x" + std::to_string(h),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D,
            w, h,
            0, // no mipmaps — we want exact pixel values
            Ogre::PF_BYTE_RGBA,
            Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        material_->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(texture_->getName());
    }

    // ── Read current property values ──────────────────────────────────────────
    const float alpha = alpha_property_->getFloat();
    const QColor cold = cold_color_property_->getColor();
    const QColor hot = hot_color_property_->getColor();
    const uint8_t a = static_cast<uint8_t>(alpha * 255.0f);

    // Precompute color channel deltas for the lerp
    const float dr = static_cast<float>(hot.red() - cold.red());
    const float dg = static_cast<float>(hot.green() - cold.green());
    const float db = static_cast<float>(hot.blue() - cold.blue());

    // ── Lock pixel buffer and fill ────────────────────────────────────────────
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);

    uint8_t *data =
        static_cast<uint8_t *>(pixel_buffer->getCurrentLock().data);

    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            // OccupancyGrid: row 0 = bottom of world
            // Ogre texture:  row 0 = top of image
            // → flip row index vertically
            const std::size_t grid_idx =
                static_cast<std::size_t>(row) * w + col;
            const std::size_t tex_row = h - 1 - row;
            const std::size_t pixel_idx = (tex_row * w + col) * 4;

            const int8_t value = grid.data[grid_idx];

            if (value == -1) {
                // Unknown cell — fully transparent
                data[pixel_idx + 0] = 128;
                data[pixel_idx + 1] = 128;
                data[pixel_idx + 2] = 128;
                data[pixel_idx + 3] = 0;
            } else {
                // Normalise [0, 100] → [0.0, 1.0] and lerp between cold and hot
                const float t = static_cast<float>(value) / 100.0f;

                data[pixel_idx + 0] =
                    static_cast<uint8_t>(static_cast<float>(cold.red()) + t * dr);
                data[pixel_idx + 1] =
                    static_cast<uint8_t>(static_cast<float>(cold.green()) + t * dg);
                data[pixel_idx + 2] =
                    static_cast<uint8_t>(static_cast<float>(cold.blue()) + t * db);
                data[pixel_idx + 3] = a;
            }
        }
    }

    pixel_buffer->unlock();
}

// ─────────────────────────────────────────────────────────────────────────────
// Plane Geometry
// ─────────────────────────────────────────────────────────────────────────────

void ThermalDisplay::updatePlane(const nav_msgs::msg::OccupancyGrid &grid) {
    const float w = static_cast<float>(grid.info.width) * grid.info.resolution;
    const float h = static_cast<float>(grid.info.height) * grid.info.resolution;

    manual_object_->clear();
    manual_object_->begin(
        material_->getName(),
        Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Single quad covering the full grid extent
    // Origin (0,0) is the bottom-left corner — matches OccupancyGrid origin
    // UV coordinates are flipped to account for the texture vertical flip above
    //
    //  3 ──────── 2
    //  │  (grid)  │
    //  0 ──────── 1
    //
    manual_object_->position(0.0f, 0.0f, 0.0f);
    manual_object_->textureCoord(0.0f, 1.0f);
    manual_object_->position(w, 0.0f, 0.0f);
    manual_object_->textureCoord(1.0f, 1.0f);
    manual_object_->position(w, h, 0.0f);
    manual_object_->textureCoord(1.0f, 0.0f);
    manual_object_->position(0.0f, h, 0.0f);
    manual_object_->textureCoord(0.0f, 0.0f);

    manual_object_->triangle(0, 1, 2);
    manual_object_->triangle(0, 2, 3);

    manual_object_->end();
};
} // namespace thermocator
// ── pluginlib registration ────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(thermocator::ThermalDisplay, rviz_common::Display)
