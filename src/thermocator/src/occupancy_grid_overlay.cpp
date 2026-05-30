#include "thermocator/occupancy_grid_overlay.hpp"

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

OccupancyGridOverlay::OccupancyGridOverlay(std::string default_topic, RgbaDefault cold, RgbaDefault hot)
    : _default_topic(std::move(default_topic)), _default_cold(cold), _default_hot(hot) {
    _cold_color_property = new rviz_common::properties::ColorProperty(
        "Cold Color", _default_cold.color,
        "Color at occupancy value 0",
        this, SLOT(onPropertyChanged()));

    _cold_alpha_property = new rviz_common::properties::FloatProperty(
        "Cold Alpha", _default_cold.alpha,
        "Opacity at occupancy value 0  (0 = transparent, 1 = opaque)",
        this, SLOT(onPropertyChanged()));
    _cold_alpha_property->setMin(0.0f);
    _cold_alpha_property->setMax(1.0f);

    // Hot endpoint group
    _hot_color_property = new rviz_common::properties::ColorProperty(
        "Hot Color", _default_hot.color,
        "Color at occupancy value 100",
        this, SLOT(onPropertyChanged()));

    _hot_alpha_property = new rviz_common::properties::FloatProperty(
        "Hot Alpha", _default_hot.alpha,
        "Opacity at occupancy value 100  (0 = transparent, 1 = opaque)",
        this, SLOT(onPropertyChanged()));
    _hot_alpha_property->setMin(0.0f);
    _hot_alpha_property->setMax(1.0f);
}

OccupancyGridOverlay::~OccupancyGridOverlay() {
    if (initialized()) {
        scene_manager_->destroyManualObject(_manual_object);
        _child_node->getParentSceneNode()->removeAndDestroyChild(_child_node);

        if (_texture)
            Ogre::TextureManager::getSingleton().remove(_texture->getHandle());
        if (_material)
            Ogre::MaterialManager::getSingleton().remove(_material->getHandle());

        Ogre::ResourceGroupManager::getSingleton().destroyResourceGroup(
            _resource_group_name);
    }
}

void OccupancyGridOverlay::onInitialize() {
    MessageFilterDisplay::onInitialize();

    if (!scene_manager_ || !scene_node_) {
        RCLCPP_FATAL(_logger, "scene_manager_ or scene_node_ is null -- aborting");
        return;
    }

    static int instance_count = 0;
    _base_name = "OccupancyGridOverlay_" + std::to_string(instance_count++);
    _logger = rclcpp::get_logger(_base_name);

    RCLCPP_INFO(_logger, "Initializing instance: %s", _base_name.c_str());

    _resource_group_name = _base_name + "_group";
    Ogre::ResourceGroupManager::getSingleton().createResourceGroup(
        _resource_group_name);

    _child_node = scene_node_->createChildSceneNode();
    _manual_object = scene_manager_->createManualObject(_base_name + "_obj");
    _manual_object->setDynamic(true);
    _child_node->attachObject(_manual_object);

    _material = Ogre::MaterialManager::getSingleton().create(
        _base_name + "_mat", _resource_group_name);
    _material->setReceiveShadows(false);

    auto *technique = _material->getTechnique(0);
    technique->setLightingEnabled(false);

    auto *pass = technique->getPass(0);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->createTextureUnitState();
    pass->getTextureUnitState(0)->setTextureFiltering(Ogre::TFO_NONE);

    qos_profile = rclcpp::QoS(1).transient_local().reliable();
    if (!_default_topic.empty()) {
        topic_property_->setValue(QString::fromStdString(_default_topic));
    }

    RCLCPP_INFO(_logger, "Initialized -- waiting for messages");
}

void OccupancyGridOverlay::reset() {
    MessageFilterDisplay::reset();
    if (_manual_object)
        _manual_object->clear();
    _last_width = 0;
    _last_height = 0;
}

void OccupancyGridOverlay::onPropertyChanged() {}

void OccupancyGridOverlay::processMessage(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    if (!context_->getFrameManager()->transform(
            msg->header, msg->info.origin, position, orientation)) {
        RCLCPP_WARN(_logger,
                    "Could not transform frame [%s] -- skipping",
                    msg->header.frame_id.c_str());
        return;
    }

    const bool dims_changed =
        msg->info.width != _last_width ||
        msg->info.height != _last_height;

    if (dims_changed) {
        updatePlane(*msg);
        _last_width = msg->info.width;
        _last_height = msg->info.height;
    }

    updateTexture(*msg);

    _child_node->setPosition(position);
    _child_node->setOrientation(orientation);
}

void OccupancyGridOverlay::updateTexture(
    const nav_msgs::msg::OccupancyGrid &grid) {
    const uint32_t w = grid.info.width;
    const uint32_t h = grid.info.height;

    if (!_texture ||
        _texture->getWidth() != w ||
        _texture->getHeight() != h) {

        if (_texture)
            Ogre::TextureManager::getSingleton().remove(_texture->getHandle());

        _texture = Ogre::TextureManager::getSingleton().createManual(
            _base_name + "_tex",
            _resource_group_name,
            Ogre::TEX_TYPE_2D,
            w, h, 0,
            Ogre::PF_BYTE_RGBA,
            Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        _material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName(_texture->getName());
    }

    const QColor cold_color = _cold_color_property->getColor();
    const QColor hot_color = _hot_color_property->getColor();
    const float cold_alpha = _cold_alpha_property->getFloat();
    const float hot_alpha = _hot_alpha_property->getFloat();

    const float cr = static_cast<float>(cold_color.red());
    const float cg = static_cast<float>(cold_color.green());
    const float cb = static_cast<float>(cold_color.blue());
    const float ca = cold_alpha * 255.0f;

    const float dr = static_cast<float>(hot_color.red()) - cr;
    const float dg = static_cast<float>(hot_color.green()) - cg;
    const float db = static_cast<float>(hot_color.blue()) - cb;
    const float da = hot_alpha * 255.0f - ca;

    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = _texture->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    uint8_t *data =
        static_cast<uint8_t *>(pixel_buffer->getCurrentLock().data);

    if (!data) {
        RCLCPP_ERROR(_logger, "Pixel buffer lock returned null");
        pixel_buffer->unlock();
        return;
    }

    for (uint32_t row = 0; row < h; ++row) {
        for (uint32_t col = 0; col < w; ++col) {
            const size_t idx = static_cast<size_t>(row) * w + col;
            const size_t pix = idx * 4;
            const int8_t val = grid.data[idx];

            if (val == -1) {
                data[pix + 0] = 0;
                data[pix + 1] = 0;
                data[pix + 2] = 0;
                data[pix + 3] = 0;
            } else {
                const float t = static_cast<float>(val) / 100.0f;
                data[pix + 0] = static_cast<uint8_t>(cr + t * dr);
                data[pix + 1] = static_cast<uint8_t>(cg + t * dg);
                data[pix + 2] = static_cast<uint8_t>(cb + t * db);
                data[pix + 3] = static_cast<uint8_t>(ca + t * da);
            }
        }
    }

    pixel_buffer->unlock();
}

void OccupancyGridOverlay::updatePlane(
    const nav_msgs::msg::OccupancyGrid &grid) {
    const float w =
        static_cast<float>(grid.info.width) * grid.info.resolution;
    const float h =
        static_cast<float>(grid.info.height) * grid.info.resolution;

    _manual_object->clear();
    _manual_object->begin(
        _material->getName(),
        Ogre::RenderOperation::OT_TRIANGLE_LIST,
        _resource_group_name);

    _manual_object->position(0.0f, 0.0f, 0.0f);
    _manual_object->textureCoord(0.0f, 0.0f);
    _manual_object->position(w, 0.0f, 0.0f);
    _manual_object->textureCoord(1.0f, 0.0f);
    _manual_object->position(w, h, 0.0f);
    _manual_object->textureCoord(1.0f, 1.0f);
    _manual_object->position(0.0f, h, 0.0f);
    _manual_object->textureCoord(0.0f, 1.0f);

    _manual_object->triangle(0, 1, 2);
    _manual_object->triangle(0, 2, 3);
    _manual_object->end();
}

} // namespace thermocator

PLUGINLIB_EXPORT_CLASS(thermocator::ThermalOverlay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(thermocator::ActionOverlay, rviz_common::Display)
