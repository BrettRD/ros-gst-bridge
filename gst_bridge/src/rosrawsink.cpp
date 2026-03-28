/* gst_bridge
 * Copyright (C) 2020-2021 Brett Downing <brettrd@brettrd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
/**
 * SECTION:element-gstrosrawsink
 *
 * The rosrawsink element, pipe raw data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 filesrc location=/bin/ls ! application/octet-stream ! rosrawsink
 * ]|
 * </refsect2>
 */
#include <gst_bridge/rosrawsink.h>

#include <memory>

GST_DEBUG_CATEGORY_STATIC(rosrawsink_debug_category);
#define GST_CAT_DEFAULT rosrawsink_debug_category

static void rosrawsink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosrawsink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec);
static void rosrawsink_init(Rosrawsink * sink);
static gboolean rosrawsink_open(RosBaseSink * ros_base_sink);
static gboolean rosrawsink_close(RosBaseSink * ros_base_sink);
static GstFlowReturn rosrawsink_render(
  RosBaseSink * sink, GstBuffer * buffer, rclcpp::Time msg_time);

enum { PROP_0, PROP_ROS_TOPIC };

static GstStaticPadTemplate rosrawsink_sink_template = GST_STATIC_PAD_TEMPLATE(
  "sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS("application/x-onvif-metadata"));

G_DEFINE_TYPE_WITH_CODE(
  Rosrawsink, rosrawsink, GST_TYPE_ROS_BASE_SINK,
  GST_DEBUG_CATEGORY_INIT(rosrawsink_debug_category, "rosrawsink", 0, "ROS raw sink debug"))

static void rosrawsink_class_init(RosrawsinkClass * klass)
{
  GObjectClass * object_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  RosBaseSinkClass * ros_base_sink_class = GST_ROS_BASE_SINK_CLASS(klass);

  object_class->set_property = rosrawsink_set_property;
  object_class->get_property = rosrawsink_get_property;

  gst_element_class_add_static_pad_template(element_class, &rosrawsink_sink_template);

  gst_element_class_set_static_metadata(
    element_class, "rosrawsink", "Sink", "Publishes raw byte data to ROS",
    "Guilherme Rodrigues <guilherme.rodrigues@ait.ac.at>");

  g_object_class_install_property(
    object_class, PROP_ROS_TOPIC,
    g_param_spec_string(
      "ros-topic", "ROS Topic", "ROS topic to publish raw data on", "raw_data",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  ros_base_sink_class->open = GST_DEBUG_FUNCPTR(rosrawsink_open);
  ros_base_sink_class->close = GST_DEBUG_FUNCPTR(rosrawsink_close);
  ros_base_sink_class->render = GST_DEBUG_FUNCPTR(rosrawsink_render);
}

static void rosrawsink_init(Rosrawsink * sink)
{
  RosBaseSink * ros_base_sink = GST_ROS_BASE_SINK(sink);
  ros_base_sink->node_name = g_strdup("gst_raw_sink_node");
  sink->pub_topic = g_strdup("gst_raw_pub");
}

static void rosrawsink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec)
{
  Rosrawsink * sink = GST_ROSRAWSINK(object);
  RosBaseSink * ros_base_sink = GST_ROS_BASE_SINK(object);

  switch (property_id) {
    case PROP_ROS_TOPIC:
      if (ros_base_sink->node_if) {
        RCLCPP_ERROR(
          ros_base_sink->node_if->logging->get_logger(), "Can't change topic name after startup");
      } else {
        g_free(sink->pub_topic);
        sink->pub_topic = g_value_dup_string(value);
      }
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

static void rosrawsink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec)
{
  Rosrawsink * sink = GST_ROSRAWSINK(object);

  switch (property_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, sink->pub_topic);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

static gboolean rosrawsink_open(RosBaseSink * ros_base_sink)
{
  GST_DEBUG_OBJECT(ros_base_sink, "open");

  Rosrawsink * sink = GST_ROSRAWSINK(ros_base_sink);
  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  sink->pub = rclcpp::create_publisher<Rosrawsink::MsgType>(
    ros_base_sink->node_if->parameters, ros_base_sink->node_if->topics, sink->pub_topic, qos);

  return TRUE;
}

static gboolean rosrawsink_close(RosBaseSink * ros_base_sink)
{
  Rosrawsink * sink = GST_ROSRAWSINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "close");
  sink->pub.reset();
  return TRUE;
}

static GstFlowReturn rosrawsink_render(RosBaseSink * ros_base_sink, GstBuffer * buf, rclcpp::Time msg_time)
{
  Rosrawsink * sink = GST_ROSRAWSINK(ros_base_sink);
  Rosrawsink::MsgType msg;
  GstMapInfo info;

  if (!gst_buffer_map(buf, &info, GST_MAP_READ)) {
    GST_ERROR_OBJECT(sink, "Failed to map GstBuffer");
    return GST_FLOW_ERROR;
  }

  msg.header.stamp = msg_time;
  msg.data = std::vector<uint8_t>(info.data, info.data + info.size);
  gst_buffer_unmap(buf, &info);

  sink->pub->publish(msg);

  return GST_FLOW_OK;
}
