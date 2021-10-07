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
 * SECTION:element-gstrosimagesink
 *
 * The rosimagesink element pipe video data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v videotestsrc ! rosimagesink node_name="gst_image" topic="/imagetopic"
 * ]|
 * Streams test tones as ROS image messages on topic.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst_bridge/rosimagesink.h>


GST_DEBUG_CATEGORY_STATIC (rosimagesink_debug_category);
#define GST_CAT_DEFAULT rosimagesink_debug_category

/* prototypes */


static void rosimagesink_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosimagesink_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static void rosimagesink_init (Rosimagesink * sink);

static gboolean rosimagesink_open (RosBaseSink * sink);
static gboolean rosimagesink_close (RosBaseSink * sink);
static gboolean rosimagesink_setcaps (GstBaseSink * gst_base_sink, GstCaps * caps);
static GstFlowReturn rosimagesink_render (RosBaseSink * base_sink, GstBuffer * buffer, rclcpp::Time msg_time);

enum
{
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_ENCODING,
};


/* pad templates */

static GstStaticPadTemplate rosimagesink_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (ROS_IMAGE_MSG_CAPS)
    );

/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rosimagesink, rosimagesink, GST_TYPE_ROS_BASE_SINK,
    GST_DEBUG_CATEGORY_INIT (rosimagesink_debug_category, "rosimagesink", 0,
        "debug category for rosimagesink element"))

static void rosimagesink_class_init (RosimagesinkClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSinkClass *basesink_class = GST_BASE_SINK_CLASS (klass);
  RosBaseSinkClass *ros_base_sink_class = GST_ROS_BASE_SINK_CLASS (klass);

  object_class->set_property = rosimagesink_set_property;
  object_class->get_property = rosimagesink_get_property;

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (element_class,
      &rosimagesink_sink_template);


  gst_element_class_set_static_metadata (element_class,
      "rosimagesink",
      "Sink",
      "a gstreamer sink that publishes image data into ROS",
      "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property (object_class, PROP_ROS_TOPIC,
      g_param_spec_string ("ros-topic", "pub-topic", "ROS topic to be published on",
      "gst_image_pub",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_FRAME_ID,
      g_param_spec_string ("ros-frame-id", "frame-id", "frame_id of the image message",
      "",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_ENCODING,
      g_param_spec_string ("ros-encoding", "encoding-string", "A hack to flexibly set the encoding string",
      "",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  //access gstreamer base sink events here
  basesink_class->set_caps = GST_DEBUG_FUNCPTR (rosimagesink_setcaps);  //gstreamer informs us what caps we're using.

  //supply the calls ros base sink needs to negotiate upstream formats and manage the publisher
  ros_base_sink_class->open = GST_DEBUG_FUNCPTR (rosimagesink_open);  //let the base sink know how we register publishers
  ros_base_sink_class->close = GST_DEBUG_FUNCPTR (rosimagesink_close);  //let the base sink know how we destroy publishers
  ros_base_sink_class->render = GST_DEBUG_FUNCPTR (rosimagesink_render); // gives us a buffer to package
}

static void rosimagesink_init (Rosimagesink * sink)
{
  RosBaseSink *ros_base_sink GST_ROS_BASE_SINK(sink);
  ros_base_sink->node_name = g_strdup("gst_image_sink_node");
  sink->pub_topic = g_strdup("gst_image_pub");
  sink->frame_id = g_strdup("image_frame");
  sink->encoding = g_strdup("");
  sink->init_caps =  g_strdup("");
}

void rosimagesink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  RosBaseSink *ros_base_sink = GST_ROS_BASE_SINK(object);
  Rosimagesink *sink = GST_ROSIMAGESINK (object);

  GST_DEBUG_OBJECT (sink, "set_property");

  switch (property_id) {
    case PROP_ROS_TOPIC:
      if(ros_base_sink->node)
      {
        RCLCPP_ERROR(ros_base_sink->logger, "can't change topic name once opened");
      }
      else
      {
        g_free(sink->pub_topic);
        sink->pub_topic = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_FRAME_ID:
      g_free(sink->frame_id);
      sink->frame_id = g_value_dup_string(value);
      break;

    case PROP_ROS_ENCODING:
      g_free(sink->encoding);
      sink->encoding = g_value_dup_string(value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosimagesink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  Rosimagesink *sink = GST_ROSIMAGESINK (object);

  GST_DEBUG_OBJECT (sink, "get_property");
  switch (property_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, sink->pub_topic);
      break;

    case PROP_ROS_FRAME_ID:
      g_value_set_string(value, sink->frame_id);
      break;

    case PROP_ROS_ENCODING:
      g_value_set_string(value, sink->encoding);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

/* open the device with given specs */
static gboolean rosimagesink_open (RosBaseSink * ros_base_sink)
{
  Rosimagesink *sink = GST_ROSIMAGESINK (ros_base_sink);
  GST_DEBUG_OBJECT (sink, "open");
  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();  //XXX add a parameter for overrides
  sink->pub = ros_base_sink->node->create_publisher<sensor_msgs::msg::Image>(sink->pub_topic, qos);
  return TRUE;
}

/* close the device */
static gboolean rosimagesink_close (RosBaseSink * ros_base_sink)
{
  Rosimagesink *sink = GST_ROSIMAGESINK (ros_base_sink);
  GST_DEBUG_OBJECT (sink, "close");
  sink->pub.reset();
  return TRUE;
}

/* check the caps, register a node and open an publisher */
static gboolean rosimagesink_setcaps (GstBaseSink * gst_base_sink, GstCaps * caps)
{
  RosBaseSink *ros_base_sink = GST_ROS_BASE_SINK (gst_base_sink);
  Rosimagesink *sink = GST_ROSIMAGESINK (ros_base_sink);

  GstStructure *caps_struct;
  gint width, height, depth, endianness, rate_num, rate_den;
  const gchar * format_str;
  GstVideoFormat format_enum;
  const GstVideoFormatInfo * format_info;


  GST_DEBUG_OBJECT (sink, "setcaps");

  if(!gst_caps_is_fixed(caps))
  {
    RCLCPP_ERROR(ros_base_sink->logger, "caps is not fixed");
  }


  if(ros_base_sink->node)
      RCLCPP_INFO(ros_base_sink->logger, "preparing video with caps '%s'",
          gst_caps_to_string(caps));

  caps_struct = gst_caps_get_structure (caps, 0);
  if(!gst_structure_get_int (caps_struct, "width", &width))
      RCLCPP_ERROR(ros_base_sink->logger, "setcaps missing width");
  if(!gst_structure_get_int (caps_struct, "height", &height))
      RCLCPP_ERROR(ros_base_sink->logger, "setcaps missing height");
  if(!gst_structure_get_fraction (caps_struct, "framerate", &rate_num, &rate_den))
      RCLCPP_ERROR(ros_base_sink->logger, "setcaps missing framerate");

  format_str = gst_structure_get_string(caps_struct, "format");

  if(format_str)
  {
    format_enum = gst_video_format_from_string (format_str);
    format_info = gst_video_format_get_info (format_enum);
    depth = format_info->pixel_stride[0];

    //allow the encoding to be overridden by parameters
    //but update it if it's blank
    if(0 == g_strcmp0(sink->init_caps, ""))
    {
      g_free(sink->init_caps);
      sink->init_caps = gst_caps_to_string(caps);
    }
    if(0 == g_strcmp0(sink->encoding, ""))
    {
      g_free(sink->encoding);
      sink->encoding = g_strdup(gst_bridge::getRosEncoding(format_enum).c_str());
    }

    RCLCPP_INFO(ros_base_sink->logger, "setcaps format string is %s ", format_str);
    RCLCPP_INFO(ros_base_sink->logger, "setcaps n_components is %d", format_info->n_components);
    RCLCPP_INFO(ros_base_sink->logger, "setcaps bits is %d", format_info->bits);
    RCLCPP_INFO(ros_base_sink->logger, "setcaps pixel_stride is %d", depth);

    if(format_info->bits < 8)
    {
      depth = depth/8;
      RCLCPP_ERROR(ros_base_sink->logger, "low bits per pixel");
    }
    endianness = GST_VIDEO_FORMAT_INFO_IS_LE(format_info) ? G_LITTLE_ENDIAN : G_BIG_ENDIAN;
  }
  else
  {
    RCLCPP_ERROR(ros_base_sink->logger, "setcaps missing format");
    if(!gst_structure_get_int (caps_struct, "endianness", &endianness))
      RCLCPP_ERROR(ros_base_sink->logger, "setcaps missing endianness");
    return false;
  }


  //collect a bunch of parameters to shoehorn into a message format
  sink->width = width;
  sink->height = height;
  sink->step = width * depth; //full row step size in bytes
  sink->endianness = endianness;  // XXX used without init
  //sink->sample_rate = rate;

  return true;
}

static GstFlowReturn rosimagesink_render (RosBaseSink * ros_base_sink, GstBuffer * buf, rclcpp::Time msg_time)
{
  GstMapInfo info;
  sensor_msgs::msg::Image msg;

  Rosimagesink *sink = GST_ROSIMAGESINK (ros_base_sink);
  GST_DEBUG_OBJECT (sink, "render");

  msg.header.stamp = msg_time;
  msg.header.frame_id = sink->frame_id;

  //auto msg = sink->pub->borrow_loaned_message();
  //msg.get().width =

  //fill the blanks
  msg.width = sink->width;
  msg.height = sink->height;
  msg.encoding = sink->encoding;
  msg.is_bigendian = (sink->endianness == G_BIG_ENDIAN);
  msg.step = sink->step;

  gst_buffer_map (buf, &info, GST_MAP_READ);
  msg.data.assign(info.data, info.data+info.size);
  gst_buffer_unmap (buf, &info);

  //publish
  sink->pub->publish(msg);

  return GST_FLOW_OK;
}
