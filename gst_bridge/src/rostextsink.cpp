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
 * SECTION:element-gstrostextsink
 *
 * The rostextsink element, pipe text data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v audiotestsrc ! rostextsink node_name="gst_text" topic="/audiotopic"
 * ]|
 * Streams test tones as ROS audio messages on topic.
 * </refsect2>
 */


#include <gst_bridge/rostextsink.h>


GST_DEBUG_CATEGORY_STATIC (rostextsink_debug_category);
#define GST_CAT_DEFAULT rostextsink_debug_category

/* prototypes */


static void rostextsink_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rostextsink_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static void rostextsink_init (Rostextsink * sink);

static gboolean rostextsink_open (RosBaseSink * ros_base_sink);
static gboolean rostextsink_close (RosBaseSink * ros_base_sink);

static GstFlowReturn rostextsink_render (RosBaseSink * sink,
  GstBuffer * buffer, rclcpp::Time msg_time);

enum
{
  PROP_0,
  PROP_ROS_TOPIC,
  // XXX we can change the publisher type here to select a more complete ros message type
};


/* pad templates */

static GstStaticPadTemplate rostextsink_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (ROS_TEXT_MSG_CAPS)
    );

/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rostextsink, rostextsink, GST_TYPE_ROS_BASE_SINK,
    GST_DEBUG_CATEGORY_INIT (rostextsink_debug_category, "rostextsink", 0,
        "debug category for rostextsink element"))

static void rostextsink_class_init (RostextsinkClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  //GstBaseSinkClass *basesink_class = GST_BASE_SINK_CLASS (klass);  //unused
  RosBaseSinkClass *ros_base_sink_class = GST_ROS_BASE_SINK_CLASS (klass);

  object_class->set_property = rostextsink_set_property;
  object_class->get_property = rostextsink_get_property;

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (element_class,
      &rostextsink_sink_template);

  gst_element_class_set_static_metadata (element_class,
      "rostextsink",
      "Sink",
      "a gstreamer sink that publishes text data into ROS",
      "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property (object_class, PROP_ROS_TOPIC,
      g_param_spec_string ("ros-topic", "pub-topic", "ROS topic to be published on",
      "gst_text_pub",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  //supply the calls ros base sink needs to negotiate upstream formats and manage the publisher
  ros_base_sink_class->open = GST_DEBUG_FUNCPTR (rostextsink_open);  //let the base sink know how we register publishers
  ros_base_sink_class->close = GST_DEBUG_FUNCPTR (rostextsink_close);  //let the base sink know how we destroy publishers
  ros_base_sink_class->render = GST_DEBUG_FUNCPTR (rostextsink_render); // gives us a buffer to package
}

static void rostextsink_init (Rostextsink * sink)
{
  RosBaseSink *ros_base_sink GST_ROS_BASE_SINK(sink);
  ros_base_sink->node_name = g_strdup("gst_text_sink_node");
  sink->pub_topic = g_strdup("gst_text_pub");
}

void rostextsink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  RosBaseSink *ros_base_sink = GST_ROS_BASE_SINK(object);
  Rostextsink *sink = GST_ROSTEXTSINK (object);

  GST_DEBUG_OBJECT (sink, "set_property");

  switch (property_id) {
    case PROP_ROS_TOPIC:
      if(ros_base_sink->node)
      {
        RCLCPP_ERROR(ros_base_sink->logger, "can't change topic name once opened");
        // XXX try harder
      }
      else
      {
        g_free(sink->pub_topic);
        sink->pub_topic = g_value_dup_string(value);
      }
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rostextsink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  Rostextsink *sink = GST_ROSTEXTSINK (object);

  GST_DEBUG_OBJECT (sink, "get_property");
  switch (property_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, sink->pub_topic);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}


/* open the device with given specs */
static gboolean rostextsink_open (RosBaseSink * ros_base_sink)
{
  Rostextsink *sink = GST_ROSTEXTSINK (ros_base_sink);
  GST_DEBUG_OBJECT (sink, "open");
  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();  //XXX add a parameter for overrides
  sink->pub = ros_base_sink->node->create_publisher<std_msgs::msg::String>(sink->pub_topic, qos);

  return TRUE;
}

/* close the device */
static gboolean rostextsink_close (RosBaseSink * ros_base_sink)
{
  Rostextsink *sink = GST_ROSTEXTSINK (ros_base_sink);
  GST_DEBUG_OBJECT (sink, "close");

  sink->pub.reset();

  return TRUE;
}


static GstFlowReturn rostextsink_render (RosBaseSink * ros_base_sink,
  GstBuffer * buf, rclcpp::Time msg_time)
{
  GstMapInfo info;
  std_msgs::msg::String msg;

  // XXX use borrowed messages, can buf be extended into the middleware?
  //    auto msg = sink->pub->borrow_loaned_message();
  //    msg.get().frames = ...

  Rostextsink *sink = GST_ROSTEXTSINK (ros_base_sink);
  GST_DEBUG_OBJECT (sink, "render");

  (void)msg_time;
  // XXX extend this to new ROS message types with at least headers
  //msg.header.stamp = msg_time;

  gst_buffer_map (buf, &info, GST_MAP_READ);
  msg.data=std::string(reinterpret_cast<char*>(info.data), info.size);

  if(GST_BUFFER_OFFSET_IS_VALID(buf))
  {
    //msg.seq_num = GST_BUFFER_OFFSET(buf);
    if(GST_BUFFER_OFFSET_END_IS_VALID(buf))
      sink->msg_seq_num = GST_BUFFER_OFFSET_END(buf);
    else
      sink->msg_seq_num = GST_BUFFER_OFFSET(buf) + info.size;
  }
  else
  {
    //msg.seq_num = sink->msg_seq_num;
    sink->msg_seq_num += info.size;
  }

  gst_buffer_unmap (buf, &info);

  //publish
  sink->pub->publish(msg);

  return GST_FLOW_OK;
}

