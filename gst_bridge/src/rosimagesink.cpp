/* GStreamer
 * Copyright (C) 2020 FIXME <fixme@example.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
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
static void rosimagesink_dispose (GObject * object);  //unused?
static void rosimagesink_finalize (GObject * object);


static GstStateChangeReturn rosimagesink_change_state (GstElement * element, GstStateChange transition);
static void rosimagesink_init (Rosimagesink * rosimagesink);
static gboolean rosimagesink_setcaps (GstBaseSink * sink, GstCaps * caps);
static GstCaps * rosimagesink_fixate (GstBaseSink * bsink, GstCaps * caps);

static GstFlowReturn rosimagesink_render (GstBaseSink * sink, GstBuffer * buffer);

static gboolean rosimagesink_open (Rosimagesink * sink);
static gboolean rosimagesink_close (Rosimagesink * sink);

// create a member function that sends a ROS message, call it from render

/*
  provide a mechanism for ROS to provide a clock
*/


enum
{
  PROP_0,
  PROP_ROS_NAME,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_ENCODING
};


/* pad templates */

static GstStaticPadTemplate rosimagesink_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw, "
        "format = (string) RGBA, "
        "framerate = (fraction) [1,max], "
        "width = (int) [1,max], "
        "height = (int) [1,max]"
        )
    );
    
    /*
        XXX much testing is required here to develop a mapping between
          sensor_msgs/image_encodings.h and
          gst/video/video-format.h
        at minimum, the formats should be:
        "format= {RGB, BGR, RGBA, BGRA}, "
        and the bit depths should be automatically set
    */
/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rosimagesink, rosimagesink, GST_TYPE_BASE_SINK,
    GST_DEBUG_CATEGORY_INIT (rosimagesink_debug_category, "rosimagesink", 0,
        "debug category for rosimagesink element"))

static void rosimagesink_class_init (RosimagesinkClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSinkClass *basesink_class = (GstBaseSinkClass *) klass;

  object_class->set_property = rosimagesink_set_property;
  object_class->get_property = rosimagesink_get_property;
  object_class->dispose = rosimagesink_dispose;
  object_class->finalize = rosimagesink_finalize;


  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (element_class,
      &rosimagesink_sink_template);


  gst_element_class_set_static_metadata (element_class,
      "rosimagesink",
      "Sink",
      "a gstreamer sink that publishes image data into ROS",
      "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property (object_class, PROP_ROS_NAME,
      g_param_spec_string ("ros-name", "node-name", "Name of the ROS node",
      "gst_image_sink_node",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_TOPIC,
      g_param_spec_string ("ros-topic", "pub-topic", "ROS topic to be published on",
      "gst_image_pub",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_ENCODING,
      g_param_spec_string ("ros-encoding", "encoding-string", "A hack to flexibly set the encoding string",
      "rgba8",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  element_class->change_state = GST_DEBUG_FUNCPTR (rosimagesink_change_state); //use state change events to open and close publishers
  basesink_class->fixate = GST_DEBUG_FUNCPTR (rosimagesink_fixate); //set caps fields to our preferred values (if possible)
  basesink_class->set_caps = GST_DEBUG_FUNCPTR (rosimagesink_setcaps);  //gstreamer informs us what caps we're using.
  //basesink_class->event = GST_DEBUG_FUNCPTR (rosimagesink_event);  //flush events can cause discontinuities (flags exist in buffers)
  //basesink_class->wait_event = GST_DEBUG_FUNCPTR (rosimagesink_wait_event); //eos events, finish rendering the output then return
  //basesink_class->get_times = GST_DEBUG_FUNCPTR (rosimagesink_get_times); //asks us for start and stop times (?)
  //basesink_class->preroll = GST_DEBUG_FUNCPTR (rosimagesink_preroll); //hands us the first buffer
  basesink_class->render = GST_DEBUG_FUNCPTR (rosimagesink_render); // gives us a buffer to forward
  //basesink_class->activate_pull = GST_DEBUG_FUNCPTR (rosimagesink_activate_pull);  //lets the sink drive the pipeline scheduling (useful for synchronising a file into a rosbag playback)

}

static void rosimagesink_init (Rosimagesink * rosimagesink)
{
  // Don't register the node or the publisher just yet,
  // wait for rosimagesink_open()
  // XXX set defaults elsewhere to keep gst-inspect consistent
  rosimagesink->node_name = g_strdup("gst_image_sink_node");
  rosimagesink->pub_topic = g_strdup("gst_image_pub");
  rosimagesink->frame_id = g_strdup("image_frame");
  rosimagesink->encoding = g_strdup("rgba8");
}

void rosimagesink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  Rosimagesink *rosimagesink = GST_ROSIMAGESINK (object);

  GST_DEBUG_OBJECT (rosimagesink, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if(rosimagesink->node)
      {
        RCLCPP_ERROR(rosimagesink->logger, "can't change node name once openned");
      }
      else
      {
        g_free(rosimagesink->node_name);
        rosimagesink->node_name = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_TOPIC:
      if(rosimagesink->node)
      {
        RCLCPP_ERROR(rosimagesink->logger, "can't change topic name once openned");
      }
      else
      {
        g_free(rosimagesink->pub_topic);
        rosimagesink->pub_topic = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_FRAME_ID:
      g_free(rosimagesink->frame_id);
      rosimagesink->frame_id = g_value_dup_string(value);
      break;

    case PROP_ROS_ENCODING:
      g_free(rosimagesink->encoding);
      rosimagesink->encoding = g_value_dup_string(value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosimagesink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  Rosimagesink *rosimagesink = GST_ROSIMAGESINK (object);

  GST_DEBUG_OBJECT (rosimagesink, "get_property");
  switch (property_id) {
    case PROP_ROS_NAME:
      g_value_set_string(value, rosimagesink->node_name);
      break;

    case PROP_ROS_TOPIC:
      g_value_set_string(value, rosimagesink->pub_topic);
      break;

    case PROP_ROS_FRAME_ID:
      g_value_set_string(value, rosimagesink->frame_id);
      break;

    case PROP_ROS_ENCODING:
      g_value_set_string(value, rosimagesink->encoding);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosimagesink_dispose (GObject * object)
{
  Rosimagesink *rosimagesink = GST_ROSIMAGESINK (object);

  GST_DEBUG_OBJECT (rosimagesink, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (rosimagesink_parent_class)->dispose (object);
}

void rosimagesink_finalize (GObject * object)
{
  Rosimagesink *rosimagesink = GST_ROSIMAGESINK (object);

  GST_DEBUG_OBJECT (rosimagesink, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (rosimagesink_parent_class)->finalize (object);
}


static GstStateChangeReturn rosimagesink_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  Rosimagesink *sink = GST_ROSIMAGESINK (element);

  switch (transition)
  {
    case GST_STATE_CHANGE_NULL_TO_READY:
    {
      if (!rosimagesink_open(sink))
      {
        GST_DEBUG_OBJECT (sink, "open failed");
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (rosimagesink_parent_class)->change_state (element, transition);

  switch (transition)
  {
    case GST_STATE_CHANGE_READY_TO_NULL:
      rosimagesink_close(sink);
      break;
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  return ret;

}

/* open the device with given specs */
static gboolean rosimagesink_open (Rosimagesink * sink)
{
  GST_DEBUG_OBJECT (sink, "open");

  sink->ros_context = std::make_shared<rclcpp::Context>();
  sink->ros_context->init(0, NULL);    // XXX should expose the init arg list
  rclcpp::NodeOptions opts = rclcpp::NodeOptions();
  opts.context(sink->ros_context); //set a context to generate the node in
  sink->node = std::make_shared<rclcpp::Node>(std::string(sink->node_name), opts);

  sink->pub = sink->node->create_publisher<sensor_msgs::msg::Image>(sink->pub_topic, 1);
  sink->logger = sink->node->get_logger();
  sink->clock = sink->node->get_clock();
  return TRUE;
}

/* close the device */
static gboolean rosimagesink_close (Rosimagesink * sink)
{
  GST_DEBUG_OBJECT (sink, "close");

  sink->clock.reset();
  sink->pub.reset();
  sink->node.reset();
  sink->ros_context->shutdown("gst closing rosimagesink");
  return TRUE;
}


static GstCaps * rosimagesink_fixate (GstBaseSink * bsink, GstCaps * caps)
{
  GstStructure *s;
  gint width, depth;

  caps = gst_caps_make_writable (caps);

  s = gst_caps_get_structure (caps, 0);

  /* fields for all formats */
  //gst_structure_fixate_field_nearest_int (s, "framerate", 25);
  gst_structure_fixate_field_nearest_int (s, "width", 640);
  gst_structure_fixate_field_nearest_int (s, "height", 480);

  /* fields for int */
  if (gst_structure_has_field (s, "depth")) {
    gst_structure_get_int (s, "width", &width);
    /* round width to nearest multiple of 8 for the depth */
    depth = GST_ROUND_UP_8 (width);
    gst_structure_fixate_field_nearest_int (s, "depth", depth);
  }
  if (gst_structure_has_field (s, "signed"))
    gst_structure_fixate_field_boolean (s, "signed", TRUE);
  if (gst_structure_has_field (s, "endianness"))
    gst_structure_fixate_field_nearest_int (s, "endianness", G_BYTE_ORDER);

  caps = GST_BASE_SINK_CLASS (rosimagesink_parent_class)->fixate (bsink, caps);

  return caps;
}


/* check the caps, register a node and open an publisher */
static gboolean rosimagesink_setcaps (GstBaseSink * sink, GstCaps * caps)
{
  GstStructure *caps_struct;
  gint width, height, depth, endianness, rate_num, rate_den;
  const gchar * format_str;
  GstVideoFormat format_enum;
  const GstVideoFormatInfo * format_info;

  Rosimagesink *rosimagesink = GST_ROSIMAGESINK (sink);

  GST_DEBUG_OBJECT (rosimagesink, "setcaps");

  if(!gst_caps_is_fixed(caps))
  {
    RCLCPP_ERROR(rosimagesink->logger, "caps is not fixed");
  }


  if(rosimagesink->node)
      RCLCPP_INFO(rosimagesink->logger, "preparing video with caps '%s'",
          gst_caps_to_string(caps));

  caps_struct = gst_caps_get_structure (caps, 0);
  if(!gst_structure_get_int (caps_struct, "width", &width))
      RCLCPP_ERROR(rosimagesink->logger, "setcaps missing width");
  if(!gst_structure_get_int (caps_struct, "height", &height))
      RCLCPP_ERROR(rosimagesink->logger, "setcaps missing height");
  if(!gst_structure_get_fraction (caps_struct, "framerate", &rate_num, &rate_den))
      RCLCPP_ERROR(rosimagesink->logger, "setcaps missing framerate");

  format_str = gst_structure_get_string(caps_struct, "format");

  if(format_str)
  {
    format_enum = gst_video_format_from_string (format_str);
    format_info = gst_video_format_get_info (format_enum);
    // XXX collect a bunch more info from format_info
    format_info = gst_video_format_get_info (format_info->unpack_format);
    
    depth = format_info->pixel_stride[0];
    
    RCLCPP_INFO(rosimagesink->logger, "setcaps format string is %s ", format_str);
    RCLCPP_INFO(rosimagesink->logger, "setcaps n_components is %d", format_info->n_components);
    RCLCPP_INFO(rosimagesink->logger, "setcaps bits is %d", format_info->bits);
    RCLCPP_INFO(rosimagesink->logger, "setcaps pixel_stride is %d", depth);

    if(format_info->bits < 8)
    {
      depth = depth/8;
      RCLCPP_ERROR(rosimagesink->logger, "low bits per pixel");
    }

    //endianness = format_info->endianness;
  }
  else
  {
    RCLCPP_ERROR(rosimagesink->logger, "setcaps missing format");
    if(!gst_structure_get_int (caps_struct, "endianness", &endianness))
        RCLCPP_ERROR(rosimagesink->logger, "setcaps missing endianness");
  }


  //collect a bunch of parameters to shoehorn into a message format
  rosimagesink->width = width;
  rosimagesink->height = height;
  rosimagesink->step = width * depth; //full row step size in bytes
  rosimagesink->endianness = endianness;
  //rosimagesink->sample_rate = rate;

  return true;
}



static GstFlowReturn rosimagesink_render (GstBaseSink * sink, GstBuffer * buf)
{
  GstMapInfo info;
  GstClockTime time;
  uint8_t* data;
  size_t length;
  auto msg = sensor_msgs::msg::Image();
  GstFlowReturn ret;

  Rosimagesink *rosimagesink = GST_ROSIMAGESINK (sink);

  //GST_DEBUG_OBJECT (rosimagesink, "render");

  time = GST_BUFFER_PTS (buf);    //XXX link gst clock to ros clock

  gst_buffer_map (buf, &info, GST_MAP_READ);
  length = info.size;
  data = info.data;
  msg.data.resize(length);
  memcpy(msg.data.data(), data, length);
  gst_buffer_unmap (buf, &info);
  data = NULL;

  //create a message (this loan should be extended upstream)
  // need to use fixed data length message to benefit from zero-copy
  
  //auto msg = rosimagesink->pub->borrow_loaned_message();
  //msg.get().frames = 

  //fill the blanks
  msg.width = rosimagesink->width;
  msg.height = rosimagesink->height;
  msg.encoding = rosimagesink->encoding;
  msg.is_bigendian = (rosimagesink->endianness == G_BIG_ENDIAN);
  msg.step = rosimagesink->step;
  
  msg.header.stamp = rosimagesink->clock->now();
  msg.header.frame_id = rosimagesink->frame_id;

  //publish
  rosimagesink->pub->publish(msg);

  return GST_FLOW_OK;
}
