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
 * SECTION:element-gstrosaudiosink
 *
 * The rosaudiosink element, pipe audio data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v audiotestsrc ! rosaudiosink node_name="gst_audio" topic="/audiotopic"
 * ]|
 * Streams test tones as ROS audio messages on topic.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include "rosaudiosink.h"


GST_DEBUG_CATEGORY_STATIC (rosaudiosink_debug_category);
#define GST_CAT_DEFAULT rosaudiosink_debug_category

/* prototypes */


static void rosaudiosink_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);

static void rosaudiosink_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);
static void rosaudiosink_dispose (GObject * object);  //unused?
static void rosaudiosink_finalize (GObject * object);


static GstStateChangeReturn rosaudiosink_change_state (GstElement * element, GstStateChange transition);
static void rosaudiosink_init (Rosaudiosink * rosaudiosink);
static gboolean rosaudiosink_setcaps (GstBaseSink * sink, GstCaps * caps);
static GstCaps * rosaudiosink_fixate (GstBaseSink * bsink, GstCaps * caps);

static GstFlowReturn rosaudiosink_render (GstBaseSink * sink, GstBuffer * buffer);

static gboolean rosaudiosink_open (Rosaudiosink * sink);
static gboolean rosaudiosink_close (Rosaudiosink * sink);

/*
  XXX provide a mechanism for ROS to provide a clock
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

/* FIXME add/remove the formats that you want to support */
static GstStaticPadTemplate rosaudiosink_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("audio/x-raw,format=S16LE,rate=[1,max],"
        "channels=[1,max],layout=interleaved")
    );


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rosaudiosink, rosaudiosink, GST_TYPE_BASE_SINK,
    GST_DEBUG_CATEGORY_INIT (rosaudiosink_debug_category, "rosaudiosink", 0,
        "debug category for rosaudiosink element"))

static void rosaudiosink_class_init (RosaudiosinkClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSinkClass *basesink_class = (GstBaseSinkClass *) klass;

  object_class->set_property = rosaudiosink_set_property;
  object_class->get_property = rosaudiosink_get_property;
  object_class->dispose = rosaudiosink_dispose;
  object_class->finalize = rosaudiosink_finalize;


  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (element_class,
      &rosaudiosink_sink_template);


  gst_element_class_set_static_metadata (element_class,
      "rosaudiosink",
      "Sink",
      "a gstreamer sink that publishes audiodata into ROS",
      "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property (object_class, PROP_ROS_NAME,
      g_param_spec_string ("ros-name", "node-name", "Name of the ROS node",
      "gst_audio_sink_node",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_TOPIC,
      g_param_spec_string ("ros-topic", "pub-topic", "ROS topic to be published on",
      "gst_audio_pub",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_FRAME_ID,
      g_param_spec_string ("ros-frame-id", "frame-id", "frame_id of the image message",
      "",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_ENCODING,
      g_param_spec_string ("ros-encoding", "encoding-string", "A hack to flexibly set the encoding string",
      "16SC1",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  element_class->change_state = GST_DEBUG_FUNCPTR (rosaudiosink_change_state); //use state change events to open and close publishers
  basesink_class->fixate = GST_DEBUG_FUNCPTR (rosaudiosink_fixate); //set caps fields to our preferred values (if possible)
  basesink_class->set_caps = GST_DEBUG_FUNCPTR (rosaudiosink_setcaps);  //gstreamer informs us what caps we're using.
  //basesink_class->event = GST_DEBUG_FUNCPTR (rosaudiosink_event);  //flush events can cause discontinuities (flags exist in buffers)
  //basesink_class->wait_event = GST_DEBUG_FUNCPTR (rosaudiosink_wait_event); //eos events, finish rendering the output then return
  //basesink_class->get_times = GST_DEBUG_FUNCPTR (rosaudiosink_get_times); //asks us for start and stop times (?)
  //basesink_class->preroll = GST_DEBUG_FUNCPTR (rosaudiosink_preroll); //hands us the first buffer
  basesink_class->render = GST_DEBUG_FUNCPTR (rosaudiosink_render); // gives us a buffer to forward
  //basesink_class->activate_pull = GST_DEBUG_FUNCPTR (rosaudiosink_activate_pull);  //lets the sink drive the pipeline scheduling (useful for synchronising a file into a rosbag playback)

}

static void rosaudiosink_init (Rosaudiosink * rosaudiosink)
{
  // Don't register the node or the publisher just yet,
  // wait for rosaudiosink_open()
  // XXX set defaults elsewhere to keep gst-inspect consistent
  rosaudiosink->node_name = g_strdup("gst_audio_sink_node");
  rosaudiosink->pub_topic = g_strdup("gst_audio_pub");
  rosaudiosink->frame_id = g_strdup("audio_frame");
  rosaudiosink->encoding = g_strdup("16SC1");
}

void rosaudiosink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if(rosaudiosink->node)
      {
        RCLCPP_ERROR(rosaudiosink->logger, "can't change node name once openned");
      }
      else
      {
        g_free(rosaudiosink->node_name);
        rosaudiosink->node_name = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_TOPIC:
      if(rosaudiosink->node)
      {
        RCLCPP_ERROR(rosaudiosink->logger, "can't change topic name once openned");
      }
      else
      {
        g_free(rosaudiosink->pub_topic);
        rosaudiosink->pub_topic = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_FRAME_ID:
      g_free(rosaudiosink->frame_id);
      rosaudiosink->frame_id = g_value_dup_string(value);
      break;

    case PROP_ROS_ENCODING:
      g_free(rosaudiosink->encoding);
      rosaudiosink->encoding = g_value_dup_string(value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosaudiosink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "get_property");
  switch (property_id) {
    case PROP_ROS_NAME:
      g_value_set_string(value, rosaudiosink->node_name);
      break;

    case PROP_ROS_TOPIC:
      g_value_set_string(value, rosaudiosink->pub_topic);
      break;

    case PROP_ROS_FRAME_ID:
      g_value_set_string(value, rosaudiosink->frame_id);
      break;

    case PROP_ROS_ENCODING:
      g_value_set_string(value, rosaudiosink->encoding);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosaudiosink_dispose (GObject * object)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (rosaudiosink_parent_class)->dispose (object);
}

void rosaudiosink_finalize (GObject * object)
{
  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (rosaudiosink, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (rosaudiosink_parent_class)->finalize (object);
}


static GstStateChangeReturn rosaudiosink_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  Rosaudiosink *sink = GST_ROSAUDIOSINK (element);

  switch (transition)
  {
    case GST_STATE_CHANGE_NULL_TO_READY:
    {
      //gst_audio_clock_reset (GST_AUDIO_CLOCK (sink->provided_clock), 0);
      if (!rosaudiosink_open(sink))
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

  ret = GST_ELEMENT_CLASS (rosaudiosink_parent_class)->change_state (element, transition);

  switch (transition)
  {
    case GST_STATE_CHANGE_READY_TO_NULL:
      rosaudiosink_close(sink);
      break;
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  return ret;

}

/* open the device with given specs */
static gboolean rosaudiosink_open (Rosaudiosink * sink)
{
  GST_DEBUG_OBJECT (sink, "open");

  sink->ros_context = std::make_shared<rclcpp::Context>();
  sink->ros_context->init(0, NULL);    // XXX should expose the init arg list
  rclcpp::NodeOptions opts = rclcpp::NodeOptions();
  opts.context(sink->ros_context); //set a context to generate the node in
  sink->node = std::make_shared<rclcpp::Node>(std::string(sink->node_name), opts);

  sink->pub = sink->node->create_publisher<audio_msgs::msg::Audio>(sink->pub_topic, 1);
  sink->logger = sink->node->get_logger();
  sink->clock = sink->node->get_clock();
  return TRUE;
}

/* close the device */
static gboolean rosaudiosink_close (Rosaudiosink * sink)
{
  GST_DEBUG_OBJECT (sink, "close");

  sink->clock.reset();
  sink->pub.reset();
  sink->node.reset();
  sink->ros_context->shutdown("gst closing rosaudiosink");
  return TRUE;
}


static GstCaps * rosaudiosink_fixate (GstBaseSink * bsink, GstCaps * caps)
{
  GstStructure *s;
  gint width, depth;

  caps = gst_caps_make_writable (caps);

  s = gst_caps_get_structure (caps, 0);

  /* fields for all formats */
  gst_structure_fixate_field_nearest_int (s, "rate", 44100);
  gst_structure_fixate_field_nearest_int (s, "channels", 2);
  gst_structure_fixate_field_nearest_int (s, "width", 16);

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

  caps = GST_BASE_SINK_CLASS (rosaudiosink_parent_class)->fixate (bsink, caps);

  return caps;
}


/* check the caps, register a node and open an publisher */
static gboolean rosaudiosink_setcaps (GstBaseSink * sink, GstCaps * caps)
{
  GstStructure *caps_struct;
  gint width, depth, channels, endianness, rate, layout;
  const gchar * format_str;
  GstAudioFormat format_enum;
  const GstAudioFormatInfo * format_info;

  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "setcaps");

  if(!gst_caps_is_fixed(caps))
  {
    RCLCPP_ERROR(rosaudiosink->logger, "caps is not fixed");
  }


  if(rosaudiosink->node)
      RCLCPP_INFO(rosaudiosink->logger, "preparing audio with caps '%s'",
          gst_caps_to_string(caps));

  caps_struct = gst_caps_get_structure (caps, 0);
  if(!gst_structure_get_int (caps_struct, "channels", &channels))
      RCLCPP_ERROR(rosaudiosink->logger, " setcaps missing channels");
  if(!gst_structure_get_int (caps_struct, "rate", &rate))
      RCLCPP_ERROR(rosaudiosink->logger, " setcaps missing rate");

  format_str = gst_structure_get_string(caps_struct, "format");

  if(format_str)
  {
    format_enum = gst_audio_format_from_string (format_str);
    format_info = gst_audio_format_get_info (format_enum);
    width = format_info->width;
    depth = format_info->depth;
    endianness = format_info->endianness;
  }
  else
  {
    RCLCPP_ERROR(rosaudiosink->logger, " setcaps missing format");
    if(!gst_structure_get_int (caps_struct, "width", &width))
        RCLCPP_ERROR(rosaudiosink->logger, " setcaps missing width");
    if(!gst_structure_get_int (caps_struct, "depth", &depth))
        RCLCPP_ERROR(rosaudiosink->logger, " setcaps missing depth");
    if(!gst_structure_get_int (caps_struct, "endianness", &endianness))
        RCLCPP_ERROR(rosaudiosink->logger, " setcaps missing endianness");
  }


  //collect a bunch of parameters to shoehorn into a message format
  rosaudiosink->channels = channels;
  rosaudiosink->stride = depth * channels;
  rosaudiosink->endianness = endianness;
  rosaudiosink->sample_rate = rate;
  rosaudiosink->layout = 0; //XXX

  return true;
}



static GstFlowReturn rosaudiosink_render (GstBaseSink * sink, GstBuffer * buf)
{
  GstMapInfo info;
  GstClockTime time;
  uint8_t* data;
  size_t length;
  auto msg = audio_msgs::msg::Audio();
  GstFlowReturn ret;

  Rosaudiosink *rosaudiosink = GST_ROSAUDIOSINK (sink);

  GST_DEBUG_OBJECT (rosaudiosink, "render");

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
  
  //auto msg = rosaudiosink->pub->borrow_loaned_message();
  //msg.get().frames = 

  //fill the blanks
  msg.frames = length/rosaudiosink->stride;
  msg.channels = rosaudiosink->channels;    
  msg.sample_rate = rosaudiosink->sample_rate;
  msg.encoding = rosaudiosink->encoding;
  msg.is_bigendian = (rosaudiosink->endianness == G_BIG_ENDIAN);
  msg.layout = rosaudiosink->layout;
  msg.step = rosaudiosink->stride;
  msg.header.stamp = rosaudiosink->clock->now();
  msg.header.frame_id = rosaudiosink->frame_id;

  //publish
  rosaudiosink->pub->publish(msg);

  return GST_FLOW_OK;
}
