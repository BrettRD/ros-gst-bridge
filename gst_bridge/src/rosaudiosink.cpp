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


#include <gst_bridge/rosaudiosink.h>


GST_DEBUG_CATEGORY_STATIC (rosaudiosink_debug_category);
#define GST_CAT_DEFAULT rosaudiosink_debug_category

/* prototypes */


static void rosaudiosink_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosaudiosink_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);
static void rosaudiosink_dispose (GObject * object);  //unused?
static void rosaudiosink_finalize (GObject * object);


static GstStateChangeReturn rosaudiosink_change_state (GstElement * element, GstStateChange transition);
static void rosaudiosink_init (Rosaudiosink * rosaudiosink);
static GstCaps * rosaudiosink_fixate (GstBaseSink * bsink, GstCaps * caps);

static gboolean rosaudiosink_setcaps (GstBaseSink * sink, GstCaps * caps);
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
  PROP_ROS_NAMESPACE,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_ENCODING,
  PROP_INIT_CAPS,
};


/* pad templates */

static GstStaticPadTemplate rosaudiosink_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (ROS_AUDIO_MSG_CAPS)
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

  g_object_class_install_property (object_class, PROP_ROS_NAMESPACE,
      g_param_spec_string ("ros-namespace", "node-namespace", "Namespace for the ROS node",
      "",
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

  g_object_class_install_property (object_class, PROP_INIT_CAPS,
      g_param_spec_string ("init-caps", "initial-caps", "optional caps filter to skip wait for first message",
      "",
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

static void rosaudiosink_init (Rosaudiosink * sink)
{
  // Don't register the node or the publisher just yet,
  // wait for rosaudiosink_open()
  // XXX set defaults elsewhere to keep gst-inspect consistent
  sink->node_name = g_strdup("gst_audio_sink_node");
  sink->node_namespace = g_strdup("");
  sink->pub_topic = g_strdup("gst_audio_pub");
  sink->frame_id = g_strdup("audio_frame");
  sink->encoding = g_strdup("16SC1");
  sink->init_caps =  g_strdup("");
}

void rosaudiosink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  Rosaudiosink *sink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (sink, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if(sink->node)
      {
        RCLCPP_ERROR(sink->logger, "can't change node name once openned");
      }
      else
      {
        g_free(sink->node_name);
        sink->node_name = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_NAMESPACE:
      if(sink->node)
      {
        RCLCPP_ERROR(sink->logger, "can't change node namespace once openned");
      }
      else
      {
        g_free(sink->node_namespace);
        sink->node_namespace = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_TOPIC:
      if(sink->node)
      {
        RCLCPP_ERROR(sink->logger, "can't change topic name once openned");
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

    case PROP_INIT_CAPS:
      if(sink->node)  // XXX wrong condition, but close enough
      {
        RCLCPP_ERROR(sink->logger, "can't change initial caps after init");
      }
      else
      {
        g_free(sink->init_caps);
        sink->init_caps = g_value_dup_string(value);
        // XXX set up the image message checks and unpack the caps
        // XXX return the init_caps in fixate(), and probably earlier than that
      }
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosaudiosink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  Rosaudiosink *sink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (sink, "get_property");
  switch (property_id) {
    case PROP_ROS_NAME:
      g_value_set_string(value, sink->node_name);
      break;

    case PROP_ROS_NAMESPACE:
      g_value_set_string(value, sink->node_namespace);
      break;

    case PROP_ROS_TOPIC:
      g_value_set_string(value, sink->pub_topic);
      break;

    case PROP_ROS_FRAME_ID:
      g_value_set_string(value, sink->frame_id);
      break;

    case PROP_ROS_ENCODING:
      g_value_set_string(value, sink->encoding);
      break;

    case PROP_INIT_CAPS:
      g_value_set_string(value, sink->init_caps);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosaudiosink_dispose (GObject * object)
{
  Rosaudiosink *sink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (sink, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (rosaudiosink_parent_class)->dispose (object);
}

void rosaudiosink_finalize (GObject * object)
{
  Rosaudiosink *sink = GST_ROSAUDIOSINK (object);

  GST_DEBUG_OBJECT (sink, "finalize");

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
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    {
      sink->ros_clock_offset = gst_bridge::sample_clock_offset(GST_ELEMENT_CLOCK(sink), sink->clock);
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
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
  sink->node = std::make_shared<rclcpp::Node>(std::string(sink->node_name), std::string(sink->node_namespace), opts);
  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();  //XXX add a parameter for overrides
  sink->pub = sink->node->create_publisher<audio_msgs::msg::Audio>(sink->pub_topic, qos);
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

static GstCaps * rosaudiosink_fixate (GstBaseSink * base_sink, GstCaps * caps)
{
  //XXX check init_caps and fixate to that
  GstStructure *s;
  gint width, depth;
  Rosaudiosink *sink = GST_ROSAUDIOSINK (base_sink);

  GST_DEBUG_OBJECT (sink, "fixate");

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

  caps = GST_BASE_SINK_CLASS (rosaudiosink_parent_class)->fixate (base_sink, caps);

  return caps;
}


/* check the caps, register a node and open an publisher */
static gboolean rosaudiosink_setcaps (GstBaseSink * base_sink, GstCaps * caps)
{
  GstAudioInfo audio_info;

  Rosaudiosink *sink = GST_ROSAUDIOSINK (base_sink);

  GST_DEBUG_OBJECT (sink, "setcaps");

  if(!gst_caps_is_fixed(caps))
  {
    RCLCPP_ERROR(sink->logger, "caps is not fixed");
    return false;
  }

  if(sink->node)
      RCLCPP_INFO(sink->logger, "preparing audio with caps '%s'",
          gst_caps_to_string(caps));

  if(gst_audio_info_from_caps(&audio_info , caps))
  {
    sink->audio_info = audio_info;
    return true;
  }

  return false;
}



static GstFlowReturn rosaudiosink_render (GstBaseSink * base_sink, GstBuffer * buf)
{
  GstMapInfo info;
  rclcpp::Time msg_time;
  GstClockTimeDiff base_time;
  audio_msgs::msg::Audio msg;

  // XXX look into borrowed messages, can buf be extended into the middleware?
  //    auto msg = sink->pub->borrow_loaned_message();
  //    msg.get().frames = ...

  Rosaudiosink *sink = GST_ROSAUDIOSINK (base_sink);
  GST_DEBUG_OBJECT (sink, "render");

  // XXX use the base sink clock synchronising features
  base_time = gst_element_get_base_time(GST_ELEMENT(sink));
  msg_time = rclcpp::Time(GST_BUFFER_PTS(buf) + base_time + sink->ros_clock_offset, sink->clock->get_clock_type());

  msg = gst_bridge::gst_audio_info_to_audio_msg(&(sink->audio_info));
  msg.header.stamp = msg_time;
  msg.header.frame_id = sink->frame_id;

  gst_buffer_map (buf, &info, GST_MAP_READ);
  msg.data.assign(info.data, info.data+info.size);
  msg.frames = info.size/GST_AUDIO_INFO_BPF(&(sink->audio_info));
  gst_buffer_unmap (buf, &info);

  //publish
  sink->pub->publish(msg);

  return GST_FLOW_OK;
}

