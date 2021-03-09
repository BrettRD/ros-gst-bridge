/* GStreamer
 * Copyright (C) 2020 BrettRD <brettrd@brettrd.com>
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

static void rosaudiosink_init (Rosaudiosink * rosaudiosink);

static gboolean rosaudiosink_open (RosBaseSink * sink);
static gboolean rosaudiosink_close (RosBaseSink * sink);
static gboolean rosaudiosink_setcaps (GstBaseSink * sink, GstCaps * caps);
static GstCaps* rosaudiosink_getcaps (GstBaseSink * sink, GstCaps * filter);

static GstFlowReturn rosaudiosink_render (GstBaseSink * sink, GstBuffer * buffer, rclcpp::Time msg_time);

/*
  XXX provide a mechanism for ROS to provide a clock
*/


enum
{
  PROP_0,
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


  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (element_class,
      &rosaudiosink_sink_template);


  gst_element_class_set_static_metadata (element_class,
      "rosaudiosink",
      "Sink",
      "a gstreamer sink that publishes audio data into ROS",
      "BrettRD <brettrd@brettrd.com>");

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
  //sink->node_name = g_strdup("gst_audio_sink_node");  //XXX overwrite the node name, but don't take over the property
  //sink->node_namespace = g_strdup("");
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
    case PROP_ROS_TOPIC:
      if(sink->node)
      {
        RCLCPP_ERROR(sink->logger, "can't change topic name once openned");
        // XXX try harder
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


/* open the device with given specs */
static gboolean rosaudiosink_open (RosBaseSink * ros_base_sink)
{
  GST_DEBUG_OBJECT (sink, "open");
  Rosaudiosink *sink = GST_ROSAUDIOSINK (ros_base_sink);

  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();  //XXX add a parameter for overrides
  sink->pub = ros_base_sink->node->create_publisher<audio_msgs::msg::Audio>(sink->pub_topic, qos);

  return TRUE;
}

/* close the device */
static gboolean rosaudiosink_close (RosBaseSink * ros_base_sink)
{
  GST_DEBUG_OBJECT (sink, "close");
  Rosaudiosink *sink = GST_ROSAUDIOSINK (ros_base_sink);

  sink->pub.reset();

  return TRUE;
}


/*
// XXX fixate is only used for pull mode which this sink can't do.
//       some of this logic needs to move to get_caps
static GstCaps * rosaudiosink_fixate (GstBaseSink * base_sink, GstCaps * caps)
{
  //XXX check init_caps and fixate to that
  GstStructure *s;
  gint width, depth;
  Rosaudiosink *sink = GST_ROSAUDIOSINK (base_sink);

  GST_DEBUG_OBJECT (sink, "fixate");

  caps = gst_caps_make_writable (caps);

  s = gst_caps_get_structure (caps, 0);

  // fields for all formats
  gst_structure_fixate_field_nearest_int (s, "rate", 44100);
  gst_structure_fixate_field_nearest_int (s, "channels", 2);
  gst_structure_fixate_field_nearest_int (s, "width", 16);

  // fields for int
  if (gst_structure_has_field (s, "depth")) {
    gst_structure_get_int (s, "width", &width);
    // round width to nearest multiple of 8 for the depth
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
*/

// gstreamer is changing the caps, try to keep up
static gboolean rosaudiosink_setcaps (RosBaseSink * ros_base_sink, GstCaps * caps)
{
  GstAudioInfo audio_info;

  Rosaudiosink *sink = GST_ROSAUDIOSINK (ros_base_sink);

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

  return false; // XXX why simply reject? ros2 messages are flexible
}

static GstCaps* rosaudiosink_getcaps (RosBaseSink * ros_base_sink, GstCaps * filter)
{
  Rosaudiosink *sink = GST_ROSAUDIOSINK (ros_base_sink);

  // XXX narrow down the filter using logic from the old fixate

  return filter;
}


static GstFlowReturn rosaudiosink_render (RosBaseSink * ros_base_sink, GstBuffer * buf, rclcpp::Time msg_time)
{
  GstMapInfo info;
  audio_msgs::msg::Audio msg;

  // XXX use borrowed messages, can buf be extended into the middleware?
  //    auto msg = sink->pub->borrow_loaned_message();
  //    msg.get().frames = ...

  Rosaudiosink *sink = GST_ROSAUDIOSINK (ros_base_sink);
  GST_DEBUG_OBJECT (sink, "render");

  msg = gst_bridge::gst_audio_info_to_audio_msg(&(sink->audio_info));
  msg.header.stamp = msg_time;
  msg.header.frame_id = sink->frame_id;

  gst_buffer_map (buf, &info, GST_MAP_READ);
  msg.data.assign(info.data, info.data+info.size);
  msg.frames = info.size/GST_AUDIO_INFO_BPF(&(sink->audio_info));
  msg.seq_num = sink->msg_seq_num;
  sink->msg_seq_num += msg.frames;

  gst_buffer_unmap (buf, &info);

  //publish
  sink->pub->publish(msg);

  return GST_FLOW_OK;
}

