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
 * SECTION:element-rosaudiosrc
 *
 * The rosaudiosrc element, pipe audio data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v rosaudiosrc ! queue ! audioconvert ! alsasink
 * ]|
 * plays audio data from ros over the default output.
 * </refsect2>
 */

//#ifdef HAVE_CONFIG_H
//#include "config.h"
//#endif

#include <gst/gst.h>
#include "rosaudiosrc.h"


GST_DEBUG_CATEGORY_STATIC (rosaudiosrc_debug_category);
#define GST_CAT_DEFAULT rosaudiosrc_debug_category

/* prototypes */


static void rosaudiosrc_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosaudiosrc_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);
static void rosaudiosrc_dispose (GObject * object);  //unused?
static void rosaudiosrc_finalize (GObject * object);


static GstStateChangeReturn rosaudiosrc_change_state (GstElement * element, GstStateChange transition);


static void rosaudiosrc_init (Rosaudiosrc * src);
static GstCaps * rosaudiosrc_fixate (GstBaseSrc * base_src, GstCaps * caps);
static GstFlowReturn rosaudiosrc_fill (GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer *buf);

static gboolean rosaudiosrc_query (GstBaseSrc * base_src, GstQuery * query);

static gboolean rosaudiosrc_open (Rosaudiosrc * src);
static gboolean rosaudiosrc_close (Rosaudiosrc * src);

//static gboolean rosaudiosrc_negotiate (GstBaseSrc * base_src);
static GstCaps* rosaudiosrc_getcaps (GstBaseSrc * base_src, GstCaps * filter);
//static GstCaps* rosaudiosrc_setcaps (GstBaseSrc * base_src, GstCaps * caps);  


/*
 * rosaudiosrc_fill needs to wait for a ros message arriving on rosaudiosrc_sub_cb
 * use the message passing pattern of GCond to block rosaudiosrc_fill until rosaudiosrc_sub_cb gets called
 */
static void rosaudiosrc_sub_cb(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg);
static audio_msgs::msg::Audio::ConstSharedPtr rosaudiosrc_wait_for_msg(Rosaudiosrc * src);

/*
  XXX provide a mechanism for ROS to provide a clock
*/


enum
{
  PROP_0,
  PROP_ROS_NAME,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_ENCODING,
  PROP_INIT_CAPS,
};


/* pad templates */

/* FIXME add/remove the formats that you want to support */
static GstStaticPadTemplate rosaudiosrc_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("audio/x-raw, " "format=S16LE, " "rate=[1,max], "
        "channels=[1,max], " "layout=interleaved")
    );


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rosaudiosrc, rosaudiosrc, GST_TYPE_BASE_SRC,
    GST_DEBUG_CATEGORY_INIT (rosaudiosrc_debug_category, "rosaudiosrc", 0,
        "debug category for rosaudiosrc element"))

static void rosaudiosrc_class_init (RosaudiosrcClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSrcClass *basesrc_class = (GstBaseSrcClass *) klass;

  object_class->set_property = rosaudiosrc_set_property;
  object_class->get_property = rosaudiosrc_get_property;
  object_class->dispose = rosaudiosrc_dispose;
  object_class->finalize = rosaudiosrc_finalize;


  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (element_class,
      &rosaudiosrc_src_template);


  gst_element_class_set_static_metadata (element_class,
      "rosaudiosrc",
      "Source/Audio",
      "a gstreamer source that transports ROS audio_msgs over gstreamer",
      "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property (object_class, PROP_ROS_NAME,
      g_param_spec_string ("ros-name", "node-name", "Name of the ROS node",
      "gst_audio_src_node",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_TOPIC,
      g_param_spec_string ("ros-topic", "sub-topic", "ROS topic to subscribe to",
      "gst_audio_sub",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_FRAME_ID,
      g_param_spec_string ("ros-frame-id", "frame-id", "frame_id of the image message",
      "",
      (GParamFlags) (G_PARAM_READABLE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_ENCODING,
      g_param_spec_string ("ros-encoding", "encoding-string", "ROS message encoding string",
      "",
      (GParamFlags) (G_PARAM_READABLE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_INIT_CAPS,
      g_param_spec_string ("init-caps", "initial-caps", "optional caps filter to skip wait for first message",
      "",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );



  element_class->change_state = GST_DEBUG_FUNCPTR (rosaudiosrc_change_state); //use state change events to open and close subscribers

  basesrc_class->fixate = GST_DEBUG_FUNCPTR (rosaudiosrc_fixate); //set caps fields to our preferred values (if possible)
  //basesrc_class->get_caps = GST_DEBUG_FUNCPTR (rosaudiosrc_getcaps);  //return caps within the filter
  //basesrc_class->negotiate = GST_DEBUG_FUNCPTR (rosaudiosrc_negotiate);  //start figuring out caps and allocators
  //basesrc_class->event = GST_DEBUG_FUNCPTR (rosaudiosrc_event);  //flush events can cause discontinuities (flags exist in buffers)
  //basesrc_class->get_times = GST_DEBUG_FUNCPTR (rosaudiosrc_get_times); //asks us for start and stop times (?)
  basesrc_class->fill = GST_DEBUG_FUNCPTR(rosaudiosrc_fill);
  basesrc_class->query = GST_DEBUG_FUNCPTR(rosaudiosrc_query);  //set the scheduling modes
}

static void rosaudiosrc_init (Rosaudiosrc * src)
{
  // Don't register the node or the subscriber just yet,
  // wait for rosaudiosrc_open()
  // XXX set defaults elsewhere to keep gst-inspect consistent
  src->node_name = g_strdup("gst_audio_src_node");
  src->sub_topic = g_strdup("gst_audio_sub");
  src->frame_id = g_strdup("");
  src->encoding = g_strdup("");
  src->init_caps = g_strdup("");

  src->msg_init = true;
  src->new_msg = false;

}

void rosaudiosrc_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (object);

  GST_DEBUG_OBJECT (src, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if(src->node)
      {
        RCLCPP_ERROR(src->logger, "can't change node name once openned");
      }
      else
      {
        g_free(src->node_name);
        src->node_name = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_TOPIC:
      if(src->node)
      {
        RCLCPP_ERROR(src->logger, "can't change topic name once openned");
      }
      else
      {
        g_free(src->sub_topic);
        src->sub_topic = g_value_dup_string(value);
      }
      break;

    case PROP_INIT_CAPS:
      if(src->msg_init)
      {
        g_free(src->init_caps);
        src->init_caps = g_value_dup_string(value);
        // XXX set up the audio message checks and unpack the caps
        src->msg_init = false;
      }
      else
      {
        RCLCPP_ERROR(src->logger, "can't change initial caps after init");
      }
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosaudiosrc_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (object);

  GST_DEBUG_OBJECT (src, "get_property");
  switch (property_id)
  {
    case PROP_ROS_NAME:
      g_value_set_string(value, src->node_name);
      break;

    case PROP_ROS_TOPIC:
      g_value_set_string(value, src->sub_topic);
      break;

    case PROP_ROS_FRAME_ID:
      g_value_set_string(value, src->frame_id);
      break;

    case PROP_ROS_ENCODING:
      g_value_set_string(value, src->encoding);
      break;

    case PROP_INIT_CAPS:
      g_value_set_string(value, src->init_caps);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosaudiosrc_dispose (GObject * object)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (object);

  GST_DEBUG_OBJECT (src, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (rosaudiosrc_parent_class)->dispose (object);
}

void rosaudiosrc_finalize (GObject * object)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (object);

  GST_DEBUG_OBJECT (src, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (rosaudiosrc_parent_class)->finalize (object);
}


static GstStateChangeReturn rosaudiosrc_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  Rosaudiosrc *src = GST_ROSAUDIOSRC (element);

  switch (transition)
  {
    case GST_STATE_CHANGE_NULL_TO_READY:
    {
      //gst_audio_clock_reset (GST_AUDIO_CLOCK (src->provided_clock), 0);
      if (!rosaudiosrc_open(src))
      {
        GST_DEBUG_OBJECT (src, "open failed");
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

  ret = GST_ELEMENT_CLASS (rosaudiosrc_parent_class)->change_state (element, transition);

  switch (transition)
  {
    case GST_STATE_CHANGE_READY_TO_NULL:
      rosaudiosrc_close(src);
      break;
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  return ret;

}

/* open the device with given specs */
static gboolean rosaudiosrc_open (Rosaudiosrc * src)
{
  using std::placeholders::_1;

  GST_DEBUG_OBJECT (src, "open");

  rclcpp::init(0, NULL);
  src->node = std::make_shared<rclcpp::Node>(src->node_name);

  auto cb = [src] (audio_msgs::msg::Audio::ConstSharedPtr msg){rosaudiosrc_sub_cb(src, msg);};
  rclcpp::QoS qos = rclcpp::QoS(10);
  src->sub = src->node->create_subscription<audio_msgs::msg::Audio>(src->sub_topic, qos, cb);

  src->logger = src->node->get_logger();
  src->clock = src->node->get_clock();
  return TRUE;
}

/* close the device */
static gboolean rosaudiosrc_close (Rosaudiosrc * src)
{
  GST_DEBUG_OBJECT (src, "close");

  src->clock.reset();
  src->sub.reset();
  src->node.reset();
  rclcpp::shutdown();
  return TRUE;
}


static GstCaps * rosaudiosrc_fixate (GstBaseSrc * base_src, GstCaps * caps)
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

  caps = GST_BASE_SRC_CLASS (rosaudiosrc_parent_class)->fixate (base_src, caps);

  return caps;
}





/* 
 * decide on caps
 * may have to wait for a first message to figure out how many channels and what encoding
 */

/*
static gboolean rosaudiosrc_negotiate (GstBaseSrc * base_src)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (base_src);
  GST_DEBUG_OBJECT (src, "negotiate");

  return false;
}
*/


/* return valid caps to parent class*/
// XXX need to provide the range of possible values 
static GstCaps* rosaudiosrc_getcaps (GstBaseSrc * base_src, GstCaps * filter)
{
  GstStructure *caps_struct;
  gint width, depth, channels, endianness, rate, layout;
  const gchar * format_str;
  GstAudioFormat format_enum;

  Rosaudiosrc *src = GST_ROSAUDIOSRC (base_src);

  GST_DEBUG_OBJECT (src, "getcaps");

  if(!gst_caps_is_fixed(filter))
  {
    RCLCPP_INFO(src->logger, "caps is not fixed");
  }

  GST_DEBUG_OBJECT (src, "getcaps with filter %s", gst_caps_to_string(filter));

  if(src->node)
      RCLCPP_INFO(src->logger, "getcaps with filter '%s'", gst_caps_to_string(filter));

  if(src->msg_init)
  {
    GST_DEBUG_OBJECT (src, "getcaps returning template early");
    return gst_pad_get_pad_template_caps (GST_BASE_SRC (src)->srcpad);
  }

  GST_DEBUG_OBJECT (src, "getcaps returning known caps");

  bool sign = true;
  endianness = src->endianness;
  width = src->stride / src->channels;
  depth = src->stride / src->channels;

  format_enum = gst_audio_format_build_integer(
      sign,
      endianness,
      width,
      depth);

  format_str = gst_audio_format_to_string(format_enum);

  return gst_caps_new_simple ("audio/x-raw",
      "format", G_TYPE_STRING, format_str,
      "rate", G_TYPE_INT, src->sample_rate,
      "channels", G_TYPE_INT, src->channels,
      "layout", G_TYPE_INT, src->layout,
      NULL);

}


static gboolean rosaudiosrc_query (GstBaseSrc * base_src, GstQuery * query)
{
  gboolean ret;

  Rosaudiosrc *src = GST_ROSAUDIOSRC (base_src);

  switch (GST_QUERY_TYPE (query)) {
    case GST_QUERY_SCHEDULING:
    {
      /* a pushsrc can by default never operate in pull mode override
       * if you want something different. */
      gst_query_set_scheduling (query, GST_SCHEDULING_FLAG_SEQUENTIAL, 1, -1,
          0);
      gst_query_add_scheduling_mode (query, GST_PAD_MODE_PUSH);

      ret = TRUE;
      break;
    }
    default:
      ret = GST_BASE_SRC_CLASS (rosaudiosrc_parent_class)->query (base_src, query);
      break;
  }
  return ret;
}



/*
 * Wait for a message to be published, then load the contents into buf
 * Also update frame_id and encoding
 * Error if the number of channels or encoding changes at runtime
 */
static GstFlowReturn rosaudiosrc_fill (GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer *buf)
{
  GstMapInfo info;
  GstClockTime time;
  size_t length;
  GstFlowReturn ret;
  
  Rosaudiosrc *src = GST_ROSAUDIOSRC (base_src);

  //wait for a message to be published
  auto msg = rosaudiosrc_wait_for_msg(src);

  GST_DEBUG_OBJECT (src, "render");


  gst_buffer_map (buf, &info, GST_MAP_READ);
  length = msg->step * msg->frames;
  info.size = length;
  memcpy(info.data, msg->data.data(), length);
  gst_buffer_unmap (buf, &info);

  time = GST_BUFFER_PTS (buf);    //XXX link gst clock to ros clock


  return GST_FLOW_OK;
}



static void rosaudiosrc_sub_cb(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg)
{
  //fetch caps from the first msg, check on subsequent
  if(src->msg_init)
  {
    src->stride = msg->step;
    src->channels = msg->channels;
    src->sample_rate = msg->sample_rate;
    src->endianness = (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN);
    src->layout = msg->layout;

    g_free(src->encoding);
    src->encoding = g_strdup(msg->encoding.c_str());


    src->msg_init = false;
  }
  else
  {
    if(!(
        (src->stride == msg->step) &&
        (src->channels == msg->channels) &&
        (src->sample_rate == msg->sample_rate) &&
        (0 == g_strcmp0(src->encoding, msg->encoding.c_str())) &&
        (src->endianness == (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN)) &&
        (src->layout == msg->layout) ))
    {
      GST_DEBUG_OBJECT (src, "ros audio message params changed during playback");
      RCLCPP_ERROR(src->logger, "audio format changed during playback");
    }
  }

  g_mutex_lock (&(src->data_mutex));
  src->current_msg = msg;
  src->new_msg = true;
  g_cond_signal (&(src->data_cond));
  g_mutex_unlock (&(src->data_mutex));
}


static audio_msgs::msg::Audio::ConstSharedPtr rosaudiosrc_wait_for_msg(Rosaudiosrc * src)
{

  if(src->msg_init)
  {
    GST_DEBUG_OBJECT (src, "ros audio filling buffer before receiving first message");
  }

  g_mutex_lock (&(src->data_mutex));
  while (!src->new_msg)
    g_cond_wait (&(src->data_cond), &(src->data_mutex));
  src->new_msg = false;
  g_mutex_unlock (&(src->data_mutex));

  return src->current_msg;
}
