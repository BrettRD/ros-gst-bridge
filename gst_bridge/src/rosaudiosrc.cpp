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


#include <gst_bridge/rosaudiosrc.h>

GST_DEBUG_CATEGORY_STATIC (rosaudiosrc_debug_category);
#define GST_CAT_DEFAULT rosaudiosrc_debug_category

/* prototypes */


static void rosaudiosrc_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosaudiosrc_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static void rosaudiosrc_init (Rosaudiosrc * src);

static gboolean rosaudiosrc_open (RosBaseSrc * ros_base_src);
static gboolean rosaudiosrc_close (RosBaseSrc * ros_base_src);
static GstFlowReturn rosaudiosrc_create (GstBaseSrc * gst_base_src, guint64 offset, guint size, GstBuffer **buf);

//static gboolean rosaudiosrc_negotiate (GstBaseSrc * gst_base_src);
//static GstCaps* rosaudiosrc_setcaps (GstBaseSrc * gst_base_src, GstCaps * caps);  //upstream returns any remaining caps preferences
static GstCaps* rosaudiosrc_getcaps (GstBaseSrc * gst_base_src, GstCaps * filter);  //set our caps preferences
static gboolean rosaudiosrc_query (GstBaseSrc * gst_base_src, GstQuery * query);
static GstCaps * rosaudiosrc_fixate (GstBaseSrc * gst_base_src, GstCaps * caps);


static void rosaudiosrc_sub_cb(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg);
static audio_msgs::msg::Audio::ConstSharedPtr rosaudiosrc_wait_for_msg(Rosaudiosrc * src);


static void rosaudiosrc_set_msg_props_from_caps_string(Rosaudiosrc * src, gchar * caps_string);
static void rosaudiosrc_set_msg_props_from_msg(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg);



enum
{
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_ENCODING,
  PROP_INIT_CAPS,
};

/* pad templates */

static GstStaticPadTemplate rosaudiosrc_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (ROS_AUDIO_MSG_CAPS)
    );


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rosaudiosrc, rosaudiosrc, GST_TYPE_ROS_BASE_SRC,
    GST_DEBUG_CATEGORY_INIT (rosaudiosrc_debug_category, "rosaudiosrc", 0,
        "debug category for rosaudiosrc element"))

static void rosaudiosrc_class_init (RosaudiosrcClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS (klass);
  RosBaseSrcClass *ros_base_src_class = GST_ROS_BASE_SRC_CLASS (klass);

  object_class->set_property = rosaudiosrc_set_property;
  object_class->get_property = rosaudiosrc_get_property;

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (element_class,
      &rosaudiosrc_src_template);


  gst_element_class_set_static_metadata (element_class,
      "rosaudiosrc",
      "Source/Audio",
      "a gstreamer source that transports ROS audio_msgs over gstreamer",
      "BrettRD <brettrd@brettrd.com>");

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


  ros_base_src_class->open = GST_DEBUG_FUNCPTR (rosaudiosrc_open);  //let the base sink know how we register publishers
  ros_base_src_class->close = GST_DEBUG_FUNCPTR (rosaudiosrc_close);  //let the base sink know how we destroy publishers
  basesrc_class->create = GST_DEBUG_FUNCPTR(rosaudiosrc_create); // allocate and fill a buffer


  basesrc_class->query = GST_DEBUG_FUNCPTR(rosaudiosrc_query);  //set the scheduling modes
  basesrc_class->get_caps = GST_DEBUG_FUNCPTR (rosaudiosrc_getcaps);  //return caps within the filter
  basesrc_class->fixate = GST_DEBUG_FUNCPTR (rosaudiosrc_fixate); //set caps fields to our preferred values (if possible)
  //basesrc_class->negotiate = GST_DEBUG_FUNCPTR (rosaudiosrc_negotiate);  //start figuring out caps and allocators
  //basesrc_class->event = GST_DEBUG_FUNCPTR (rosaudiosrc_event);  //flush events can cause discontinuities (flags exist in buffers)
  //basesrc_class->get_times = GST_DEBUG_FUNCPTR (rosaudiosrc_get_times); //asks us for start and stop times (?)
}

static void rosaudiosrc_init (Rosaudiosrc * src)
{
  RosBaseSrc *ros_base_src GST_ROS_BASE_SRC(src);
  ros_base_src->node_name = g_strdup("gst_audio_src_node");
  src->sub_topic = g_strdup("gst_audio_sub");
  src->frame_id = g_strdup("");
  src->encoding = g_strdup("");
  src->init_caps = g_strdup("");

  src->msg_init = true;
  src->msg_queue_max = 1;
  // XXX why does queue segfault without expicit construction?
  src->msg_queue = std::queue<audio_msgs::msg::Audio::ConstSharedPtr>();

  /* configure basesrc to be a live source */
  gst_base_src_set_live (GST_BASE_SRC (src), TRUE); // XXX revise this
  /* make basesrc output a segment in time */
  gst_base_src_set_format (GST_BASE_SRC (src), GST_FORMAT_TIME); // XXX revise this
  /* make basesrc set timestamps on outgoing buffers based on the running_time
   * when they were captured */
  gst_base_src_set_do_timestamp (GST_BASE_SRC (src), TRUE); // XXX revise this

}

void rosaudiosrc_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (object);
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (object);

  GST_DEBUG_OBJECT (src, "set_property");

  switch (property_id)
  {
    case PROP_ROS_TOPIC:
      if(ros_base_src->node)
      {
        RCLCPP_ERROR(ros_base_src->logger, "can't change topic name once opened");
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
        rosaudiosrc_set_msg_props_from_caps_string(src, src->init_caps);
      }
      else
      {
        RCLCPP_ERROR(ros_base_src->logger, "can't change initial caps after init");
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

static void rosaudiosrc_set_msg_props_from_caps_string(Rosaudiosrc * src, gchar * caps_string)
{
  GstAudioInfo audio_info;

  GstCaps * caps = gst_caps_from_string(caps_string);
  if(gst_audio_info_from_caps(&audio_info , caps))
  {
    src->audio_info = audio_info;
  }
  src->msg_init = false;
}

static void rosaudiosrc_set_msg_props_from_msg(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (src);

  GstAudioFormat fmt;

  if(0 == g_strcmp0(src->encoding, "")) fmt = gst_bridge::getGstAudioFormat(msg->encoding);
  else fmt = gst_bridge::getGstAudioFormat(src->encoding);

  gst_audio_info_set_format(&(src->audio_info),
    fmt,
    msg->sample_rate,
    msg->channels,
    NULL);

  if((uint32_t)GST_AUDIO_INFO_BPF(&(src->audio_info)) != msg->step)
      RCLCPP_ERROR(ros_base_src->logger, "audio format misunderstood, step %d != %d",
      GST_AUDIO_INFO_BPF(&(src->audio_info)), msg->step);
  if(GST_AUDIO_INFO_ENDIANNESS(&(src->audio_info)) != ((msg->is_bigendian == 1) ? G_BIG_ENDIAN : G_LITTLE_ENDIAN))
      RCLCPP_ERROR(ros_base_src->logger, "audio format misunderstood, endianness %d != %d",
      GST_AUDIO_INFO_ENDIANNESS(&(src->audio_info)), ((msg->is_bigendian == 1) ? G_BIG_ENDIAN : G_LITTLE_ENDIAN));
  if(GST_AUDIO_INFO_LAYOUT(&(src->audio_info)) != ((msg->layout == audio_msgs::msg::Audio::LAYOUT_INTERLEAVED) ? GST_AUDIO_LAYOUT_INTERLEAVED : GST_AUDIO_LAYOUT_NON_INTERLEAVED))
      RCLCPP_ERROR(ros_base_src->logger, "audio format misunderstood, layout %d != %d",
      GST_AUDIO_INFO_LAYOUT(&(src->audio_info)), ((msg->layout == audio_msgs::msg::Audio::LAYOUT_INTERLEAVED) ? GST_AUDIO_LAYOUT_INTERLEAVED : GST_AUDIO_LAYOUT_NON_INTERLEAVED));

  size_t blocksize = msg->step * msg->frames;
  gst_base_src_set_blocksize(GST_BASE_SRC (src), blocksize);

  src->msg_init = false;
}


/* open the subscription with given specs */
static gboolean rosaudiosrc_open (RosBaseSrc * ros_base_src)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (ros_base_src);

  using std::placeholders::_1;

  GST_DEBUG_OBJECT (src, "open");

  // ROS can't cope with some forms of std::bind being passed as subscriber callbacks,
  // lambdas seem to be the preferred case for these instances
  auto cb = [src] (audio_msgs::msg::Audio::ConstSharedPtr msg){rosaudiosrc_sub_cb(src, msg);};
  rclcpp::QoS qos = rclcpp::SensorDataQoS();  //XXX add a parameter for overrides
  src->sub = ros_base_src->node->create_subscription<audio_msgs::msg::Audio>(src->sub_topic, qos, cb);

  return TRUE;
}

/* close the device */
static gboolean rosaudiosrc_close (RosBaseSrc * ros_base_src)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (ros_base_src);

  GST_DEBUG_OBJECT (src, "close");

  src->sub.reset();
  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  while(src->msg_queue.size() > 0)
  {
    src->msg_queue.pop();
  }

  return TRUE;
}


static GstCaps * rosaudiosrc_fixate (GstBaseSrc * gst_base_src, GstCaps * caps)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (gst_base_src);
  Rosaudiosrc *src = GST_ROSAUDIOSRC (gst_base_src);

  GstStructure *s;
  gint width, depth;


  GST_DEBUG_OBJECT (src, "fixate");

  caps = gst_caps_make_writable (caps);

  s = gst_caps_get_structure (caps, 0);

  /* fields for all formats */
  gst_structure_fixate_field_nearest_int (s, "rate", 44100);
  gst_structure_fixate_field_nearest_int (s, "channels", 2);

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

  caps = GST_BASE_SRC_CLASS (rosaudiosrc_parent_class)->fixate (gst_base_src, caps);

  if(ros_base_src->node)
      RCLCPP_INFO(ros_base_src->logger, "preparing audio with caps '%s'",
          gst_caps_to_string(caps));

  return caps;
}





/*
 * decide on caps
 * may have to wait for a first message to figure out how many channels and what encoding
 */

/*
static gboolean rosaudiosrc_negotiate (GstBaseSrc * gst_base_src)
{
  Rosaudiosrc *src = GST_ROSAUDIOSRC (gst_base_src);
  GST_DEBUG_OBJECT (src, "negotiate");

  return false;
}
*/


/* return valid caps to parent class*/
// XXX need to provide the range of possible values
static GstCaps* rosaudiosrc_getcaps (GstBaseSrc * gst_base_src, GstCaps * filter)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (gst_base_src);
  Rosaudiosrc *src = GST_ROSAUDIOSRC (gst_base_src);

  static audio_msgs::msg::Audio::ConstSharedPtr msg;
  GstCaps * caps;

  GST_DEBUG_OBJECT (src, "getcaps");
/*
  if(!gst_caps_is_fixed(filter))
  {
    RCLCPP_INFO(ros_base_src->logger, "caps is not fixed");
  }
*/
  GST_DEBUG_OBJECT (src, "getcaps with filter %s", gst_caps_to_string(filter));

  if(ros_base_src->node)
      RCLCPP_INFO(ros_base_src->logger, "getcaps with filter '%s'", gst_caps_to_string(filter));

  // if init_caps is not set, we wait for the first message
  // if init_caps is set, we don't wait
  if(0 == g_strcmp0(src->init_caps, ""))
  {
    if(!ros_base_src->node)
    {
      GST_DEBUG_OBJECT (src, "getcaps with node not ready, returning template");
      return gst_pad_get_pad_template_caps (GST_BASE_SRC (src)->srcpad);
    }
    GST_DEBUG_OBJECT (src, "getcaps with node ready, waiting for message");
    RCLCPP_INFO(ros_base_src->logger, "waiting for first message");
    msg = rosaudiosrc_wait_for_msg(src);

    rosaudiosrc_set_msg_props_from_msg(src, msg); //XXX generalise this to return audio_info instead of relying on side-effects

    caps = gst_audio_info_to_caps(&(src->audio_info));
    GST_DEBUG_OBJECT (src, "getcaps returning %s from first msg", gst_caps_to_string(caps));

    return caps;
  }
  else
  {
    caps = gst_caps_from_string(src->init_caps);
    if(gst_audio_info_from_caps(&(src->audio_info) , caps))
    {
      GST_DEBUG_OBJECT (src, "getcaps returning %s from init_caps", gst_caps_to_string(caps));
      src->msg_init = false;  //start checking message consistency
      return caps;
    }
    GST_DEBUG_OBJECT (src, "init_caps did not parse: '%s'", src->init_caps);
    return gst_pad_get_pad_template_caps (GST_BASE_SRC (src)->srcpad);

  }
}


static gboolean rosaudiosrc_query (GstBaseSrc * gst_base_src, GstQuery * query)
{
  gboolean ret;

  Rosaudiosrc *src = GST_ROSAUDIOSRC (gst_base_src);
  GST_DEBUG_OBJECT (src, "query");

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
      ret = GST_BASE_SRC_CLASS (rosaudiosrc_parent_class)->query (gst_base_src, query);
      break;
  }
  return ret;
}



/*
 * Wait for a message to be published, then load the contents into buf
 * Also update frame_id and encoding
 * Error if the number of channels or encoding changes at runtime
 */
static GstFlowReturn rosaudiosrc_create (GstBaseSrc * gst_base_src, guint64 offset, guint size, GstBuffer **buf)
{
  GstMapInfo info;
  GstClockTimeDiff base_time;
  size_t length;
  GstFlowReturn ret = GST_FLOW_OK;
  GstBuffer *res_buf;

  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (gst_base_src);
  Rosaudiosrc *src = GST_ROSAUDIOSRC (gst_base_src);

  GST_DEBUG_OBJECT (src, "create");

  if(!ros_base_src->node)
  {
    GST_DEBUG_OBJECT (src, "ros audio creating buffer before node init");
  }
  else if(src->msg_init)
  {
    GST_DEBUG_OBJECT (src, "ros audio creating buffer before receiving first message");
  }

  auto msg = rosaudiosrc_wait_for_msg(src);
  { //scope the mutex lock
    std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
    src->msg_queue.pop();
  }
  // XXX check sequence number and pad the buffer

  length = msg->data.size();
  if (*buf == NULL) {
    /* downstream did not provide us with a buffer to fill, allocate one
     * ourselves
     * XXX pass the vector memory on directly */
    ret = GST_BASE_SRC_CLASS (rosaudiosrc_parent_class)->alloc (gst_base_src, offset, length, &res_buf);
    if (G_UNLIKELY (ret != GST_FLOW_OK))
      GST_DEBUG_OBJECT (src, "Failed to allocate buffer of %lu bytes", length);
    *buf = res_buf;
    size = length;
  } else {
    /* downstream provided a buffer to fill
     * XXX pass the buffer to the ros subscription allocator */
    res_buf = *buf;
  }

  if(length != size)
    GST_DEBUG_OBJECT (src, "size mismatch, %ld, %d", length, size);

  gst_buffer_map (*buf, &info, GST_MAP_READ);
  info.size = length;
  memcpy(info.data, msg->data.data(), length);
  gst_buffer_unmap (*buf, &info);

  base_time = gst_element_get_base_time(GST_ELEMENT(src));
  GST_BUFFER_PTS (*buf) = rclcpp::Time(msg->header.stamp).nanoseconds() - ros_base_src->ros_clock_offset - base_time;   // XXX +basetime?

  return GST_FLOW_OK;
}



static void rosaudiosrc_sub_cb(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (src);

  //GST_DEBUG_OBJECT (src, "ros cb called");
  //RCLCPP_DEBUG(ros_base_src->logger, "ros cb called");

  //fetch caps from the first msg, check on subsequent
  if(!(src->msg_init))
    {
    if((uint32_t) GST_AUDIO_INFO_BPF(&(src->audio_info)) != msg->step)
      RCLCPP_ERROR(ros_base_src->logger, "audio format changed during playback, step %d != %d",
        GST_AUDIO_INFO_BPF(&(src->audio_info)), msg->step);
    if((uint32_t) GST_AUDIO_INFO_CHANNELS(&(src->audio_info)) != msg->channels)
      RCLCPP_ERROR(ros_base_src->logger, "audio format changed during playback, channels %d != %d",
        GST_AUDIO_INFO_CHANNELS(&(src->audio_info)), msg->channels);
    if(GST_AUDIO_INFO_RATE(&(src->audio_info)) != msg->sample_rate)
      RCLCPP_ERROR(ros_base_src->logger, "audio format changed during playback, sample_rate %d != %d",
        GST_AUDIO_INFO_RATE(&(src->audio_info)), msg->sample_rate);
    if(gst_bridge::getRosEncoding(GST_AUDIO_INFO_FORMAT(&(src->audio_info))) != msg->encoding.c_str() )
      RCLCPP_ERROR(ros_base_src->logger, "audio format changed during playback, encoding %s != %s",
        gst_bridge::getRosEncoding(GST_AUDIO_INFO_FORMAT(&(src->audio_info))), msg->encoding.c_str()); // XXX account for the override
    if(GST_AUDIO_INFO_ENDIANNESS(&(src->audio_info)) != (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN))
      RCLCPP_ERROR(ros_base_src->logger, "audio format changed during playback, endianness %d != %d",
        GST_AUDIO_INFO_ENDIANNESS(&(src->audio_info)), (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN));
    if(GST_AUDIO_INFO_LAYOUT(&(src->audio_info)) != msg->layout)  // XXX really do need a converter beteen the enums
      RCLCPP_ERROR(ros_base_src->logger, "audio format changed during playback, layout %d != %d",
        GST_AUDIO_INFO_LAYOUT(&(src->audio_info)), msg->layout);
  }

  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  src->msg_queue.push(msg);
  while(src->msg_queue.size() > src->msg_queue_max)
  {
    src->msg_queue.pop();
    RCLCPP_WARN(ros_base_src->logger, "dropping message");
  }
  src->msg_queue_cv.notify_one();
}


static audio_msgs::msg::Audio::ConstSharedPtr rosaudiosrc_wait_for_msg(Rosaudiosrc * src)
{
  //RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (src);

  GST_DEBUG_OBJECT (src, "wait for msg");

  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  while(src->msg_queue.empty())
  {
    src->msg_queue_cv.wait(lck);
  }
  auto msg = src->msg_queue.front();

  return msg;

}
