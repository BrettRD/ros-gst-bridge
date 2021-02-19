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


#include <gst_bridge/rosaudiosrc.h>


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

static GstFlowReturn rosaudiosrc_create (GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer **buf);

static gboolean rosaudiosrc_query (GstBaseSrc * base_src, GstQuery * query);

static gboolean rosaudiosrc_open (Rosaudiosrc * src);
static gboolean rosaudiosrc_close (Rosaudiosrc * src);

//static gboolean rosaudiosrc_negotiate (GstBaseSrc * base_src);
static GstCaps* rosaudiosrc_getcaps (GstBaseSrc * base_src, GstCaps * filter);  //set our caps preferences
//static GstCaps* rosaudiosrc_setcaps (GstBaseSrc * base_src, GstCaps * caps);  //upstream returns any remaining caps preferences


/*
 * rosaudiosrc_create needs to wait for a ros message arriving on rosaudiosrc_sub_cb
 * excuse the gross mix of C++ and C styling going on here, it had to happen somewhere.
 */
static void rosaudiosrc_sub_cb(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg);
static audio_msgs::msg::Audio::ConstSharedPtr rosaudiosrc_wait_for_msg(Rosaudiosrc * src);


static void rosaudiosrc_set_msg_props_from_caps_string(Rosaudiosrc * src, gchar * caps_string);
static void rosaudiosrc_set_msg_props_from_msg(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg);

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

static GstStaticPadTemplate rosaudiosrc_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (ROS_AUDIO_MSG_CAPS)
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

  g_object_class_install_property (object_class, PROP_ROS_NAMESPACE,
      g_param_spec_string ("ros-namespace", "node-namespace", "Namespace for the ROS node",
      "",
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
  basesrc_class->get_caps = GST_DEBUG_FUNCPTR (rosaudiosrc_getcaps);  //return caps within the filter
  //basesrc_class->negotiate = GST_DEBUG_FUNCPTR (rosaudiosrc_negotiate);  //start figuring out caps and allocators
  //basesrc_class->event = GST_DEBUG_FUNCPTR (rosaudiosrc_event);  //flush events can cause discontinuities (flags exist in buffers)
  //basesrc_class->get_times = GST_DEBUG_FUNCPTR (rosaudiosrc_get_times); //asks us for start and stop times (?)
  basesrc_class->create = GST_DEBUG_FUNCPTR(rosaudiosrc_create); // allocate and fill a buffer
  basesrc_class->query = GST_DEBUG_FUNCPTR(rosaudiosrc_query);  //set the scheduling modes
}

static void rosaudiosrc_init (Rosaudiosrc * src)
{
  // Don't register the node or the subscriber just yet,
  // wait for rosaudiosrc_open()
  // XXX set defaults elsewhere to keep gst-inspect consistent
  src->node_name = g_strdup("gst_audio_src_node");
  src->node_namespace = g_strdup("");
  src->sub_topic = g_strdup("gst_audio_sub");
  src->frame_id = g_strdup("");
  src->encoding = g_strdup("");
  src->init_caps = g_strdup("");

  src->msg_init = true;

  /* configure basesrc to be a live source */
  gst_base_src_set_live (GST_BASE_SRC (src), TRUE);
  /* make basesrc output a segment in time */
  gst_base_src_set_format (GST_BASE_SRC (src), GST_FORMAT_TIME);
  /* make basesrc set timestamps on outgoing buffers based on the running_time
   * when they were captured */
  gst_base_src_set_do_timestamp (GST_BASE_SRC (src), TRUE);

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

    case PROP_ROS_NAMESPACE:
      if(src->node)
      {
        RCLCPP_ERROR(src->logger, "can't change node namespace once openned");
      }
      else
      {
        g_free(src->node_namespace);
        src->node_namespace = g_value_dup_string(value);
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
        rosaudiosrc_set_msg_props_from_caps_string(src, src->init_caps);
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

    case PROP_ROS_NAMESPACE:
      g_value_set_string(value, src->node_namespace);
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

  GstAudioFormat fmt;
  if(0 == g_strcmp0(src->encoding, "")) fmt = gst_bridge::getGstAudioFormat(msg->encoding);
  else fmt = gst_bridge::getGstAudioFormat(src->encoding);

  gst_audio_info_set_format(&(src->audio_info),
    fmt,
    msg->sample_rate,
    msg->channels,
    NULL);

  if((uint32_t)GST_AUDIO_INFO_BPF(&(src->audio_info)) != msg->step)
      RCLCPP_ERROR(src->logger, "audio format misunderstood, step %d != %d",
      GST_AUDIO_INFO_BPF(&(src->audio_info)), msg->step);
  if(GST_AUDIO_INFO_ENDIANNESS(&(src->audio_info)) != ((msg->is_bigendian == 1) ? G_BIG_ENDIAN : G_LITTLE_ENDIAN))
      RCLCPP_ERROR(src->logger, "audio format misunderstood, endianness %d != %d",
      GST_AUDIO_INFO_ENDIANNESS(&(src->audio_info)), ((msg->is_bigendian == 1) ? G_BIG_ENDIAN : G_LITTLE_ENDIAN));
  if(GST_AUDIO_INFO_LAYOUT(&(src->audio_info)) != ((msg->layout == audio_msgs::msg::Audio::LAYOUT_INTERLEAVED) ? GST_AUDIO_LAYOUT_INTERLEAVED : GST_AUDIO_LAYOUT_NON_INTERLEAVED))
      RCLCPP_ERROR(src->logger, "audio format misunderstood, layout %d != %d",
      GST_AUDIO_INFO_LAYOUT(&(src->audio_info)), ((msg->layout == audio_msgs::msg::Audio::LAYOUT_INTERLEAVED) ? GST_AUDIO_LAYOUT_INTERLEAVED : GST_AUDIO_LAYOUT_NON_INTERLEAVED));

  size_t blocksize = msg->step * msg->frames;
  gst_base_src_set_blocksize(GST_BASE_SRC (src), blocksize);

  src->msg_init = false;
}


static GstStateChangeReturn rosaudiosrc_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  Rosaudiosrc *src = GST_ROSAUDIOSRC (element);

  switch (transition)
  {
    case GST_STATE_CHANGE_NULL_TO_READY:
    {
      if (!rosaudiosrc_open(src))
      {
        GST_DEBUG_OBJECT (src, "open failed");
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    }
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    {
      src->ros_clock_offset = gst_bridge::sample_clock_offset(GST_ELEMENT_CLOCK(src), src->clock);
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (rosaudiosrc_parent_class)->change_state (element, transition);

  switch (transition)
  {
    case GST_STATE_CHANGE_READY_TO_NULL:
    {
      rosaudiosrc_close(src);
      break;
    }
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

  src->ros_context = std::make_shared<rclcpp::Context>();
  src->ros_context->init(0, NULL);    // XXX should expose the init arg list
  auto opts = rclcpp::NodeOptions();
  opts.context(src->ros_context); //set a context to generate the node in
  src->node = std::make_shared<rclcpp::Node>(std::string(src->node_name), std::string(src->node_namespace), opts);

  // A local ros context requires an executor to spin() on
  auto ex_args = rclcpp::executor::ExecutorArgs();
  ex_args.context = src->ros_context;
  src->ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(ex_args);
  src->ros_executor->add_node(src->node);

  // ROS can't cope with some forms of std::bind being passed as subscriber callbacks,
  // lambdas seem to be the preferred case for these instances
  auto cb = [src] (audio_msgs::msg::Audio::ConstSharedPtr msg){rosaudiosrc_sub_cb(src, msg);};
  rclcpp::QoS qos = rclcpp::SensorDataQoS();  //XXX add a parameter for overrides
  src->sub = src->node->create_subscription<audio_msgs::msg::Audio>(src->sub_topic, qos, cb);

  src->logger = src->node->get_logger();
  src->clock = src->node->get_clock();
  //src->ros_executor->spin();
  src->ros_executor->spin_some();


  return TRUE;
}

/* close the device */
static gboolean rosaudiosrc_close (Rosaudiosrc * src)
{
  GST_DEBUG_OBJECT (src, "close");

  src->clock.reset();
  src->sub.reset();
  src->node.reset();
  src->ros_context->shutdown("gst closing rosaudiosrc");
  return TRUE;
}


static GstCaps * rosaudiosrc_fixate (GstBaseSrc * base_src, GstCaps * caps)
{
  GstStructure *s;
  gint width, depth;

  Rosaudiosrc *src = GST_ROSAUDIOSRC (base_src);

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

  caps = GST_BASE_SRC_CLASS (rosaudiosrc_parent_class)->fixate (base_src, caps);

  if(src->node)
      RCLCPP_INFO(src->logger, "preparing audio with caps '%s'",
          gst_caps_to_string(caps));

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

  static audio_msgs::msg::Audio::ConstSharedPtr msg;
  GstCaps * caps;

  Rosaudiosrc *src = GST_ROSAUDIOSRC (base_src);

  GST_DEBUG_OBJECT (src, "getcaps");
/*
  if(!gst_caps_is_fixed(filter))
  {
    RCLCPP_INFO(src->logger, "caps is not fixed");
  }
*/
  GST_DEBUG_OBJECT (src, "getcaps with filter %s", gst_caps_to_string(filter));

  if(src->node)
      RCLCPP_INFO(src->logger, "getcaps with filter '%s'", gst_caps_to_string(filter));
  
  // if init_caps is not set, we wait for the first message
  // if init_caps is set, we don't wait
  if(0 == g_strcmp0(src->init_caps, ""))
  {
    if(!src->node)
    {
      GST_DEBUG_OBJECT (src, "getcaps with node not ready, returning template");
      return gst_pad_get_pad_template_caps (GST_BASE_SRC (src)->srcpad);
    }
    GST_DEBUG_OBJECT (src, "getcaps with node ready, waiting for message");
    RCLCPP_INFO(src->logger, "waiting for first message");
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


static gboolean rosaudiosrc_query (GstBaseSrc * base_src, GstQuery * query)
{
  gboolean ret;

  Rosaudiosrc *src = GST_ROSAUDIOSRC (base_src);
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
static GstFlowReturn rosaudiosrc_create (GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer **buf)
{
  GstMapInfo info;
  GstClockTimeDiff base_time;
  size_t length;
  GstFlowReturn ret = GST_FLOW_OK;
  GstBuffer *res_buf;

  Rosaudiosrc *src = GST_ROSAUDIOSRC (base_src);

  GST_DEBUG_OBJECT (src, "create");

  if(!src->node)
  {
    GST_DEBUG_OBJECT (src, "ros audio creating buffer before node init");
  }
  else if(src->msg_init)
  {
    GST_DEBUG_OBJECT (src, "ros audio creating buffer before receiving first message");
  }

  auto msg = rosaudiosrc_wait_for_msg(src);

  length = msg->data.size();
  if (*buf == NULL) {
    /* downstream did not provide us with a buffer to fill, allocate one
     * ourselves 
     * XXX pass the vector memory on directly */
    ret = GST_BASE_SRC_CLASS (rosaudiosrc_parent_class)->alloc (base_src, offset, length, &res_buf);
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
  GST_BUFFER_PTS (*buf) = rclcpp::Time(msg->header.stamp).nanoseconds() - src->ros_clock_offset - base_time;

  return GST_FLOW_OK;
}



static void rosaudiosrc_sub_cb(Rosaudiosrc * src, audio_msgs::msg::Audio::ConstSharedPtr msg)
{
  //GST_DEBUG_OBJECT (src, "ros cb called");
  //RCLCPP_DEBUG(src->logger, "ros cb called");

  //fetch caps from the first msg, check on subsequent
  if(!(src->msg_init))
    {
    if((uint32_t) GST_AUDIO_INFO_BPF(&(src->audio_info)) != msg->step)
      RCLCPP_ERROR(src->logger, "audio format changed during playback, step %d != %d",
        GST_AUDIO_INFO_BPF(&(src->audio_info)), msg->step);
    if((uint32_t) GST_AUDIO_INFO_CHANNELS(&(src->audio_info)) != msg->channels)
      RCLCPP_ERROR(src->logger, "audio format changed during playback, channels %d != %d",
        GST_AUDIO_INFO_CHANNELS(&(src->audio_info)), msg->channels);
    if(GST_AUDIO_INFO_RATE(&(src->audio_info)) != msg->sample_rate)
      RCLCPP_ERROR(src->logger, "audio format changed during playback, sample_rate %d != %d",
        GST_AUDIO_INFO_RATE(&(src->audio_info)), msg->sample_rate);
    if(gst_bridge::getRosEncoding(GST_AUDIO_INFO_FORMAT(&(src->audio_info))) != msg->encoding.c_str() )
      RCLCPP_ERROR(src->logger, "audio format changed during playback, encoding %s != %s",
        gst_bridge::getRosEncoding(GST_AUDIO_INFO_FORMAT(&(src->audio_info))), msg->encoding.c_str()); // XXX account for the override
    if(GST_AUDIO_INFO_ENDIANNESS(&(src->audio_info)) != (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN))
      RCLCPP_ERROR(src->logger, "audio format changed during playback, endianness %d != %d",
        GST_AUDIO_INFO_ENDIANNESS(&(src->audio_info)), (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN));
    if(GST_AUDIO_INFO_LAYOUT(&(src->audio_info)) != msg->layout)  // XXX really do need a converter beteen the enums
      RCLCPP_ERROR(src->logger, "audio format changed during playback, layout %d != %d",
        GST_AUDIO_INFO_LAYOUT(&(src->audio_info)), msg->layout);
  }

  src->new_msg.set_value(msg);
}


static audio_msgs::msg::Audio::ConstSharedPtr rosaudiosrc_wait_for_msg(Rosaudiosrc * src)
{
  std::promise<audio_msgs::msg::Audio::ConstSharedPtr> new_msg;
  src->new_msg = std::move(new_msg);
  std::shared_future<audio_msgs::msg::Audio::ConstSharedPtr> fut(src->new_msg.get_future());

  rclcpp::FutureReturnCode ret;
  
  do
  {
    ret = src->ros_executor->spin_until_future_complete(fut);
    if(ret == rclcpp::FutureReturnCode::INTERRUPTED)
    {
      RCLCPP_INFO(src->logger, "wait for cb got interrupted");
    }
  }
  while(ret != rclcpp::FutureReturnCode::SUCCESS);

  return fut.get();
}
