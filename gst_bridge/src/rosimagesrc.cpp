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
 * SECTION:element-rosimagesrc
 *
 * The rosimagesrc element, pipe image data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v rosimagesrc ! queue ! ximagesink
 * ]|
 * plays video data from ros over the default output.
 * </refsect2>
 */


#include <gst_bridge/rosimagesrc.h>

GST_DEBUG_CATEGORY_STATIC (rosimagesrc_debug_category);
#define GST_CAT_DEFAULT rosimagesrc_debug_category

/* prototypes */


static void rosimagesrc_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosimagesrc_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static void rosimagesrc_init (Rosimagesrc * src);

static gboolean rosimagesrc_open (RosBaseSrc * ros_base_src);
static gboolean rosimagesrc_close (RosBaseSrc * ros_base_src);
static GstFlowReturn rosimagesrc_create (GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer **buf);

//static gboolean rosimagesrc_negotiate (GstBaseSrc * base_src);
//static GstCaps* rosimagesrc_setcaps (GstBaseSrc * base_src, GstCaps * caps);  //upstream returns any remaining caps preferences
static gboolean rosimagesrc_query (GstBaseSrc * base_src, GstQuery * query);
static GstCaps* rosimagesrc_getcaps (GstBaseSrc * base_src, GstCaps * filter);  //set our caps preferences
static GstCaps * rosimagesrc_fixate (GstBaseSrc * base_src, GstCaps * caps);


static void rosimagesrc_sub_cb(Rosimagesrc * src, sensor_msgs::msg::Image::ConstSharedPtr msg);
static sensor_msgs::msg::Image::ConstSharedPtr rosimagesrc_wait_for_msg(Rosimagesrc * src);


static void rosimagesrc_set_msg_props_from_caps_string(Rosimagesrc * src, gchar * caps_string);
static void rosimagesrc_set_msg_props_from_msg(Rosimagesrc * src, sensor_msgs::msg::Image::ConstSharedPtr msg);
static void rosimagesrc_set_msg_props(Rosimagesrc * src, int width, int height, size_t step, gint endianness, gchar* encoding);


enum
{
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_ENCODING,
  PROP_INIT_CAPS,
};

/* pad templates */

static GstStaticPadTemplate rosimagesrc_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (ROS_IMAGE_MSG_CAPS)
    );


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rosimagesrc, rosimagesrc, GST_TYPE_ROS_BASE_SRC,
    GST_DEBUG_CATEGORY_INIT (rosimagesrc_debug_category, "rosimagesrc", 0,
        "debug category for rosimagesrc element"))

static void rosimagesrc_class_init (RosimagesrcClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS (klass);
  RosBaseSrcClass *ros_base_src_class = GST_ROS_BASE_SRC_CLASS (klass);

  object_class->set_property = rosimagesrc_set_property;
  object_class->get_property = rosimagesrc_get_property;

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (element_class,
      &rosimagesrc_src_template);


  gst_element_class_set_static_metadata (element_class,
      "rosimagesrc",
      "Source/Video",
      "a gstreamer source that transports ROS image_msgs over gstreamer",
      "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property (object_class, PROP_ROS_TOPIC,
      g_param_spec_string ("ros-topic", "sub-topic", "ROS topic to subscribe to",
      "gst_image_sub",
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


  ros_base_src_class->open = GST_DEBUG_FUNCPTR (rosimagesrc_open);  //let the base sink know how we register publishers
  ros_base_src_class->close = GST_DEBUG_FUNCPTR (rosimagesrc_close);  //let the base sink know how we destroy publishers
  basesrc_class->create = GST_DEBUG_FUNCPTR(rosimagesrc_create);


  basesrc_class->get_caps = GST_DEBUG_FUNCPTR (rosimagesrc_getcaps);  //return caps within the filter
  basesrc_class->query = GST_DEBUG_FUNCPTR(rosimagesrc_query);  //set the scheduling modes
  basesrc_class->fixate = GST_DEBUG_FUNCPTR (rosimagesrc_fixate); //set caps fields to our preferred values (if possible)
  //basesrc_class->negotiate = GST_DEBUG_FUNCPTR (rosimagesrc_negotiate);  //start figuring out caps and allocators
  //basesrc_class->event = GST_DEBUG_FUNCPTR (rosimagesrc_event);  //flush events can cause discontinuities (flags exist in buffers)
  //basesrc_class->get_times = GST_DEBUG_FUNCPTR (rosimagesrc_get_times); //asks us for start and stop times (?)
}

static void rosimagesrc_init (Rosimagesrc * src)
{
  RosBaseSrc *ros_base_src GST_ROS_BASE_SRC(src);
  ros_base_src->node_name = g_strdup("gst_image_src_node");
  src->sub_topic = g_strdup("gst_image_sub");
  src->frame_id = g_strdup("");
  src->encoding = g_strdup("");
  src->init_caps = g_strdup("");

  src->msg_init = true;
  src->msg_queue_max = 1;
  // XXX why does queue segfault without expicit construction?
  src->msg_queue = std::queue<sensor_msgs::msg::Image::ConstSharedPtr>();

  /* configure basesrc to be a live source */
  gst_base_src_set_live (GST_BASE_SRC (src), TRUE);
  /* make basesrc output a segment in time */
  gst_base_src_set_format (GST_BASE_SRC (src), GST_FORMAT_TIME);
  /* make basesrc set timestamps on outgoing buffers based on the running_time
   * when they were captured */
  gst_base_src_set_do_timestamp (GST_BASE_SRC (src), TRUE);

}

void rosimagesrc_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (object);
  Rosimagesrc *src = GST_ROSIMAGESRC (object);

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
        rosimagesrc_set_msg_props_from_caps_string(src, src->init_caps);
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

void rosimagesrc_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  Rosimagesrc *src = GST_ROSIMAGESRC (object);

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

static void rosimagesrc_set_msg_props_from_caps_string(Rosimagesrc * src, gchar * caps_string)
{
  int width;
  int height;
  size_t step;
  gint endianness;
  gchar* encoding;

  GstCaps * caps = gst_caps_from_string(caps_string);
  GstStructure * caps_struct = gst_caps_get_structure (caps, 0);

  if(!gst_structure_get_int (caps_struct, "width", &width))
    GST_DEBUG_OBJECT (src, "caps_init missing width");

  if(!gst_structure_get_int (caps_struct, "height", &height))
    GST_DEBUG_OBJECT (src, "caps_init missing height");

  const gchar * format_str = gst_structure_get_string(caps_struct, "format");
  if(!format_str)
    GST_DEBUG_OBJECT (src, "caps_init missing format");

  GstVideoFormat format = gst_video_format_from_string (format_str);

  step = gst_video_format_get_info(format)->pixel_stride[0];
  endianness = G_LITTLE_ENDIAN;  // XXX pull this from somewhere

  // XXX this check is redundant right now, we should allow overrides by making ros-encoding READWRITE
  if(0 == g_strcmp0(src->encoding, ""))
  {
    encoding = g_strdup(gst_bridge::getRosEncoding(format).c_str());
  }
  else
  {
    encoding = NULL;
  }
  rosimagesrc_set_msg_props(src, width, height, step, endianness, encoding);


}
static void rosimagesrc_set_msg_props_from_msg(Rosimagesrc * src, sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  int width = msg->width;
  int height = msg->height;
  size_t step = msg->step / msg->width;
  gint endianness = (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN);
  gchar* encoding = g_strdup(msg->encoding.c_str());

  rosimagesrc_set_msg_props(src, width, height, step, endianness, encoding);
}

static void rosimagesrc_set_msg_props(Rosimagesrc * src, int width, int height, size_t step, gint endianness, gchar* encoding)
{
  src->width = width;
  src->height = height;
  src->step = step;
  src->endianness = endianness;

  if(encoding)
  {
    g_free(src->encoding);
    src->encoding = g_strdup(encoding);
  }

  size_t blocksize = src->step * src->width * src->height;
  gst_base_src_set_blocksize(GST_BASE_SRC (src), blocksize);

  src->msg_init = false;
}


/* open the subscription with given specs */
static gboolean rosimagesrc_open (RosBaseSrc * ros_base_src)
{
  Rosimagesrc *src = GST_ROSIMAGESRC (ros_base_src);

  using std::placeholders::_1;

  GST_DEBUG_OBJECT (src, "open");

  // ROS can't cope with some forms of std::bind being passed as subscriber callbacks,
  // lambdas seem to be the preferred case for these instances
  auto cb = [src] (sensor_msgs::msg::Image::ConstSharedPtr msg){rosimagesrc_sub_cb(src, msg);};
  rclcpp::QoS qos = rclcpp::SensorDataQoS();  //XXX add a parameter for overrides
  src->sub = ros_base_src->node->create_subscription<sensor_msgs::msg::Image>(src->sub_topic, qos, cb);

  return TRUE;
}

/* close the device */
static gboolean rosimagesrc_close (RosBaseSrc * ros_base_src)
{
  Rosimagesrc *src = GST_ROSIMAGESRC (ros_base_src);

  GST_DEBUG_OBJECT (src, "close");

  //XXX dereference is as close as foxy gets to unsubscribe
  src->sub.reset();
  //empty the queue
  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  while(src->msg_queue.size() > 0)
  {
    src->msg_queue.pop();
  }

  return TRUE;
}


static GstCaps * rosimagesrc_fixate (GstBaseSrc * base_src, GstCaps * caps)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (base_src);

  GstStructure *s;
  gint width, depth;

  Rosimagesrc *src = GST_ROSIMAGESRC (base_src);

  GST_DEBUG_OBJECT (src, "fixate");

  caps = gst_caps_make_writable (caps);

  s = gst_caps_get_structure (caps, 0);

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

  caps = GST_BASE_SRC_CLASS (rosimagesrc_parent_class)->fixate (base_src, caps);

  if(ros_base_src->node)
      RCLCPP_INFO(ros_base_src->logger, "preparing video with caps '%s'",
          gst_caps_to_string(caps));

  return caps;
}

/* return valid caps to parent class*/
static GstCaps* rosimagesrc_getcaps (GstBaseSrc * base_src, GstCaps * filter)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (base_src);

  const gchar * format_str;
  GstVideoFormat format_enum;
  static sensor_msgs::msg::Image::ConstSharedPtr msg;
  GstCaps * caps;

  Rosimagesrc *src = GST_ROSIMAGESRC (base_src);

  GST_DEBUG_OBJECT (src, "getcaps");

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
    msg = rosimagesrc_wait_for_msg(src);  // XXX need to fix API, the action happens in a side-effect

    format_enum = gst_bridge::getGstVideoFormat(std::string(src->encoding));
    format_str = gst_video_format_to_string(format_enum);

    caps = gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, format_str,
        "height", G_TYPE_INT, src->height,
        "width", G_TYPE_INT, src->width,
        NULL);

    gchar* caps_str = gst_caps_to_string(caps);
    GST_DEBUG_OBJECT (src, "getcaps after first message returning %s", caps_str);
    g_free(caps_str);

    return caps;
  }
  else
  {
    caps = gst_caps_from_string(src->init_caps);
    GST_DEBUG_OBJECT (src, "getcaps returning %s from init_caps", gst_caps_to_string(caps));
    return caps;
  }
}


static gboolean rosimagesrc_query (GstBaseSrc * base_src, GstQuery * query)
{
  gboolean ret;

  //Rosimagesrc *src = GST_ROSIMAGESRC (base_src);

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
      ret = GST_BASE_SRC_CLASS (rosimagesrc_parent_class)->query (base_src, query);
      break;
  }
  return ret;
}



/*
 * Wait for a message to be published, then load the contents into buf
 * Also update frame_id and encoding
 * Error if the number of channels or encoding changes at runtime
 */
static GstFlowReturn rosimagesrc_create (GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer **buf)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (base_src);
  Rosimagesrc *src = GST_ROSIMAGESRC (base_src);

  GstMapInfo info;
  GstClockTimeDiff base_time;
  size_t length;
  GstFlowReturn ret = GST_FLOW_OK;
  GstBuffer *res_buf;

  GST_DEBUG_OBJECT (src, "create");

  if(!ros_base_src->node)
  {
    GST_DEBUG_OBJECT (src, "ros image creating buffer before node init");
  }
  else if(src->msg_init)
  {
    GST_DEBUG_OBJECT (src, "ros image creating buffer before receiving first message");
  }

  auto msg = rosimagesrc_wait_for_msg(src);
  { //scope the mutex lock
    std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
    src->msg_queue.pop();   // XXX we can stop dropping the first message during preroll now
  }

  // XXX check message contains anything

  length = msg->data.size();
  if (*buf == NULL) {
    /* downstream did not provide us with a buffer to fill, allocate one
     * ourselves
     * XXX pass the vector memory on directly */
    ret = GST_BASE_SRC_CLASS (rosimagesrc_parent_class)->alloc (base_src, offset, length, &res_buf);
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

  // XXX check the buffer exists, and check info.size > length
  gst_buffer_map (*buf, &info, GST_MAP_READ);
  info.size = length;
  memcpy(info.data, msg->data.data(), length);
  gst_buffer_unmap (*buf, &info);

  base_time = gst_element_get_base_time(GST_ELEMENT(src));
  GST_BUFFER_PTS (*buf) = rclcpp::Time(msg->header.stamp).nanoseconds() - ros_base_src->ros_clock_offset - base_time;

  return ret;
}

static void rosimagesrc_sub_cb(Rosimagesrc * src, sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (src);
  //GST_DEBUG_OBJECT (src, "ros cb called");
  //RCLCPP_DEBUG(ros_base_src->logger, "ros cb called");

  //fetch caps from the first msg, check on subsequent
  if(src->msg_init)
  {
    rosimagesrc_set_msg_props_from_msg(src, msg);
  }
  else
  {
    if(!(src->step == msg->step / msg->width))
      RCLCPP_ERROR(ros_base_src->logger, "image format changed during playback, step %d != %d", src->step, msg->step/msg->width);
    if(!(src->height == (int) msg->height))
      RCLCPP_ERROR(ros_base_src->logger, "image format changed during playback, height %d != %d", src->height, msg->height);
    if(!(src->width == (int) msg->width))
      RCLCPP_ERROR(ros_base_src->logger, "image format changed during playback, width %d != %d", src->width, msg->width);
    if(!(src->endianness == (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN)))
      RCLCPP_ERROR(ros_base_src->logger, "image format changed during playback, endianness %d != %d", src->endianness, (msg->is_bigendian ? G_BIG_ENDIAN : G_LITTLE_ENDIAN));
    if(!(0 == g_strcmp0(src->encoding, msg->encoding.c_str())))
      RCLCPP_ERROR(ros_base_src->logger, "image format changed during playback, encoding %s != %s", src->encoding, msg->encoding.c_str());

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


static sensor_msgs::msg::Image::ConstSharedPtr rosimagesrc_wait_for_msg(Rosimagesrc * src)
{
  //RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (src);

  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  while(src->msg_queue.empty())
  {
    src->msg_queue_cv.wait(lck);
  }
  auto msg = src->msg_queue.front();

  return msg;
}
