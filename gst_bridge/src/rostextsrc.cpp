/* gst_bridge
 * Copyright (C) 2021 Clyde McQueen <clyde@mcqueen.net>
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
 * SECTION:element-gstrostextsrc
 *
 * The rostextsrc element subscribes to a ROS2 topic and feeds text into a pipeline.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v rostextsrc topic="/string" ! txt. videotestsrc ! textoverlay name=txt ! autovideosink
 * ]|
 * Subscribe to /string topic and overlay on test video.
 * </refsect2>
 */

#include <gst_bridge/rostextsrc.h>

GST_DEBUG_CATEGORY_STATIC (rostextsrc_debug_category);
#define GST_CAT_DEFAULT rostextsrc_debug_category

/* prototypes */


static void rostextsrc_set_property (GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);
static void rostextsrc_get_property (GObject * object, guint prop_id, GValue * value, GParamSpec * pspec);

static void rostextsrc_init (Rostextsrc * src);

static gboolean rostextsrc_open (RosBaseSrc * ros_base_src);
static gboolean rostextsrc_close (RosBaseSrc * ros_base_src);

static GstFlowReturn rostextsrc_create (GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer **buf);
static gboolean rostextsrc_query (GstBaseSrc * base_src, GstQuery * query);

static void rostextsrc_sub_cb(Rostextsrc * src, std_msgs::msg::String::ConstSharedPtr msg);
static std_msgs::msg::String::ConstSharedPtr rostextsrc_wait_for_msg(Rostextsrc * src);


enum
{
  PROP_0,
  PROP_SILENT,
  PROP_ROS_TOPIC,
};

/* pad templates */

static GstStaticPadTemplate rostextsrc_src_template = GST_STATIC_PAD_TEMPLATE ("src",
  GST_PAD_SRC,
  GST_PAD_ALWAYS,
  GST_STATIC_CAPS (ROS_TEXT_MSG_CAPS)
);


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (Rostextsrc, rostextsrc, GST_TYPE_ROS_BASE_SRC,
  GST_DEBUG_CATEGORY_INIT (rostextsrc_debug_category, "rostextsrc", 0,
    "debug category for rostextsrc element"))

static void rostextsrc_class_init (RostextsrcClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS (klass);
  RosBaseSrcClass *ros_base_src_class = GST_ROS_BASE_SRC_CLASS (klass);

  object_class->set_property = rostextsrc_set_property;
  object_class->get_property = rostextsrc_get_property;

  gst_element_class_add_pad_template (element_class,
    gst_static_pad_template_get (&rostextsrc_src_template));


  gst_element_class_set_static_metadata (element_class,
    "rostextsrc",
    "Source/Text",
    "a gstreamer source that transports ROS strings over gstreamer",
    "Clyde McQueen <clyde@mcqueen.net>");

  g_object_class_install_property (object_class, PROP_SILENT,
    g_param_spec_boolean ("silent", "Silent", "Produce verbose output ?",
      FALSE,
      G_PARAM_READWRITE));

  g_object_class_install_property (object_class, PROP_ROS_TOPIC,
    g_param_spec_string ("topic", "Topic", "ROS topic to subscribe to",
      "string",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  ros_base_src_class->open = GST_DEBUG_FUNCPTR (rostextsrc_open);  //let the base sink know how we register publishers
  ros_base_src_class->close = GST_DEBUG_FUNCPTR (rostextsrc_close);  //let the base sink know how we destroy publishers

  basesrc_class->create = GST_DEBUG_FUNCPTR(rostextsrc_create);
  basesrc_class->query = GST_DEBUG_FUNCPTR(rostextsrc_query);  //set the scheduling modes
}

static void rostextsrc_init (Rostextsrc * src)
{
  RosBaseSrc *ros_base_src GST_ROS_BASE_SRC(src);
  ros_base_src->node_name = g_strdup("gst_text_src_node");
  src->silent = FALSE;
  src->sub_topic = g_strdup("string");

  src->msg_queue_max = 1;
  // XXX why does queue segfault without expicit construction?
  src->msg_queue = std::queue<std_msgs::msg::String::ConstSharedPtr>();

  /* configure basesrc to be a live source */
  gst_base_src_set_live (GST_BASE_SRC (src), TRUE);
  /* make basesrc output a segment in time */
  gst_base_src_set_format (GST_BASE_SRC (src), GST_FORMAT_TIME);
  /* make basesrc set timestamps on outgoing buffers based on the running_time
   * when they were captured */
  gst_base_src_set_do_timestamp (GST_BASE_SRC (src), TRUE);

}

static void rostextsrc_set_property (GObject * object, guint prop_id,
  const GValue * value, GParamSpec * pspec)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (object);
  Rostextsrc *src = GST_ROSTEXTSRC (object);

  switch (prop_id) {
    case PROP_SILENT:
      src->silent = g_value_get_boolean (value);
      break;

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

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void rostextsrc_get_property (GObject * object, guint prop_id,
  GValue * value, GParamSpec * pspec)
{
  Rostextsrc *src = GST_ROSTEXTSRC (object);

  switch (prop_id) {
    case PROP_SILENT:
      g_value_set_boolean (value, src->silent);
      break;

    case PROP_ROS_TOPIC:
      g_value_set_string(value, src->sub_topic);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}


/* open the subscription with given specs */
static gboolean rostextsrc_open (RosBaseSrc * ros_base_src)
{
  Rostextsrc *src = GST_ROSTEXTSRC (ros_base_src);

  using std::placeholders::_1;

  GST_DEBUG_OBJECT (src, "open");

  // ROS can't cope with some forms of std::bind being passed as subscriber callbacks,
  // lambdas seem to be the preferred case for these instances
  auto cb = [src] (std_msgs::msg::String::ConstSharedPtr msg){rostextsrc_sub_cb(src, msg);};
  rclcpp::QoS qos = rclcpp::SensorDataQoS();  //XXX add a parameter for overrides
  src->sub = ros_base_src->node->create_subscription<std_msgs::msg::String>(src->sub_topic, qos, cb);

  return TRUE;
}

/* close the device */
static gboolean rostextsrc_close (RosBaseSrc * ros_base_src)
{
  Rostextsrc *src = GST_ROSTEXTSRC (ros_base_src);

  GST_DEBUG_OBJECT (src, "close");

  //XXX dereference is as close as foxy gets to unsubscribe
  src->sub.reset();
  //empty the queue
  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  while(!src->msg_queue.empty())
  {
    src->msg_queue.pop();
  }

  return TRUE;
}

static gboolean rostextsrc_query (GstBaseSrc * base_src, GstQuery * query)
{
  gboolean ret;

  //Rostextsrc *src = GST_ROSTEXTSRC (base_src);

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
      ret = GST_BASE_SRC_CLASS (rostextsrc_parent_class)->query (base_src, query);
      break;
  }
  return ret;
}



/*
 * Wait for a message to be published, then load the contents into buf
 * Also update frame_id and encoding
 * Error if the number of channels or encoding changes at runtime
 */
static GstFlowReturn rostextsrc_create (GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer **buf)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (base_src);
  Rostextsrc *src = GST_ROSTEXTSRC (base_src);

  GstMapInfo info;
  GstClockTimeDiff base_time;
  size_t length;
  GstFlowReturn ret = GST_FLOW_OK;
  GstBuffer *res_buf;

  GST_DEBUG_OBJECT (src, "create");

  if(!ros_base_src->node)
  {
    GST_DEBUG_OBJECT (src, "ros text creating buffer before node init");
  }
  else if(false /* src->msg_init */)
  {
    GST_DEBUG_OBJECT (src, "ros text creating buffer before receiving first message");
  }

  auto msg = rostextsrc_wait_for_msg(src);
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
    ret = GST_BASE_SRC_CLASS (rostextsrc_parent_class)->alloc (base_src, offset, length, &res_buf);
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
  // String message does not have a header, use node->now() TODO call now() in the cb and save in the queue
  // GST_BUFFER_PTS (*buf) = rclcpp::Time(msg->header.stamp).nanoseconds() - ros_base_src->ros_clock_offset - base_time;
  GST_BUFFER_PTS (*buf) = ros_base_src->node->now().nanoseconds() - ros_base_src->ros_clock_offset - base_time;
  
  // TODO explore configurable message types
  //GST_BUFFER_DURATION (*buf) = GST_CLOCK_TIME_NONE;
  //GST_BUFFER_DURATION (*buf) = 0;
  GST_BUFFER_DURATION (*buf) = 1000000000L;

  GST_DEBUG_OBJECT (src, "Sending text '%s', %" GST_TIME_FORMAT " + %"
    GST_TIME_FORMAT, msg->data.c_str(), GST_TIME_ARGS (GST_BUFFER_PTS (*buf)),
    GST_TIME_ARGS (GST_BUFFER_DURATION (*buf)));

  return ret;
}

static void rostextsrc_sub_cb(Rostextsrc * src, std_msgs::msg::String::ConstSharedPtr msg)
{
  RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (src);
  GST_DEBUG_OBJECT (src, "ros cb called");
  RCLCPP_DEBUG(ros_base_src->logger, "ros cb called with %s", msg->data.c_str());

  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  src->msg_queue.push(msg);
  while(src->msg_queue.size() > src->msg_queue_max)
  {
    src->msg_queue.pop();
    RCLCPP_WARN(ros_base_src->logger, "dropping message");
  }
  src->msg_queue_cv.notify_one();
}


static std_msgs::msg::String::ConstSharedPtr rostextsrc_wait_for_msg(Rostextsrc * src)
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
