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
 * SECTION:element-gstrosbasesink
 *
 * The rosbasesink element, pipe audio data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v audiotestsrc ! rosbasesink node_name="gst_audio" topic="/audiotopic"
 * ]|
 * Streams test tones as ROS audio messages on topic.
 * </refsect2>
 */


#include <gst_bridge/rosbasesink.h>


GST_DEBUG_CATEGORY_STATIC (rosbasesink_debug_category);
#define GST_CAT_DEFAULT rosbasesink_debug_category

/* prototypes */


static void rosbasesink_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosbasesink_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static GstStateChangeReturn rosbasesink_change_state (GstElement * element, GstStateChange transition);
static void rosbasesink_init (RosBaseSink * rosbasesink);

static GstFlowReturn rosbasesink_render (GstBaseSink * sink, GstBuffer * buffer);


static gboolean rosbasesink_open (RosBaseSink * sink);
static gboolean rosbasesink_close (RosBaseSink * sink);
static void spin_wrapper(RosBaseSink * sink);

/*
  XXX provide a mechanism for ROS to provide a clock
*/


enum
{
  PROP_0,
  PROP_ROS_NAME,
  PROP_ROS_NAMESPACE,
  PROP_ROS_START_TIME,
};


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (RosBaseSink, rosbasesink, GST_TYPE_BASE_SINK,
    GST_DEBUG_CATEGORY_INIT (rosbasesink_debug_category, "rosbasesink", 0,
        "debug category for rosbasesink element"))

static void rosbasesink_class_init (RosBaseSinkClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseSinkClass *basesink_class = GST_BASE_SINK_CLASS (klass);

  object_class->set_property = rosbasesink_set_property;
  object_class->get_property = rosbasesink_get_property;


  gst_element_class_set_static_metadata (element_class,
      "rosbasesink",
      "Sink",
      "a gstreamer sink class for handling boilerplate ROS2 interactions",
      "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property (object_class, PROP_ROS_NAME,
      g_param_spec_string ("ros-name", "node-name", "Name of the ROS node",
      "gst_base_sink_node",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_NAMESPACE,
      g_param_spec_string ("ros-namespace", "node-namespace", "Namespace for the ROS node",
      "",
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  g_object_class_install_property (object_class, PROP_ROS_START_TIME,
      g_param_spec_uint64 ("ros-start-time", "ros-start-time", "ROS time (nanoseconds) of the first message",
      0, (guint64)(-1), GST_CLOCK_TIME_NONE,
      (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS))
  );

  element_class->change_state = GST_DEBUG_FUNCPTR (rosbasesink_change_state); //use state change events to open and close publishers
  basesink_class->render = GST_DEBUG_FUNCPTR (rosbasesink_render); // gives us a buffer to forward

}

static void rosbasesink_init (RosBaseSink * sink)
{
  sink->node_name = g_strdup("gst_base_sink_node");
  sink->node_namespace = g_strdup("");
  sink->stream_start_prop = GST_CLOCK_TIME_NONE;
}

void rosbasesink_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  RosBaseSink *sink = GST_ROS_BASE_SINK (object);

  GST_DEBUG_OBJECT (sink, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if(sink->node)
      {
        RCLCPP_ERROR(sink->logger, "can't change node name once opened");
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
        RCLCPP_ERROR(sink->logger, "can't change node namespace once opened");
      }
      else
      {
        g_free(sink->node_namespace);
        sink->node_namespace = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_START_TIME:
      if(sink->node)
      {
        RCLCPP_ERROR(sink->logger, "can't change start_time once opened");
      }
      else
      {
        sink->stream_start_prop = g_value_get_uint64(value);
      }
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosbasesink_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  RosBaseSink *sink = GST_ROS_BASE_SINK (object);

  GST_DEBUG_OBJECT (sink, "get_property");
  switch (property_id) {
    case PROP_ROS_NAME:
      g_value_set_string(value, sink->node_name);
      break;

    case PROP_ROS_NAMESPACE:
      g_value_set_string(value, sink->node_namespace);
      break;

    case PROP_ROS_START_TIME:
      g_value_set_uint64(value, sink->stream_start.nanoseconds());
      // XXX this allows inspection via props,
      //      but may cause confusion because it does not show the actual prop
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}


static GstStateChangeReturn rosbasesink_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  RosBaseSink *sink = GST_ROS_BASE_SINK (element);

  switch (transition)
  {
    case GST_STATE_CHANGE_NULL_TO_READY:
    {
      if (!rosbasesink_open(sink))
      {
        GST_DEBUG_OBJECT (sink, "open failed");
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    }
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    {
      if(GST_CLOCK_TIME_IS_VALID(sink->stream_start_prop))
      {
        sink->stream_start = rclcpp::Time(
          sink->stream_start_prop, sink->clock->get_clock_type());
        RCLCPP_INFO(sink->logger, "stream_start overridden to %ld", sink->stream_start.nanoseconds());
      }
      else
      {
        sink->stream_start = sink->clock->now();
        RCLCPP_INFO(sink->logger, "stream_start at %ld", sink->stream_start.nanoseconds());
      }

      sink->ros_clock_offset = gst_bridge::sample_clock_offset(GST_ELEMENT_CLOCK(sink), sink->stream_start);
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (rosbasesink_parent_class)->change_state (element, transition);

  switch (transition)
  {
    case GST_STATE_CHANGE_READY_TO_NULL:
      rosbasesink_close(sink);
      break;
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  return ret;

}

/* open the device with given specs */
static gboolean rosbasesink_open (RosBaseSink * sink)
{
  RosBaseSinkClass *sink_class = GST_ROS_BASE_SINK_GET_CLASS (sink);
  gboolean result = TRUE;
  GST_DEBUG_OBJECT (sink, "open");

  sink->ros_context = std::make_shared<rclcpp::Context>();
  sink->ros_context->init(0, NULL);    // XXX should expose the init arg list
  auto opts = rclcpp::NodeOptions();
  opts.context(sink->ros_context); //set a context to generate the node in
  sink->node = std::make_shared<rclcpp::Node>(std::string(sink->node_name), std::string(sink->node_namespace), opts);

  auto ex_args = rclcpp::ExecutorOptions();
  ex_args.context = sink->ros_context;
  sink->ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(ex_args);
  sink->ros_executor->add_node(sink->node);

  // allow sub-class to create publishers on sink->node
  if(sink_class->open)
    result = sink_class->open(sink);


  sink->logger = sink->node->get_logger();
  sink->clock = sink->node->get_clock();

  //sink->ros_executor->spin_some();
  sink->spin_thread = std::thread{&spin_wrapper, sink};
  return result;
}

/* close the device */
static gboolean rosbasesink_close (RosBaseSink * sink)
{
  RosBaseSinkClass *sink_class = GST_ROS_BASE_SINK_GET_CLASS (sink);
  gboolean result = TRUE;

  GST_DEBUG_OBJECT (sink, "close");

  sink->clock.reset();

  //allow sub-class to clean up before destroying ros context
  if(sink_class->close)
    result = sink_class->close(sink);

  // XXX do something with result
  //XXX executor
  sink->ros_executor->cancel();
  sink->spin_thread.join();

  sink->node.reset();
  sink->ros_context->shutdown("gst closing rosbasesink");
  return result;
}

static void spin_wrapper(RosBaseSink * sink)
{
  sink->ros_executor->spin();
}


static GstFlowReturn rosbasesink_render (GstBaseSink * base_sink, GstBuffer * buf)
{
  rclcpp::Time msg_time;
  GstClockTimeDiff base_time;

  RosBaseSink *sink = GST_ROS_BASE_SINK (base_sink);
  RosBaseSinkClass *sink_class = GST_ROS_BASE_SINK_GET_CLASS (sink);

  GST_DEBUG_OBJECT (sink, "render");

  // XXX look at the base sink clock synchronising features
  base_time = gst_element_get_base_time(GST_ELEMENT(sink));
  msg_time = rclcpp::Time(GST_BUFFER_PTS(buf) + base_time + sink->ros_clock_offset, sink->clock->get_clock_type());

  if(NULL != sink_class->render)
    return sink_class->render(sink, buf, msg_time);

  if(sink->node)
    RCLCPP_WARN(sink->logger, "rosbasesink render function not set, dropping buffer");

  return GST_FLOW_OK;
}

