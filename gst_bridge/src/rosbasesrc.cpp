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
 * SECTION:element-rosbasesrc
 *
 * The rosbasesrc element, tie ROS2 into a src element.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v rosaudiosrc ! queue ! audioconvert ! alsasink
 * ]|
 * plays audio data from ros over the default output.
 * </refsect2>
 */


#include <gst_bridge/rosbasesrc.h>


GST_DEBUG_CATEGORY_STATIC (rosbasesrc_debug_category);
#define GST_CAT_DEFAULT rosbasesrc_debug_category

/* prototypes */


static void rosbasesrc_set_property (GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosbasesrc_get_property (GObject * object, guint property_id, GValue * value, GParamSpec * pspec);


static GstStateChangeReturn rosbasesrc_change_state (GstElement * element, GstStateChange transition);
static void rosbasesrc_init (RosBaseSrc * src);



static gboolean rosbasesrc_open (RosBaseSrc * src);
static gboolean rosbasesrc_close (RosBaseSrc * src);
static void spin_wrapper(RosBaseSrc * src);


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

G_DEFINE_TYPE_WITH_CODE (RosBaseSrc, rosbasesrc, GST_TYPE_BASE_SRC,
    GST_DEBUG_CATEGORY_INIT (rosbasesrc_debug_category, "rosbasesrc", 0,
        "debug category for rosbasesrc element"))

static void rosbasesrc_class_init (RosBaseSrcClass * klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  //GstBaseSrcClass *basesrc_class = (GstBaseSrcClass *) klass;

  object_class->set_property = rosbasesrc_set_property;
  object_class->get_property = rosbasesrc_get_property;

  gst_element_class_set_static_metadata (element_class,
      "rosbasesrc",
      "Source",
      "a gstreamer source class to place ROS nodes in gstreamer",
      "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property (object_class, PROP_ROS_NAME,
      g_param_spec_string ("ros-name", "node-name", "Name of the ROS node",
      "ros_base_src_node",
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

  element_class->change_state = GST_DEBUG_FUNCPTR (rosbasesrc_change_state); //use state change events to open and close subscribers

  //basesrc_class->create() // there's no reason for the base class to shim in here


}

static void rosbasesrc_init (RosBaseSrc * src)
{
  src->node_name = g_strdup("ros_base_src_node");
  src->node_namespace = g_strdup("");
  src->stream_start_prop = GST_CLOCK_TIME_NONE;
}

void rosbasesrc_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  RosBaseSrc *src = GST_ROS_BASE_SRC (object);

  GST_DEBUG_OBJECT (src, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if(src->node)
      {
        RCLCPP_ERROR(src->logger, "can't change node name once opened");
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
        RCLCPP_ERROR(src->logger, "can't change node namespace once opened");
      }
      else
      {
        g_free(src->node_namespace);
        src->node_namespace = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_START_TIME:
      if(src->node)
      {
        RCLCPP_ERROR(src->logger, "can't change start_time once opened");
      }
      else
      {
        src->stream_start_prop = g_value_get_uint64(value);
      }
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void rosbasesrc_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  RosBaseSrc *src = GST_ROS_BASE_SRC (object);

  GST_DEBUG_OBJECT (src, "get_property");
  switch (property_id)
  {
    case PROP_ROS_NAME:
      g_value_set_string(value, src->node_name);
      break;

    case PROP_ROS_NAMESPACE:
      g_value_set_string(value, src->node_namespace);
      break;

    case PROP_ROS_START_TIME:
      g_value_set_uint64(value, src->stream_start.nanoseconds());
      // XXX this allows inspection via props,
      //      but may cause confusion because it does not show the actual prop
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}


static GstStateChangeReturn rosbasesrc_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  RosBaseSrc *src = GST_ROS_BASE_SRC (element);

  switch (transition)
  {
    case GST_STATE_CHANGE_NULL_TO_READY:
    {
      if (!rosbasesrc_open(src))
      {
        GST_DEBUG_OBJECT (src, "open failed");
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    }
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    {
      if(GST_CLOCK_TIME_IS_VALID(src->stream_start_prop))
      {
        src->stream_start = rclcpp::Time(
          src->stream_start_prop, src->clock->get_clock_type());
        RCLCPP_INFO(src->logger, "stream_start overridden to %ld", src->stream_start.nanoseconds());
      }
      else
      {
        src->stream_start = src->clock->now();
        RCLCPP_INFO(src->logger, "stream_start at %ld", src->stream_start.nanoseconds());
      }

      src->ros_clock_offset = gst_bridge::sample_clock_offset(GST_ELEMENT_CLOCK(src), src->stream_start);
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    //XXX stop the subscription
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (rosbasesrc_parent_class)->change_state (element, transition);

  switch (transition)
  {
    case GST_STATE_CHANGE_READY_TO_NULL:
    {
      rosbasesrc_close(src);
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
static gboolean rosbasesrc_open (RosBaseSrc * src)
{
  RosBaseSrcClass *src_class = GST_ROS_BASE_SRC_GET_CLASS (src);
  using std::placeholders::_1;

  gboolean result = TRUE;

  GST_DEBUG_OBJECT (src, "open");

  src->ros_context = std::make_shared<rclcpp::Context>();
  src->ros_context->init(0, NULL);    // XXX should expose the init arg list
  auto opts = rclcpp::NodeOptions();
  opts.context(src->ros_context); //set a context to generate the node in
  src->node = std::make_shared<rclcpp::Node>(std::string(src->node_name), std::string(src->node_namespace), opts);

  auto ex_args = rclcpp::ExecutorOptions();
  ex_args.context = src->ros_context;
  src->ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(ex_args);
  src->ros_executor->add_node(src->node);

  // allow sub-class to create subscribers on src->node
  if(src_class->open)
    result = src_class->open(src);

  src->logger = src->node->get_logger();
  src->clock = src->node->get_clock();

  src->spin_thread = std::thread{&spin_wrapper, src};

  return result;
}

/* close the device */
static gboolean rosbasesrc_close (RosBaseSrc * src)
{
  RosBaseSrcClass *src_class = GST_ROS_BASE_SRC_GET_CLASS (src);

  GST_DEBUG_OBJECT (src, "close");
  gboolean result = TRUE;

  src->clock.reset();

  //allow sub-class to clean up before destroying ros context
  if(src_class->close)
    result = src_class->close(src);

  //stop the executor
  src->ros_executor->cancel();
  src->spin_thread.join();
  src->ros_context->shutdown("gst closing rosbasesrc");

  //release anything held by shared pointer
  src->ros_context.reset();
  src->ros_executor.reset();
  src->node.reset();
  src->clock.reset();
  return result;
}

static void spin_wrapper(RosBaseSrc * src)
{
  src->ros_executor->spin();
}


