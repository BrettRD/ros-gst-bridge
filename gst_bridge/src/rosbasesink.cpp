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

#include <cstdint>
#include <optional>

GST_DEBUG_CATEGORY_STATIC(rosbasesink_debug_category);
#define GST_CAT_DEFAULT rosbasesink_debug_category

/* prototypes */

static void rosbasesink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosbasesink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static GstStateChangeReturn rosbasesink_change_state(
  GstElement * element, GstStateChange transition);
static void rosbasesink_init(RosBaseSink * rosbasesink);

static GstFlowReturn rosbasesink_render(GstBaseSink * sink, GstBuffer * buffer);

static gboolean rosbasesink_open(RosBaseSink * sink);
static gboolean rosbasesink_close(RosBaseSink * sink);

/*
  XXX provide a mechanism for ROS to provide a clock
*/

enum { PROP_0, PROP_ROS_NAME, PROP_ROS_NAMESPACE, PROP_ROS_START_TIME, PROP_TIMESTAMP_MODE, PROP_TIMESTAMP_CONVERSION_MODE };

std::optional<rclcpp::Time> get_reference_timestamp(GstBuffer * buf)
{
  GstReferenceTimestampMeta * ref_meta = gst_buffer_get_reference_timestamp_meta(buf, nullptr);

  if (ref_meta) {
    rclcpp::Time time_stamp{};

    constexpr uint64_t NTP_UNIX_OFFSET_SECS = 2'208'988'800ULL;
    constexpr uint64_t NTP_UNIX_OFFSET_NS = NTP_UNIX_OFFSET_SECS * 1'000'000'000ULL;

    uint64_t corrected_ns = ref_meta->timestamp - NTP_UNIX_OFFSET_NS;

    GstClockTime ntp_time = corrected_ns;

    time_stamp = rclcpp::Time(static_cast<int64_t>(ntp_time));

    return time_stamp;
  }

  return std::nullopt;
}

static GType gst_ros_timestamp_mode_get_type(void)
{
  static GType type = 0;
  if (!type) {
    static const GEnumValue values[] = {
      {TIMESTAMP_MODE_ROS_OFFSET, "Ros Offset Adjusted", "ros-offset"},
      {TIMESTAMP_MODE_REFERENCE, "Reference Timestamp (NTP)", "reference"},
      {TIMESTAMP_MODE_PTS, "Presentation Timestamp", "pts"},
      {0, NULL, NULL}};
    type = g_enum_register_static("GstRosTimestampMode", values);
  }
  return type;
}

static GType gst_timestamp_conversion_mode_get_type(void)
{
  static GType type = 0;
  if (!type) {
    static const GEnumValue values[] = {
      {CONVERSION_MODE_NONE, "No timestamp conversion", "none"},
      {CONVERSION_MODE_NTP_2_UNIX, "NTP to UNIX", "ntp2unix"},
      {0, NULL, NULL}};
    type = g_enum_register_static("GstTimestampConversionMode", values);
  }
  return type;
}

rcl_clock_type_t get_clock_type_from(RosBaseSink * sink)
{
  return sink->node_if->clock->get_clock()->get_clock_type();
}

/* class initialization */

G_DEFINE_TYPE_WITH_CODE(
  RosBaseSink, rosbasesink, GST_TYPE_BASE_SINK,
  GST_DEBUG_CATEGORY_INIT(
    rosbasesink_debug_category, "rosbasesink", 0, "debug category for rosbasesink element"))

static void rosbasesink_class_init(RosBaseSinkClass * klass)
{
  GObjectClass * object_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSinkClass * basesink_class = GST_BASE_SINK_CLASS(klass);

  object_class->set_property = rosbasesink_set_property;
  object_class->get_property = rosbasesink_get_property;

  gst_element_class_set_static_metadata(
    element_class, "rosbasesink", "Sink",
    "a gstreamer sink class for handling boilerplate ROS2 interactions",
    "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property(
    object_class, PROP_ROS_NAME,
    g_param_spec_string(
      "ros-name", "node-name", "Name of the ROS node", "gst_base_sink_node",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_NAMESPACE,
    g_param_spec_string(
      "ros-namespace", "node-namespace", "Namespace for the ROS node", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_START_TIME,
    g_param_spec_uint64(
      "ros-start-time", "ros-start-time", "ROS time (nanoseconds) of the first message", 0,
      (guint64)(-1), GST_CLOCK_TIME_NONE,
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_TIMESTAMP_MODE,
    g_param_spec_enum(
      "timestamp-mode", "Timestamp Mode", "How to generate ROS timestamps from incoming buffers",
      GST_TYPE_ROS_TIMESTAMP_MODE,
      TIMESTAMP_MODE_ROS_OFFSET,  // default
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  
  g_object_class_install_property(
    object_class, PROP_TIMESTAMP_CONVERSION_MODE,
    g_param_spec_enum(
      "timestamp-conversion-mode", "Timestamp Conversion Mode", "Convert incoming timestamps",
      GST_TYPE_TIMESTAMP_CONVERSION_MODE,
      CONVERSION_MODE_NONE,  // default
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  element_class->change_state = GST_DEBUG_FUNCPTR(
    rosbasesink_change_state);  //use state change events to open and close publishers
  basesink_class->render = GST_DEBUG_FUNCPTR(rosbasesink_render);  // gives us a buffer to forward
}

static void rosbasesink_init(RosBaseSink * sink)
{
  sink->node_name = g_strdup("gst_base_sink_node");
  sink->node_namespace = g_strdup("");
  sink->stream_start_prop = GST_CLOCK_TIME_NONE;
}

void rosbasesink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec)
{
  RosBaseSink * sink = GST_ROS_BASE_SINK(object);

  GST_DEBUG_OBJECT(sink, "set_property");

  switch (property_id) {
    case PROP_ROS_NAME:
      if (sink->node_if) {
        RCLCPP_ERROR(sink->node_if->logging->get_logger(), "can't change node name once opened");
      } else {
        g_free(sink->node_name);
        sink->node_name = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_NAMESPACE:
      if (sink->node_if) {
        RCLCPP_ERROR(
          sink->node_if->logging->get_logger(), "can't change node namespace once opened");
      } else {
        g_free(sink->node_namespace);
        sink->node_namespace = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_START_TIME:
      if (sink->node_if) {
        RCLCPP_ERROR(sink->node_if->logging->get_logger(), "can't change start_time once opened");
      } else {
        sink->stream_start_prop = g_value_get_uint64(value);
      }
      break;

    case PROP_TIMESTAMP_MODE:
      sink->timestamp_mode = (RosTimestampMode)g_value_get_enum(value);
      break;

    case PROP_TIMESTAMP_CONVERSION_MODE:
      sink->timestamp_conversion_mode = (TimestampConversionMode)g_value_get_enum(value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

void rosbasesink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec)
{
  RosBaseSink * sink = GST_ROS_BASE_SINK(object);

  GST_DEBUG_OBJECT(sink, "get_property");
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

    case PROP_TIMESTAMP_MODE:
      g_value_set_enum(value, sink->timestamp_mode);
      break;
    
    case PROP_TIMESTAMP_CONVERSION_MODE:
      g_value_set_enum(value, sink->timestamp_conversion_mode);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

static GstStateChangeReturn rosbasesink_change_state(
  GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  RosBaseSink * sink = GST_ROS_BASE_SINK(element);

  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY: {
      if (!rosbasesink_open(sink)) {
        GST_DEBUG_OBJECT(sink, "open failed");
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    }
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING: {
      if (GST_CLOCK_TIME_IS_VALID(sink->stream_start_prop)) {
        sink->stream_start = rclcpp::Time(
          sink->stream_start_prop, sink->node_if->clock->get_clock()->get_clock_type());
        RCLCPP_INFO(
          sink->node_if->logging->get_logger(), "stream_start overridden to %ld",
          sink->stream_start.nanoseconds());
      } else {
        sink->stream_start = sink->node_if->clock->get_clock()->now();
        RCLCPP_INFO(
          sink->node_if->logging->get_logger(), "stream_start at %ld",
          sink->stream_start.nanoseconds());
      }

      sink->ros_clock_offset =
        gst_bridge::sample_clock_offset(GST_ELEMENT_CLOCK(sink), sink->stream_start);
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS(rosbasesink_parent_class)->change_state(element, transition);

  switch (transition) {
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
static gboolean rosbasesink_open(RosBaseSink * sink)
{
  // XXX revision: check the sink->node_if contains valid pointers
  //   only build an isolated node if the node_if hasn't been set up

  RosBaseSinkClass * sink_class = GST_ROS_BASE_SINK_GET_CLASS(sink);
  gboolean result = TRUE;
  GST_DEBUG_OBJECT(sink, "open");

  if (nullptr == sink->node_if) {
    // XXX this can be a call to a gst interface
    rosbaseimp_open(&(sink->local_node), sink->node_name, sink->node_namespace);
    sink->node_if = gst_bridge::collect_all_node_interfaces(sink->local_node.node);
  }

  // allow sub-class to create publishers on sink->node
  if (sink_class->open) result = sink_class->open(sink);
  return result;
}

/* close the device */
static gboolean rosbasesink_close(RosBaseSink * sink)
{
  RosBaseSinkClass * sink_class = GST_ROS_BASE_SINK_GET_CLASS(sink);
  gboolean result = TRUE;

  GST_DEBUG_OBJECT(sink, "close");

  //allow sub-class to clean up before destroying ros context
  if (sink_class->close) result = sink_class->close(sink);

  // if the node exists, destruct it, and reset node_if.
  if (nullptr != sink->local_node.node) {
    sink->node_if.reset();
    rosbaseimp_close(&(sink->local_node));
  }
  // if the node doesn't exist, hang onto the node_if

  return result;
}

int64_t ntp_to_unix(const int64_t &ntp_nsec)
{
  static constexpr int64_t NTP_UNIX_OFFSET_SEC = 2208988800LL;
  return ntp_nsec - (NTP_UNIX_OFFSET_SEC * 1000000000LL);;
}

static GstFlowReturn rosbasesink_render(GstBaseSink * base_sink, GstBuffer * buf)
{
  rclcpp::Time msg_time;

  RosBaseSink * sink = GST_ROS_BASE_SINK(base_sink);
  RosBaseSinkClass * sink_class = GST_ROS_BASE_SINK_GET_CLASS(sink);

  GST_DEBUG_OBJECT(sink, "render");

  switch (sink->timestamp_mode) {
    case TIMESTAMP_MODE_REFERENCE: {
      auto reference_time = get_reference_timestamp(buf);

      if (reference_time.has_value()) {
        msg_time = *reference_time;
      } else {
        if (sink->node_if)
          RCLCPP_WARN(sink->node_if->logging->get_logger(), "no reference timestamp found");
      }
      break;
    }
    case TIMESTAMP_MODE_PTS: {
      msg_time = rclcpp::Time(GST_BUFFER_PTS(buf), get_clock_type_from(sink));
      break;
    }
    case TIMESTAMP_MODE_ROS_OFFSET: {
      // XXX look at the base sink clock synchronising features
      GstClockTimeDiff base_time = gst_element_get_base_time(GST_ELEMENT(sink));

      msg_time = rclcpp::Time(
        GST_BUFFER_PTS(buf) + base_time + sink->ros_clock_offset, get_clock_type_from(sink));
    }
  }

  switch (sink->timestamp_conversion_mode) {
    case CONVERSION_MODE_NONE:
      // do nothing
      break;
    case CONVERSION_MODE_NTP_2_UNIX: {
      // convert NTP to UNIX time
      msg_time = rclcpp::Time(ntp_to_unix(msg_time.nanoseconds()), msg_time.get_clock_type());
      break;
    }
  }

  if (NULL != sink_class->render) return sink_class->render(sink, buf, msg_time);

  if (sink->node_if)
    RCLCPP_WARN(
      sink->node_if->logging->get_logger(), "rosbasesink render function not set, dropping buffer");

  return GST_FLOW_OK;
}
