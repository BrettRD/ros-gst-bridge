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

#ifndef _GST_ROS_BASE_SINK_H_
#define _GST_ROS_BASE_SINK_H_

#include <gst/base/gstbasesink.h>
#include <gst_bridge/gst_bridge.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>


G_BEGIN_DECLS
#define GST_TYPE_ROS_BASE_SINK   (rosbasesink_get_type())
#define GST_ROS_BASE_SINK(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROS_BASE_SINK,RosBaseSink))
#define GST_ROS_BASE_SINK_CAST(obj)        ((RosBaseSink*)obj)
#define GST_ROS_BASE_SINK_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROS_BASE_SINK,RosBaseSinkClass))
#define GST_ROS_BASE_SINK_GET_CLASS(obj)    (G_TYPE_INSTANCE_GET_CLASS ((obj), GST_TYPE_ROS_BASE_SINK, RosBaseSinkClass))
#define GST_IS_ROS_BASE_SINK(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROS_BASE_SINK))
#define GST_IS_ROS_BASE_SINK_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROS_BASE_SINK))

typedef struct _RosBaseSink RosBaseSink;
typedef struct _RosBaseSinkClass RosBaseSinkClass;

struct _RosBaseSink
{
  GstBaseSink parent;
  gchar* node_name;
  gchar* node_namespace;

  rclcpp::Context::SharedPtr ros_context;
  rclcpp::Executor::SharedPtr ros_executor;
  rclcpp::Node::SharedPtr node;
  rclcpp::Logger logger;
  rclcpp::Clock::SharedPtr clock;

  std::thread spin_thread;

  rclcpp::Time stream_start;
  rcl_time_point_value_t stream_start_prop; //uint64_t, equiv to GST_TYPE_CLOCK_TIME
  GstClockTimeDiff ros_clock_offset;
};

struct _RosBaseSinkClass
{
  GstBaseSinkClass parent_class;


  /*
   * called shortly after the node is created
   * register publishers with ROS (context, node, clock, and logger are handled for you)
   * called at gstbasesink->change_state()  GST_STATE_CHANGE_NULL_TO_READY
   * timers and reconf callbacks are currently broken, needs a new thread with an executor, patches welcome
   */
  gboolean (*open) (RosBaseSink * sink);


  /*
   * destroy the ros publisher(s) and unregister your callbacks and timers and prepare for ros_context->shutdown()
   * called at gstbasesink->change_state()  GST_STATE_CHANGE_READY_TO_NULL
   * timers and reconf callbacks are currently broken, needs a new thread with an executor, patches welcome
   */
  gboolean (*close) (RosBaseSink * sink);


  /*
   * gstreamer tells us what caps we should set up to deal with
   * this may be called several times, including during playback?
   */
  gboolean (*set_caps) (RosBaseSink * sink, GstCaps * caps);


  /*
   * publish the message
   * msg_time is derived from buf and offset by rostime at pipeline playtime
   * rosbasesink will calculate msg_time, you can bypass that by using gstbasesink's render() instead
   */
  GstFlowReturn (*render) (RosBaseSink * base_sink, GstBuffer * buf, rclcpp::Time msg_time);

};

GType rosbasesink_get_type (void);

G_END_DECLS

#endif
