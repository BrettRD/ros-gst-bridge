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

#ifndef _GST_ROS_BASE_SRC_H_
#define _GST_ROS_BASE_SRC_H_

#include <gst/base/gstbasesrc.h>
#include <gst_bridge/gst_bridge.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>

G_BEGIN_DECLS

#define GST_TYPE_ROS_BASE_SRC   (rosbasesrc_get_type())
#define GST_ROS_BASE_SRC(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROS_BASE_SRC,RosBaseSrc))
#define GST_ROS_BASE_SRC_CAST(obj)        ((RosBaseSrc*)obj)
#define GST_ROS_BASE_SRC_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROS_BASE_SRC,RosBaseSrcClass))
#define GST_ROS_BASE_SRC_GET_CLASS(obj)    (G_TYPE_INSTANCE_GET_CLASS ((obj), GST_TYPE_ROS_BASE_SRC, RosBaseSrcClass))
#define GST_IS_ROS_BASE_SRC(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROS_BASE_SRC))
#define GST_IS_ROS_BASE_SRC_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROS_BASE_SRC))

typedef struct _RosBaseSrc RosBaseSrc;
typedef struct _RosBaseSrcClass RosBaseSrcClass;

struct _RosBaseSrc
{
  GstBaseSrc parent;
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

struct _RosBaseSrcClass
{
  GstBaseSrcClass parent_class;


  /*
   * called shortly after the node is created
   * register subscription with ROS (context, node, clock, and logger are handled for you)
   * called at gstbasesrc->change_state()  GST_STATE_CHANGE_NULL_TO_READY
   * timers and reconf callbacks are currently broken, needs a new thread with an executor, patches welcome
   */
  gboolean (*open) (RosBaseSrc * src);


  /*
   * destroy the ros subscription(s) and unregister your callbacks and timers and prepare for ros_context->shutdown()
   * called at gstbasesrc->change_state()  GST_STATE_CHANGE_READY_TO_NULL
   * timers and reconf callbacks are currently broken, needs a new thread with an executor, patches welcome
   */
  gboolean (*close) (RosBaseSrc * src);

};

GType rosbasesrc_get_type (void);

G_END_DECLS

#endif
