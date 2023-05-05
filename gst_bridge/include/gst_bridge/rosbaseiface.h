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


/*
Proper GStreamer interface definition, not yet used

#ifndef _GST_ROS_BASE_IFACE_H_
#define _GST_ROS_BASE_IFACE_H_

#include <gst/gst.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>

G_BEGIN_DECLS

#define GST_TYPE_ROS_BASE (rosbase_get_type())
G_DECLARE_INTERFACE(RosBase, ros_base, ROS, BASE, GObject)


typedef struct _RosBaseInterface RosBaseInterface;


struct _RosBaseInterface
{
  GTypeInterface parent_iface;

  gboolean (*open)(RosBaseInterface * iface);

  gboolean (*close)(RosBaseInterface * iface);

};

gboolean rosbase_open(RosBase *self);
gboolean rosbase_close(RosBase *self);

*/


// helper object to collect implementation prerequisites
typedef struct _RosBaseImp RosBaseImp;
struct _RosBaseImp{
  // interface variables
  gchar * node_name;
  gchar * node_namespace;

  rclcpp::Context::SharedPtr ros_context;
  rclcpp::Executor::SharedPtr ros_executor;
  rclcpp::Node::SharedPtr node;
  std::thread spin_thread;

  rclcpp::Time stream_start;
  rcl_time_point_value_t stream_start_prop;  //uint64_t, equiv to GST_TYPE_CLOCK_TIME
  GstClockTimeDiff ros_clock_offset;
}

gboolean rosbase_open(RosBase *self);
gboolean rosbase_close(RosBase *self);

/*
GType rosbase_get_type(void);

G_END_DECLS
*/
#endif
