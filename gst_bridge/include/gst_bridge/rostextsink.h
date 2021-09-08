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

#ifndef _GST_ROSTEXTSINK_H_
#define _GST_ROSTEXTSINK_H_

#include <gst/base/gstbasesink.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesink.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


G_BEGIN_DECLS

#define GST_TYPE_ROSTEXTSINK   (rostextsink_get_type())
#define GST_ROSTEXTSINK(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROSTEXTSINK,Rostextsink))
#define GST_ROSTEXTSINK_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROSTEXTSINK,RostextsinkClass))
#define GST_IS_ROSTEXTSINK(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROSTEXTSINK))
#define GST_IS_ROSTEXTSINK_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROSTEXTSINK))

typedef struct _Rostextsink Rostextsink;
typedef struct _RostextsinkClass RostextsinkClass;

struct _Rostextsink
{
  RosBaseSink parent;

  gchar* pub_topic;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

  uint64_t msg_seq_num;
};

struct _RostextsinkClass
{
  RosBaseSinkClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType rostextsink_get_type (void);

G_END_DECLS

#endif
