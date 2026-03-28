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

#ifndef _GST_ROSRAWSINK_H_
#define _GST_ROSRAWSINK_H_

#include <gst/base/gstbasesink.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesink.h>

#include <gst_msgs/msg/stream_stamped_data.hpp>
#include <rclcpp/rclcpp.hpp>

G_BEGIN_DECLS

#define GST_TYPE_ROSRAWSINK (rosrawsink_get_type())
#define GST_ROSRAWSINK(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROSRAWSINK, Rosrawsink))
#define GST_ROSRAWSINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROSRAWSINK, RosrawsinkClass))
#define GST_IS_ROSRAWSINK(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_ROSRAWSINK))
#define GST_IS_ROSRAWSINK_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_ROSRAWSINK))

typedef struct _Rosrawsink Rosrawsink;
typedef struct _RosrawsinkClass RosrawsinkClass;

struct _Rosrawsink
{
  using MsgType = gst_msgs::msg::StreamStampedData;

  RosBaseSink parent;

  gchar * pub_topic;

  rclcpp::Publisher<MsgType>::SharedPtr pub;

  uint64_t msg_seq_num;
};

struct _RosrawsinkClass
{
  RosBaseSinkClass parent_class;
};

GType rosrawsink_get_type(void);

G_END_DECLS

#endif  // _GST_ROSRAWSINK_H_
