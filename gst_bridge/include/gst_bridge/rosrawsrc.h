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

#ifndef _GST_ROSRAWSRC_H_
#define _GST_ROSRAWSRC_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesrc.h>

#include <condition_variable>  // std::condition_variable
#include <gst_msgs/msg/stream_stamped_data.hpp>
#include <mutex>  // std::mutex, std::unique_lock
#include <queue>  // std::queue

G_BEGIN_DECLS

#define GST_TYPE_ROSRAWSRC (rosrawsrc_get_type())
#define GST_ROSRAWSRC(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROSRAWSRC, Rosrawsrc))
#define GST_ROSRAWSRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROSRAWSRC, RosrawsrcClass))
#define GST_IS_ROSRAWSRC(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_ROSRAWSRC))
#define GST_IS_ROSRAWSRC_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_ROSRAWSRC))

typedef struct _Rosrawsrc Rosrawsrc;
typedef struct _RosrawsrcClass RosrawsrcClass;

struct _Rosrawsrc
{
  using MsgType = gst_msgs::msg::StreamStampedData;

  RosBaseSrc parent;
  GstPad * srcpad;
  gboolean silent;
  gchar * sub_topic;
  gchar * caps_string;

  rclcpp::Subscription<MsgType>::SharedPtr sub;

  bool started;
  size_t msg_queue_max;
  std::queue<MsgType::ConstSharedPtr> msg_queue;
  std::mutex msg_queue_mtx;
  std::condition_variable msg_queue_cv;
};

struct _RosrawsrcClass
{
  RosBaseSrcClass parent_class;
};

GType rosrawsrc_get_type(void);

G_END_DECLS

#endif  // _GST_ROSRAWSRC_H_
