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

#ifndef _GST_ROSTEXTSRC_H_
#define _GST_ROSTEXTSRC_H_

#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesrc.h>
#include <std_msgs/msg/string.hpp>

#include <queue>  // std::queue
#include <mutex>  // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

G_BEGIN_DECLS

#define GST_TYPE_ROSTEXTSRC   (rostextsrc_get_type())
#define GST_ROSTEXTSRC(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROSTEXTSRC,Rostextsrc))
#define GST_ROSTEXTSRC_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROSTEXTSRC,RostextsrcClass))
#define GST_IS_ROSTEXTSRC(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROSTEXTSRC))
#define GST_IS_ROSTEXTSRC_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROSTEXTSRC))

typedef struct _Rostextsrc Rostextsrc;
typedef struct _RostextsrcClass RostextsrcClass;

struct _Rostextsrc
{
  RosBaseSrc parent;
  GstPad *srcpad;
  gboolean silent;
  gchar* sub_topic;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

  // XXX this is too much boilerplate.
  size_t msg_queue_max;
  std::queue<std_msgs::msg::String::ConstSharedPtr> msg_queue;
  std::mutex msg_queue_mtx;
  std::condition_variable msg_queue_cv;
};

struct _RostextsrcClass
{
  RosBaseSrcClass parent_class;
};

GType rostextsrc_get_type (void);

G_END_DECLS

#endif  // _GST_ROSTEXTSRC_H_