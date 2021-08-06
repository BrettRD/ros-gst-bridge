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

#ifndef _GST_ROSAUDIOSRC_H_
#define _GST_ROSAUDIOSRC_H_

#include <gst/audio/audio-format.h>
#include <gst/base/gstbasesrc.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesrc.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <audio_msgs/msg/audio.hpp>
#include <queue>  // std::queue
#include <mutex>  // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

G_BEGIN_DECLS

#define GST_TYPE_ROSAUDIOSRC   (rosaudiosrc_get_type())
#define GST_ROSAUDIOSRC(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROSAUDIOSRC,Rosaudiosrc))
#define GST_ROSAUDIOSRC_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROSAUDIOSRC,RosaudiosrcClass))
#define GST_IS_ROSAUDIOSRC(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROSAUDIOSRC))
#define GST_IS_ROSAUDIOSRC_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROSAUDIOSRC))

typedef struct _Rosaudiosrc Rosaudiosrc;
typedef struct _RosaudiosrcClass RosaudiosrcClass;

struct _Rosaudiosrc
{
  RosBaseSrc parent;
  gchar* sub_topic;
  gchar* frame_id;
  gchar* encoding;
  gchar* init_caps;

  bool msg_init;

  // XXX this is too much boilerplate.
  size_t msg_queue_max;
  std::queue<audio_msgs::msg::Audio::ConstSharedPtr> msg_queue;
  std::mutex msg_queue_mtx;
  std::condition_variable msg_queue_cv;

  rclcpp::Subscription<audio_msgs::msg::Audio>::SharedPtr sub;

  GstAudioInfo audio_info;
  uint64_t msg_seq_num;
};

struct _RosaudiosrcClass
{
  RosBaseSrcClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType rosaudiosrc_get_type (void);

G_END_DECLS

#endif
