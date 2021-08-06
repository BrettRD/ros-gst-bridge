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

#ifndef _GST_ROSAUDIOSINK_H_
#define _GST_ROSAUDIOSINK_H_

#include <gst/audio/audio-format.h>
#include <gst/base/gstbasesink.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesink.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <audio_msgs/msg/audio.hpp>


G_BEGIN_DECLS

#define GST_TYPE_ROSAUDIOSINK   (rosaudiosink_get_type())
#define GST_ROSAUDIOSINK(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROSAUDIOSINK,Rosaudiosink))
#define GST_ROSAUDIOSINK_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROSAUDIOSINK,RosaudiosinkClass))
#define GST_IS_ROSAUDIOSINK(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROSAUDIOSINK))
#define GST_IS_ROSAUDIOSINK_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROSAUDIOSINK))

typedef struct _Rosaudiosink Rosaudiosink;
typedef struct _RosaudiosinkClass RosaudiosinkClass;

struct _Rosaudiosink
{
  RosBaseSink parent;

  gchar* pub_topic;
  gchar* frame_id;
  gchar* encoding; //msg encoding override string (for hacking)
  gchar* init_caps; //a hack to allow skipping preroll

  rclcpp::Publisher<audio_msgs::msg::Audio>::SharedPtr pub;

  GstAudioInfo audio_info;
  uint64_t msg_seq_num;
};

struct _RosaudiosinkClass
{
  RosBaseSinkClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType rosaudiosink_get_type (void);

G_END_DECLS

#endif
