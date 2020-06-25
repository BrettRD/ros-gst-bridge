/* GStreamer
 * Copyright (C) 2020 FIXME <fixme@example.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#ifndef _GST_ROSAUDIOSINK_H_
#define _GST_ROSAUDIOSINK_H_

#include <gst/audio/gstaudiosink.h>


#include "rclcpp/rclcpp.hpp"
#include "audio_msgs/msg/audio.hpp"


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
  GstAudioSink base_rosaudiosink;
  gchar* node_name;
  gchar* pub_topic;
  gchar* frame_id;
  gchar* encoding; //image topic encoding string

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<audio_msgs::msg::Audio>::SharedPtr pub;
  rclcpp::Logger logger;
  rclcpp::Clock::SharedPtr clock;

  int channels;    //number of audio channels
  int sample_rate; //sample rate in Hz
  size_t stride;   //bytes per frame
  gint endianness;  
  GstAudioLayout layout;

};

struct _RosaudiosinkClass
{
  GstAudioSinkClass base_rosaudiosink_class;
};

GType rosaudiosink_get_type (void);

G_END_DECLS

#endif
