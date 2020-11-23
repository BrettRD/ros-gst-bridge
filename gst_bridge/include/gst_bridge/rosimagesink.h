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

#ifndef _GST_ROSIMAGESINK_H_
#define _GST_ROSIMAGESINK_H_

#include <gst/video/video-format.h>
#include <gst/base/gstbasesink.h>
#include <gst_bridge/gst_bridge.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


G_BEGIN_DECLS

#define GST_TYPE_ROSIMAGESINK   (rosimagesink_get_type())
#define GST_ROSIMAGESINK(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROSIMAGESINK,Rosimagesink))
#define GST_ROSIMAGESINK_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROSIMAGESINK,RosimagesinkClass))
#define GST_IS_ROSIMAGESINK(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROSIMAGESINK))
#define GST_IS_ROSIMAGESINK_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROSIMAGESINK))

typedef struct _Rosimagesink Rosimagesink;
typedef struct _RosimagesinkClass RosimagesinkClass;

struct _Rosimagesink
{
  GstBaseSink parent;
  gchar* node_name;
  gchar* node_namespace;
  gchar* pub_topic;
  gchar* frame_id;
  gchar* encoding; //image topic encoding string
  gchar* init_caps; //optional caps override (used for limited apis)

  rclcpp::Context::SharedPtr ros_context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
  rclcpp::Logger logger;
  rclcpp::Clock::SharedPtr clock;
  GstClockTimeDiff ros_clock_offset;

  int height;
  int width;
  
  size_t step;   //bytes per pixel
  gint endianness;
};

struct _RosimagesinkClass
{
  GstBaseSinkClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType rosimagesink_get_type (void);

G_END_DECLS

#endif
