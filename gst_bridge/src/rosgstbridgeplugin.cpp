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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <gst/gst.h>
#include <gst_bridge/rosaudiosink.h>
#include <gst_bridge/rosimagesink.h>
#include <gst_bridge/rostextsink.h>
#include <gst_bridge/rosaudiosrc.h>
#include <gst_bridge/rosimagesrc.h>
#include <gst_bridge/rostextsrc.h>


static gboolean
plugin_init (GstPlugin * plugin)
{

  /* FIXME Remember to set the rank if it's an element that is meant
     to be autoplugged by decodebin. */

  gst_element_register (plugin, "rosaudiosink", GST_RANK_NONE,
      GST_TYPE_ROSAUDIOSINK);

  gst_element_register (plugin, "rosimagesink", GST_RANK_NONE,
      GST_TYPE_ROSIMAGESINK);

  gst_element_register (plugin, "rostextsink", GST_RANK_NONE,
      GST_TYPE_ROSTEXTSINK);

  gst_element_register (plugin, "rosaudiosrc", GST_RANK_NONE,
      GST_TYPE_ROSAUDIOSRC);

  gst_element_register (plugin, "rosimagesrc", GST_RANK_NONE,
      GST_TYPE_ROSIMAGESRC);

  gst_element_register (plugin, "rostextsrc", GST_RANK_NONE,
    GST_TYPE_ROSTEXTSRC);


  return true;
}


#ifndef VERSION
#define VERSION "0.0.0"
#endif
#ifndef PACKAGE
#define PACKAGE "ros_gst_bridge"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "ros_gst_bridge"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "https://github.com/BrettRD/ros-gst-bridge"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    rosgstbridge,
    "ROS topic bridge elements",
    plugin_init, VERSION, "LGPL", PACKAGE_NAME, GST_PACKAGE_ORIGIN)

