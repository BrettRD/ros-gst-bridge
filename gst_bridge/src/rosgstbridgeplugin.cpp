/*
 * GStreamer
 * Copyright (C) 2020 FIXME <fixme@example.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Alternatively, the contents of this file may be used under the
 * GNU Lesser General Public License Version 2.1 (the "LGPL"), in
 * which case the following provisions apply instead of the ones
 * mentioned above:
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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <gst/gst.h>
#include <gst_bridge/rosaudiosink.h>
#include <gst_bridge/rosimagesink.h>
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

