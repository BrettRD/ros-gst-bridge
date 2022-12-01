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

#include <gst_bridge/gst_bridge.h>

namespace gst_bridge
{

GstClockTimeDiff sample_clock_offset(GstClock* gst_clock, rclcpp::Time stream_start)
{
  GstClockTime g_time = gst_clock_get_time (gst_clock); //gst time now
  GstClockTime r_time = stream_start.nanoseconds();  //ros time now
  return r_time - g_time;  //instantaneous offset between ros and gst
}


// convert between ROS and GST types, only fully transparent mappings
GstVideoFormat getGstVideoFormat(const std::string & encoding)
{
  if (encoding == sensor_msgs::image_encodings::MONO8)       {return GST_VIDEO_FORMAT_GRAY8;}
  if (encoding == sensor_msgs::image_encodings::MONO16)      {return GST_VIDEO_FORMAT_GRAY16_LE;}
  if (encoding == sensor_msgs::image_encodings::RGB8)        {return GST_VIDEO_FORMAT_RGB;}
  if (encoding == sensor_msgs::image_encodings::BGR8)        {return GST_VIDEO_FORMAT_BGR;}
  if (encoding == sensor_msgs::image_encodings::RGBA8)       {return GST_VIDEO_FORMAT_RGBA;}
  if (encoding == sensor_msgs::image_encodings::BGRA8)       {return GST_VIDEO_FORMAT_BGRA;}
  if (encoding == sensor_msgs::image_encodings::YUV422)      {return GST_VIDEO_FORMAT_UYVY;}
  if (encoding == sensor_msgs::image_encodings::YUV422_YUY2) {return GST_VIDEO_FORMAT_YUY2;}
  return GST_VIDEO_FORMAT_UNKNOWN;
}

std::string getRosEncoding(GstVideoFormat format)
{
  if (format == GST_VIDEO_FORMAT_GRAY8)     {return sensor_msgs::image_encodings::MONO8;}
  if (format == GST_VIDEO_FORMAT_GRAY16_LE) {return sensor_msgs::image_encodings::MONO16;}
  if (format == GST_VIDEO_FORMAT_RGB)       {return sensor_msgs::image_encodings::RGB8;}
  if (format == GST_VIDEO_FORMAT_BGR)       {return sensor_msgs::image_encodings::BGR8;}
  if (format == GST_VIDEO_FORMAT_RGBA)      {return sensor_msgs::image_encodings::RGBA8;}
  if (format == GST_VIDEO_FORMAT_BGRA)      {return sensor_msgs::image_encodings::BGRA8;}
  if (format == GST_VIDEO_FORMAT_UYVY)      {return sensor_msgs::image_encodings::YUV422;}
  if (format == GST_VIDEO_FORMAT_YUY2)      {return sensor_msgs::image_encodings::YUV422_YUY2;}
  return "unknown";
}


GstAudioFormat getGstAudioFormat(const std::string & encoding)
{
  return gst_audio_format_from_string(encoding.c_str());
}
std::string getRosEncoding(GstAudioFormat format)
{
  return std::string(gst_audio_format_to_string(format));
}

/*
 * Unpack a GstAudioInfo struct into ROS audio message metadata fields
 * this sets frames to zero, and does not fill the header.
 */
audio_msgs::msg::Audio gst_audio_info_to_audio_msg(GstAudioInfo * audio_info)
{
  audio_msgs::msg::Audio msg = audio_msgs::msg::Audio();
  msg.channels = GST_AUDIO_INFO_CHANNELS(audio_info);
  msg.sample_rate = GST_AUDIO_INFO_RATE(audio_info);
  msg.encoding = getRosEncoding(GST_AUDIO_INFO_FORMAT(audio_info));  //equiv to GST_AUDIO_INFO_NAME(audio_info);
  msg.is_bigendian = (GST_AUDIO_INFO_ENDIANNESS(audio_info) == G_BIG_ENDIAN);
  if(GST_AUDIO_INFO_LAYOUT(audio_info) == GST_AUDIO_LAYOUT_INTERLEAVED) msg.layout = audio_msgs::msg::Audio::LAYOUT_INTERLEAVED;
  if(GST_AUDIO_INFO_LAYOUT(audio_info) == GST_AUDIO_LAYOUT_NON_INTERLEAVED) msg.layout = audio_msgs::msg::Audio::LAYOUT_NON_INTERLEAVED;
  msg.step = GST_AUDIO_INFO_BPF(audio_info);  //bytes per frame
  msg.frames = 0; //not known from caps
  return msg;
}


}  //namespace gst_bridge

