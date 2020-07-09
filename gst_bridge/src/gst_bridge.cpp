
#include <gst_bridge/gst_bridge.h>
#include <string>


namespace gst_bridge
{

// convert between ROS and GST types, only fully transparent mappings
GstVideoFormat getGstVideoFormat(const std::string & encoding)
{
  if (encoding == sensor_msgs::image_encodings::MONO8)  {return GST_VIDEO_FORMAT_GRAY8;}
  if (encoding == sensor_msgs::image_encodings::MONO16) {return GST_VIDEO_FORMAT_GRAY16_LE;}
  if (encoding == sensor_msgs::image_encodings::RGB8)   {return GST_VIDEO_FORMAT_RGB;}  //XXX check these
  if (encoding == sensor_msgs::image_encodings::BGR8)   {return GST_VIDEO_FORMAT_BGR;}
  if (encoding == sensor_msgs::image_encodings::RGBA8)  {return GST_VIDEO_FORMAT_RGBA;}
  if (encoding == sensor_msgs::image_encodings::BGRA8)  {return GST_VIDEO_FORMAT_BGRA;}
  return GST_VIDEO_FORMAT_UNKNOWN;
}

std::string getRosEncoding(GstVideoFormat format)
{
  if (format == GST_VIDEO_FORMAT_GRAY8)     {return sensor_msgs::image_encodings::MONO8;}
  if (format == GST_VIDEO_FORMAT_GRAY16_LE) {return sensor_msgs::image_encodings::MONO16;}
  if (format == GST_VIDEO_FORMAT_RGB)       {return sensor_msgs::image_encodings::RGB8;}  //XXX check the 24b versions, had trouble with lurking padding bytes
  if (format == GST_VIDEO_FORMAT_BGR)       {return sensor_msgs::image_encodings::BGR8;}
  if (format == GST_VIDEO_FORMAT_RGBA)      {return sensor_msgs::image_encodings::RGBA8;}
  if (format == GST_VIDEO_FORMAT_BGRA)      {return sensor_msgs::image_encodings::BGRA8;}
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
}

