/*
(BSD License) to go with ROS2

*/

#ifndef GST_BRIDGE__GST_BRIDGE_H_
#define GST_BRIDGE__GST_BRIDGE_H_

#include <gst/gst.h>

#include <gst/video/video-format.h>
#include <gst/audio/audio-format.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <audio_msgs/msg/audio.hpp>

#define GST_BRIDGE_GST_VIDEO_FORMAT_LIST "{ GRAY8, GRAY16_LE, RGB, BGR, RGBA, BGRA }"
#define GST_BRIDGE_GST_AUDIO_FORMAT_LIST "S16LE"    // "Any"


#define ROS_IMAGE_MSG_CAPS                            \
  "video/x-raw, "                                     \
  "format = " GST_BRIDGE_GST_VIDEO_FORMAT_LIST ", "   \
  "framerate = " GST_VIDEO_FPS_RANGE ", "             \
  "width = " GST_VIDEO_SIZE_RANGE ", "                \
  "height = " GST_VIDEO_SIZE_RANGE " "

#define ROS_AUDIO_MSG_CAPS                            \
  "audio/x-raw, "                                     \
  "format = " GST_BRIDGE_GST_AUDIO_FORMAT_LIST ", "   \
  "rate = " GST_AUDIO_RATE_RANGE ", "                 \
  "channels = " GST_AUDIO_CHANNELS_RANGE ","          \
  "layout = interleaved"

//support rpicamsrc compressed feeds over DDS?
#define H264_CAPS                                     \
  "video/x-h264, "                                    \
  "width = " GST_VIDEO_SIZE_RANGE ", "                \
  "height = " GST_VIDEO_SIZE_RANGE ", "               \
  "framerate = " GST_VIDEO_FPS_RANGE ", "             \
  "stream-format = (string) byte-stream, "            \
  "alignment = (string) nal, "                        \
  "profile = (string) { constrained-baseline, baseline, main, high }"


namespace gst_bridge
{

// convert between ROS and GST types
GstVideoFormat getGstVideoFormat(const std::string & encoding);
GstAudioFormat getGstAudioFormat(const std::string & encoding);

std::string getRosEncoding(GstVideoFormat);
std::string getRosEncoding(GstAudioFormat);

audio_msgs::msg::Audio gst_audio_info_to_audio_msg(GstAudioInfo * audio_info);

/*
// convert between GST and CV
// these should cover the edge cases that ROS doesn't know about
GstVideoFormat getGstVideoFormatFromCvType(int cv_type);
GstAudioFormat getGstAudioFormatFromCvType(int cv_type);

int getCvTypeFromGstVideoFormat(GstVideoFormat);
int getCvTypeFromGstAudioFormat(GstAudioFormat);


// audio specific conversions from ROS to CV
int getCvTypeFromRosEncoding(const std::string & encoding);
*/

}

#endif //GST_BRIDGE__GST_BRIDGE_H_