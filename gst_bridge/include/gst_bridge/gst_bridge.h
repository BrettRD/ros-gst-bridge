/*
(BSD License) to go with ROS2

*/

#ifndef GST_BRIDGE__GST_BRIDGE_H_
#define GST_BRIDGE__GST_BRIDGE_H_


#include <gst/video/video-format.h>
#include <gst/audio/audio-format.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <audio_msgs/msg/audio.hpp>

#define GST_BRIDGE_GST_VIDEO_FORMAT_LIST "{ GRAY8, GRAY16_LE, RGB, BGR, RGBA, BGRA }"
#define GST_BRIDGE_GST_AUDIO_FORMAT_LIST "Any"


namespace gst_bridge
{

// convert between ROS and GST types
GstVideoFormat getGstVideoFormat(const std::string & encoding);
GstAudioFormat getGstAudioFormat(const std::string & encoding);

std::string getRosEncoding(GstVideoFormat);
std::string getRosEncoding(GstAudioFormat);

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