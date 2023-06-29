# ROS2 Analogue of GstMultiFileSink
# https://gstreamer.freedesktop.org/documentation/multifile/multifilesink.html?gi-language=c

std_msgs/Header header

string filename # the filename where the buffer was written.
int32 index # index of the buffer.
uint64 timestamp # the timestamp of the buffer.
uint64 stream_time # the stream time of the buffer.
uint64 running_time # the running_time of the buffer.
uint64 duration # the duration of the buffer.
uint64 offset # the offset of the buffer that triggered the message.
uint64 offset_end # the offset-end of the buffer that triggered the message.