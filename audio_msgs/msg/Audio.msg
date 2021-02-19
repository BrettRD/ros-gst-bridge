# This message contains uncompressed audio

std_msgs/Header header  # Header timestamp should be acquisition time the beginning of the buffer
                        # Header frame_id should be meaningful to the location of the transducers

uint64 seq_num          # accumulator of number of frames that came before this message

uint32 frames           # Number of sample-rate intervals in this buffer
uint32 channels         # number of channels of audio
int32 sample_rate       # the sample rate of the audio in Hz (number type following Gstreamer)

                        # Values for encoding should follow Image message OpenCV convention
string encoding         # Encoding of samples, channel meaning, ordering, etc
                        # this field should be extended

uint8 is_bigendian      # is this data bigendian?
uint8 layout            # are audio samples interleaved into frames?

uint8 LAYOUT_INTERLEAVED = 0
uint8 LAYOUT_NON_INTERLEAVED = 1

uint32 step             # audio frame size in bytes
uint8[] data            # actual matrix data, size is (step * frames)