// This is a TensorFlow Lite model file that has been converted into a C data
// array using the tensorflow.lite.util.convert_bytes_to_c_source() function.
// This form is useful for compiling into a binary for devices that don't have a
// file system.

#ifndef TENSORFLOW_LITE_UTIL_FC_MODEL_DATA_H_
#define TENSORFLOW_LITE_UTIL_FC_MODEL_DATA_H_

#define MODEL_IMAGE_WIDTH   28
#define MODEL_IMAGE_HEIGHT  28
#define NUM_CHANNELS         1 // BW
#define kCategory  10


extern const unsigned char fc_model[];
extern const int fc_model_len;

#endif  // TENSORFLOW_LITE_UTIL_SINE_MODEL_DATA_H_
