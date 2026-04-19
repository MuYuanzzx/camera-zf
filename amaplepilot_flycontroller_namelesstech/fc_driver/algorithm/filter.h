#ifndef __FILTER_H
#define __FILTER_H

#include "stdbool.h"

void set_cutoff_frequency(float sample_frequent, float cutoff_frequent,filter_parameter *lpf);
float butterworth(float curr_input,filter_buffer *buf,filter_parameter *lpf);

																																										 
#endif

