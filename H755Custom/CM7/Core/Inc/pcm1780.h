/*
 * pcm1780.h
 *
 *  Created on: Apr 4, 2025
 *      Author: joshuayu
 */

#ifndef INC_PCM1780_H_
#define INC_PCM1780_H_

#define SINE_SAMPLES 440

void fill_buffer_with_square_wave(int32_t *buf, uint32_t num_samples);
void fill_buffer_with_triangle_wave(int32_t *buf, uint32_t num_samples);






#endif /* INC_PCM1780_H_ */
