/*
 * pcm1780.c
 *
 *  Created on: Apr 4, 2025
 *      Author: joshuayu
 */


void fill_buffer_with_square_wave(int32_t *buf, uint32_t num_samples)
 {
     // Fill up a 100 Hz square wave
     // 44.1 kHz sample rate -> 441 samples in 100 Hz -> toggle every 220 samples
     int toggle_period = 440;
     int count = 0;
     int wave_state = 1;
     int32_t magnitude = 8000000;
 
     for(int i = 0; i < num_samples; i++)
     {
         buf[i] = magnitude * wave_state;
         count++;
         if(count >= toggle_period)
         {
             count = 0;
             wave_state = wave_state * (-1); // toggle
         }
     }
 }

void fill_buffer_with_triangle_wave(int32_t *buf, uint32_t num_samples)
 {
     for (int32_t i = 0; i < num_samples; i++)
     {
         buf[(i)*2] = (i*1000);
         buf[(i)*2+1] = (i*1000);
     }
 }