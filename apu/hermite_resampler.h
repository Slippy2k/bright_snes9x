/* Simple resampler based on bsnes's ruby audio library */

#ifndef __HERMITE_RESAMPLER_H
#define __HERMITE_RESAMPLER_H

#include "resampler.h"

extern int audio_output_method;
extern float audio_output_max;

#undef CLAMP
#undef SHORT_CLAMP
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SHORT_CLAMP(n) ((short) CLAMP((n), -32768, 32767))

class HermiteResampler : public Resampler
{
    protected:

        float r_step;
        float r_frac;
        int   r_left[4], r_right[4];

        static inline float
        hermite (float mu1, float a, float b, float c, float d)
        {
					float out = 0;

					switch(audio_output_method) {
						// default
						case 0: {
							float mu2, mu3, m0, m1, a0, a1, a2, a3;

							mu2 = mu1 * mu1;
							mu3 = mu2 * mu1;

							m0 = (c - a) * 0.5;
							m1 = (d - b) * 0.5;

							a0 = +2 * mu3 - 3 * mu2 + 1;
							a1 =      mu3 - 2 * mu2 + mu1;
							a2 =      mu3 -     mu2;
							a3 = -2 * mu3 + 3 * mu2;

							out = (a0 * b) + (a1 * m0) + (a2 * m1) + (a3 * c);
							break;
						}

						// linear
						case 1: {
							out = (1.0 - mu1) * b + (mu1 * c);
							break;
						}

						// hermite
						case 2:
						case 3:
						case 4:
						case 5:
						case 6:
						case 7: {
							// 0.12 -- 16.0
							int tension;

							switch(audio_output_method)
							{
							case 2: tension = 0.250; break;
							case 3: tension = 0.300; break;
							case 4: tension = 0.350; break;
							case 5: tension = 0.500; break;
							case 6: tension = 0.750; break;
							default: tension = 0.1000; break;
							}

							/*
							a = tension [0..1]

							-a  2-a  a-2    a
							2a  a-3  3-2a  -a
							-a   0    a     0
							 0   1    0     0
							*/

							float A=a, B=b, C=c, D=d;
							float a_ = ((-A - B + C + D) * tension) + (2*B - 2*C);
							float b_ = ((2*A + B - 2*C - D) * tension) + (-3*B + 3*C);
							float c_ = (-A + C) * tension;
							float d_ = B;

							float temp = 0;
							temp = (a_+temp) * mu1;
							temp = (b_+temp) * mu1;
							temp = (c_+temp) * mu1;
							temp =  d_+temp;
							out = temp;
							break;
						}
					}	// switch

					float out_max = out;
					out *= 32768.0;
					out /= audio_output_max;

					if( out > 32767.0 ) {
						if(log_cb) log_cb(RETRO_LOG_INFO,"output audio max = %X (%d %d)\n",out_max,audio_output_max,out);
						audio_output_max = out_max;
						out = 32767.0;
					}

					else if( out < -32768.0 ) {
						if(log_cb) log_cb(RETRO_LOG_INFO,"output audio max = -%X (%d %d)\n",-out_max,audio_output_max,out);
						audio_output_max = -out_max;
						out = -32768.0;
					}

					return out;
        }

    public:
        HermiteResampler (int num_samples) : Resampler (num_samples)
        {
            clear ();
        }

        ~HermiteResampler ()
        {
        }

        void
        time_ratio (double ratio)
        {
            r_step = ratio;
            clear ();
        }

        void
        clear (void)
        {
            ring_buffer::clear ();
            r_frac = 1.0;
            r_left [0] = r_left [1] = r_left [2] = r_left [3] = 0;
            r_right[0] = r_right[1] = r_right[2] = r_right[3] = 0;
        }

        void
        read (short *data, int num_samples)
        {
			//If we are outputting the exact same ratio as the input, pull directly from the input buffer
			if (r_step == 1.0)
			{
				ring_buffer::pull((unsigned char*)data, num_samples * sizeof(short));
				return;
			}

            int i_position = start >> 1;
            int max_samples = buffer_size >> 1;
            short *internal_buffer = (short *) buffer;
            int o_position = 0;
            int consumed = 0;

            while (o_position < num_samples && consumed < buffer_size)
            {
                int s_left = internal_buffer[i_position];
                int s_right = internal_buffer[i_position + 1];
                float hermite_val[2];

                while (r_frac <= 1.0 && o_position < num_samples)
                {
                    hermite_val[0] = hermite (r_frac, r_left [0], r_left [1], r_left [2], r_left [3]);
                    hermite_val[1] = hermite (r_frac, r_right[0], r_right[1], r_right[2], r_right[3]);
                    data[o_position]     = SHORT_CLAMP (hermite_val[0]);
                    data[o_position + 1] = SHORT_CLAMP (hermite_val[1]);

                    o_position += 2;

                    r_frac += r_step;
                }

                if (r_frac > 1.0)
                {
                    r_left [0] = r_left [1];
                    r_left [1] = r_left [2];
                    r_left [2] = r_left [3];
                    r_left [3] = s_left;

                    r_right[0] = r_right[1];
                    r_right[1] = r_right[2];
                    r_right[2] = r_right[3];
                    r_right[3] = s_right;

                    r_frac -= 1.0;

                    i_position += 2;
                    if (i_position >= max_samples)
                        i_position -= max_samples;
                    consumed += 2;
                }
            }

            size -= consumed << 1;
            start += consumed << 1;
            if (start >= buffer_size)
                start -= buffer_size;
        }

        inline int
        avail (void)
        {
			//If we are outputting the exact same ratio as the input, find out directly from the input buffer
			if (r_step == 1.0)
			{
				return (ring_buffer::space_filled() + sizeof(short) - 1) / sizeof(short);
			}

			return (int) floor (((size >> 2) - r_frac) / r_step) * 2;
        }
};

#endif /* __HERMITE_RESAMPLER_H */
