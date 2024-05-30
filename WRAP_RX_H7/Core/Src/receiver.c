#include <math.h>
#include <stdint.h>
#include "constants.h"
#include "receiver.h"

void normalize(const uint16_t * samples, float * norm_samples);
int timing_recovery(float * filtered_samps, float * symbs, params_r * params);
float wrap_to_pi(const float x);
int sign(float x);

float zc[ADC_BUF_LEN-1];
/*
Takes samples and turns them into symbols.
samples: samples from ADC
symbs: symbs at the output
params: parameters that need to be stored intermediately
returns length of symbs array. symbs array must be
allocated for longer than samples/sps + some margin
*/

void normalize(const uint16_t * samples, float * norm_samples) {
        // Normalize signal
    float var = 0, mean = 0;
    // find mean
    for (int i = 0; i < ADC_BUF_LEN; i++) {
        mean += (float)samples[i];
    }
    mean /= ADC_BUF_LEN;
    // find sample variance
    for (int i = 0; i < ADC_BUF_LEN; i++) {
        float temp = (float)samples[i]-mean;
        var += temp * temp;
    }
    var = var / (ADC_BUF_LEN-1);
    var = sqrt(var)*25;

    // normalize
    // divide by 60 arbitrary, just done to get to an ampltiude I used to tune gain values
    for (int i = 0; i < ADC_BUF_LEN; i++) {
        norm_samples[i] = (((float)samples[i]) - mean)/var;
    }
}

float symb = 0;
float prev_symb = 0;
float err = 0;
int i_fake = 0;
float tr_ph = 0;
float tr_int = 0;
float zc_samp = 0;
float max_integrator_value = 50;
float max_phase_value = 100;

int timing_recovery(float* rrc_samps_in, float* symbs_out, params_r* params) {

	if (TIMING_RECOVERY_METHOD == 0) {
		// OLD METHOD
		const float kp_PLL = 0.1;
		const float ki_PLL = 0.05;
		const float margin = 0.75;

		float sps = params->sps;
		int integrator = 0; //params->TR_integrator;
		int error = 0, bit_len = 0;
		float prev_phase = 0; // params->TR_phase
		float phase;

		// calculate zero crossings
		for (int i = 0; i < ADC_BUF_LEN-1; i++) {
			int temp = copysign(1, (rrc_samps_in[i+1] * rrc_samps_in[i]));
			zc[i] = !(temp+1);
		}

		// timing recovery
		for (int i = 1; i < ADC_BUF_LEN; i++) {
			phase = prev_phase + 2*M_PI/sps;
			phase = wrap_to_pi(phase);
			if (phase < -M_PI * margin && prev_phase > M_PI * margin) {
				symbs_out[bit_len] = (int)(rrc_samps_in[i]/fabs(rrc_samps_in[i]));
				bit_len++;
			}
			if (bit_len==SYMBOL_BUFF) {
				break;
			}
			if (zc[i-1]){
				error = phase;
				integrator = integrator + error * ki_PLL;
				sps = SPS + error*kp_PLL + integrator;
			}
			prev_phase = phase;
		}
		params->sps = sps;
		params->TR_phase = remainder(phase, 2*M_PI);
		params->TR_integrator = remainder(integrator, 2*M_PI);

		return bit_len;
	} else if (TIMING_RECOVERY_METHOD == 1) {
		const float Kp = 8.5 / 25.0;
		const float Ki = 0.1 / 25.0;
		float error = 0;
		int i = 1;

		params->TR_phase = fmod(params->TR_phase, params->sps);


		int offset = 0;
		int offset2 = 1;
		prev_symb = 25 * rrc_samps_in[lrintf(offset*params->sps)];
		symbs_out[0] = prev_symb;

		while ((i < SYMBOL_BUFF) && (lrintf((i+offset+offset2)*params->sps + params->TR_phase) < ADC_BUF_LEN)) {
			symb = 25 * rrc_samps_in[lrintf((i+offset)*params->sps + params->TR_phase)];

			// Mueller-Muller timing error detector
			error = sign(prev_symb) * symb - sign(symb) * prev_symb;

			err = error;

			symbs_out[i] = symb / fabs(symb);

			params->TR_phase += Kp * error + Ki * params->TR_integrator;
			params->TR_integrator += error;

			if (params->TR_integrator > max_integrator_value) {
				params->TR_integrator = max_integrator_value;
			} else if (params->TR_integrator < -max_integrator_value) {
				params->TR_integrator = -max_integrator_value;
			}

			if (params->TR_phase > max_phase_value) {
				params->TR_phase = max_phase_value;
			} else if (params->TR_phase < -max_phase_value) {
				params->TR_phase = -max_phase_value;
			}

			tr_ph = params->TR_phase;
			tr_int = params->TR_integrator;

			prev_symb = symb;
			i++;
			i_fake = i;
		}

		return i;
	} else {
		const float Kp = 8.5 / 25.0;
		const float Ki = 0.1 / 25.0;
		float error = 0;
		int i = 1;

		params->TR_phase = fmod(params->TR_phase, params->sps);

		int offset = 0;
		int offset2 = 1;
		prev_symb = 25 * rrc_samps_in[lrintf(offset*params->sps)];
		symbs_out[0] = prev_symb;

		while ((i < SYMBOL_BUFF) && (lrintf((i+offset+offset2)*params->sps + params->TR_phase) < ADC_BUF_LEN)) {
			symb = 25 * rrc_samps_in[lrintf((i+offset)*params->sps + params->TR_phase)];

			// Gardner timing error detector
			zc_samp = rrc_samps_in[lrintf((i+offset)*params->sps - params->sps/2 + params->TR_phase)];
			error = zc_samp * (prev_symb - symb);

			err = error;


			symbs_out[i] = symb / fabs(symb);

			params->TR_phase += Kp * error + Ki * params->TR_integrator;
			params->TR_integrator += error;

			if (params->TR_integrator > max_integrator_value) {
				params->TR_integrator = max_integrator_value;
			} else if (params->TR_integrator < -max_integrator_value) {
				params->TR_integrator = -max_integrator_value;
			}

			if (params->TR_phase > max_phase_value) {
				params->TR_phase = max_phase_value;
			} else if (params->TR_phase < -max_phase_value) {
				params->TR_phase = -max_phase_value;
			}

			tr_ph = params->TR_phase;
			tr_int = params->TR_integrator;

			prev_symb = symb;
			i++;
			i_fake = i;
		}

		return i;
	}
}


int sign(float x) {
    if (x > 0) {
        return 1;
    } else if (x < 0) {
        return -1;
    } else {
        return 0;
    }
}

float wrap_to_pi(const float x) {
    float mod_x = remainder(x,2*M_PI);
    return (mod_x > M_PI) ? mod_x - M_PI : mod_x;
}
