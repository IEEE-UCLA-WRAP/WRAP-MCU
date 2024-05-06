#include <math.h>
#include <stdint.h>
#include "constants.h"
#include "receiver.h"

void normalize(const uint16_t * samples, float * norm_samples);
int timing_recovery(float * filtered_samps, float * symbs, params_r * params);
float wrap_to_pi(const float x);

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



int timing_recovery(float * filtered_samps, float * symbs, params_r * params) {
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
        int temp = copysign(1, (filtered_samps[i+1] * filtered_samps[i]));
        zc[i] = !(temp+1);
    }

    // timing recovery
    for (int i = 1; i < ADC_BUF_LEN; i++) {
        phase = prev_phase + 2*M_PI/sps;
        phase = wrap_to_pi(phase);
        if (phase < -M_PI * margin && prev_phase > M_PI * margin) {
            symbs[bit_len] = (int)(filtered_samps[i]/fabs(filtered_samps[i]));
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
}

float wrap_to_pi(const float x) {
    float mod_x = remainder(x,2*M_PI);
    return (mod_x > M_PI) ? mod_x - M_PI : mod_x;
}
