/*
 * receiver.h
 *
 *  Created on: May 20, 2023
 *      Author: natha
 */

#ifndef SRC_RECEIVER_H_
#define SRC_RECEIVER_H_

#include "constants.h"

#define ORDER 5     				// low pass filter order

// parameters to pass between calls of receiver
typedef struct params_r {
	float CL_integrator; // integrator
    float CL_phase; // costas loop phase
    float TR_integrator;
    float TR_phase; // timing recovery phase
    float sps;      // last sps
} params_r;

static float lp[ORDER + 1] = {0.238053579626575,0.064423148855147,0.0684583456734438,0.0684583456734438,0.0644231488551473,0.238053579626575};
static float key[15] = {1, 1, 1, -1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1}; // packet header
void normalize(const uint16_t * samples, float * norm_samples);
int timing_recovery(float * filtered_samps, float * symbs, params_r * params);
float wrap_to_pi(const float x);

#endif /* SRC_RECEIVER_H_ */
