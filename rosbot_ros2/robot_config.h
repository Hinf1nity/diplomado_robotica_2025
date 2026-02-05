#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

// --- Physical Dimensions (Constants) ---
const float WHEEL_BASE = 0.22; 
const float WHEEL_RADIUS = 0.035; 

// --- Motor/Encoder Parameters ---

const int ENCODER_TPR = 4600; // verificar
const int MAX_PWM_SPEED = 255; 
// const int MAX_WHEEL_VEL = 0.15; // m/s
const int MAX_WHEEL_VEL = 0.07;

// encoders
const uint left_enc_ch1 = 34;
const uint left_enc_ch2 = 35;

const uint right_enc_ch1 = 19;
const uint right_enc_ch2 = 23;

#endif // ROBOTCONFIG_H
