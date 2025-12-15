#include "bdot.h"
#include "math.h"
void compute_mcon(const float mag[3], const float gyro[3], float k, float m_con[3])
{
    float B_norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]); // normalizing mag field
    float b[3] = {0,0,0};
    if(B_norm != 0){ // avoid division by 0
        for(int j = 0; j < 3; j++) b[j] = mag[j] / B_norm; // hat{b}

        m_con[0] = gyro[1] * b[2] - gyro[2] * b[1];
        m_con[1] = -(gyro[0] * b[2] - gyro[2] * b[0]); // omega x hat{b}
        m_con[2] = gyro[0] * b[1] - gyro[1] * b[0];

        for(int j = 0; j < 3; j++) {
            m_con[j] = (m_con[j] * k) / B_norm;  // m = (k / norm(B)) * (omega x hat{b}) 
        }
    }
    else{for(int j = 0; j < 3; j++) m_con[j] = 0;}
}
void compute_duty_cycle(const float m_con[3], const float coil_turn[3] ,const float coil_area[3] ,const float reg_coil[3] ,const float VDD_coil[3] ,float duty_cycle[3] ,uint8_t direction[3])
{
    for (int i = 0; i < 3; i++)
    {
        if (coil_turn[i] == 0.0f || coil_area[i] == 0.0f || VDD_coil[i] == 0.0f)
        {
            duty_cycle[i] = 0.0f;
            direction[i] = 0u;
            continue;
        }

        float I_cmd = m_con[i] / (coil_turn[i] * coil_area[i]);      // A
        float duty  = (I_cmd * reg_coil[i]) / VDD_coil[i];           // signed fraction

        direction[i] = (duty >= 0.0f) ? 1u : 0u;                     // sign → direction
        duty = fabsf(duty) * 100.0f;                                 // convert to percent
        if (duty > 100.0f) duty = 100.0f;                            // clamp

        duty_cycle[i] = duty;                                        // 0..100%
    }
}


