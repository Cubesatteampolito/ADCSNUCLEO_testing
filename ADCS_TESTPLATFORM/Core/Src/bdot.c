#include "bdot.h"

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
    for(int i = 0; i < 3; i++) {
        m_con[i] = 0.0f; // Zeroing output for safe
    //A=0.07225 
    //n=300
        i_requred[i]=m_con[i]/(A* n);
        v_required[i]=i_required[i]*Rmagnetorquer[i];
    //Rmagnetorquer=30.5,r

    }
}
