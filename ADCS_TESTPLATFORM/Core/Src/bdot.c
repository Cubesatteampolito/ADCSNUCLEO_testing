/* BDOT Algorithm implementation + duty cycle computation 
*
* Data from magnetometer and gyroscope are used to compute the spacecraft's magnetic dipole momentum.  
* From the momentum, the duty cycle for the PWM is computed and used to actuate. 
* The magnetorquers should then produce this dipole moment to generate a torque that opposes angular velocity. 
*/

#include "bdot.h"
#include "math.h"

/* Return the magnetic dipole moment required to compute the duty cycle starting from mag and gyro readings. */
void compute_mcon
(
    const float mag[3],         // Magnetometer readings
    const float gyro[3],        // Gyroscope readings
    float k,                    // BDOT gain
    float m_con[3]              // Results buffer
) {
    /* compute the norm of the magnetometer vector */
    float b_norm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);               // normalizing magnetic field
    float b[3] = {0, 0, 0};

    if(b_norm != 0)     /* check norm value to avoid division by 0 when normalizing */
    {                                                                        
        /* normalize magnetometer readings to get a unit direction vector (b_hat = b / b_norm) */
        for(int j = 0; j < 3; j++) b[j] = mag[j] / b_norm;                                  

        /* compute the control dipole moment as the vector product (omega x b_hat) between the angular velocity vector omega and the normalized magnetic field vector b_hat */
        m_con[0] =   gyro[1] * b[2] - gyro[2] * b[1];
        m_con[1] = -(gyro[0] * b[2] - gyro[2] * b[0]);                            
        m_con[2] =   gyro[0] * b[1] - gyro[1] * b[0];

        /* normalize the dipole moment after multiplying it by the BDOT gain */
        for(int j = 0; j < 3; j++) 
        {
            m_con[j] = (m_con[j] * k) / b_norm;                                             // m = (k / norm(B)) * (omega x hat{b}) 
        }
    }
    else
    {
        for(int j = 0; j < 3; j++) 
        {
            m_con[j] = 0;
        }
    }
}

/* Update duty_cycle[] to configure PWM on the magnetorquers */
void compute_duty_cycle(
    const float m_con[3],           // Angular momentum computed via BDOT
    const float coil_turn[3],       // n. of physical turns in each coil 
    const float coil_area[3],       // physical area of the coil
    const float reg_coil[3],        // Coil's electric resistance
    const float VDD_coil[3],        // Supply voltage for each coil
    float duty_cycle[3],            // Results buffer
    uint8_t direction[3]            // Computed direction for each coil
) {
    /* For each of the three axes */
    for (int i = 0; i < 3; i++)
    {
        /* Safe duty cycle setting in case of unexpected values */
        if (coil_turn[i] == 0.0f || coil_area[i] == 0.0f || VDD_coil[i] == 0.0f)
        {
            duty_cycle[i] = 0.0f;
            direction[i] = 0u;
            continue;
        }

        float I_cmd = m_con[i] / (coil_turn[i] * coil_area[i]);         // I = angular_momentum / (turns * area)
        float duty  = (I_cmd * reg_coil[i]) / VDD_coil[i];              // duty = (I * R) / VDD. Signed result

        direction[i] = (duty >= 0.0f) ? 1u : 0u;                        // The sign represents the direction
        duty = fabsf(duty) * 100.0f;                                    // Convert to percentage
        if (duty > 80.0f) duty = 80.0f;                                 // Clamp to 80% max to avoid over actuation (testing purposes)

        duty_cycle[i] = duty;                                           // [0,80]%
    }
}


