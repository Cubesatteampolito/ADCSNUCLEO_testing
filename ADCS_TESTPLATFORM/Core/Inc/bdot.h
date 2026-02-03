/**
 * @file bdot.h
 * @author Lazzaro Francesco Sangiovanni (s342674@studenti.polito.it)
 * @brief BDOT Algorithm Implementation
 * 
 */
#ifndef BDOT_H
#define BDOT_H

#include <stdint.h>
#include <math.h>

/**
 * Compute the BDOT commanded dipole moment:
 *
 * Inputs:
 *   mag[3]  : magnetic field vector [Tesla]
 *   gyro[3] : angular rate vector [rad/s]
 *   k       : BDOT gain
 *
 * Output:
 *   m_con[3]: dipole moment command [A·m²]
 *
 */
void compute_mcon(const float mag[3], const float gyro[3], float k, float m_con[3]);

/**
 * Compute the duty cycle for each magnetorquer coil:
 *
 * Inputs:
 * bruh iwill do this later i am out of time
 *
 * Output:
 *   m_con[3]: dipole moment command [A·m²]
 *
 */
void compute_duty_cycle(const float m_con[3], const float coil_turn[3] ,const float coil_area[3] ,const float reg_coil[3] ,const float VDD_coil[3] ,float duty_cycle[3] ,uint8_t direction[3]);



#endif // BDOT_H
