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
 * @brief Compute the BDOT commanded dipole moment.
 * 
 * This function computes the commanded dipole moment for the BDOT algorithm based on the
 * magnetometer readings, gyroscope readings, and a gain factor. The magnetic field is
 * normalized to ensure that the commanded dipole moment is appropriately scaled.
 * 
 * @param mag[3] magnetometer readings (magnetic field vector in Tesla)
 * @param gyro[3] gyroscope readings (angular rate vector in rad/s)
 * @param k BDOT gain (tuning parameter for the control algorithm)
 * @param m_con[3] output buffer for the computed commanded dipole moment (in A·m²)
 */
void compute_mcon(const float mag[3], const float gyro[3], float k, float m_con[3]);


/**
 * @brief Compute the duty cycle for each magnetorquer coil.
 * 
 * The function computes the duty cycle for each magnetorquer coil based on the
 * commanded dipole moment and the coil parameters. The direction of the current
 * is also determined based on the sign of the commanded dipole moment.
 * 
 * @param m_con[3] commanded dipole moment [A·m²]
 * @param coil_turn[3] number of turns for each coil
 * @param coil_area[3] area of each coil [m²]
 * @param reg_coil[3] resistance of each coil [Ohm]
 * @param VDD_coil[3] supply voltage for each coil [V]
 * @param duty_cycle[3] output duty cycle for each coil (0 to 1)
 * @param direction[3] output direction for each coil (0 for positive, 1 for negative)
 */
void compute_duty_cycle(const float m_con[3], const float coil_turn[3] ,const float coil_area[3] ,const float reg_coil[3] ,const float VDD_coil[3] ,float duty_cycle[3] ,uint8_t direction[3]);



#endif // BDOT_H
