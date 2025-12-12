/**
 * @file bdot.h
 * @author Lazzaro Francesco Sangiovanni (s342674@studenti.polito.it)
 * @brief bdot algorithm implementation
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

#endif // BDOT_H
