/**
 * @file Utilities.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */
#include <cmath>

#include <CentroidalMPCWalking/Utilities.h>

float CentroidalMPCWalking::roundoff(float value, unsigned char prec)
{
    const float pow_10 = std::pow(10.0f, (float)prec);
    return std::round(value * pow_10) / pow_10;
}
