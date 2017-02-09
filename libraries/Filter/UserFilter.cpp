/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/// @file	User.cpp
/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
//
#include <inttypes.h>
#include <AP_Math/AP_Math.h>
#include "Filter.h"
#include "UserFilter.h"


template <class T,  uint8_t FILTER_SIZE>
void UserFilter<T,FILTER_SIZE>::update(T sample, uint32_t timestamp)
{
    uint8_t i = FilterWithBuffer<T,FILTER_SIZE>::sample_index;
    uint8_t i1;
    if (i == 0) {
        i1 = FILTER_SIZE-1;
    } else {
        i1 = i-1;
    }
    if (_timestamps[i1] == timestamp) {
        // this is not a new timestamp - ignore
        return;
    }

    // add timestamp before we apply to FilterWithBuffer
    _timestamps[i] = timestamp;

    // call parent's apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);

    _new_data = true;
}

template <class T,  uint8_t FILTER_SIZE>
T UserFilter<T,FILTER_SIZE>::apply(T sample)
{
    // update sample
    //
    // do filter
    //
    // output
}

template <class T,  uint8_t FILTER_SIZE>
void UserFilter<T,FILTER_SIZE>::init(filter_t ft, freq_t cutoff)
{
    if(4 == FILTER_SIZE)
    {
        switch(ft)
        {
            case ft_chebyI:
                // b=cheI_ba(cutoff*FILTER_SIZE); 
                // a=cheI_ba(cutoff*FILTER_SIZE + FILTER_SIZE); 
                break;
            case ft_chebyII:
                break;
            case ft_elliptic:
                break;
            default:
                break;
        }
    }
}

// reset - clear all samples
template <class T, uint8_t FILTER_SIZE>
void UserFilter<T,FILTER_SIZE>::reset(void)
{
    // call parent's apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::reset();
}

// add new instances as needed here
template void UserFilter<float,3>::update(float sample, uint32_t timestamp);
template float UserFilter<float,3>::apply(float sample);
template void UserFilter<float,3>::reset(void);
template void UserFilter<float,4>::update(float sample, uint32_t timestamp);
template float UserFilter<float,4>::apply(float sample);
template void UserFilter<float,4>::reset(void);
template void UserFilter<float,5>::update(float sample, uint32_t timestamp);
template float UserFilter<float,5>::apply(float sample);
template void UserFilter<float,5>::reset(void);
template void UserFilter<float,6>::update(float sample, uint32_t timestamp);
template float UserFilter<float,6>::apply(float sample);
template void UserFilter<float,6>::reset(void);
template void UserFilter<float,7>::update(float sample, uint32_t timestamp);
template float UserFilter<float,7>::apply(float sample);
template void UserFilter<float,7>::reset(void);

template void UserFilter<double,3>::update(double sample, uint32_t timestamp);
template double UserFilter<double,3>::apply(double sample);
template void UserFilter<double,3>::reset(void);
template void UserFilter<double,4>::update(double sample, uint32_t timestamp);
template double UserFilter<double,4>::apply(double sample);
template void UserFilter<double,4>::reset(void);
template void UserFilter<double,5>::update(double sample, uint32_t timestamp);
template double UserFilter<double,5>::apply(double sample);
template void UserFilter<double,5>::reset(void);
template void UserFilter<double,6>::update(double sample, uint32_t timestamp);
template double UserFilter<double,6>::apply(double sample);
template void UserFilter<double,6>::reset(void);
template void UserFilter<double,7>::update(double sample, uint32_t timestamp);
template double UserFilter<double,7>::apply(double sample);
template void UserFilter<double,7>::reset(void);

