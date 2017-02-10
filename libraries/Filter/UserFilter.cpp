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

extern const AP_HAL::HAL& hal;

#define N_ORDER 4
#define USER_SAMPLE_RATE 8000
// #define USER_SAMPLE_RATE 1000
#define TEST_FILTER  1

// 8KHz
#if USER_SAMPLE_RATE == 8000
double ba_chebyI[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
double ba_chebyII[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
double ba_elliptic[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
#elif USER_SAMPLE_RATE == 1000
double ba_chebyI[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
double ba_chebyII[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
double ba_elliptic[UserFilterDouble_Size5::freq_num][(N_ORDER+1)*2] = {
   };
#endif



template <class T,  uint8_t FILTER_SIZE>
void UserFilter<T,FILTER_SIZE>::update(T sample, uint32_t timestamp)
{

    _new_data = true;
}

#define FORMER(curr, n, array_size) ((curr >= n)?(curr - n):(curr + array_size - n)) 
template <class T,  uint8_t FILTER_SIZE>
T UserFilter<T,FILTER_SIZE>::apply(T sample)
{
    uint16_t curr_idx = _sample_idx;
    if(!_first)
    {
        // update state
        filter_state[_sample_idx++] = sample;
        // wrap index if necessary
        if( _sample_idx >= FILTER_SIZE )
            _sample_idx = 0;

        // filter  
        filter_out[curr_idx] = b[0]*filter_state[curr_idx] 
            + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_SIZE)] 
            - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_SIZE)] 
            + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_SIZE)] 
            - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_SIZE)] 
            + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_SIZE)] 
            - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_SIZE)]
            + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_SIZE)]
            - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_SIZE)];

        if (isnan(filter_out[curr_idx]) || isinf(filter_out[curr_idx])) {

            filter_out[curr_idx] = filter_state[curr_idx]; 
        }

    }
    else
    {

        filter_state[_sample_idx] = sample;
        filter_out[_sample_idx] = sample;

        _sample_idx++;
        if(_sample_idx == FILTER_SIZE)
        {
            _first = false;
            _sample_idx = 0;
        }
    }

    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return filter_out[curr_idx];
}

template <class T,  uint8_t FILTER_SIZE>
Vector3f UserFilter<T,FILTER_SIZE>::apply3d(Vector3f sample)
{
    uint16_t curr_idx = _sample_idx;
    Vector3f ret;
#if TEST_FILTER 
    static uint32_t incr = 0;
    if((0 == incr%4000) || (1 == incr%4000) || (2 == incr%4000))
    {
        hal.util->prt("filter b: %.19f, %.19f, %.19f, %.19f, %.19f", 
                b[0], b[1], b[2], b[3], b[4]);
        hal.util->prt("filter a: %.19f, %.19f, %.19f, %.19f, %.19f", 
                a[0], a[1], a[2], a[3], a[4]);
#if 0
        hal.util->prt("ChebyII filter idx: curr_idx %d, %d, %d, %d, %d", 
                curr_idx,
                FORMER(curr_idx, 1, FILTER_MAX_TAP), 
                FORMER(curr_idx, 2, FILTER_MAX_TAP), 
                FORMER(curr_idx, 3, FILTER_MAX_TAP), 
                FORMER(curr_idx, 4, FILTER_MAX_TAP)); 
        uint8_t med_f_len = _imu.get_mean_filter_former() + _imu.get_mean_filter_latter();
        hal.util->prt("Median filter idx: len: %d, curr_idx %d", med_f_len, curr_idx);
            for(uint8_t med_idx = 0; med_idx < med_f_len; med_idx++)
            {
                uint8_t jj = (med_f_len - med_idx);
                hal.util->prt("in idx: <%d>", FORMER(curr_idx, jj, MED_TAP));
            }
#endif

    }
    incr++;
#endif
    if(!_first)
    {
        // update state
        filter_state[_sample_idx++] = sample;
        // wrap index if necessary
        if( _sample_idx >= FILTER_SIZE )
            _sample_idx = 0;

        // filter x 
        filter_out_3d[curr_idx].x = b[0]*filter_state_3d[curr_idx].x 
            + b[1]*filter_state_3d[FORMER(curr_idx, 1, FILTER_SIZE)].x 
            - a[1]*filter_out_3d[FORMER(curr_idx, 1, FILTER_SIZE)].x 
            + b[2]*filter_state_3d[FORMER(curr_idx, 2, FILTER_SIZE)].x 
            - a[2]*filter_out_3d[FORMER(curr_idx, 2, FILTER_SIZE)].x 
            + b[3]*filter_state_3d[FORMER(curr_idx, 3, FILTER_SIZE)].x 
            - a[3]*filter_out_3d[FORMER(curr_idx, 3, FILTER_SIZE)].x
            + b[4]*filter_state_3d[FORMER(curr_idx, 4, FILTER_SIZE)].x
            - a[4]*filter_out_3d[FORMER(curr_idx, 4, FILTER_SIZE)].x;

        if (isnan(filter_out_3d[curr_idx].x) || isinf(filter_out_3d[curr_idx].x)) {

            filter_out_3d[curr_idx].x = filter_state_3d[curr_idx].x; 
        }

        // filter y 
        filter_out_3d[curr_idx].y = b[0]*filter_state_3d[curr_idx].y 
            + b[1]*filter_state_3d[FORMER(curr_idx, 1, FILTER_SIZE)].y 
            - a[1]*filter_out_3d[FORMER(curr_idx, 1, FILTER_SIZE)].y 
            + b[2]*filter_state_3d[FORMER(curr_idx, 2, FILTER_SIZE)].y 
            - a[2]*filter_out_3d[FORMER(curr_idx, 2, FILTER_SIZE)].y 
            + b[3]*filter_state_3d[FORMER(curr_idx, 3, FILTER_SIZE)].y 
            - a[3]*filter_out_3d[FORMER(curr_idx, 3, FILTER_SIZE)].y
            + b[4]*filter_state_3d[FORMER(curr_idx, 4, FILTER_SIZE)].y
            - a[4]*filter_out_3d[FORMER(curr_idx, 4, FILTER_SIZE)].y;

        if (isnan(filter_out_3d[curr_idx].y) || isinf(filter_out_3d[curr_idx].y)) {

            filter_out_3d[curr_idx].y = filter_state_3d[curr_idx].y; 
        }

        // filter z 
        filter_out_3d[curr_idx].z = b[0]*filter_state_3d[curr_idx].z 
            + b[1]*filter_state_3d[FORMER(curr_idx, 1, FILTER_SIZE)].z 
            - a[1]*filter_out_3d[FORMER(curr_idx, 1, FILTER_SIZE)].z 
            + b[2]*filter_state_3d[FORMER(curr_idx, 2, FILTER_SIZE)].z 
            - a[2]*filter_out_3d[FORMER(curr_idx, 2, FILTER_SIZE)].z 
            + b[3]*filter_state_3d[FORMER(curr_idx, 3, FILTER_SIZE)].z 
            - a[3]*filter_out_3d[FORMER(curr_idx, 3, FILTER_SIZE)].z
            + b[4]*filter_state_3d[FORMER(curr_idx, 4, FILTER_SIZE)].z
            - a[4]*filter_out_3d[FORMER(curr_idx, 4, FILTER_SIZE)].z;

        if (isnan(filter_out_3d[curr_idx].z) || isinf(filter_out_3d[curr_idx].z)) {

            filter_out_3d[curr_idx].z = filter_state_3d[curr_idx].z; 
        }

        ret.x = filter_out_3d[curr_idx].x;
        ret.y = filter_out_3d[curr_idx].y;
        ret.z = filter_out_3d[curr_idx].z;

    }
    else
    {


        filter_state_3d[curr_idx].x = sample.x;
        filter_state_3d[curr_idx].y = sample.y;
        filter_state_3d[curr_idx].z = sample.z;
        filter_out_3d[curr_idx].x = sample.x;
        filter_out_3d[curr_idx].y = sample.y;
        filter_out_3d[curr_idx].z = sample.z;

        _sample_idx++;
        if(_sample_idx == FILTER_SIZE)
        {
            _first = false;
            _sample_idx = 0;
        }
        ret = sample;
    }

    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}


template <class T,  uint8_t FILTER_SIZE>
void UserFilter<T,FILTER_SIZE>::init(filter_t ft, freq_t fr_t)
{
    hal.util->prt("UserFilter: type: %d, cutoff: %dHz", ft, fr_t*5);
    
    if(5 == FILTER_SIZE)
    {
        switch(ft)
        {
            case ft_chebyI:
                b = &ba_chebyI[0][0] + (N_ORDER+1)*2*fr_t;
                a = b + (N_ORDER+1);
                break;
            case ft_chebyII:
                b = &ba_chebyII[0][0] + (N_ORDER+1)*2*fr_t;
                a = b + (N_ORDER+1);
                break;
            case ft_elliptic:
                b = &ba_elliptic[0][0] + (N_ORDER+1)*2*fr_t;
                a = b + (N_ORDER+1);
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
    // FilterWithBuffer<T,FILTER_SIZE>::reset();
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
template void UserFilter<double,5>::init(filter_t ft, freq_t fr_t);
template void UserFilter<double,6>::update(double sample, uint32_t timestamp);
template double UserFilter<double,6>::apply(double sample);
template void UserFilter<double,6>::reset(void);
template void UserFilter<double,7>::update(double sample, uint32_t timestamp);
template double UserFilter<double,7>::apply(double sample);
template void UserFilter<double,7>::reset(void);

