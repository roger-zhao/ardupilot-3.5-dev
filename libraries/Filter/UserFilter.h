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

//
/// @file	User.h
/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"

// 1st parameter <T> is the type of data being filtered.
// 2nd parameter <FILTER_SIZE> is the number of elements in the filter
template <class T, uint8_t FILTER_SIZE>
class UserFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    enum filter_t{
        ft_chebyI = 0,
        ft_chebyII,
        ft_elliptic,
        ft_num
    };

    enum freq_t{
        freq_5Hz = 0,
        freq_10Hz,
        freq_15Hz,
        freq_20Hz,
        freq_25Hz,
        freq_30Hz,
        freq_35Hz,
        freq_40Hz,
        freq_45Hz,
        freq_50Hz,
        freq_55Hz,
        freq_60Hz,
        freq_num 
    };

    // constructor
    UserFilter(filter_t ft, freq_t cutoff) : FilterWithBuffer<T,FILTER_SIZE>() {
        reset();
        init(ft, cutoff);
    };

    // update - Add a new raw value to the filter, but don't recalculate
    void update(T sample, uint32_t timestamp);

    T apply(T sample);


    // init coeffs
    void        init(filter_t ft, freq_t cutoff);

    // reset - clear the filter
    virtual void        reset();

private:
    bool            _new_data;
    T               b[FILTER_SIZE];
    T               a[FILTER_SIZE];

    // microsecond timestamps for samples. This is needed
    // to cope with non-uniform time spacing of the data
    uint32_t        _timestamps[FILTER_SIZE];
};

typedef UserFilter<float,3> UserFilterFloat_Size3;
typedef UserFilter<float,4> UserFilterFloat_Size4;
typedef UserFilter<float,5> UserFilterFloat_Size5;
typedef UserFilter<float,6> UserFilterFloat_Size6;
typedef UserFilter<float,7> UserFilterFloat_Size7;

typedef UserFilter<double,3> UserFilterDouble_Size3;
typedef UserFilter<double,4> UserFilterDouble_Size4;
typedef UserFilter<double,5> UserFilterDouble_Size5;
typedef UserFilter<double,6> UserFilterDouble_Size6;
typedef UserFilter<double,7> UserFilterDouble_Size7;
