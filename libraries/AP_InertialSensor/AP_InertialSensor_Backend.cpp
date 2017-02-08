#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <DataFlash/DataFlash.h>
#include <AP_Module/AP_Module.h>

// AB ZhaoYJ@2016-11-30 for user-defined 4 order chebyII filter
#define FILTER_TYPE 15 // 7 filters, 4 order with b & a

#define SAMPLE_RATE 1000 // AC3.4+ 1000Hz
// #define SAMPLE_RATE 400 // AC3.4- 400Hz
#define ELLIPTIC_80DB 1

#define CHK_FT_TAP 0

#if SAMPLE_RATE == 400
const double ba[FILTER_TYPE][5*2] = {
    // 0: fc=10Hz
    {0.0009877867510385,-0.003762348901931, 0.005553744695291,-0.003762348901931,
    0.0009877867510385,
    1,   -3.878129734999,    5.641762572816,   -3.648875955419,
    0.8852477379956},
    // 1: fc=20Hz
    {0.001066578484441,-0.003520583754742, 0.004979264107821,-0.003520583754742,
    0.001066578484441,
    1,    -3.75490235187,    5.294313666885,    -3.32185631444,
    0.7825162529919},
    // 2: fc=30Hz
    {0.001235431141961,-0.003233484708145, 0.004349236721367,-0.003233484708145,
    0.001235431141961,
    1,   -3.628903305608,    4.954076395023,   -3.014454340388,
    0.6896343805619},
    // 3: 40Hz
    {0.001504023202492,-0.002833704229474, 0.003768968353254,-0.002833704229474,
    0.001504023202492,
    1,   -3.498597652843,    4.617828546747,     -2.7230591526,
    0.604937864995},
    // 4: 50Hz
    {0.001893594048567,-0.002220262954039, 0.003389066536478,-0.002220262954039,
    0.001893594048567,
    1,   -3.362256889209,    4.282608240118,   -2.444765517273,
    0.527149895089},
    // 5: 60Hz
    {0.002440006636488,-0.001243697363317, 0.003428681549348,-0.001243697363317,
    0.002440006636488,
    1,   -3.217868090935,    3.945654810763,   -2.177266033495,
    0.4553006137634},
    // 6: 100Hz
    {0.007904657035683,   0.0130194351176,  0.01766923337388,   0.0130194351176,
    0.007904657035683,
    1,    -2.50166997543,     2.53332880823,   -1.188409920757,
    0.2162685056381},
#if 0
    // 7: 140Hz
    {    0.03491867202533,    0.109221934686,     0.15231860983,    0.109221934686,
            0.03491867202533,
                1,   -1.283404870008,    1.021296455388,  -0.3523374727386,
                    0.05504571061107},
    // 8: 200Hz
    {    0.08893723310995,   0.3209252123518,   0.4657669085511,   0.3209252123518,
            0.08893723310995,
                1,  -0.2139521883892,   0.5177818092894, -0.03877694557932,
                     0.0204391241538},
#endif
    // elliptic
    // 7: 10Hz
    {0.001622516136187,-0.002828375511362, 0.003810675479761,-0.002828375511362,
    0.001622516136187,
    1,   -3.530382593166,    4.717748289788,   -2.825146415162,
    0.6391810755881},
    // 8: 15Hz 
    {0.00258873589755,-0.001233000776317, 0.003639805616231,-0.001233000776317,
    0.00258873589755,
    1,   -3.283002014681,    4.133347122059,   -2.355458472614,
    0.5114709985556},
    // 9: 20Hz
    {0.004107551662318, 0.002010215185723, 0.005793408195504, 0.002010215185723,
    0.004107551662318,
    1,   -3.030285807338,    3.593357746771,   -1.954731115222,
    0.4097061641801},
    // 10: 25Hz
    {0.006307797121228, 0.007695269382593,  0.01159372061391, 0.007695269382593,
    0.006307797121228,
    1,   -2.774294863632,     3.10056644764,   -1.615273296793,
    0.3286412048251},
    // 11: 30Hz
    {0.009321247485553,  0.01656082346783,  0.02224586730533,  0.01656082346783,
    0.009321247485553,
    1,    -2.51686389898,    2.656330329843,   -1.329451603217,
    0.2640692636513},
    // 12: 35Hz
    {0.01326931768189,  0.02924921742757,  0.03878281187879,  0.02924921742757,
    0.01326931768189,
    1,   -2.259583229684,    2.260845108374,   -1.089950945088,
    0.2126328889613},
#if ELLIPTIC_80DB 
    // 13: 10Hz 80db
    {0.0003980052194648,2.560038094574e-05,  0.00052507500309,2.560038094574e-05,
    0.0003980052194648,
    1,   -3.528358568107,    4.712330426217,   -2.820372195743,
    0.6377739974605},
    // 14: 6Hz 80db
    {0.0001911564530998,-0.0003001208385711,0.0004122437690402,-0.0003001208385711,
    0.0001911564530998,
    1,   -3.721661619608,    5.210745694536,   -3.252226509473,
    0.7633369440482},
#else
    // 13: 40Hz
    {0.01825647670023,  0.04629083875403,  0.06204752664347,  0.04629083875403,
    0.01825647670023,
    1,   -2.003795946524,    1.913402001585,  -0.8899384841482,
    0.1716659149431},
    // 14: 50Hz
    {0.03167074869554,  0.09500734129155,   0.1312278441234,  0.09500734129155,
    0.03167074869554,
    1,   -1.500899021154,    1.356646489988,  -0.5839458813532,
    0.1131673951732},
#endif
};
#elif SAMPLE_RATE == 1000
const double ba[FILTER_TYPE][5*2] = {
    // 0: fc=10Hz
    {  9.941241125902e-05,-0.0003945263576618,0.0005902402068445,-0.0003945263576618,
    9.941241125902e-05,
    1,   -3.972458622135,    5.917754504397,   -3.918130095139,
    0.972834225191},
    // 1: fc=20Hz
    {  0.0001003734958431,-0.0003890744437156,0.0005775969901159,-0.0003890744437156,
          0.0001003734958431,
            1,   -3.944864674435,     5.83610895405,   -3.837599574185,
                 0.9463554896637},
    // 2: fc=30Hz
    {   0.000102862236555,-0.0003835252095693, 0.000562306474394,-0.0003835252095693,
           0.000102862236555,
              1,   -3.917165192517,    5.754909352487,   -3.758241150432,
                   0.9204979709902},
    // 3: 40Hz
    {  0.0001068876168914,-0.0003776890948126,0.0005446874763283,-0.0003776890948126,
          0.0001068876168914,
            1,     -3.8893064155,    5.674005151862,   -3.679896540922,
                 0.8952008890803},
    // 4: 50Hz
    {  0.0001124883762028,-0.0003713038989948,0.0005251459150399,-0.0003713038989948,
          0.0001124883762028,
            1,   -3.861233372095,    5.593248173458,   -3.602415197646,
                 0.8704079111519},
    // 5: 60Hz
    {  0.0001197336574328,-0.0003640298656109,0.0005041830349917,-0.0003640298656109,
          0.0001197336574328,
            1,   -3.832889447413,    5.512491568755,   -3.525653162673,
                 0.8460666319496},
    // 6: 100Hz
    {    0.00016772222904,-0.0003160025650004,0.0004202992163563,-0.0003160025650004,
            0.00016772222904,
                1,   -3.715588329731,     5.18652198639,   -3.223093823181,
                     0.7522839050674},
#if 0
    // 7: 140Hz
    {    0.03491867202533,    0.109221934686,     0.15231860983,    0.109221934686,
            0.03491867202533,
                1,   -1.283404870008,    1.021296455388,  -0.3523374727386,
                    0.05504571061107},
    // 8: 200Hz
    {    0.08893723310995,   0.3209252123518,   0.4657669085511,   0.3209252123518,
            0.08893723310995,
                1,  -0.2139521883892,   0.5177818092894, -0.03877694557932,
                     0.0204391241538},
#endif
    // elliptic
    // 7: 10Hz
    {  0.0001365293649764,-0.0003551533699655,0.0004773999437498,-0.0003551533699655,
          0.0001365293649764,
            1,    -3.81612017141,    5.468667637301,   -3.487687994098,
                 0.8351807202926},
    // 8: 15Hz 
    {  0.0001911802771726,-0.0003000902507431,0.0004122248786295,-0.0003000902507431,
          0.0001911802771726,
            1,   -3.721617458581,    5.210625006096,    -3.25211605352,
                 0.7633031053408},
    // 9: 20Hz
    {  0.0002759610582909,-0.0001858490563426,0.0004075153360141,-0.0001858490563426,
          0.0002759610582909,
            1,   -3.625620035462,    4.958311097943,   -3.029764714886,
                 0.6976619794845},
    // 10: 25Hz
    {  0.0003980885129365,2.576616211313e-05,0.0005251917953061,2.576616211313e-05,
          0.0003980885129365,
            1,    -3.52828560094,     4.71214368799,   -2.820210784204,
                 0.6377269711995},
    // 11: 30Hz
    {  0.0005663497151324,0.0003784008300013,0.0008349291084672,0.0003784008300013,
          0.0005663497151324,
            1,   -3.429768711272,    4.472488606159,   -2.623003094254,
                 0.5830103539969},
    // 12: 35Hz
    {  0.0007906376344732,0.0009193435980036, 0.001411473231841,0.0009193435980036,
          0.0007906376344732,
            1,   -3.330220028609,    4.239661627194,   -2.437669304545,
                 0.5330639730933},
    // deprecated
#if ELLIPTIC_80DB 
    // 13: 10Hz 80db
    {0.0003980052194648,2.560038094574e-05,  0.00052507500309,2.560038094574e-05,
    0.0003980052194648,
    1,   -3.528358568107,    4.712330426217,   -2.820372195743,
    0.6377739974605},
    // 14: 6Hz 80db
    {0.0001911564530998,-0.0003001208385711,0.0004122437690402,-0.0003001208385711,
    0.0001911564530998,
    1,   -3.721661619608,    5.210745694536,   -3.252226509473,
    0.7633369440482},
#else
    // 13: 40Hz
    {0.01825647670023,  0.04629083875403,  0.06204752664347,  0.04629083875403,
    0.01825647670023,
    1,   -2.003795946524,    1.913402001585,  -0.8899384841482,
    0.1716659149431},
    // 14: 50Hz
    {0.03167074869554,  0.09500734129155,   0.1312278441234,  0.09500734129155,
    0.03167074869554,
    1,   -1.500899021154,    1.356646489988,  -0.5839458813532,
    0.1131673951732},
#endif
};
#endif

const extern AP_HAL::HAL& hal;

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &imu) :
    _imu(imu)
{
    _sem = hal.util->new_semaphore();
}

void AP_InertialSensor_Backend::_rotate_and_correct_accel(uint8_t instance, Vector3f &accel) 
{
    /*
      accel calibration is always done in sensor frame with this
      version of the code. That means we apply the rotation after the
      offsets and scaling.
     */

    // rotate for sensor orientation
    accel.rotate(_imu._accel_orientation[instance]);
    
    // apply offsets
    accel -= _imu._accel_offset[instance];

    // apply scaling
    const Vector3f &accel_scale = _imu._accel_scale[instance].get();
    accel.x *= accel_scale.x;
    accel.y *= accel_scale.y;
    accel.z *= accel_scale.z;

    // rotate to body frame
    accel.rotate(_imu._board_orientation);
}

void AP_InertialSensor_Backend::_rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro) 
{
    // rotate for sensor orientation
    gyro.rotate(_imu._gyro_orientation[instance]);
    
    // gyro calibration is always assumed to have been done in sensor frame
    gyro -= _imu._gyro_offset[instance];

    gyro.rotate(_imu._board_orientation);
}

/*
  rotate gyro vector and add the gyro offset
 */
void AP_InertialSensor_Backend::_publish_gyro(uint8_t instance, const Vector3f &gyro)
{
    _imu._gyro[instance] = gyro;
    _imu._gyro_healthy[instance] = true;

    if (_imu._gyro_raw_sample_rates[instance] <= 0) {
        return;
    }

    // publish delta angle
    _imu._delta_angle[instance] = _imu._delta_angle_acc[instance];
    _imu._delta_angle_dt[instance] = _imu._delta_angle_acc_dt[instance];
    _imu._delta_angle_valid[instance] = true;
}

void AP_InertialSensor_Backend::_notify_new_gyro_raw_sample(uint8_t instance,
                                                            const Vector3f &gyro,
                                                            uint64_t sample_us)
{
    float dt;

    if (_imu._gyro_raw_sample_rates[instance] <= 0) {
        return;
    }

    dt = 1.0f / _imu._gyro_raw_sample_rates[instance];

    // call gyro_sample hook if any
    AP_Module::call_hook_gyro_sample(instance, dt, gyro);

    // push gyros if optical flow present
    if (hal.opticalflow)
        hal.opticalflow->push_gyro(gyro.x, gyro.y, dt);
    
    // compute delta angle
    Vector3f delta_angle = (gyro + _imu._last_raw_gyro[instance]) * 0.5f * dt;

    // compute coning correction
    // see page 26 of:
    // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
    // Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
    // see also examples/coning.py
    Vector3f delta_coning = (_imu._delta_angle_acc[instance] +
                             _imu._last_delta_angle[instance] * (1.0f / 6.0f));
    delta_coning = delta_coning % delta_angle;
    delta_coning *= 0.5f;

    if (_sem->take(0)) {
        // integrate delta angle accumulator
        // the angles and coning corrections are accumulated separately in the
        // referenced paper, but in simulation little difference was found between
        // integrating together and integrating separately (see examples/coning.py)
        _imu._delta_angle_acc[instance] += delta_angle + delta_coning;
        _imu._delta_angle_acc_dt[instance] += dt;

        // save previous delta angle for coning correction
        _imu._last_delta_angle[instance] = delta_angle;
        _imu._last_raw_gyro[instance] = gyro;

#if USER_FILTER

    // imu_acc = _accel_filter.apply(imu_acc);

    // imu_gyro = _gyro_filter.apply(imu_gyro);
    //
    uint8_t gyro_user_ft = _imu.get_gyro_user_filter();

    Vector3f _gyro_filtered;

#if CHK_FT_TAP 
    static uint32_t gyro_cnt = 0;
#endif

    if(gyro_user_ft < 0xF) // ChebyII
    {
        _gyro_filtered = _gyro_user_filter(gyro, gyro_user_ft);
    }
    else if(gyro_user_ft == 0xF) // median
    {
        _gyro_filtered = _gyro_median_filter(gyro);
    }
    else if(gyro_user_ft & 0x10) // median + ChebyII(Hz)
    {
        _gyro_filtered = _gyro_median_filter(gyro);
        _gyro_filtered = _gyro_user_filter(_gyro_filtered, gyro_user_ft & 0xF);
    }
    else if(gyro_user_ft & 0x20) // ChebyII(20Hz) + median 
    {
#if CHK_FT_TAP 
        if((0 == (gyro_cnt%4000)) || (1 == (gyro_cnt%4000)))
        {
            hal.util->prt("[%d us] gyro ft: %d, med_tap: %d", AP_HAL::micros(), gyro_user_ft, _imu.get_med_tap_gyro());
        }
#endif
        _gyro_filtered = _gyro_user_filter(gyro, gyro_user_ft & 0xF);
        _gyro_filtered = _gyro_median_filter(_gyro_filtered);
    }
#if CHK_FT_TAP 
    gyro_cnt++;
#endif

        _imu._gyro_filtered[instance] = _gyro_filtered;
#else
        _imu._gyro_filtered[instance] = _imu._gyro_filter[instance].apply(gyro);
#endif

        if (_imu._gyro_filtered[instance].is_nan() || _imu._gyro_filtered[instance].is_inf()) {
            _imu._gyro_filter[instance].reset();
        }
        _imu._new_gyro_data[instance] = true;
        _sem->give();
    }

    DataFlash_Class *dataflash = get_dataflash();
    if (dataflash != nullptr) {
        uint64_t now = AP_HAL::micros64();
        struct log_GYRO pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GYR1_MSG+instance)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            GyrX      : gyro.x,
            GyrY      : gyro.y,
            GyrZ      : gyro.z
        };
        dataflash->WriteBlock(&pkt, sizeof(pkt));
    }
}

/*
  rotate accel vector, scale and add the accel offset
 */
void AP_InertialSensor_Backend::_publish_accel(uint8_t instance, const Vector3f &accel)
{
    _imu._accel[instance] = accel;
    _imu._accel_healthy[instance] = true;

    if (_imu._accel_raw_sample_rates[instance] <= 0) {
        return;
    }

    // publish delta velocity
    _imu._delta_velocity[instance] = _imu._delta_velocity_acc[instance];
    _imu._delta_velocity_dt[instance] = _imu._delta_velocity_acc_dt[instance];
    _imu._delta_velocity_valid[instance] = true;


    if (_imu._accel_calibrator != nullptr && _imu._accel_calibrator[instance].get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
        Vector3f cal_sample = _imu._delta_velocity[instance];

        //remove rotation
        cal_sample.rotate_inverse(_imu._board_orientation);

        // remove scale factors
        const Vector3f &accel_scale = _imu._accel_scale[instance].get();
        cal_sample.x /= accel_scale.x;
        cal_sample.y /= accel_scale.y;
        cal_sample.z /= accel_scale.z;
        
        //remove offsets
        cal_sample += _imu._accel_offset[instance].get() * _imu._delta_velocity_dt[instance] ;

        _imu._accel_calibrator[instance].new_sample(cal_sample, _imu._delta_velocity_dt[instance]);
    }
}

void AP_InertialSensor_Backend::_notify_new_accel_raw_sample(uint8_t instance,
                                                             const Vector3f &accel,
                                                             uint64_t sample_us,
                                                             bool fsync_set)
{
    float dt;

    if (_imu._accel_raw_sample_rates[instance] <= 0) {
        return;
    }

    dt = 1.0f / _imu._accel_raw_sample_rates[instance];

    // call gyro_sample hook if any
    AP_Module::call_hook_accel_sample(instance, dt, accel, fsync_set);
    
    _imu.calc_vibration_and_clipping(instance, accel, dt);

    if (_sem->take(0)) {
        // delta velocity
        _imu._delta_velocity_acc[instance] += accel * dt;
        _imu._delta_velocity_acc_dt[instance] += dt;

#if USER_FILTER

    // imu_acc = _accel_filter.apply(imu_acc);

    // imu_gyro = _gyro_filter.apply(imu_gyro);
    //
    uint8_t acc_user_ft = _imu.get_accl_user_filter();

    Vector3f _accel_filtered;

#if CHK_FT_TAP 
    static uint32_t acc_gyro_cnt = 0;
#endif

    if(acc_user_ft < 0xF) // ChebyII
    {
        _accel_filtered = _accel_user_filter(accel, acc_user_ft);
    }
    else if(acc_user_ft == 0xF) // median
    {
        _accel_filtered = _accel_median_filter(accel);
    }
    else if(acc_user_ft & 0x10) // median + ChebyII(Hz)
    {
        _accel_filtered = _accel_median_filter(accel);
        _accel_filtered = _accel_user_filter(_accel_filtered, acc_user_ft & 0xF);
    }
    else if(acc_user_ft & 0x20) // ChebyII(Hz) + median 
    {
#if CHK_FT_TAP 
        if((0 == (acc_gyro_cnt%4000)) || (1 == (acc_gyro_cnt%4000)))
        {
            hal.util->prt("[%d us] acc ft: %d, med_tap: %d", AP_HAL::micros(), acc_user_ft, _imu.get_med_tap_acc());
        }
        acc_gyro_cnt++; 
#endif
        _accel_filtered = _accel_user_filter(accel, acc_user_ft & 0xF);
        _accel_filtered = _accel_median_filter(_accel_filtered);
    }

    _imu._accel_filtered[instance] = _accel_filtered;
#else
        _imu._accel_filtered[instance] = _imu._accel_filter[instance].apply(accel);
#endif

        if (_imu._accel_filtered[instance].is_nan() || _imu._accel_filtered[instance].is_inf()) {
            _imu._accel_filter[instance].reset();
        }

        _imu.set_accel_peak_hold(instance, _imu._accel_filtered[instance]);

        _imu._new_accel_data[instance] = true;
        _sem->give();
    }

    DataFlash_Class *dataflash = get_dataflash();
    if (dataflash != nullptr) {
        uint64_t now = AP_HAL::micros64();
        struct log_ACCEL pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ACC1_MSG+instance)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            AccX      : accel.x,
            AccY      : accel.y,
            AccZ      : accel.z
        };
        dataflash->WriteBlock(&pkt, sizeof(pkt));
    }
}

void AP_InertialSensor_Backend::_set_accel_max_abs_offset(uint8_t instance,
                                                          float max_offset)
{
    _imu._accel_max_abs_offsets[instance] = max_offset;
}

// set accelerometer error_count
void AP_InertialSensor_Backend::_set_accel_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._accel_error_count[instance] = error_count;
}

// set gyro error_count
void AP_InertialSensor_Backend::_set_gyro_error_count(uint8_t instance, uint32_t error_count)
{
    _imu._gyro_error_count[instance] = error_count;
}

// increment accelerometer error_count
void AP_InertialSensor_Backend::_inc_accel_error_count(uint8_t instance)
{
    _imu._accel_error_count[instance]++;
}

// increment gyro error_count
void AP_InertialSensor_Backend::_inc_gyro_error_count(uint8_t instance)
{
    _imu._gyro_error_count[instance]++;
}

// return the requested sample rate in Hz
uint16_t AP_InertialSensor_Backend::get_sample_rate_hz(void) const
{
    // enum can be directly cast to Hz
    return (uint16_t)_imu._sample_rate;
}

/*
  publish a temperature value for an instance
 */
void AP_InertialSensor_Backend::_publish_temperature(uint8_t instance, float temperature)
{
    _imu._temperature[instance] = temperature;

    /* give the temperature to the control loop in order to keep it constant*/
    if (instance == 0) {
        hal.util->set_imu_temp(temperature);
    }
}

/*
  common gyro update function for all backends
 */
void AP_InertialSensor_Backend::update_gyro(uint8_t instance)
{    
    if (!_sem->take(0)) {
        return;
    }

    if (_imu._new_gyro_data[instance]) {
        _publish_gyro(instance, _imu._gyro_filtered[instance]);
        _imu._new_gyro_data[instance] = false;
    }

    // possibly update filter frequency
    if (_last_gyro_filter_hz[instance] != _gyro_filter_cutoff()) {
        _imu._gyro_filter[instance].set_cutoff_frequency(_gyro_raw_sample_rate(instance), _gyro_filter_cutoff());
        _last_gyro_filter_hz[instance] = _gyro_filter_cutoff();
    }

    _sem->give();
}

/*
  common accel update function for all backends
 */
void AP_InertialSensor_Backend::update_accel(uint8_t instance)
{    
    if (!_sem->take(0)) {
        return;
    }

    if (_imu._new_accel_data[instance]) {
        _publish_accel(instance, _imu._accel_filtered[instance]);
        _imu._new_accel_data[instance] = false;
    }
    
    // possibly update filter frequency
    if (_last_accel_filter_hz[instance] != _accel_filter_cutoff()) {
        _imu._accel_filter[instance].set_cutoff_frequency(_accel_raw_sample_rate(instance), _accel_filter_cutoff());
        _last_accel_filter_hz[instance] = _accel_filter_cutoff();
    }

    _sem->give();
}


#if USER_FILTER 
#define TEST_FILTER 0

#define FORMER(curr, n, array_size) ((curr >= n)?(curr - n):(curr + array_size - n)) 

static double median_filter(double *pimu_in, uint8_t median_len)
{
    int i,j;
    double ret;  
    double bTemp;  

    for (j = 0; j < (median_len - 1); j++)  
    {  
        for (i = 0; i < (median_len - j - 1); i++)  
        {  
            if (pimu_in[i] > pimu_in[i + 1])  
            {  
                bTemp = pimu_in[i];  
                pimu_in[i] = pimu_in[i + 1];  
                pimu_in[i + 1] = bTemp;  
            }  
        }  
    }  

    // 计算中值  
    if ((median_len & 1) > 0)  
    {  
        // 数组有奇数个元素，返回中间一个元素  
        ret = pimu_in[median_len / 2];  
    }  
    else  
    {  
        // 数组有偶数个元素，返回中间两个元素平均值  
        ret = (pimu_in[median_len / 2 - 1] + pimu_in[median_len / 2]) / 2;  
    }  
  
    return ret;  

}


Vector3f AP_InertialSensor_Backend::_accel_user_filter(Vector3f _accl_in, uint8_t _uf)
{
    //  for ChebyII
#define FILTER_MAX_TAP 8
    static Vector3d filter_state[FILTER_MAX_TAP]; 
    static Vector3d filter_out[FILTER_MAX_TAP]; 
    static uint8_t curr_idx = 0;
    static bool first = true;
    Vector3f ret;
    // Chebyshev II
    const double *b;
    const double *a;


    {
        if(_uf >= FILTER_TYPE) 
        {
            hal.util->prt("[Err] acc filter type wrong: %d", _uf);
            return ret;
        }

        b = &ba[0][0] + (N_ORDER+1)*2*_uf;
        a = b + (N_ORDER+1);

#if TEST_FILTER 
    static uint32_t incr = 0;
    // if((0 == incr%4000) || (1 == incr%4000) || (2 == incr%4000))
    {
#if 0
        hal.util->prt("acc filter: %d", _imu.get_accl_user_filter());
        hal.util->prt("gyro filter: %d", _imu.get_gyro_user_filter());
        hal.util->prt("mean filter former: %d", _imu.get_mean_filter_former());
        hal.util->prt("mean filter latter: %d", _imu.get_mean_filter_latter());
        hal.util->prt("sizeof ba: %d", sizeof(ba));
        hal.util->prt("filter b: %.19f, %.19f, %.19f, %.19f, %.19f", 
                b[0], b[1], b[2], b[3], b[4]);
        hal.util->prt("filter a: %.19f, %.19f, %.19f, %.19f, %.19f", 
                a[0], a[1], a[2], a[3], a[4]);
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
        if(!first)
        {
            // update state
            filter_state[curr_idx].x = _accl_in.x;
            filter_state[curr_idx].y = _accl_in.y;
            filter_state[curr_idx].z = _accl_in.z;
            // filter x: 
            filter_out[curr_idx].x = b[0]*filter_state[curr_idx].x 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].x 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].x 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].x 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].x 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].x 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].x
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].x
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].x;

            if (isnan(filter_out[curr_idx].x) || isinf(filter_out[curr_idx].x)) {

                filter_out[curr_idx].x = filter_state[curr_idx].x; 
            }

            // filter y
            filter_out[curr_idx].y = b[0]*filter_state[curr_idx].y 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].y 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].y 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].y 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].y 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].y 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].y
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].y
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].y;

            if (isnan(filter_out[curr_idx].y) || isinf(filter_out[curr_idx].y)) {

                filter_out[curr_idx].y = filter_state[curr_idx].y; 
            }

            // filter z
            filter_out[curr_idx].z = b[0]*filter_state[curr_idx].z 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].z 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].z 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].z 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].z 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].z 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].z
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].z
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].z;

            if (isnan(filter_out[curr_idx].z) || isinf(filter_out[curr_idx].z)) {

                filter_out[curr_idx].z = filter_state[curr_idx].z; 
            }

            ret.x = filter_out[curr_idx].x;
            ret.y = filter_out[curr_idx].y;
            ret.z = filter_out[curr_idx].z;

            // update filter postion
            curr_idx++;
            curr_idx &= FILTER_MAX_TAP - 1;
            
        }
        else
        {

            filter_state[curr_idx].x = _accl_in.x;
            filter_state[curr_idx].y = _accl_in.y;
            filter_state[curr_idx].z = _accl_in.z;
            filter_out[curr_idx].x = _accl_in.x;
            filter_out[curr_idx].y = _accl_in.y;
            filter_out[curr_idx].z = _accl_in.z;

            curr_idx++;
            if(curr_idx == FILTER_MAX_TAP)
            {
                first = false;
                curr_idx = 0;
            }
            ret = _accl_in;
        }
    }

    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}

Vector3f AP_InertialSensor_Backend::_accel_median_filter(Vector3f _accl_in)
{
    // for median filter: circular buff 16
#define MED_TAP 64
    static Vector3d med_filter_in[MED_TAP];
    static uint8_t curr_idx = 0;
    static bool first = true;
    Vector3f ret;

    if(!first)
    {
        uint8_t med_len = _imu.get_med_tap_acc() + 1; // include current in
        if(med_len > 1)
        {
            double med_in_x[MED_TAP]; 
            double med_in_y[MED_TAP];
            double med_in_z[MED_TAP];
            med_filter_in[curr_idx].x = _accl_in.x;
            med_filter_in[curr_idx].y = _accl_in.y;
            med_filter_in[curr_idx].z = _accl_in.z;
            for(uint8_t med_idx = 0; med_idx < med_len; med_idx++)
            {
                uint8_t dist = med_len - 1 - med_idx;
                med_in_x[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].x;
                med_in_y[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].y;
                med_in_z[med_idx] = med_filter_in[FORMER(curr_idx, dist, MED_TAP)].z;
            }
            ret.x = median_filter(med_in_x, med_len);  
#if 0
            static uint32_t cnt_xx = 0;
            if((0 == (cnt_xx%1000)) || (1 == (cnt_xx%1000)))
            {
                hal.util->prt("ffffmedian x: <%f>, med_len: %d", ret.x, med_len);
                hal.util->prt("median x: before <%f>, current <%f>, average <%f>", med_in_x[med_len/2 - 1], med_in_x[med_len/2], (med_in_x[med_len/2 - 1]+med_in_x[med_len/2])/2);
            }
            cnt_xx++;
#endif

            ret.y = median_filter(med_in_y, med_len);  
            ret.z = median_filter(med_in_z, med_len);  
            curr_idx++;
            curr_idx &= MED_TAP - 1;
        }
        else
        {
            // hal.util->prt("[Err] acc mean filter param wrong: ");
            return _accl_in;
        }
    }
    else
    {
        med_filter_in[curr_idx].x = _accl_in.x;
        med_filter_in[curr_idx].y = _accl_in.y;
        med_filter_in[curr_idx].z = _accl_in.z;

        curr_idx++;
        if(curr_idx == MED_TAP)
        {
            first = false;
            curr_idx = 0;
        }
        ret = _accl_in;
    }


    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}

Vector3f AP_InertialSensor_Backend::_gyro_user_filter(Vector3f _gyro_in, uint8_t _uf)
{
    //  for ChebyII
#define FILTER_MAX_TAP 8
    static Vector3d filter_state[FILTER_MAX_TAP]; 
    static Vector3d filter_out[FILTER_MAX_TAP]; 
    static uint8_t curr_idx = 0;
    static bool first = true;
    Vector3f ret;
    // Chebyshev II
    const double *b;
    const double *a;


    {
        if(_uf >= FILTER_TYPE) 
        {
            hal.util->prt("[Err] gyro filter type wrong: %d", _uf);
            return ret;
        }

        b = &ba[0][0] + (N_ORDER+1)*2*_uf;
        a = b + (N_ORDER+1);

#if TEST_FILTER 
    static uint32_t incr = 0;
    // if((0 == incr%4000) || (1 == incr%4000) || (2 == incr%4000))
    {
#if 0
        hal.util->prt("acc filter: %d", _imu.get_accl_user_filter());
        hal.util->prt("gyro filter: %d", _imu.get_gyro_user_filter());
        hal.util->prt("mean filter former: %d", _imu.get_mean_filter_former());
        hal.util->prt("mean filter latter: %d", _imu.get_mean_filter_latter());
        hal.util->prt("sizeof ba: %d", sizeof(ba));
        hal.util->prt("filter b: %.19f, %.19f, %.19f, %.19f, %.19f", 
                b[0], b[1], b[2], b[3], b[4]);
        hal.util->prt("filter a: %.19f, %.19f, %.19f, %.19f, %.19f", 
                a[0], a[1], a[2], a[3], a[4]);
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
        if(!first)
        {
            // update state
            filter_state[curr_idx].x = _gyro_in.x;
            filter_state[curr_idx].y = _gyro_in.y;
            filter_state[curr_idx].z = _gyro_in.z;
            // filter x: 
            filter_out[curr_idx].x = b[0]*filter_state[curr_idx].x 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].x 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].x 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].x 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].x 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].x 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].x
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].x
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].x;

            if (isnan(filter_out[curr_idx].x) || isinf(filter_out[curr_idx].x)) {

                filter_out[curr_idx].x = filter_state[curr_idx].x; 
            }

            // filter y
            filter_out[curr_idx].y = b[0]*filter_state[curr_idx].y 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].y 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].y 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].y 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].y 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].y 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].y
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].y
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].y;

            if (isnan(filter_out[curr_idx].y) || isinf(filter_out[curr_idx].y)) {

                filter_out[curr_idx].y = filter_state[curr_idx].y; 
            }

            // filter z
            filter_out[curr_idx].z = b[0]*filter_state[curr_idx].z 
                + b[1]*filter_state[FORMER(curr_idx, 1, FILTER_MAX_TAP)].z 
                - a[1]*filter_out[FORMER(curr_idx, 1, FILTER_MAX_TAP)].z 
                + b[2]*filter_state[FORMER(curr_idx, 2, FILTER_MAX_TAP)].z 
                - a[2]*filter_out[FORMER(curr_idx, 2, FILTER_MAX_TAP)].z 
                + b[3]*filter_state[FORMER(curr_idx, 3, FILTER_MAX_TAP)].z 
                - a[3]*filter_out[FORMER(curr_idx, 3, FILTER_MAX_TAP)].z
                + b[4]*filter_state[FORMER(curr_idx, 4, FILTER_MAX_TAP)].z
                - a[4]*filter_out[FORMER(curr_idx, 4, FILTER_MAX_TAP)].z;

            if (isnan(filter_out[curr_idx].z) || isinf(filter_out[curr_idx].z)) {

                filter_out[curr_idx].z = filter_state[curr_idx].z; 
            }

            ret.x = filter_out[curr_idx].x;
            ret.y = filter_out[curr_idx].y;
            ret.z = filter_out[curr_idx].z;

            // update filter postion
            curr_idx++;
            curr_idx &= FILTER_MAX_TAP - 1;
            
        }
        else
        {
            filter_state[curr_idx].x = _gyro_in.x;
            filter_state[curr_idx].y = _gyro_in.y;
            filter_state[curr_idx].z = _gyro_in.z;
            filter_out[curr_idx].x = _gyro_in.x;
            filter_out[curr_idx].y = _gyro_in.y;
            filter_out[curr_idx].z = _gyro_in.z;
            curr_idx++;
            if(curr_idx == FILTER_MAX_TAP)
            {
                first = false;
                curr_idx = 0;
            }
            ret = _gyro_in;
        }
    }

    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}

Vector3f AP_InertialSensor_Backend::_gyro_median_filter(Vector3f _gyro_in)
{
    // for median filter: circular buff 16
#define MED_TAP 64
    static Vector3d gyro_med_filter_in[MED_TAP];
    static uint8_t curr_idx = 0;
    static bool first = true;
    Vector3f ret;
    if(!first)
    {
        uint8_t med_len = _imu.get_med_tap_gyro() + 1; // include current in
        if(med_len > 1)
        {
            double med_in_x[MED_TAP]; 
            double med_in_y[MED_TAP];
            double med_in_z[MED_TAP];
            gyro_med_filter_in[curr_idx].x = _gyro_in.x;
            gyro_med_filter_in[curr_idx].y = _gyro_in.y;
            gyro_med_filter_in[curr_idx].z = _gyro_in.z;
            for(uint8_t med_idx = 0; med_idx < med_len; med_idx++)
            {
                uint8_t dist = med_len - 1 - med_idx;
                med_in_x[med_idx] = gyro_med_filter_in[FORMER(curr_idx, dist, MED_TAP)].x;
                med_in_y[med_idx] = gyro_med_filter_in[FORMER(curr_idx, dist, MED_TAP)].y;
                med_in_z[med_idx] = gyro_med_filter_in[FORMER(curr_idx, dist, MED_TAP)].z;
            }
            ret.x = median_filter(med_in_x, med_len);  
            ret.y = median_filter(med_in_y, med_len);  
            ret.z = median_filter(med_in_z, med_len);  
            curr_idx++;
            curr_idx &= MED_TAP - 1;
        }
        else
        {
            // hal.util->prt("[Err] acc mean filter param wrong: ");
            return _gyro_in;
        }
    }
    else
    {
        gyro_med_filter_in[curr_idx].x = _gyro_in.x;
        gyro_med_filter_in[curr_idx].y = _gyro_in.y;
        gyro_med_filter_in[curr_idx].z = _gyro_in.z;

        curr_idx++;
        if(curr_idx == MED_TAP)
        {
            first = false;
            curr_idx = 0;
        }
        ret = _gyro_in;
    }


    // hal.util->prt("[ %d us] ICM20689 filter end", hal.scheduler->micros()); 
    return ret;
}

#endif
