/*
    Kalman Filter:
    SimpleKalmanFilter from denyssene is used as a reference      
    https://github.com/denyssene/SimpleKalmanFilter
*/

#ifndef _kalman_h
#define _kalman_h

class Kalman {
public:
    Kalman(float mea_e, float est_e, float q) {
        setParameters(mea_e, est_e, q);
    }
    
    Kalman(float mea_e, float q) {
        setParameters(mea_e, mea_e, q);
    }
    
    void setParameters(float mea_e, float est_e, float q) {
        _err_measure = mea_e;
        _err_estimate = est_e;
        _q = q;
    }
    
    void setParameters(float mea_e, float q) {
        setParameters(mea_e, mea_e, q);
    }
    
    float updateFilter(float value) {		
        float _kalman_gain, _current_estimate;
        _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
        _current_estimate = _last_estimate + _kalman_gain * (value - _last_estimate);
        _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
        _last_estimate=_current_estimate;
        return _current_estimate;
    }

private:
    float _err_measure = 0.0;
    float _err_estimate = 0.0;
    float _q = 0.0;
    float _last_estimate = 0.0;
};
#endif