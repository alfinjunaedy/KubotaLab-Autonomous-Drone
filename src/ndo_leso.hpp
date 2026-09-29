#pragma once
#include <cmath>
#include <algorithm>

namespace ndo {

class LESO {
public:
    // pwm_min/pwm_max : ESC/RC PWM range (1000-2000)
    // k_t             : thrust-to-throttle gain [N per NORMALIZED throttle unit]
    // mass            : vehicle mass [kg]
    // omega_o         : observer bandwidth [rad/s] -- start ~6-8, tune from clean data
    // d_hat_max       : anti-windup clamp on the disturbance estimate [m/s^2]
    // g               : gravity [m/s^2]
    void configure(float pwm_min, float pwm_max, float k_t, float mass,
                   float omega_o, float d_hat_max = 15.0f, float g = 9.81f) {
        pwm_min_ = pwm_min;
        pwm_max_ = pwm_max;
        b0_ = k_t / mass;
        g_ = g;
        beta1_ = 3.0f * omega_o;
        beta2_ = 3.0f * omega_o * omega_o;
        beta3_ = omega_o * omega_o * omega_o;
        d_hat_max_ = d_hat_max;
        reset();
    }

    void reset() {
        z_hat_ = 0.0f; vz_hat_ = 0.0f; d_hat_ = 0.0f;
        initialized_ = false;
    }

    // z_meas                     : current altitude estimate [m], e.g. pos_neu[2]
    // pwm_applied_last_cycle     : the ACTUAL PWM sent to the throttle channel
    //                              last cycle (post-correction), NOT the raw
    //                              pre-correction PID output.
    // dt                         : loop period [s]
    float update(float z_meas, float pwm_applied_last_cycle, float dt) {
        float u_norm = pwmToNorm(pwm_applied_last_cycle);
        if (!initialized_) {
            z_hat_ = z_meas;   
            initialized_ = true;
        }
        float e = z_hat_ - z_meas;
        float accel_model = b0_ * u_norm - g_;     
        z_hat_  += dt * (vz_hat_ - beta1_ * e);
        vz_hat_ += dt * (accel_model + d_hat_ - beta2_ * e);
        d_hat_  += dt * (-beta3_ * e);
        d_hat_ = std::max(-d_hat_max_, std::min(d_hat_max_, d_hat_));
        float correction_norm = -d_hat_ / b0_;
        const float kMaxCorrectionNorm = 0.15f;
        correction_norm = std::max(-kMaxCorrectionNorm, std::min(kMaxCorrectionNorm, correction_norm));
        return correction_norm;
    }

    float dHat()  const { return d_hat_; }
    float zHat()  const { return z_hat_; }
    float vzHat() const { return vz_hat_; }

private:
    float pwmToNorm(float pwm) const { return (pwm - pwm_min_) / (pwm_max_ - pwm_min_); }

    float pwm_min_ = 1000.0f, pwm_max_ = 2000.0f;
    float b0_ = 1.0f, g_ = 9.81f;
    float beta1_ = 0, beta2_ = 0, beta3_ = 0, d_hat_max_ = 15.0f;
    float z_hat_ = 0.0f, vz_hat_ = 0.0f, d_hat_ = 0.0f;
    bool initialized_ = false;
};
}