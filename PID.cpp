/**
 * @file PID.cpp
 * @brief PID Implementation file
 * 
 * @author Charles Waldron
 * @date 2026-10-26
 */

#include <PID.h>

namespace GNC
{
    namespace Tools
    {
        PID::PID(float kp,
                float ki,
                float kd,
                float outputMin,
                float outputMax,
                float integralMin,
                float integralMax)
                : kp_{kp},
                  ki_{ki},
                  kd_{kd},
                  outputMin_{outputMin},
                  outputMax_{outputMax},
                  integralMin_{integralMin},
                  integralMax_{integralMax},
                  integral_{0.0f},
                  prevError_{0.0f}{}

        float PID::update(float setPoint, float measurement, float dt) {
            if (dt <= 0.0f) {
                return 0.0f;
            }

            const float error = setPoint - measurement;

            integral_ += error * dt;

            integral_ = clamp(integral_, integralMin_, integralMax_);

            const float derivative = (error - prevError_) / dt;

            const float proportionalTerm = kp_ * error;
            const float integralTerm = ki_ * integral_;
            const float derivativeTerm = kd_ * derivative;

            float output = proportionalTerm + integralTerm + derivativeTerm;

            output = clamp(output, outputMin_, outputMax_);

            prevError_ = error;

            return output;
        }

        float PID::clamp(float value, float min, float max) {
            if (value < min) {
                return min;
            }
            if (value > max) {
                return max;
            }

            return value;
        }

        void PID::reset() {
            integral_ = 0.0f;
            prevError_ = 0.0f;
        }
    } // namespace PID
    
} // namespace GNC
