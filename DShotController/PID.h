/**
 * @file PID.cpp
 * @brief A PID controller class
 * 
 * @author Charles Waldron
 * @date 2026-10-26
 */
#pragma once

namespace GNC
{
    namespace Tools
    {
        /**
        * @brief A PID controller for a single I/O mapping
        */
        class PID
        {
        private:
            static float clamp (float value, float min, float max);
            float kp_; ///< Proportional controller value
            float ki_; ///< Integral controller value
            float kd_; ///< Derivative controller value

            float outputMin_; ///< Output min clamp
            float outputMax_; ///< Output max clamp

            float integralMin_; ///< Integral min clamp
            float integralMax_; ///< Integral max clamp

            float integral_; ///< Integral value
            float prevError_; ///< Error for the last update

        public:
            /**
             * @brief PID constructor 
             * 
             * @param kp The proportional constant
             * @param ki The integral constant
             * @param kd The derivative constant
             * @param outputMin The minimum value the output is clamped to
             * @param outputMax The maximum value the output is clamped to
             * @param integralMin The minimum value the integral error can be
             * @param integralMax The maximum value the integral error can be
             */
            PID(float kp,
                float ki,
                float kd,
                float outputMin,
                float outputMax,
                float integralMin,
                float integralMax);

            /**
             * @brief Update the error in the PID controller
             * 
             * @param setPoint The desired value to move towards
             * @param measurement The value to input to the controller
             * @param dt The change in time since the last measurement
             * @return The output clamped from [min, max] to apply 
             */
            float update(float setPoint, float measurement, float dt);

            /**
             * @brief Reset the integral and previous error
             * 
             * @return Nothing
             */
            void reset();
        };

        
    } // namespace Tools
    
} // namespace GNC
