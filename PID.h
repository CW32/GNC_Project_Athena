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
        class PID
        {
        private:
            /* data */

            static float clamp (float value, float min, float max);
            float kp_;
            float ki_;
            float kd_;

            float outputMin_;
            float outputMax_;

            float integralMin_;
            float integralMax_;

            float integral_;
            float prevError_;

        public:
            PID(float kp,
                float ki,
                float kd,
                float outputMin,
                float outputMax,
                float integralMin,
                float integralMax);

            float update(float setPoint, float measurement, float dt);

            void reset();
        };

        
    } // namespace Tools
    
} // namespace GNC
