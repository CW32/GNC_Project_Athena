/**
 * @file LowPassFilter.cpp
 * @brief Show example usage of the data logging library
 * 
 * @author Charles Waldron
 * @date 2026-11-01
 */

#pragma once

namespace GNC
{
    namespace Tools
    {
        /**
         * @brief A low pass filter object for computing filtered values
         */
        class LPF
        {
            private:
                float alpha_; ///< The constant filter value
                float last_; ///< The last value the filter computed

            public:

                /**
                 * @brief LPF object constructor 
                 * 
                 * @param alpha The inital filter constant
                 * @param initValue The first value of the filter
                 *
                 * @return An LPF object
                 */
                LPF(float alpha = .15f, float initValue = 0) : alpha_{alpha}, last_{initValue} {};

                /**
                 * @brief Comptues the new filter value
                 * @param value The new data
                 * 
                 * @return The filters current value
                 */
                inline float updateFilter(float value) {
                    last_ = (alpha_ * (last_)) + ((1 - alpha_) * value);
                    return last_;
                }

                /**
                 * @brief Return the current value of the filter
                 * @return The last computed filter value
                 */
                inline float getValue() const {
                    return last_;
                }
        };
    } // namespace Tools

} // namespace GNC
