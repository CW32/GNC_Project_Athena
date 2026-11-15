/**
 * @file NumericalDerivative.h
 * @brief Numerical Derivative implementations in cpp for arduino use
 * 
 * @author Charles Waldron
 * @date 2026-11-01
 */
#pragma once
#include <cstdint>

namespace GNC
{
    namespace Tools
    {
        /**
         * @brief Calculate the numerical derivative from two points
         * @param y2 the second y dependent variable
         * @param y1 the first y dependent variable
         * @param x2 the second independent variable
         * @param x1 the first independent variable
         * 
         * @return The computed derivative value
         */
        inline float numericalDerivative(float y2, float y1, float x2, float x1) {

            const float dx = x2 - x1;

            if (dx == 0.0f) {
                return 0.0f;
            }

            return (y2 - y1) / dx;
        }

        /**
         * @brief Calculate the numerical derivative from two points and a step
         * @param y2 the second y dependent variable
         * @param y1 the first y dependent variable
         * @param h the independent variable step
         * 
         * @return The computed derivative value
         */
        inline float numericalDerivative(float y2, float y1, float h) {

            if (h == 0.0f) {
                return 0.0f;
            }

            return (y2 - y1) / h;
        }

        /**
         * @brief Calculate the numerical derivative from two points and a step
         * @param y2 the second y dependent variable
         * @param y1 the first y dependent variable
         * @param h the independent variable step
         * 
         * @return The computed derivative value
         */
        inline float numericalDerivative(float y2, float y1, uint32_t dt) {
            // dt takes the units it came from (micros or millis)

            if (dt == 0U) {
                return 0.0f;
            }

            return (y2 - y1) / static_cast<float>(dt);
        }
    } // namespace Tools

} // namespace GNC