#pragma once

#include <janus/janus.hpp>

namespace vulcan::tf {

/**
 * @brief Apply rate limiting to a value change (stateless).
 *
 * Limits the delta from current to commanded value.
 *
 * @tparam Scalar Variable type
 * @param current Current value
 * @param commanded Commanded value
 * @param rate_max Maximum rate magnitude [units/s]
 * @param dt Time step [s]
 * @return Scalar Rate-limited next value
 */
template <typename Scalar>
Scalar rate_limit(const Scalar &current, const Scalar &commanded,
                  double rate_max, double dt) {
    Scalar delta = commanded - current;
    Scalar max_delta = static_cast<Scalar>(rate_max * dt);
    return current + janus::clamp(delta, -max_delta, max_delta);
}

/**
 * @brief Apply saturation/clamping (stateless).
 *
 * @tparam Scalar Variable type
 * @param value Value to saturate
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @return Scalar Saturated value
 */
template <typename Scalar>
Scalar saturate(const Scalar &value, const Scalar &min, const Scalar &max) {
    return janus::clamp(value, min, max);
}

/**
 * @brief Apply deadband (stateless).
 *
 * Returns zero for inputs within deadband, otherwise input minus deadband.
 *
 * @tparam Scalar Variable type
 * @param input Input value
 * @param deadband_width Deadband half-width (symmetric around zero)
 * @return Scalar Output with deadband applied
 */
template <typename Scalar>
Scalar deadband(const Scalar &input, double deadband_width) {
    Scalar db = static_cast<Scalar>(deadband_width);
    Scalar sign_input = janus::where(input < Scalar(0), Scalar(-1), Scalar(1));
    Scalar abs_input = janus::where(input < Scalar(0), -input, input);

    return janus::where(abs_input <= db, Scalar(0),
                        sign_input * (abs_input - db));
}

/**
 * @brief Apply hysteresis (stateless computation).
 *
 * Given input and last output, computes new output with hysteresis.
 *
 * @tparam Scalar Variable type
 * @param input Current input command
 * @param last_output Previous output
 * @param hysteresis_width Hysteresis half-width
 * @return Scalar New output
 */
template <typename Scalar>
Scalar hysteresis(const Scalar &input, const Scalar &last_output,
                  double hysteresis_width) {
    Scalar h = static_cast<Scalar>(hysteresis_width);
    Scalar upper = last_output + h;
    Scalar lower = last_output - h;

    return janus::where(input > upper, input - h,
                        janus::where(input < lower, input + h, last_output));
}

} // namespace vulcan::tf
