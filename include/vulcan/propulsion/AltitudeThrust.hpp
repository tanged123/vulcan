#pragma once

#include <janus/janus.hpp>
#include <vulcan/quantity/Quantity.hpp>

namespace vulcan::propulsion {

using namespace vulcan::units;

/**
 * @brief Calculates thrust adjusted for ambient pressure (altitude).
 *
 * Standard correction: F = F_vac - P_atm * A_e
 *
 * Note: The implementation plan proposed F = F_vac - (P_atm - P_e) * A_e.
 * However, since F_vac = F_mom + P_e * A_e, substituting into the standard
 * equation: F = F_mom + (P_e - P_atm) * A_e = (F_mom + P_e * A_e) - P_atm * A_e
 *   = F_vac - P_atm * A_e
 *
 * The P_exit parameter is kept in the signature for compatibility with the plan
 * but is not used in the standard calculation derived from F_vac.
 *
 * @tparam Scalar Variable type
 * @param F_vac Thrust in vacuum [N]
 * @param P_atm Ambient atmospheric pressure [Pa]
 * @param P_exit Nozzle exit pressure [Pa] (Unused for F_vac based calculation)
 * @param A_exit Nozzle exit area [m^2]
 * @return Altitude-compensated thrust [N]
 */
template <typename Scalar>
Quantity<N, Scalar> altitude_thrust(Quantity<N, Scalar> F_vac,
                                    Quantity<Pa, Scalar> P_atm, double P_exit,
                                    double A_exit) {
    (void)P_exit;
    return Quantity<N, Scalar>{F_vac.value() - P_atm.value() * A_exit};
}

/**
 * @brief Calculates the thrust coefficient.
 *
 * C_F = F / (P_c * A_t)
 *
 * @tparam Scalar Variable type
 * @param thrust Thrust force [N]
 * @param P_chamber Chamber pressure [Pa]
 * @param A_throat Throat area [m^2]
 * @return Thrust coefficient [dimensionless]
 */
template <typename Scalar>
Quantity<dimensionless, Scalar>
thrust_coefficient(Quantity<N, Scalar> thrust, Quantity<Pa, Scalar> P_chamber,
                   double A_throat) {
    return Quantity<dimensionless, Scalar>{thrust.value() /
                                           (P_chamber.value() * A_throat)};
}

} // namespace vulcan::propulsion
