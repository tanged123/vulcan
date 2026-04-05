// Eigen NumTraits specialization for vulcan::Quantity<Unit, Rep>
//
// This enables Eigen fixed-size types (Vec3, Mat3, etc.) to hold
// vulcan::Quantity elements, e.g. janus::Vec3<Quantity<m>>.
//
// The specialization is generic over Unit — one definition covers all units.
// It inherits from NumTraits<Rep> so that cost constants and precision match
// the underlying representation (double, float, casadi::MX, ...).
#pragma once

#include <vulcan/quantity/Quantity.hpp>

#include <Eigen/Core>

namespace Eigen {

template <auto Unit, typename Rep>
struct NumTraits<vulcan::Quantity<Unit, Rep>> : NumTraits<Rep> {
    using Real = vulcan::Quantity<Unit, Rep>;
    using NonInteger = vulcan::Quantity<Unit, Rep>;
    using Nested = vulcan::Quantity<Unit, Rep>;
    using Literal = vulcan::Quantity<Unit, Rep>;

    enum {
        IsComplex = 0,
        IsInteger = 0,
        IsSigned = NumTraits<Rep>::IsSigned,
        RequireInitialization = 1,
        ReadCost = NumTraits<Rep>::ReadCost,
        AddCost = NumTraits<Rep>::AddCost,
        MulCost = NumTraits<Rep>::MulCost
    };

    static inline Real epsilon() { return Real{NumTraits<Rep>::epsilon()}; }
    static inline Real dummy_precision() {
        return Real{NumTraits<Rep>::dummy_precision()};
    }
    static inline Real highest() { return Real{NumTraits<Rep>::highest()}; }
    static inline Real lowest() { return Real{NumTraits<Rep>::lowest()}; }
};

} // namespace Eigen
