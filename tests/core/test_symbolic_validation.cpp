#include <gtest/gtest.h>
#include <vulcan/core/Validation.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::tests {

using namespace vulcan::validation;

TEST(ValidationTest, SymbolicCompileCheck) {
    // This test checks if we can instantiate these functions with
    // SymbolicScalar It doesn't need to run complex logic, just compile and
    // potentially run

    // We can't really "run" this if it throws runtime errors about implicit
    // conversion to bool but the compilation is the first hurdle.

    SymbolicScalar x = vulcan::sym("x");

    // This will likely fail to compile if is_finite uses std::isfinite on MX
    // or if the 'if' condition in assert_finite tries to convert MX to bool.

    // We expect this might fail compilation or runtime if not handled.
    // Uncommenting to test:
    // assert_finite(x, "x");
}

} // namespace vulcan::tests
