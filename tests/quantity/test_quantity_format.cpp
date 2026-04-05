// Tests for Quantity::to_string() formatting
#include <gtest/gtest.h>
#include <vulcan/quantity/QuantityFormat.hpp>

namespace vu = vulcan::units;

// =============================================================================
// Numeric formatting
// =============================================================================

TEST(QuantityFormat, MetresContainsValueAndSymbol) {
    auto s = vulcan::Quantity<vu::m>(10000.0).to_string();
    EXPECT_NE(s.find("10000"), std::string::npos) << "got: " << s;
    EXPECT_NE(s.find("m"), std::string::npos) << "got: " << s;
}

TEST(QuantityFormat, KelvinContainsValueAndSymbol) {
    auto s = vulcan::Quantity<vu::K>(288.15).to_string();
    EXPECT_NE(s.find("288.15"), std::string::npos) << "got: " << s;
    EXPECT_NE(s.find("K"), std::string::npos) << "got: " << s;
}

TEST(QuantityFormat, DimensionlessContainsValue) {
    auto s = vulcan::Quantity<vu::dimensionless>(0.85).to_string();
    EXPECT_NE(s.find("0.85"), std::string::npos) << "got: " << s;
}
