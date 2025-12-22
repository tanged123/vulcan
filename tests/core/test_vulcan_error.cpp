#include <gtest/gtest.h>
#include <stdexcept>
#include <vulcan/core/VulcanError.hpp>
#include <vulcan/io/Signal.hpp>

// Assuming janus::JanusError is available and derives from std::exception or
// similar We will test that we can catch VulcanError as JanusError and
// std::runtime_error if applicable

TEST(VulcanErrorTests, InheritanceHierarchy) {
    vulcan::VulcanError err("test error");

    // Verify it inherits from std::runtime_error
    std::runtime_error *re = dynamic_cast<std::runtime_error *>(&err);
    EXPECT_TRUE(re != nullptr);
}

TEST(VulcanErrorTests, SpecificErrorTypes) {
    // IOError
    vulcan::IOError io_err("file not found");
    EXPECT_STREQ(io_err.what(), "[vulcan] IO: file not found");

    // SignalError
    vulcan::SignalError sig_err("bad signal");
    EXPECT_STREQ(sig_err.what(), "[vulcan] Signal: bad signal");

    // AtmosphereError
    vulcan::AtmosphereError atm_err("too high");
    EXPECT_STREQ(atm_err.what(), "[vulcan] Atmosphere: too high");

    // CoordinateError
    vulcan::CoordinateError coord_err("bad lla");
    EXPECT_STREQ(coord_err.what(), "[vulcan] Coordinate: bad lla");

    // GravityError
    vulcan::GravityError grav_err("bad mu");
    EXPECT_STREQ(grav_err.what(), "[vulcan] Gravity: bad mu");
}

TEST(VulcanErrorTests, Catching) {
    // Catch as base class
    try {
        throw vulcan::SignalError("oops");
    } catch (const vulcan::VulcanError &e) {
        EXPECT_STREQ(e.what(), "[vulcan] Signal: oops");
    } catch (...) {
        FAIL() << "Should have caught as VulcanError";
    }

    // Catch as std::exception
    try {
        throw vulcan::SignalError("oops");
    } catch (const std::exception &e) {
        EXPECT_STREQ(e.what(), "[vulcan] Signal: oops");
    } catch (...) {
        FAIL() << "Should have caught as std::exception";
    }
}