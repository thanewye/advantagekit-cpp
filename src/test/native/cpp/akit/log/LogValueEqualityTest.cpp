#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "akit/log/LogStorage.h"

namespace {
    using akit::LogValue;

    TEST(LogValueEqualityTest, NaNEqualsNaN) {
        EXPECT_EQ(LogValue{std::numeric_limits<double>::quiet_NaN()}, LogValue{std::numeric_limits<double>::quiet_NaN()});
        EXPECT_EQ(LogValue{std::numeric_limits<float>::quiet_NaN()}, LogValue{std::numeric_limits<float>::quiet_NaN()});
    }

    TEST(LogValueEqualityTest, NaNPayloadsAreEqual) {
        EXPECT_EQ(LogValue{std::nan("1")}, LogValue{std::nan("2")});
    }

    TEST(LogValueEqualityTest, NegativeZeroDiffersFromZero) {
        EXPECT_NE(LogValue{0.0}, LogValue{-0.0});
        EXPECT_NE(LogValue{0.0f}, LogValue{-0.0f});
    }

    TEST(LogValueEqualityTest, ArraysUseFloatingBitsEquality) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const std::vector<double> withNaN{1.0, nan};
        const std::vector<double> ones{1.0, 1.0};
        EXPECT_EQ(LogValue{withNaN}, LogValue{withNaN});
        EXPECT_NE(LogValue{std::vector<double>{0.0}}, LogValue{std::vector<double>{-0.0}});
        EXPECT_NE(LogValue{std::vector<double>{1.0}}, LogValue{ones});
        EXPECT_EQ(LogValue{std::vector<float>{std::numeric_limits<float>::quiet_NaN()}}, LogValue{std::vector<float>{std::numeric_limits<float>::quiet_NaN()}});
    }

    TEST(LogValueEqualityTest, OrdinaryValuesCompareByContent) {
        EXPECT_EQ(LogValue{1.5}, LogValue{1.5});
        EXPECT_NE(LogValue{1.5}, LogValue{2.5});
        EXPECT_NE(LogValue{1.0}, LogValue{1.0f});
        EXPECT_NE(LogValue(1.0, "", "meters"), LogValue(1.0, "", "feet"));
    }
} // namespace
