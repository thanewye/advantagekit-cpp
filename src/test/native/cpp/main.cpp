#include <hal/HAL.h>

#include "gtest/gtest.h"

int main(int argc, char** argv) {
    HAL_Initialize(500, 0);
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
