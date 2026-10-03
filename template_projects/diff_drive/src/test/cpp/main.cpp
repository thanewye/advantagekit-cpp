#include <catch2/catch_session.hpp>
#include <wpi/hal/HAL.h>

int main(int argc, char** argv) {
    HAL_Initialize();
    Catch::Session session;
    session.configData().allowZeroTests = true;
    return session.run(argc, argv);
}
