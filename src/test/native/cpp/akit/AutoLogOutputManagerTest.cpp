#include <string>
#include <string_view>

#include <frc/geometry/Rotation2d.h>
#include <gtest/gtest.h>
#include <units/length.h>
#include <wpi/struct/Struct.h>

#include "akit/LoggedRobot.h"
#include "akit/Logger.h"
#include "akit/autolog/AutoLogOutput.h"
#include "akit/autolog/AutoLogOutputManager.h"

enum class TestStructuredMode : uint8_t { kIdle = 0, kMoving = 1 };

namespace wpi {
    template<> struct Struct<TestStructuredMode> {
        static constexpr std::string_view GetTypeName() { return "TestStructuredMode"; }
        static constexpr size_t GetSize() { return 1; }
        static constexpr std::string_view GetSchema() { return "uint8 value"; }

        static TestStructuredMode Unpack(std::span<const uint8_t> data) {
            return data.empty() ? TestStructuredMode::kIdle : static_cast<TestStructuredMode>(data[0]);
        }

        static void Pack(std::span<uint8_t> data, TestStructuredMode value) {
            if (!data.empty()) {
                data[0] = static_cast<uint8_t>(value);
            }
        }
    };

    static_assert(StructSerializable<TestStructuredMode>);
} // namespace wpi

class GlobalScopeAutoLog {
public:
    GlobalScopeAutoLog() { velocity = 5.5; }

    AUTOLOG_OUTPUT(double, velocity);
};

namespace {
    using akit::Logger;
    using namespace units::literals;

    class ValidationLoggedRobot : public akit::LoggedRobot {
    public:
        ValidationLoggedRobot()
            : LoggedRobot() {}
    };

    void EnsureLoggedRobotValidationSatisfied() {
        static ValidationLoggedRobot robot;
    }

    namespace nested::deeper {
        class NamespacedAutoLog {
        public:
            NamespacedAutoLog() { velocity = 3.5; }

            AUTOLOG_OUTPUT(double, velocity);
        };
    } // namespace nested::deeper

    class UnitAutoLog {
    public:
        UnitAutoLog() { distance = 4.25; }

        AUTOLOG_OUTPUT(double, distance, akit::DefaultKey, units::meter_t);
    };

    class StrongUnitAutoLog {
    public:
        StrongUnitAutoLog() { distance = 0.3048_m; }

        AUTOLOG_OUTPUT(units::meter_t, distance);
    };

    class PrefixedAutoLog {
    public:
        explicit PrefixedAutoLog(std::string prefix)
            : prefix_(std::move(prefix)) {
            velocity = 2.0;
        }

    private:
        std::string prefix_;
        SET_LOG_PREFIX(prefix_);

    public:
        AUTOLOG_OUTPUT(double, velocity);
    };

    class SupplierAutoLog {
    public:
        SupplierAutoLog() = default;

        void SetRawDistance(const double value) { rawDistance_ = value; }

        void SetMode(const TestStructuredMode mode) { mode_ = mode; }

    private:
        double rawDistance_ = 0.0;
        TestStructuredMode mode_ = TestStructuredMode::kIdle;

    public:
        AUTOLOG_OUTPUT_SUPPLIER(appliedVoltage, rawDistance_ * 2.0);
        AUTOLOG_OUTPUT_SUPPLIER(distanceMeters, rawDistance_, akit::DefaultKey, units::meter_t);
        AUTOLOG_OUTPUT_SUPPLIER(mode, mode_, "Supplier/Mode", akit::NoUnitTag, akit::ForceSerializable);
    };

    class ForceSerializableAutoLog {
    public:
        ForceSerializableAutoLog() { mode = TestStructuredMode::kMoving; }

        AUTOLOG_OUTPUT(TestStructuredMode, mode, "Superstructure/Mode", akit::NoUnitTag, akit::ForceSerializable);
    };

    class DuplicateKeyAutoLog {
    public:
        AUTOLOG_OUTPUT(double, velocity);
    };

    static_assert(!akit::detail::kSupportsExplicitUnit<std::string, units::meter_t>);

    TEST(AutoLogOutputManagerTest, DefaultFieldKeyUsesClassNameAndCapitalizedMemberName) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        nested::deeper::NamespacedAutoLog value;
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/NamespacedAutoLog/Velocity"));
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/RealOutputs/NamespacedAutoLog/Velocity").value), 3.5);

        Logger::End();
        Logger::Clear();
    }

    TEST(AutoLogOutputManagerTest, GlobalScopeClassResolvesKeyToClassNameOnly) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        GlobalScopeAutoLog value;
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/GlobalScopeAutoLog/Velocity"));
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/RealOutputs/GlobalScopeAutoLog/Velocity").value), 5.5);

        Logger::End();
        Logger::Clear();
    }

    TEST(AutoLogOutputManagerTest, ExplicitUnitMetadataUsesNamedWpilibUnit) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        UnitAutoLog value;
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& stored = Logger::GetCurrentStorage().values.at("/RealOutputs/UnitAutoLog/Distance");
        ASSERT_TRUE(stored.unitStr.has_value());
        EXPECT_EQ(*stored.unitStr, units::meter_t{0}.name());
        EXPECT_DOUBLE_EQ(std::get<double>(stored.value), 4.25);

        Logger::End();
        Logger::Clear();
    }

    TEST(AutoLogOutputManagerTest, StrongUnitsUseExistingLoggerPathForMetadata) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        StrongUnitAutoLog value;
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& stored = Logger::GetCurrentStorage().values.at("/RealOutputs/StrongUnitAutoLog/Distance");
        ASSERT_TRUE(stored.unitStr.has_value());
        EXPECT_EQ(*stored.unitStr, units::meter_t{0}.name());
        EXPECT_NEAR(std::get<double>(stored.value), 0.3048, 1e-9);

        Logger::End();
        Logger::Clear();
    }

    TEST(AutoLogOutputManagerTest, SetLogPrefixDisambiguatesRepeatedInstances) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        PrefixedAutoLog left{"LeftModule"};
        PrefixedAutoLog right{"RightModule"};
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/LeftModule/Velocity"));
        ASSERT_TRUE(values.contains("/RealOutputs/RightModule/Velocity"));

        Logger::End();
        Logger::Clear();
    }

    TEST(AutoLogOutputManagerTest, SupplierRegistrationsLogComputedValuesAndForcedStructSerialization) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        SupplierAutoLog value;
        value.SetRawDistance(2.25);
        value.SetMode(TestStructuredMode::kMoving);
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& values = Logger::GetCurrentStorage().values;
        ASSERT_TRUE(values.contains("/RealOutputs/SupplierAutoLog/AppliedVoltage"));
        EXPECT_DOUBLE_EQ(std::get<double>(values.at("/RealOutputs/SupplierAutoLog/AppliedVoltage").value), 4.5);

        const auto& distance = values.at("/RealOutputs/SupplierAutoLog/DistanceMeters");
        ASSERT_TRUE(distance.unitStr.has_value());
        EXPECT_EQ(*distance.unitStr, units::meter_t{0}.name());

        const auto& mode = values.at("/RealOutputs/Supplier/Mode");
        EXPECT_EQ(mode.customTypeStr, wpi::GetStructTypeString<TestStructuredMode>());

        Logger::End();
        Logger::Clear();
    }

    TEST(AutoLogOutputManagerTest, ForceSerializableWrappedFieldUsesStructPathInsteadOfEnumStringPath) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        ForceSerializableAutoLog value;
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& stored = Logger::GetCurrentStorage().values.at("/RealOutputs/Superstructure/Mode");
        EXPECT_EQ(stored.customTypeStr, wpi::GetStructTypeString<TestStructuredMode>());

        Logger::End();
        Logger::Clear();
    }

    TEST(AutoLogOutputManagerTest, DestroyedObjectsUnregisterTheirCallbacks) {
        EXPECT_EXIT(
            {
                Logger::Clear();
                {
                    nested::deeper::NamespacedAutoLog first;
                    (void)first;
                }
                nested::deeper::NamespacedAutoLog second;
                (void)second;
                std::exit(0);
            },
            testing::ExitedWithCode(0), "");
    }

    TEST(AutoLogOutputManagerTest, LoggerClearResetsManagerStateForReRegistration) {
        EXPECT_EXIT(
            {
                Logger::Clear();
                nested::deeper::NamespacedAutoLog first;
                (void)first;
                Logger::Clear();
                nested::deeper::NamespacedAutoLog second;
                (void)second;
                std::exit(0);
            },
            testing::ExitedWithCode(0), "");
    }

    TEST(AutoLogOutputManagerTest, DuplicateKeysFailFastAtRegistration) {
        EXPECT_DEATH(
            {
                Logger::Clear();
                DuplicateKeyAutoLog first;
                DuplicateKeyAutoLog second;
                (void)first;
                (void)second;
            },
            "Duplicate autolog output key");
    }

    TEST(AutoLogOutputManagerTest, AutoLogTimingMetricIsRecorded) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        nested::deeper::NamespacedAutoLog value;
        Logger::Start();
        Logger::PeriodicAfterUser();

        const auto& values = Logger::GetCurrentStorage().values;
        EXPECT_TRUE(values.contains("/RealOutputs/Logger/AutoLogMS"));

        Logger::End();
        Logger::Clear();
    }
} // namespace
