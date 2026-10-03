#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/telemetry/MockTelemetryBackend.hpp>
#include <wpi/telemetry/Telemetry.hpp>
#include <wpi/telemetry/TelemetryRegistry.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>

#include "akit/LoggedRobot.h"
#include "akit/Logger.h"
#include "akit/log/LogStorage.h"
#include "akit/log/LogTable.h"
#include "akit/mechanism/LoggedMechanism2d.h"
#include "akit/mechanism/LoggedMechanismLigament2d.h"

namespace {
    using akit::Logger;
    using akit::LogStorage;
    using akit::LogTable;
    using akit::LogValue;
    using akit::mechanism::LoggedMechanism2d;
    using akit::mechanism::LoggedMechanismLigament2d;
    using namespace wpi::units::literals;

    class ValidationLoggedRobot : public akit::LoggedRobot {
    public:
        ValidationLoggedRobot()
            : LoggedRobot() {}
    };

    void EnsureLoggedRobotValidationSatisfied() {
        static ValidationLoggedRobot robot;
    }

    TEST(LoggedMechanism2dTest, LogOutputWritesTreeInAdvantageScopeFormat) {
        LoggedMechanism2d mechanism(3.0, 4.0, wpi::util::Color8Bit{0, 0, 32});
        auto* root = mechanism.GetRoot("Root", 1.0, 0.5);
        auto* arm = root->Append<LoggedMechanismLigament2d>("Arm", 2.0, 90_deg, 6.0, wpi::util::Color8Bit{255, 0, 0});
        arm->Append<LoggedMechanismLigament2d>("Wrist", 0.5, 45_deg);

        LogStorage storage;
        mechanism.LogOutput(LogTable(storage).GetSubtable("Mech"));

        const auto& values = storage.values;
        EXPECT_EQ(values.at("/Mech/.type"), LogValue(std::string("Mechanism2d")));
        EXPECT_FALSE(values.contains("/Mech/.controllable"));
        EXPECT_EQ(values.at("/Mech/dims"), LogValue(std::vector<double>{3.0, 4.0}));
        EXPECT_EQ(values.at("/Mech/backgroundColor"), LogValue(std::string("#000020")));
        EXPECT_EQ(values.at("/Mech/Root/position"), LogValue(std::vector<double>{1.0, 0.5}));
        EXPECT_EQ(values.at("/Mech/Root/Arm/.type"), LogValue(std::string("line")));
        EXPECT_EQ(values.at("/Mech/Root/Arm/angle"), LogValue(90.0));
        EXPECT_EQ(values.at("/Mech/Root/Arm/length"), LogValue(2.0));
        EXPECT_EQ(values.at("/Mech/Root/Arm/color"), LogValue(std::string("#FF0000")));
        EXPECT_EQ(values.at("/Mech/Root/Arm/weight"), LogValue(6.0));
        EXPECT_EQ(values.at("/Mech/Root/Arm/Wrist/angle"), LogValue(45.0));
        EXPECT_EQ(values.at("/Mech/Root/Arm/Wrist/length"), LogValue(0.5));
    }

    TEST(LoggedMechanism2dTest, TelemetryLogMatchesWpilibMechanism2dLayout) {
        using wpi::telemetry::MockTelemetryBackend;
        auto backend = std::make_shared<MockTelemetryBackend>();
        wpi::telemetry::TelemetryRegistry::RegisterBackend("/", backend);

        LoggedMechanism2d mechanism(3.0, 4.0, wpi::util::Color8Bit{0, 0, 32});
        auto* root = mechanism.GetRoot("Root", 1.0, 0.5);
        root->Append<LoggedMechanismLigament2d>("Arm", 2.0, 90_deg, 6.0, wpi::util::Color8Bit{255, 0, 0});
        wpi::telemetry::Log("Mech", mechanism);

        const auto type = backend->GetLastValue<MockTelemetryBackend::LogStringValue>("/Mech/.type");
        ASSERT_TRUE(type.has_value());
        EXPECT_EQ(type->value, "Mechanism2d");
        EXPECT_EQ(backend->GetLastValue<std::vector<double>>("/Mech/dims"), (std::vector<double>{3.0, 4.0}));
        EXPECT_EQ(backend->GetLastValue<std::vector<double>>("/Mech/Root/position"), (std::vector<double>{1.0, 0.5}));
        const auto armType = backend->GetLastValue<MockTelemetryBackend::LogStringValue>("/Mech/Root/Arm/.type");
        ASSERT_TRUE(armType.has_value());
        EXPECT_EQ(armType->value, "line");
        EXPECT_EQ(backend->GetLastValue<double>("/Mech/Root/Arm/angle"), 90.0);
        EXPECT_EQ(backend->GetLastValue<double>("/Mech/Root/Arm/length"), 2.0);
        const auto armColor = backend->GetLastValue<MockTelemetryBackend::LogStringValue>("/Mech/Root/Arm/color");
        ASSERT_TRUE(armColor.has_value());
        EXPECT_EQ(armColor->value, "#FF0000");

        wpi::telemetry::TelemetryRegistry::Reset();
    }

    TEST(LoggedMechanism2dTest, SettersAreReflectedInNextLogOutput) {
        LoggedMechanism2d mechanism(1.0, 1.0);
        auto* root = mechanism.GetRoot("Root", 0.0, 0.0);
        auto* arm = root->Append<LoggedMechanismLigament2d>("Arm", 1.0, 0_deg);

        root->SetPosition(0.25, 0.75);
        arm->SetAngle(30_deg);
        arm->SetLength(1.5);

        LogStorage storage;
        mechanism.LogOutput(LogTable(storage));

        EXPECT_EQ(storage.values.at("/Root/position"), LogValue(std::vector<double>{0.25, 0.75}));
        EXPECT_EQ(storage.values.at("/Root/Arm/angle"), LogValue(30.0));
        EXPECT_EQ(storage.values.at("/Root/Arm/length"), LogValue(1.5));
    }

    TEST(LoggedMechanism2dTest, GetRootReturnsExistingRootForSameName) {
        LoggedMechanism2d mechanism(1.0, 1.0);
        auto* first = mechanism.GetRoot("Root", 0.0, 0.0);
        auto* second = mechanism.GetRoot("Root", 5.0, 5.0);
        EXPECT_EQ(first, second);
    }

    TEST(LoggedMechanism2dTest, AppendingDuplicateNameThrows) {
        LoggedMechanism2d mechanism(1.0, 1.0);
        auto* root = mechanism.GetRoot("Root", 0.0, 0.0);
        root->Append<LoggedMechanismLigament2d>("Arm", 1.0, 0_deg);
        EXPECT_ANY_THROW(root->Append<LoggedMechanismLigament2d>("Arm", 1.0, 0_deg));
    }

    TEST(LoggedMechanism2dTest, Generate3dMechanismFollowsLigamentChainDepthFirst) {
        LoggedMechanism2d mechanism(3.0, 3.0);
        auto* root = mechanism.GetRoot("Root", 1.0, 0.5);
        auto* arm = root->Append<LoggedMechanismLigament2d>("Arm", 2.0, 90_deg);
        arm->Append<LoggedMechanismLigament2d>("Wrist", 0.5, 0_deg);

        const std::vector<wpi::math::Pose3d> poses = mechanism.Generate3dMechanism();
        ASSERT_EQ(poses.size(), 2u);

        EXPECT_NEAR(poses[0].X().value(), 1.0, 1e-9);
        EXPECT_NEAR(poses[0].Y().value(), 0.0, 1e-9);
        EXPECT_NEAR(poses[0].Z().value(), 0.5, 1e-9);
        EXPECT_NEAR(wpi::units::degree_t{poses[0].Rotation().Y()}.value(), -90.0, 1e-6);

        EXPECT_NEAR(poses[1].X().value(), 1.0, 1e-9);
        EXPECT_NEAR(poses[1].Y().value(), 0.0, 1e-9);
        EXPECT_NEAR(poses[1].Z().value(), 2.5, 1e-9);
        EXPECT_NEAR(wpi::units::degree_t{poses[1].Rotation().Y()}.value(), -90.0, 1e-6);
    }

    TEST(LoggedMechanism2dTest, RecordOutputWritesUnderRealOutputs) {
        EnsureLoggedRobotValidationSatisfied();
        Logger::Clear();

        LoggedMechanism2d mechanism(1.0, 2.0);
        mechanism.GetRoot("Root", 0.5, 0.0)->Append<LoggedMechanismLigament2d>("Arm", 1.0, 10_deg);

        Logger::Start();
        Logger::RecordOutput("Superstructure/Mechanism", mechanism);

        const auto& values = Logger::GetCurrentStorage().values;
        EXPECT_EQ(values.at("/RealOutputs/Superstructure/Mechanism/.type"), LogValue(std::string("Mechanism2d")));
        EXPECT_EQ(values.at("/RealOutputs/Superstructure/Mechanism/Root/Arm/angle"), LogValue(10.0));

        Logger::End();
        Logger::Clear();
    }
} // namespace
