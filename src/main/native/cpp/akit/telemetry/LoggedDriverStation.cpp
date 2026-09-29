#include "pch.h"

#include "akit/telemetry/LoggedDriverStation.h"

#include <string>
#include <string_view>
#include <vector>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/DriverStation.h>

namespace {
    std::string Trim(const std::string_view value) {
        size_t start = 0;
        while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start]))) {
            start++;
        }

        size_t end = value.size();
        while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
            end--;
        }

        return std::string(value.substr(start, end - start));
    }
} // namespace

namespace akit {
    void LoggedDriverStation::SaveToLog(const LogTable& table) {
        int32_t status = 0;

        table.Put("AllianceStation", static_cast<int64_t>(HAL_GetAllianceStation(&status)));
        table.Put("EventName", Trim(frc::DriverStation::GetEventName()));
        table.Put("GameSpecificMessage", Trim(frc::DriverStation::GetGameSpecificMessage()));
        table.Put("MatchNumber", frc::DriverStation::GetMatchNumber());
        table.Put("ReplayNumber", frc::DriverStation::GetReplayNumber());
        table.Put("MatchType", static_cast<int64_t>(frc::DriverStation::GetMatchType()));
        table.Put("MatchTime", frc::DriverStation::GetMatchTime());

        table.Put("Enabled", frc::DriverStation::IsEnabled());
        table.Put("Autonomous", frc::DriverStation::IsAutonomous());
        table.Put("Test", frc::DriverStation::IsTest());
        table.Put("EmergencyStop", frc::DriverStation::IsEStopped());
        table.Put("FMSAttached", frc::DriverStation::IsFMSAttached());
        table.Put("DSAttached", frc::DriverStation::IsDSAttached());

        for (int id = 0; id < frc::DriverStation::kJoystickPorts; id++) {
            LogTable joystickTable = table.GetSubtable("Joystick" + std::to_string(id));

            HAL_JoystickDescriptor desc{};
            HAL_GetJoystickDescriptor(id, &desc);
            joystickTable.Put("Name", Trim(desc.name));
            joystickTable.Put("Type", static_cast<int64_t>(desc.type));
            joystickTable.Put("Xbox", static_cast<bool>(desc.isXbox));
            joystickTable.Put("ButtonCount", desc.buttonCount);

            HAL_JoystickButtons buttons{};
            HAL_GetJoystickButtons(id, &buttons);
            joystickTable.Put("ButtonValues", static_cast<int64_t>(buttons.buttons));

            HAL_JoystickPOVs povs{};
            HAL_GetJoystickPOVs(id, &povs);
            std::vector<int> povValues(povs.povs, povs.povs + povs.count);
            joystickTable.Put("POVs", std::span<const int>(povValues));

            HAL_JoystickAxes axes{};
            HAL_GetJoystickAxes(id, &axes);
            std::vector<float> axisValues(axes.axes, axes.axes + axes.count);
            joystickTable.Put("AxisValues", std::span<const float>(axisValues));

            std::vector<int> axisTypes(axes.count);
            for (int i = 0; i < axes.count; i++) {
                axisTypes[i] = desc.axisTypes[i];
            }
            joystickTable.Put("AxisTypes", std::span<const int>(axisTypes));
        }
    }

    void LoggedDriverStation::ReplayFromLog(const LogTable& table) {
        using frc::sim::DriverStationSim;

        const auto allianceStation = table.Get("AllianceStation", 0);
        switch (allianceStation) {
        case HAL_AllianceStationID_kRed1:
            DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kRed1);
            break;
        case HAL_AllianceStationID_kRed2:
            DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kRed2);
            break;
        case HAL_AllianceStationID_kRed3:
            DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kRed3);
            break;
        case HAL_AllianceStationID_kBlue1:
            DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kBlue1);
            break;
        case HAL_AllianceStationID_kBlue2:
            DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kBlue2);
            break;
        case HAL_AllianceStationID_kBlue3:
            DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kBlue3);
            break;
        default:
            DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kUnknown);
            break;
        }
        DriverStationSim::SetEventName(table.Get("EventName", std::string{}));
        DriverStationSim::SetGameSpecificMessage(table.Get("GameSpecificMessage", std::string{}));
        DriverStationSim::SetMatchNumber(table.Get("MatchNumber", 0));
        DriverStationSim::SetReplayNumber(table.Get("ReplayNumber", 0));
        const auto matchType = table.Get("MatchType", 0);
        switch (matchType) {
        case 1:
            DriverStationSim::SetMatchType(frc::DriverStation::MatchType::kPractice);
            break;
        case 2:
            DriverStationSim::SetMatchType(frc::DriverStation::MatchType::kQualification);
            break;
        case 3:
            DriverStationSim::SetMatchType(frc::DriverStation::MatchType::kElimination);
            break;
        default:
            DriverStationSim::SetMatchType(frc::DriverStation::MatchType::kNone);
            break;
        }
        DriverStationSim::SetMatchTime(table.Get("MatchTime", -1.0));

        const bool dsAttached = table.Get("DSAttached", false);
        DriverStationSim::SetEnabled(table.Get("Enabled", false));
        DriverStationSim::SetAutonomous(table.Get("Autonomous", false));
        DriverStationSim::SetTest(table.Get("Test", false));
        DriverStationSim::SetEStop(table.Get("EmergencyStop", false));
        DriverStationSim::SetFmsAttached(table.Get("FMSAttached", false));
        DriverStationSim::SetDsAttached(dsAttached);

        for (int id = 0; id < frc::DriverStation::kJoystickPorts; id++) {
            const LogTable joystickTable = table.GetSubtable("Joystick" + std::to_string(id));
            DriverStationSim::SetJoystickName(id, joystickTable.Get("Name", std::string{}));

            const auto joystickType = joystickTable.Get("Type", 0);
            DriverStationSim::SetJoystickType(id, joystickType);
            DriverStationSim::SetJoystickIsXbox(id, joystickTable.Get("Xbox", false));
            DriverStationSim::SetJoystickButtonCount(id, joystickTable.Get("ButtonCount", 0));
            DriverStationSim::SetJoystickButtons(id, static_cast<uint32_t>(joystickTable.Get("ButtonValues", static_cast<int64_t>(0))));

            const auto povValues = joystickTable.Get("POVs", std::span<const int>{});
            const auto povCount = static_cast<int>(povValues.size());
            DriverStationSim::SetJoystickPOVCount(id, povCount);
            for (int pov = 0; pov < povCount; pov++) {
                DriverStationSim::SetJoystickPOV(id, pov, povValues[pov]);
            }

            const auto axisValues = joystickTable.Get("AxisValues", std::vector<float>{});
            auto axisTypes = joystickTable.Get("AxisTypes", std::span<const int>{});
            axisTypes.resize(axisValues.size(), 0);

            const auto axisCount = static_cast<int>(axisValues.size());
            DriverStationSim::SetJoystickAxisCount(id, axisCount);
            for (int axis = 0; axis < axisCount; axis++) {
                DriverStationSim::SetJoystickAxis(id, axis, axisValues[axis]);
                DriverStationSim::SetJoystickAxisType(id, axis, axisTypes[axis]);
            }
        }

        if (dsAttached) {
            DriverStationSim::NotifyNewData();
        }
    }
} // namespace akit
