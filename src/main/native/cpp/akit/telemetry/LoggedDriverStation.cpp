#include "pch.h"

#include "akit/telemetry/LoggedDriverStation.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include <wpi/driverstation/MatchType.hpp>
#include <wpi/driverstation/internal/DriverStationBackend.hpp>
#include <wpi/hal/DriverStation.h>
#include <wpi/hal/DriverStationTypes.hpp>
#include <wpi/hal/simulation/DriverStationData.h>
#include <wpi/simulation/DriverStationSim.hpp>

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

    template<size_t N> std::string NullTerminatedString(const char (&buffer)[N]) {
        return std::string(buffer, strnlen(buffer, N));
    }

    int AvailableToCount(const uint64_t available) {
        return std::countr_one(available);
    }

    std::string TouchpadKey(const int touchpad, std::string_view suffix) {
        return "Touchpad/" + std::to_string(touchpad) + "/" + std::string(suffix);
    }

    std::string FingerKey(const int touchpad, const int finger, std::string_view suffix) {
        return TouchpadKey(touchpad, "Finger/" + std::to_string(finger) + "/" + std::string(suffix));
    }
} // namespace

namespace akit {
    void LoggedDriverStation::SaveToLog(const LogTable& table) {
        int32_t status = 0;

        HAL_MatchInfo matchInfo{};
        HAL_GetMatchInfo(&matchInfo);
        HAL_GameData gameData{};
        HAL_GetGameData(&gameData);

        table.Put("AllianceStation", static_cast<int64_t>(HAL_GetAllianceStation(&status)));
        table.Put("EventName", NullTerminatedString(matchInfo.eventName));
        table.Put("GameData", NullTerminatedString(gameData.gameData));
        table.Put("MatchNumber", static_cast<int64_t>(matchInfo.matchNumber));
        table.Put("ReplayNumber", static_cast<int64_t>(matchInfo.replayNumber));
        table.Put("MatchType", static_cast<int64_t>(matchInfo.matchType));
        table.Put("MatchTime", HAL_GetMatchTime(&status), "seconds");

        HAL_ControlWord controlWord{};
        HAL_GetControlWord(&controlWord);
        table.Put("Enabled", static_cast<bool>(HAL_ControlWord_IsEnabled(controlWord)));
        table.Put("RobotMode", static_cast<wpi::hal::RobotMode>(HAL_ControlWord_GetRobotMode(controlWord)));
        table.Put("OpModeId", HAL_ControlWord_GetOpModeId(controlWord));
        table.Put("OpMode", wpi::internal::DriverStationBackend::GetOpMode());
        table.Put("EmergencyStop", static_cast<bool>(HAL_ControlWord_IsEStopped(controlWord)));
        table.Put("FMSAttached", static_cast<bool>(HAL_ControlWord_IsFMSAttached(controlWord)));
        table.Put("DSAttached", static_cast<bool>(HAL_ControlWord_IsDSAttached(controlWord)));

        for (int id = 0; id < HAL_MAX_JOYSTICKS; id++) {
            LogTable joystickTable = table.GetSubtable("Joystick" + std::to_string(id));

            HAL_JoystickDescriptor descriptor{};
            HAL_GetJoystickDescriptor(id, &descriptor);
            joystickTable.Put("Name", Trim(NullTerminatedString(descriptor.name)));
            joystickTable.Put("Type", static_cast<int64_t>(descriptor.gamepadType));
            joystickTable.Put("IsGamepad", static_cast<bool>(descriptor.isGamepad));
            joystickTable.Put("SupportedOutputs", static_cast<int64_t>(descriptor.supportedOutputs));

            HAL_JoystickButtons buttons{};
            HAL_GetJoystickButtons(id, &buttons);
            joystickTable.Put("ButtonsAvailable", static_cast<int64_t>(buttons.available));
            joystickTable.Put("ButtonValues", static_cast<int64_t>(buttons.buttons));

            HAL_JoystickPOVs povs{};
            HAL_GetJoystickPOVs(id, &povs);
            joystickTable.Put("POVsAvailable", static_cast<int64_t>(povs.available));
            const int povCount = std::min(AvailableToCount(povs.available), HAL_MAX_JOYSTICK_POVS);
            std::vector<int> povValues(povs.povs, povs.povs + povCount);
            joystickTable.Put("POVValues", std::span<const int>(povValues));

            HAL_JoystickAxes axes{};
            HAL_GetJoystickAxes(id, &axes);
            joystickTable.Put("AxesAvailable", static_cast<int64_t>(axes.available));
            const int axisCount = std::min(AvailableToCount(axes.available), HAL_MAX_JOYSTICK_AXES);
            std::vector<float> axisValues(axes.axes, axes.axes + axisCount);
            joystickTable.Put("AxisValues", std::span<const float>(axisValues));
            std::vector<int> axisRawValues(axes.raw, axes.raw + axisCount);
            joystickTable.Put("AxisRawValues", std::span<const int>(axisRawValues));

            HAL_JoystickTouchpads touchpads{};
            HAL_GetJoystickTouchpads(id, &touchpads);
            const int touchpadCount = std::min<int>(touchpads.count, HAL_MAX_JOYSTICK_TOUCHPADS);
            joystickTable.Put("TouchpadCount", static_cast<int64_t>(touchpadCount));
            for (int t = 0; t < touchpadCount; t++) {
                const HAL_JoystickTouchpad& touchpad = touchpads.touchpads[t];
                const int fingerCount = std::min<int>(touchpad.count, HAL_MAX_JOYSTICK_TOUCHPAD_FINGERS);
                joystickTable.Put(TouchpadKey(t, "FingerCount"), static_cast<int64_t>(fingerCount));
                for (int f = 0; f < fingerCount; f++) {
                    const HAL_JoystickTouchpadFinger& finger = touchpad.fingers[f];
                    joystickTable.Put(FingerKey(t, f, "Down"), finger.down != 0);
                    joystickTable.Put(FingerKey(t, f, "X"), finger.x);
                    joystickTable.Put(FingerKey(t, f, "Y"), finger.y);
                }
            }
        }
    }

    void LoggedDriverStation::ReplayFromLog(const LogTable& table) {
        using wpi::sim::DriverStationSim;

        switch (table.Get("AllianceStation", 0)) {
        case HAL_ALLIANCE_STATION_RED_1:
            DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::RED_1);
            break;
        case HAL_ALLIANCE_STATION_RED_2:
            DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::RED_2);
            break;
        case HAL_ALLIANCE_STATION_RED_3:
            DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::RED_3);
            break;
        case HAL_ALLIANCE_STATION_BLUE_1:
            DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::BLUE_1);
            break;
        case HAL_ALLIANCE_STATION_BLUE_2:
            DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::BLUE_2);
            break;
        case HAL_ALLIANCE_STATION_BLUE_3:
            DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::BLUE_3);
            break;
        default:
            DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::UNKNOWN);
            break;
        }
        DriverStationSim::SetEventName(table.Get("EventName", std::string{}));
        DriverStationSim::SetGameData(table.Get("GameData", std::string{}));
        DriverStationSim::SetMatchNumber(table.Get("MatchNumber", 0));
        DriverStationSim::SetReplayNumber(table.Get("ReplayNumber", 0));
        switch (table.Get("MatchType", 0)) {
        case HAL_MATCH_TYPE_PRACTICE:
            DriverStationSim::SetMatchType(wpi::MatchType::PRACTICE);
            break;
        case HAL_MATCH_TYPE_QUALIFICATION:
            DriverStationSim::SetMatchType(wpi::MatchType::QUALIFICATION);
            break;
        case HAL_MATCH_TYPE_ELIMINATION:
            DriverStationSim::SetMatchType(wpi::MatchType::ELIMINATION);
            break;
        default:
            DriverStationSim::SetMatchType(wpi::MatchType::NONE);
            break;
        }
        DriverStationSim::SetMatchTime(table.Get("MatchTime", -1.0));

        const bool dsAttached = table.Get("DSAttached", false);
        DriverStationSim::SetEnabled(table.Get("Enabled", false));
        DriverStationSim::SetRobotMode(table.Get("RobotMode", wpi::hal::RobotMode::UNKNOWN));
        DriverStationSim::SetOpMode(table.Get("OpModeId", static_cast<int64_t>(0)));
        DriverStationSim::SetEStop(table.Get("EmergencyStop", false));
        DriverStationSim::SetFmsAttached(table.Get("FMSAttached", false));
        DriverStationSim::SetDsAttached(dsAttached);

        for (int id = 0; id < HAL_MAX_JOYSTICKS; id++) {
            const LogTable joystickTable = table.GetSubtable("Joystick" + std::to_string(id));
            DriverStationSim::SetJoystickName(id, joystickTable.Get("Name", std::string{}));
            DriverStationSim::SetJoystickGamepadType(id, joystickTable.Get("Type", 0));
            DriverStationSim::SetJoystickIsGamepad(id, joystickTable.Get("IsGamepad", false));
            DriverStationSim::SetJoystickSupportedOutputs(id, joystickTable.Get("SupportedOutputs", 0));

            DriverStationSim::SetJoystickButtonsAvailable(id, static_cast<uint64_t>(joystickTable.Get("ButtonsAvailable", static_cast<int64_t>(0))));
            HALSIM_SetJoystickButtonsValue(id, static_cast<uint64_t>(joystickTable.Get("ButtonValues", static_cast<int64_t>(0))));

            DriverStationSim::SetJoystickPOVsAvailable(id, joystickTable.Get("POVsAvailable", 0));
            const auto povValues = joystickTable.Get("POVValues", std::span<const int>{});
            const int povCount = std::min(static_cast<int>(povValues.size()), HAL_MAX_JOYSTICK_POVS);
            for (int pov = 0; pov < povCount; pov++) {
                HALSIM_SetJoystickPOV(id, pov, static_cast<HAL_JoystickPOV>(povValues[pov]));
            }

            DriverStationSim::SetJoystickAxesAvailable(id, joystickTable.Get("AxesAvailable", 0));
            const auto axisValues = joystickTable.Get("AxisValues", std::span<const float>{});
            const int axisCount = std::min(static_cast<int>(axisValues.size()), HAL_MAX_JOYSTICK_AXES);
            for (int axis = 0; axis < axisCount; axis++) {
                DriverStationSim::SetJoystickAxis(id, axis, axisValues[axis]);
            }

            const int touchpadCount = std::clamp(joystickTable.Get("TouchpadCount", 0), 0, HAL_MAX_JOYSTICK_TOUCHPADS);
            std::array<uint8_t, HAL_MAX_JOYSTICK_TOUCHPADS> fingerCounts{};
            for (int t = 0; t < touchpadCount; t++) {
                fingerCounts[t] = static_cast<uint8_t>(std::clamp(joystickTable.Get(TouchpadKey(t, "FingerCount"), 0), 0, HAL_MAX_JOYSTICK_TOUCHPAD_FINGERS));
            }
            HALSIM_SetJoystickTouchpadCounts(id, static_cast<uint8_t>(touchpadCount), fingerCounts.data());
            for (int t = 0; t < touchpadCount; t++) {
                for (int f = 0; f < fingerCounts[t]; f++) {
                    const bool down = joystickTable.Get(FingerKey(t, f, "Down"), false);
                    const float x = joystickTable.Get(FingerKey(t, f, "X"), 0.0f);
                    const float y = joystickTable.Get(FingerKey(t, f, "Y"), 0.0f);
                    HALSIM_SetJoystickTouchpadFinger(id, t, f, down, x, y);
                }
            }
        }

        if (dsAttached) {
            DriverStationSim::NotifyNewData();
        }
    }
} // namespace akit
