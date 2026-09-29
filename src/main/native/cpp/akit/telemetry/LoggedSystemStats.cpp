#include "pch.h"

#include "akit/telemetry/LoggedSystemStats.h"

#include <array>
#include <chrono>

#include <frc/RobotController.h>
#include <hal/HAL.h>
#include <networktables/NetworkTableInstance.h>

namespace akit {
    void LoggedSystemStats::SaveToLog(LogTable stats) {
        int32_t status = 0;
        stats.Put("FPGAVersion", static_cast<int64_t>(HAL_GetFPGAVersion(&status)));
        stats.Put("FPGARevision", static_cast<int64_t>(HAL_GetFPGARevision(&status)));

        WPI_String serialNum{};
        HAL_GetSerialNumber(&serialNum);
        std::string serial(serialNum.str, serialNum.len);
        WPI_FreeString(&serialNum);
        stats.Put("SerialNumber", std::string_view{serial});

        stats.Put("Comments", frc::RobotController::GetComments());
        stats.Put("TeamNumber", static_cast<int64_t>(HAL_GetTeamNumber()));
        stats.Put("FPGAButton", frc::RobotController::GetUserButton());
        stats.Put("SystemActive", frc::RobotController::IsSysActive());
        stats.Put("BrownedOut", frc::RobotController::IsBrownedOut());
        stats.Put("CommsDisableCount", static_cast<int64_t>(frc::RobotController::GetCommsDisableCount()));
        stats.Put("RSLState", frc::RobotController::GetRSLState());
        stats.Put("SystemTimeValid", static_cast<bool>(HAL_GetSystemTimeValid(&status)));

        stats.Put("BatteryVoltage", frc::RobotController::GetBatteryVoltage().value());
        stats.Put("BatteryCurrent", frc::RobotController::GetInputCurrent());

        stats.Put("3v3Rail/Voltage", frc::RobotController::GetVoltage3V3());
        stats.Put("3v3Rail/Current", frc::RobotController::GetCurrent3V3());
        stats.Put("3v3Rail/Active", frc::RobotController::GetEnabled3V3());
        stats.Put("3v3Rail/CurrentFaults", static_cast<int64_t>(frc::RobotController::GetFaultCount3V3()));

        stats.Put("5vRail/Voltage", frc::RobotController::GetVoltage5V());
        stats.Put("5vRail/Current", frc::RobotController::GetCurrent5V());
        stats.Put("5vRail/Active", frc::RobotController::GetEnabled5V());
        stats.Put("5vRail/CurrentFaults", static_cast<int64_t>(frc::RobotController::GetFaultCount5V()));

        stats.Put("6vRail/Voltage", frc::RobotController::GetVoltage6V());
        stats.Put("6vRail/Current", frc::RobotController::GetCurrent6V());
        stats.Put("6vRail/Active", frc::RobotController::GetEnabled6V());
        stats.Put("6vRail/CurrentFaults", static_cast<int64_t>(frc::RobotController::GetFaultCount6V()));

        stats.Put("BrownoutVoltage", frc::RobotController::GetBrownoutVoltage().value());
        stats.Put("CPUTempCelsius", frc::RobotController::GetCPUTemp().value());

        auto can = frc::RobotController::GetCANStatus();
        stats.Put("CANBus/Utilization", can.percentBusUtilization);
        stats.Put("CANBus/OffCount", static_cast<int64_t>(can.busOffCount));
        stats.Put("CANBus/TxFullCount", static_cast<int64_t>(can.txFullCount));
        stats.Put("CANBus/ReceiveErrorCount", static_cast<int64_t>(can.receiveErrorCount));
        stats.Put("CANBus/TransmitErrorCount", static_cast<int64_t>(can.transmitErrorCount));

        stats.Put("EpochTimeMicros",
                  static_cast<int64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()));

        LogTable ntClients = stats.GetSubtable("NTClients");
        auto ntConnections = nt::NetworkTableInstance::GetDefault().GetConnections();
        std::unordered_set<std::string> currentRemoteIds;

        for (const auto& connection : ntConnections) {
            lastNTRemoteIds_.erase(connection.remote_id);
            currentRemoteIds.insert(connection.remote_id);

            LogTable ntClient = ntClients.GetSubtable(connection.remote_id);
            ntClient.Put("Connected", true);
            ntClient.Put("IPAddress", connection.remote_ip);
            ntClient.Put("RemotePort", static_cast<int64_t>(connection.remote_port));

            std::array<uint8_t, 4> protocolVersion{
                static_cast<uint8_t>((connection.protocol_version >> 24) & 0xFF),
                static_cast<uint8_t>((connection.protocol_version >> 16) & 0xFF),
                static_cast<uint8_t>((connection.protocol_version >> 8) & 0xFF),
                static_cast<uint8_t>(connection.protocol_version & 0xFF),
            };
            ntClient.Put("ProtocolVersion", std::span<const uint8_t>(protocolVersion));
        }

        for (const auto& remoteId : lastNTRemoteIds_) {
            ntClients.Put(remoteId + "/Connected", false);
        }
        lastNTRemoteIds_ = std::move(currentRemoteIds);
    }
} // namespace akit
