#include "pch.h"

#include "akit/telemetry/LoggedSystemStats.h"

#include <array>
#include <string>
#include <string_view>

#include <wpi/math/geometry/Quaternion.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>

#include "akit/telemetry/detail/SystemReader.h"

namespace akit {
    namespace {
        detail::SystemReader& Reader() {
            static detail::SystemReader reader;
            return reader;
        }

        void LogNetworkDirectionStatus(const LogTable& table, const detail::NetworkDirectionStatus& status) {
            table.Put("Bandwidth", status.bandwidthKbps * 1.0e-3, "megabits per second");
            table.Put("Kilobytes", status.bytes / 1024.0, "kilobytes");
            table.Put("Dropped", status.dropped);
            table.Put("Errors", status.errors);
            table.Put("Packets", status.packets);
        }

        void LogNetworkStatus(const LogTable& table, const detail::NetworkStatus& status) {
            LogNetworkDirectionStatus(table.GetSubtable("RX"), status.rx);
            LogNetworkDirectionStatus(table.GetSubtable("TX"), status.tx);
        }

        void LogCANInfo(const LogTable& table, const detail::CANInfo& info) {
            table.Put("MaxBandwidth", info.maxBandwidthMbps, "megabits per second");
            table.Put("FD", info.isFd);
            table.Put("Available", info.isAvailable);
            table.Put("InterfaceUp", info.isUp);
            table.Put("Utilization", info.utilizationPercent, "percent");
            table.Put("Framerate", info.fps);
        }

        void LogVector3(const LogTable& table, const detail::Vector3& vector, std::string_view unit) {
            table.Put("X", vector.x, unit);
            table.Put("Y", vector.y, unit);
            table.Put("Z", vector.z, unit);
        }
    } // namespace

    void LoggedSystemStats::SaveToLog(LogTable stats) {
        const detail::SystemData data = Reader().Read();

        stats.Put("BatteryVoltage", data.batteryVoltage, "volts");
        stats.Put("WatchdogActive", data.watchdogActive);
        stats.Put("IOFrequency", data.ioFrequency);
        stats.Put("IORXFrequency", data.ioRxFrequency);
        stats.Put("TeamNumber", data.teamNumber);
        stats.Put("EpochTime", static_cast<double>(data.epochTime / 1000), "microseconds");
        stats.Put("EpochTimeValid", data.epochTimeValid);

        stats.Put("Faults/Brownout", data.faultBrownout);
        stats.Put("Faults/CANBusDown", data.faultCanbusDown);
        stats.Put("Faults/CANBusUnavail", data.faultCanbusUnavail);
        stats.Put("Faults/Display", data.faultDisplay);
        stats.Put("Faults/IMU", data.faultIMU);
        stats.Put("Faults/IO", data.faultIO);
        stats.Put("Faults/RSL", data.faultRSL);
        stats.Put("Faults/USB", data.faultUSB);

        stats.Put("FaultCounts/Brownout", data.faultCountBrownout);
        stats.Put("FaultCounts/CANBusDown", data.faultCountCanbusDown);
        stats.Put("FaultCounts/CANBusUnavail", data.faultCountCanbusUnavail);
        stats.Put("FaultCounts/Display", data.faultCountDisplay);
        stats.Put("FaultCounts/IMU", data.faultCountIMU);
        stats.Put("FaultCounts/IO", data.faultCountIO);
        stats.Put("FaultCounts/RSL", data.faultCountRSL);
        stats.Put("FaultCounts/USB", data.faultCountUSB);

        LogNetworkStatus(stats.GetSubtable("Network/Ethernet"), data.networkEthernet);
        LogNetworkStatus(stats.GetSubtable("Network/WiFi"), data.networkWiFi);
        LogNetworkStatus(stats.GetSubtable("Network/USBTether"), data.networkUSBTether);
        for (int bus = 0; bus < detail::kNumCANBuses; bus++) {
            const LogTable busTable = stats.GetSubtable("Network/CAN" + std::to_string(bus));
            LogNetworkStatus(busTable, data.networkCAN[bus]);
            LogCANInfo(busTable, data.networkCANInfo[bus]);
        }

        stats.Put("CPU/Utilization", data.cpuPercent, "percent");
        stats.Put("CPU/Temperature", data.cpuTemp, "celcius");

        stats.Put("Memory/Usage", data.memoryUsageBytes * 1.0e-6, "megabytes");
        stats.Put("Memory/Total", data.memoryTotalBytes * 1.0e-6, "megabytes");
        stats.Put("Memory/Utilization", data.memoryPercent, "percent");

        stats.Put("Storage/Usage", data.storageUsageBytes * 1.0e-6, "megabytes");
        stats.Put("Storage/Total", data.storageTotalBytes * 1.0e-6, "megabytes");
        stats.Put("Storage/Utilization", data.storagePercent, "percent");

        stats.Put("3v3Current", data.current3v3, "amps");
        stats.Put("OS/Hash", std::string_view{data.osHash});
        stats.Put("OS/Slot", std::string_view{data.osSlot});
        stats.Put("OS/Version", std::string_view{data.osVersion});

        LogVector3(stats.GetSubtable("IMU/AccelRaw"), data.imuAccelRaw, "G");
        LogVector3(stats.GetSubtable("IMU/GyroRates"), data.imuGyroRates, "degrees per second");
        LogVector3(stats.GetSubtable("IMU/GyroEuler/Flat"), data.imuGyroEulerFlat, "degrees");
        LogVector3(stats.GetSubtable("IMU/GyroEuler/Landscape"), data.imuGyroEulerLandscape, "degrees");
        LogVector3(stats.GetSubtable("IMU/GyroEuler/Portrait"), data.imuGyroEulerPortrait, "degrees");
        const auto& quaternion = data.imuGyroQuaternion;
        stats.Put("IMU/Gyro3d", wpi::math::Rotation3d{wpi::math::Quaternion{quaternion.w, quaternion.x, quaternion.y, quaternion.z}});
        stats.Put("IMU/GyroYaw/Flat", wpi::math::Rotation2d{wpi::units::radian_t{data.imuGyroYawFlat}});
        stats.Put("IMU/GyroYaw/Landscape", wpi::math::Rotation2d{wpi::units::radian_t{data.imuGyroYawLandscape}});
        stats.Put("IMU/GyroYaw/Portrait", wpi::math::Rotation2d{wpi::units::radian_t{data.imuGyroYawPortrait}});

        LogTable ntClients = stats.GetSubtable("NTClients");
        const auto ntConnections = wpi::nt::NetworkTableInstance::GetDefault().GetConnections();
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
