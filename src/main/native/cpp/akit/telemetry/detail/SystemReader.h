#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <wpi/nt/BooleanTopic.hpp>
#include <wpi/nt/DoubleArrayTopic.hpp>
#include <wpi/nt/DoubleTopic.hpp>
#include <wpi/nt/IntegerTopic.hpp>
#include <wpi/nt/StringTopic.hpp>
#include <wpi/nt/StructArrayTopic.hpp>
#include <wpi/nt/StructTopic.hpp>

#include "akit/telemetry/detail/SystemcoreStructs.h"

namespace akit::detail {
    struct NetworkDirectionStatus {
        int64_t bandwidthKbps = 0;
        int64_t bytes = 0;
        int64_t dropped = 0;
        int64_t errors = 0;
        int64_t packets = 0;
    };

    struct NetworkStatus {
        NetworkDirectionStatus rx;
        NetworkDirectionStatus tx;
    };

    struct CANInfo {
        double maxBandwidthMbps = 0.0;
        bool isFd = false;
        bool isAvailable = false;
        bool isUp = false;
        double utilizationPercent = 0.0;
        double fps = 0.0;
    };

    struct Vector3 {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Vector4 {
        double w = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    inline constexpr int kNumCANBuses = 5;

    struct SystemData {
        double batteryVoltage = 0.0;
        bool watchdogActive = false;
        int64_t ioFrequency = 0;
        int64_t ioRxFrequency = 0;
        int64_t teamNumber = -1;
        int64_t epochTime = 0;
        bool epochTimeValid = false;

        bool faultBrownout = false;
        bool faultCanbusDown = false;
        bool faultCanbusUnavail = false;
        bool faultDisplay = false;
        bool faultIMU = false;
        bool faultIO = false;
        bool faultRSL = false;
        bool faultUSB = false;

        int64_t faultCountBrownout = 0;
        int64_t faultCountCanbusDown = 0;
        int64_t faultCountCanbusUnavail = 0;
        int64_t faultCountDisplay = 0;
        int64_t faultCountIMU = 0;
        int64_t faultCountIO = 0;
        int64_t faultCountRSL = 0;
        int64_t faultCountUSB = 0;

        NetworkStatus networkEthernet;
        NetworkStatus networkWiFi;
        NetworkStatus networkUSBTether;
        std::array<NetworkStatus, kNumCANBuses> networkCAN{};
        std::array<CANInfo, kNumCANBuses> networkCANInfo{};

        double cpuPercent = 0.0;
        double cpuTemp = 0.0;
        double current3v3 = 0.0;
        std::string osHash;
        std::string osSlot;
        std::string osVersion;

        int64_t memoryUsageBytes = 0;
        int64_t memoryTotalBytes = 0;
        double memoryPercent = 0.0;

        int64_t storageUsageBytes = 0;
        int64_t storageTotalBytes = 0;
        double storagePercent = 0.0;

        Vector3 imuAccelRaw;
        Vector3 imuGyroRates;
        Vector3 imuGyroEulerFlat;
        Vector3 imuGyroEulerLandscape;
        Vector3 imuGyroEulerPortrait;
        Vector4 imuGyroQuaternion;
        double imuGyroYawFlat = 0.0;
        double imuGyroYawLandscape = 0.0;
        double imuGyroYawPortrait = 0.0;
    };

    /** Reads Systemcore system stats, mostly from the system server NetworkTables instance, synchronously on each call. */
    class SystemReader {
    public:
        SystemReader();

        SystemData Read();

    private:
        wpi::nt::BooleanSubscriber watchdogActiveSubscriber_;
        wpi::nt::IntegerSubscriber ioFrequencySubscriber_;
        wpi::nt::IntegerSubscriber ioRxFrequencySubscriber_;
        wpi::nt::IntegerSubscriber teamNumberSubscriber_;
        wpi::nt::BooleanSubscriber epochTimeValidSubscriber_;

        wpi::nt::StructSubscriber<IOFaults> faultsDataSubscriber_;
        wpi::nt::IntegerSubscriber faultCanbusDownSubscriber_;
        wpi::nt::IntegerSubscriber faultCanbusUnavailSubscriber_;
        wpi::nt::IntegerSubscriber faultCountCanbusDownSubscriber_;
        wpi::nt::IntegerSubscriber faultCountCanbusUnavailSubscriber_;

        wpi::nt::DoubleArraySubscriber networkEthernetSubscriber_;
        wpi::nt::DoubleArraySubscriber networkWiFiSubscriber_;
        wpi::nt::DoubleArraySubscriber networkUsb0Subscriber_;
        wpi::nt::DoubleArraySubscriber networkUsb1Subscriber_;
        std::array<wpi::nt::DoubleArraySubscriber, kNumCANBuses> networkCANSubscribers_;
        wpi::nt::StructArraySubscriber<CanBusInfoEntry> networkCANInfoSubscriber_;
        wpi::nt::DoubleArraySubscriber networkCANUtilizationSubscriber_;
        wpi::nt::DoubleArraySubscriber networkCANFramerateSubscriber_;

        wpi::nt::DoubleSubscriber cpuPercentSubscriber_;
        wpi::nt::DoubleSubscriber cpuTempSubscriber_;
        wpi::nt::DoubleSubscriber current3v3Subscriber_;
        wpi::nt::StringSubscriber osHashSubscriber_;
        wpi::nt::StringSubscriber osSlotSubscriber_;
        wpi::nt::StringSubscriber osVersionSubscriber_;

        wpi::nt::IntegerSubscriber memoryUsageBytesSubscriber_;
        wpi::nt::IntegerSubscriber memoryTotalBytesSubscriber_;
        wpi::nt::DoubleSubscriber memoryPercentSubscriber_;

        wpi::nt::IntegerSubscriber storageUsageBytesSubscriber_;
        wpi::nt::IntegerSubscriber storageTotalBytesSubscriber_;
        wpi::nt::DoubleSubscriber storagePercentSubscriber_;
    };
} // namespace akit::detail
