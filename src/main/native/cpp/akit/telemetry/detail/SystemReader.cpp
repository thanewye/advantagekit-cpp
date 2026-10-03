#include "pch.h"

#include "akit/telemetry/detail/SystemReader.h"

#include <algorithm>
#include <string>

#include <wpi/hal/IMU.h>
#include <wpi/hal/Power.h>
#include <wpi/hal/SystemServer.h>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/util/timestamp.h>

namespace akit::detail {
    namespace {
        constexpr size_t kNetworkStatusFieldCount = 10;

        NetworkStatus ToNetworkStatus(const std::vector<double>& values) {
            std::array<double, kNetworkStatusFieldCount> fields{};
            std::copy_n(values.begin(), std::min(values.size(), fields.size()), fields.begin());
            NetworkStatus status;
            status.rx.bytes = static_cast<int64_t>(fields[0]);
            status.tx.bytes = static_cast<int64_t>(fields[1]);
            status.rx.packets = static_cast<int64_t>(fields[2]);
            status.tx.packets = static_cast<int64_t>(fields[3]);
            status.rx.errors = static_cast<int64_t>(fields[4]);
            status.tx.errors = static_cast<int64_t>(fields[5]);
            status.rx.dropped = static_cast<int64_t>(fields[6]);
            status.tx.dropped = static_cast<int64_t>(fields[7]);
            status.rx.bandwidthKbps = static_cast<int64_t>(fields[8]);
            status.tx.bandwidthKbps = static_cast<int64_t>(fields[9]);
            return status;
        }

        template<typename HALVector> Vector3 ToVector3(const HALVector& value) {
            return Vector3{value.x, value.y, value.z};
        }
    } // namespace

    SystemReader::SystemReader() {
        const wpi::nt::NetworkTableInstance instance{HAL_GetSystemServerHandle()};
        const auto netcommTable = instance.GetTable("Netcomm");
        const auto sysTable = instance.GetTable("sys");
        const auto diagnosticsTable = instance.GetTable("diagnostics");

        watchdogActiveSubscriber_ = instance.GetBooleanTopic("/Netcomm/Control/WatchdogActive").Subscribe(false);
        ioFrequencySubscriber_ = sysTable->GetIntegerTopic("iofreq").Subscribe(0);
        ioRxFrequencySubscriber_ = sysTable->GetIntegerTopic("iorxfreq").Subscribe(0);
        teamNumberSubscriber_ = sysTable->GetIntegerTopic("teamnum").Subscribe(-1);
        epochTimeValidSubscriber_ = netcommTable->GetBooleanTopic("Control/HasSetWallClock").Subscribe(false);

        const auto faultsTable = sysTable->GetSubTable("faults");
        faultsDataSubscriber_ = faultsTable->GetStructTopic<IOFaults>("data").Subscribe({});
        faultCanbusDownSubscriber_ = faultsTable->GetIntegerTopic("canbus_down").Subscribe(0);
        faultCanbusUnavailSubscriber_ = faultsTable->GetIntegerTopic("canbus_unavail").Subscribe(0);

        const auto faultCountsTable = sysTable->GetSubTable("faultcounts");
        faultCountCanbusDownSubscriber_ = faultCountsTable->GetIntegerTopic("canbus_down").Subscribe(0);
        faultCountCanbusUnavailSubscriber_ = faultCountsTable->GetIntegerTopic("canbus_unavail").Subscribe(0);

        const std::vector<double> networkDefault(kNetworkStatusFieldCount, 0.0);
        networkEthernetSubscriber_ = diagnosticsTable->GetDoubleArrayTopic("eth0").Subscribe(networkDefault);
        networkWiFiSubscriber_ = diagnosticsTable->GetDoubleArrayTopic("wlan0").Subscribe(networkDefault);
        networkUsb0Subscriber_ = diagnosticsTable->GetDoubleArrayTopic("usb0").Subscribe(networkDefault);
        networkUsb1Subscriber_ = diagnosticsTable->GetDoubleArrayTopic("usb1").Subscribe(networkDefault);
        for (int bus = 0; bus < kNumCANBuses; bus++) {
            networkCANSubscribers_[bus] = diagnosticsTable->GetDoubleArrayTopic("can_s" + std::to_string(bus)).Subscribe(networkDefault);
        }
        networkCANInfoSubscriber_ = diagnosticsTable->GetStructArrayTopic<CanBusInfoEntry>("canbusinfo").Subscribe({});
        const std::vector<double> perBusDefault(kNumCANBuses, 0.0);
        networkCANUtilizationSubscriber_ = diagnosticsTable->GetDoubleArrayTopic("canbusutil").Subscribe(perBusDefault);
        networkCANFramerateSubscriber_ = diagnosticsTable->GetDoubleArrayTopic("canbusfps").Subscribe(perBusDefault);

        cpuPercentSubscriber_ = sysTable->GetDoubleTopic("cpu").Subscribe(0.0);
        cpuTempSubscriber_ = sysTable->GetDoubleTopic("temp").Subscribe(0.0);
        current3v3Subscriber_ = sysTable->GetDoubleTopic("current3v3").Subscribe(0.0);
        osHashSubscriber_ = sysTable->GetStringTopic("oshash").Subscribe("");
        osSlotSubscriber_ = sysTable->GetStringTopic("osslot").Subscribe("");
        osVersionSubscriber_ = sysTable->GetStringTopic("osver").Subscribe("");

        memoryUsageBytesSubscriber_ = sysTable->GetIntegerTopic("ram").Subscribe(0);
        memoryTotalBytesSubscriber_ = sysTable->GetIntegerTopic("ramtotal").Subscribe(0);
        memoryPercentSubscriber_ = sysTable->GetDoubleTopic("ramutil").Subscribe(0);

        storageUsageBytesSubscriber_ = sysTable->GetIntegerTopic("storage").Subscribe(0);
        storageTotalBytesSubscriber_ = sysTable->GetIntegerTopic("storagetotal").Subscribe(0);
        storagePercentSubscriber_ = sysTable->GetDoubleTopic("storageutil").Subscribe(0);
    }

    SystemData SystemReader::Read() {
        int32_t status = 0;
        SystemData data;

        data.batteryVoltage = HAL_GetVinVoltage(&status);
        data.watchdogActive = watchdogActiveSubscriber_.Get();
        data.ioFrequency = ioFrequencySubscriber_.Get();
        data.ioRxFrequency = ioRxFrequencySubscriber_.Get();
        data.teamNumber = teamNumberSubscriber_.Get();
        data.epochTime = static_cast<int64_t>(WPI_GetSystemTime());
        data.epochTimeValid = epochTimeValidSubscriber_.Get();

        const IOFaults faults = faultsDataSubscriber_.Get();
        data.faultBrownout = faults.brownout;
        data.faultCanbusDown = faultCanbusDownSubscriber_.Get() != 0;
        data.faultCanbusUnavail = faultCanbusUnavailSubscriber_.Get() != 0;
        data.faultDisplay = faults.display;
        data.faultIMU = faults.imu;
        data.faultIO = faults.io;
        data.faultRSL = faults.rsl;
        data.faultUSB = faults.usb;

        data.faultCountBrownout = faults.brownoutCount;
        data.faultCountCanbusDown = faultCountCanbusDownSubscriber_.Get();
        data.faultCountCanbusUnavail = faultCountCanbusUnavailSubscriber_.Get();
        data.faultCountDisplay = faults.displayCount;
        data.faultCountIMU = faults.imuCount;
        data.faultCountIO = faults.ioCount;
        data.faultCountRSL = faults.rslCount;
        data.faultCountUSB = faults.usbCount;

        data.networkEthernet = ToNetworkStatus(networkEthernetSubscriber_.Get());
        data.networkWiFi = ToNetworkStatus(networkWiFiSubscriber_.Get());
        const auto usb0Status = networkUsb0Subscriber_.Get();
        const auto usb1Status = networkUsb1Subscriber_.Get();
        std::vector<double> usbStatus(kNetworkStatusFieldCount, 0.0);
        for (size_t i = 0; i < std::min({usb0Status.size(), usb1Status.size(), usbStatus.size()}); i++) {
            usbStatus[i] = usb0Status[i] + usb1Status[i];
        }
        data.networkUSBTether = ToNetworkStatus(usbStatus);
        for (int bus = 0; bus < kNumCANBuses; bus++) {
            data.networkCAN[bus] = ToNetworkStatus(networkCANSubscribers_[bus].Get());
        }

        const auto canInfo = networkCANInfoSubscriber_.Get();
        const auto canUtilization = networkCANUtilizationSubscriber_.Get();
        const auto canFramerate = networkCANFramerateSubscriber_.Get();
        for (size_t bus = 0; bus < kNumCANBuses; bus++) {
            CANInfo& info = data.networkCANInfo[bus];
            if (bus < canInfo.size()) {
                const CanBusInfoEntry& entry = canInfo[bus];
                info.maxBandwidthMbps = entry.fd ? entry.dataMbps : entry.nominalMbps;
                info.isFd = entry.fd;
                info.isAvailable = entry.avail;
                info.isUp = entry.up;
            }
            if (bus < canUtilization.size()) info.utilizationPercent = canUtilization[bus];
            if (bus < canFramerate.size()) info.fps = canFramerate[bus];
        }

        data.cpuPercent = cpuPercentSubscriber_.Get();
        data.cpuTemp = cpuTempSubscriber_.Get();
        data.current3v3 = current3v3Subscriber_.Get();
        data.osHash = osHashSubscriber_.Get();
        data.osSlot = osSlotSubscriber_.Get();
        data.osVersion = osVersionSubscriber_.Get();

        data.memoryUsageBytes = memoryUsageBytesSubscriber_.Get();
        data.memoryTotalBytes = memoryTotalBytesSubscriber_.Get();
        data.memoryPercent = memoryPercentSubscriber_.Get();

        data.storageUsageBytes = storageUsageBytesSubscriber_.Get();
        data.storageTotalBytes = storageTotalBytesSubscriber_.Get();
        data.storagePercent = storagePercentSubscriber_.Get();

        HAL_Acceleration3d acceleration{};
        HAL_GetIMUAcceleration(&acceleration, &status);
        data.imuAccelRaw = ToVector3(acceleration);

        HAL_GyroRate3d gyroRates{};
        HAL_GetIMUGyroRates(&gyroRates, &status);
        data.imuGyroRates = ToVector3(gyroRates);

        HAL_EulerAngles3d eulerFlat{};
        HAL_GetIMUEulerAnglesFlat(&eulerFlat, &status);
        data.imuGyroEulerFlat = ToVector3(eulerFlat);

        HAL_EulerAngles3d eulerLandscape{};
        HAL_GetIMUEulerAnglesLandscape(&eulerLandscape, &status);
        data.imuGyroEulerLandscape = ToVector3(eulerLandscape);

        HAL_EulerAngles3d eulerPortrait{};
        HAL_GetIMUEulerAnglesPortrait(&eulerPortrait, &status);
        data.imuGyroEulerPortrait = ToVector3(eulerPortrait);

        HAL_Quaternion quaternion{};
        HAL_GetIMUQuaternion(&quaternion, &status);
        data.imuGyroQuaternion = Vector4{quaternion.w, quaternion.x, quaternion.y, quaternion.z};

        int64_t yawTimestamp = 0;
        data.imuGyroYawFlat = HAL_GetIMUYawFlat(&yawTimestamp);
        data.imuGyroYawLandscape = HAL_GetIMUYawLandscape(&yawTimestamp);
        data.imuGyroYawPortrait = HAL_GetIMUYawPortrait(&yawTimestamp);

        return data;
    }
} // namespace akit::detail
