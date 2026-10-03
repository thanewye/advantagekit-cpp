#include "pch.h"

#include "akit/AlertLogger.h"

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <wpi/util/Alert.h>
#include <wpi/util/string.hpp>

#include "akit/Logger.h"

namespace akit {
    namespace {
        struct ActiveAlert {
            std::string text;
            int64_t activeStartTime;
            int32_t level;
        };

        std::set<std::string>& KnownGroups() {
            static std::set<std::string> groups;
            return groups;
        }

        std::map<std::string, std::vector<ActiveAlert>> ReadActiveAlertsByGroup() {
            std::map<std::string, std::vector<ActiveAlert>> activeByGroup;
            const int32_t alertCount = WPI_GetNumAlerts();
            if (alertCount <= 0) return activeByGroup;

            std::vector<WPI_AlertInfo> alerts(static_cast<size_t>(alertCount));
            const int32_t readCount = WPI_GetAlerts(alerts.data(), alertCount);
            for (int32_t i = 0; i < readCount; i++) {
                const WPI_AlertInfo& info = alerts[i];
                std::string group{wpi::util::to_string_view(&info.group)};
                KnownGroups().insert(group);
                if (info.activeStartTime != 0) {
                    activeByGroup[group].push_back(ActiveAlert{std::string{wpi::util::to_string_view(&info.text)}, info.activeStartTime, info.level});
                }
            }
            WPI_FreeAlerts(alerts.data(), readCount);
            return activeByGroup;
        }
    } // namespace

    void AlertLogger::Periodic() {
        auto activeByGroup = ReadActiveAlertsByGroup();

        for (const std::string& group : KnownGroups()) {
            std::vector<ActiveAlert>& groupAlerts = activeByGroup[group];
            std::sort(groupAlerts.begin(), groupAlerts.end(), [](const ActiveAlert& a, const ActiveAlert& b) {
                if (a.activeStartTime != b.activeStartTime) return a.activeStartTime > b.activeStartTime;
                return a.text < b.text;
            });

            std::vector<std::string> errors;
            std::vector<std::string> warnings;
            std::vector<std::string> infos;
            for (const ActiveAlert& alert : groupAlerts) {
                switch (alert.level) {
                case WPI_ALERT_HIGH:
                    errors.push_back(alert.text);
                    break;
                case WPI_ALERT_MEDIUM:
                    warnings.push_back(alert.text);
                    break;
                case WPI_ALERT_LOW:
                    infos.push_back(alert.text);
                    break;
                default:
                    break;
                }
            }

            Logger::RecordOutput(group + "/.type", "Alerts");
            Logger::RecordOutput(group + "/errors", std::span<const std::string>(errors));
            Logger::RecordOutput(group + "/warnings", std::span<const std::string>(warnings));
            Logger::RecordOutput(group + "/infos", std::span<const std::string>(infos));
        }
    }
} // namespace akit
