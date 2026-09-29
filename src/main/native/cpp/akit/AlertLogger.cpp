#include "pch.h"

#include "akit/AlertLogger.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayTopic.h>

#include "akit/Logger.h"

namespace akit {
    namespace {
        using SubscriberMap = std::unordered_map<std::string, nt::StringArraySubscriber>;

        SubscriberMap& ErrorSubscribers() {
            static SubscriberMap subscribers;
            return subscribers;
        }

        SubscriberMap& WarningSubscribers() {
            static SubscriberMap subscribers;
            return subscribers;
        }

        SubscriberMap& InfoSubscribers() {
            static SubscriberMap subscribers;
            return subscribers;
        }

        void EnsureSubscriber(SubscriberMap& subscribers, const std::shared_ptr<nt::NetworkTable>& smartDashboard, const std::string& group,
                              const std::string& suffix) {
            if (subscribers.contains(group)) return;
            static const std::vector<std::string> emptyValue;
            subscribers.emplace(group, smartDashboard->GetStringArrayTopic(group + "/" + suffix).Subscribe(emptyValue));
        }
    } // namespace

    void AlertLogger::Periodic() {
        auto smartDashboard = nt::NetworkTableInstance::GetDefault().GetTable("/SmartDashboard");
        for (const auto& group : smartDashboard->GetSubTables()) {
            if (frc::SmartDashboard::GetString(group + "/.type", "") != "Alerts") continue;

            EnsureSubscriber(ErrorSubscribers(), smartDashboard, group, "errors");
            EnsureSubscriber(WarningSubscribers(), smartDashboard, group, "warnings");
            EnsureSubscriber(InfoSubscribers(), smartDashboard, group, "infos");

            Logger::RecordOutput(group + "/.type", "Alerts");

            const auto errors = ErrorSubscribers().at(group).Get();
            const auto warnings = WarningSubscribers().at(group).Get();
            const auto infos = InfoSubscribers().at(group).Get();

            Logger::RecordOutput(group + "/errors", std::span<const std::string>(errors));
            Logger::RecordOutput(group + "/warnings", std::span<const std::string>(warnings));
            Logger::RecordOutput(group + "/infos", std::span<const std::string>(infos));
        }
    }
} // namespace akit
