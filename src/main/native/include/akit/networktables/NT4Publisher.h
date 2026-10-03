#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <wpi/nt/GenericEntry.hpp>
#include <wpi/nt/IntegerTopic.hpp>
#include <wpi/nt/NetworkTable.hpp>

#include "akit/log/LogDataReceiver.h"

namespace akit::networktables {
    class NT4Publisher : public LogDataReceiver {
        LogStorage lastStorage_;
        std::shared_ptr<wpi::nt::NetworkTable> akitTable_;
        wpi::nt::IntegerPublisher timestampPublisher_;
        std::unordered_map<std::string, wpi::nt::GenericPublisher> publishers_;
        [[nodiscard]] std::string GetNT4Type(const LogValue& val) const;
        wpi::nt::GenericPublisher& GetOrCreatePublisher(const std::string& key, const LogValue& value);
        std::unordered_map<std::string, std::string> units_;

    public:
        NT4Publisher();
        void PutTable(const LogTable& table) override;
    };
} // namespace akit::networktables
