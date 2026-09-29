#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <networktables/GenericEntry.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>

#include "akit/log/LogDataReceiver.h"

namespace akit::networktables {
    class NT4Publisher : public LogDataReceiver {
        LogStorage lastStorage_;
        std::shared_ptr<nt::NetworkTable> akitTable_;
        nt::IntegerPublisher timestampPublisher_;
        std::unordered_map<std::string, nt::GenericPublisher> publishers_;
        [[nodiscard]] std::string GetNT4Type(const LogValue& val) const;
        nt::GenericPublisher& GetOrCreatePublisher(const std::string& key, const LogValue& value);
        std::unordered_map<std::string, std::string> units_;

    public:
        NT4Publisher();
        void PutTable(const LogTable& table) override;
    };
} // namespace akit::networktables
