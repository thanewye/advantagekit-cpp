#include "pch.h"

#include "akit/networktables/NT4Publisher.h"

#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/util/json.hpp>

namespace akit::networktables {
    namespace {
        std::string NormalizeTopicKey(const std::string& key) {
            if (!key.empty() && key.front() == '/') return key.substr(1);
            return key;
        }
    } // namespace

    NT4Publisher::NT4Publisher() {
        akitTable_ = wpi::nt::NetworkTableInstance::GetDefault().GetTable("/AdvantageKit");
        timestampPublisher_ = akitTable_->GetIntegerTopic(kTimestampKey.substr(1))
                                  .PublishEx(wpi::nt::IntegerTopic::TYPE_STRING, wpi::util::json::object("unit", "nanoseconds", "mutable", "false"),
                                             wpi::nt::PubSubOptions{.sendAll = true});
    }

    void NT4Publisher::PutTable(const LogTable& table) {
        const int64_t timestamp = table.GetTimestamp();
        const auto& values = table.GetAll();

        timestampPublisher_.Set(timestamp, timestamp);

        for (const auto& [key, value] : values) {
            auto oldValue = lastStorage_.values.find(key);
            if (oldValue != lastStorage_.values.end() && value == oldValue->second) continue;

            auto& publisher = GetOrCreatePublisher(key, value);
            const auto topicKey = NormalizeTopicKey(key);

            if (value.unitStr.has_value() && !value.unitStr->empty()) {
                auto existing = units_.find(topicKey);
                if (existing == units_.end() || existing->second != *value.unitStr) {
                    akitTable_->GetTopic(topicKey).SetProperty("unit", "\"" + *value.unitStr + "\"");
                    units_.insert_or_assign(topicKey, *value.unitStr);
                }
            }

            std::visit(
                [&]<typename T_>(const T_& typedValue) {
                    using T = std::decay_t<T_>;
                    if constexpr (std::is_same_v<T, bool>) publisher.SetBoolean(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, int64_t>) publisher.SetInteger(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, float>) publisher.SetFloat(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, double>) publisher.SetDouble(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::string>) publisher.SetString(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<uint8_t>>) publisher.SetRaw(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<bool>>) {
                        std::vector<int> wpilibArray;
                        wpilibArray.reserve(typedValue.size());
                        for (const auto value : typedValue)
                            wpilibArray.push_back(value);
                        publisher.SetBooleanArray(wpilibArray, timestamp);
                    } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) publisher.SetIntegerArray(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<float>>) publisher.SetFloatArray(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<double>>) publisher.SetDoubleArray(typedValue, timestamp);
                    else if constexpr (std::is_same_v<T, std::vector<std::string>>) publisher.SetStringArray(typedValue, timestamp);
                },
                value.value);
        }
        lastStorage_.values = values;
        lastStorage_.timestamp = timestamp;
    }

    std::string NT4Publisher::GetNT4Type(const LogValue& val) const {
        return val.GetNT4Type();
    }

    wpi::nt::GenericPublisher& NT4Publisher::GetOrCreatePublisher(const std::string& key, const LogValue& value) {
        const auto topicKey = NormalizeTopicKey(key);
        auto publisher = publishers_.find(topicKey);

        // publisher does not exist, create
        if (publisher == publishers_.end()) {
            wpi::nt::Topic topic = akitTable_->GetTopic(topicKey);
            auto unit = value.unitStr;

            // construct publisher within the emplace (black magic)
            auto [createdPublisher, inserted] = publishers_.emplace(
                topicKey, topic.GenericPublishEx(GetNT4Type(value), wpi::util::json::object("mutable", "false"), wpi::nt::PubSubOptions{.sendAll = true}));

            if (unit.has_value() && !unit->empty()) {
                topic.SetProperty("unit", "\"" + unit.value() + "\"");
                units_.emplace(topicKey, *unit);
            }

            return createdPublisher->second;
        }
        return publisher->second;
    }
} // namespace akit::networktables
