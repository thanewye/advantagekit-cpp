#pragma once

#include <concepts>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/nt/StringArrayTopic.hpp>
#include <wpi/nt/StringTopic.hpp>
#include <wpi/tunables/Selectable.hpp>
#include <wpi/util/json.hpp>

#include "akit/Logger.h"
#include "akit/networktables/LoggedNetworkInput.h"
#include "akit/networktables/detail/SelectableAccess.h"
#include "akit/util/LinkedHashMap.h"

namespace akit::networktables {
    /**
     * NT chooser using the Selectable dashboard layout whose selected option is logged and is replayable
     *
     * @tparam V The value type associated with each option
     */
    template<typename V> requires std::copy_constructible<V> && std::default_initializable<V>
    class LoggedNetworkChooser : public LoggedNetworkInput {
    public:
        explicit LoggedNetworkChooser(std::string_view key)
            : key_(key) {
            auto instance = wpi::nt::NetworkTableInstance::GetDefault();
            const std::string topicPath = key.starts_with('/') ? std::string(key) : "/" + std::string(key);

            typePublisher_ = instance.GetStringTopic(topicPath + "/.type").PublishEx("string", wpi::util::json::object("mutable", true));
            typePublisher_.Set("Selectable");

            defaultPublisher_ = instance.GetStringTopic(topicPath + "/default").PublishEx("string", wpi::util::json::object("mutable", false));
            defaultPublisher_.Set(defaultChoice_);

            optionsPublisher_ = instance.GetStringArrayTopic(topicPath + "/options").PublishEx("string[]", wpi::util::json::object("mutable", false));
            optionsPublisher_.Set({});

            activePublisher_ = instance.GetStringTopic(topicPath + "/selected/value").PublishEx("string", wpi::util::json::object("mutable", true));
            activePublisher_.Set("");

            selectedSubscriber_ =
                instance.GetStringTopic(topicPath + "/selected/tune").Subscribe("", wpi::nt::PubSubOptions{.excludePublisher = activePublisher_.GetHandle()});

            LoggedNetworkChooser<V>::Periodic();
            Logger::RegisterDashboardInput(this);
        }

        LoggedNetworkChooser(std::string_view key, wpi::tunables::Selectable<V>& selectable)
            : LoggedNetworkChooser(key) {
            const std::string defaultOption = detail::SelectableDefault(selectable);
            for (auto& [optionKey, value] : detail::SelectableOptions(selectable)) {
                if (optionKey == defaultOption) AddDefault(optionKey, std::move(value));
                else Add(optionKey, std::move(value));
            }
        }

        void Add(std::string_view name, V object) {
            options_.put(std::string(name), std::move(object));
            PublishOptions();
            if (!selectedValue_.has_value()) selectedValue_ = defaultChoice_;
        }

        void AddDefault(std::string_view name, V object) {
            Add(name, std::move(object));
            SetDefault(name);
        }

        void Remove(std::string_view name) {
            const std::string optionKey(name);
            if (!options_.erase(optionKey)) return;
            PublishOptions();
            if (optionKey == defaultChoice_) SetDefault("");
        }

        void SetDefault(std::string_view name) {
            defaultChoice_ = std::string(name);
            defaultPublisher_.Set(defaultChoice_);
        }

        void Clear() {
            options_ = {};
            PublishOptions();
            SetDefault("");
        }

        [[deprecated("Use Add")]] void AddOption(std::string_view name, V object) { Add(name, std::move(object)); }

        [[deprecated("Use AddDefault")]] void AddDefaultOption(std::string_view name, V object) { AddDefault(name, std::move(object)); }

        [[nodiscard]] V Get() const {
            if (selectedValue_.has_value() && !selectedValue_->empty()) {
                if (const V* selected = options_.get(*selectedValue_)) return *selected;
            }
            if (const V* defaultValue = options_.get(defaultChoice_)) return *defaultValue;
            return V{};
        }

        [[nodiscard]] V GetSelected() const { return Get(); }

        void OnChange(std::function<void(V)> listener) { listener_ = std::move(listener); }

        void Periodic() override {
            if (!Logger::HasReplaySource()) {
                for (auto& update : selectedSubscriber_.ReadQueue()) {
                    selectedValue_ = std::move(update.value);
                }
                if (!selectedValue_.has_value()) {
                    std::string current = selectedSubscriber_.Get();
                    if (!current.empty()) selectedValue_ = std::move(current);
                }
                if (!selectedValue_.has_value() || selectedValue_->empty() || !options_.contains(*selectedValue_)) {
                    selectedValue_ = defaultChoice_;
                }
                activePublisher_.Set(*selectedValue_);
            }

            std::string loggedValue = selectedValue_.value_or("");
            Logger::ProcessDashboardInput(kPrefix, RemoveSlash(key_), loggedValue, defaultChoice_);
            selectedValue_ = std::move(loggedValue);

            if (previousValue_ != selectedValue_) {
                if (listener_) listener_(Get());
                previousValue_ = selectedValue_;
            }
        }

    private:
        void PublishOptions() {
            std::vector<std::string> names;
            names.reserve(options_.size());
            for (const auto& [name, value] : options_)
                names.push_back(name);
            optionsPublisher_.Set(names);
        }

        std::string key_;
        util::LinkedHashMap<std::string, V> options_;
        std::string defaultChoice_;
        std::optional<std::string> selectedValue_;
        std::optional<std::string> previousValue_;
        std::function<void(V)> listener_;

        wpi::nt::StringPublisher typePublisher_;
        wpi::nt::StringPublisher defaultPublisher_;
        wpi::nt::StringArrayPublisher optionsPublisher_;
        wpi::nt::StringPublisher activePublisher_;
        wpi::nt::StringSubscriber selectedSubscriber_;
    };
} // namespace akit::networktables
