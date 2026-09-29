#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "akit/Logger.h"
#include "akit/networktables/LoggedNetworkInput.h"
#include "akit/networktables/detail/SendableChooserAccess.h"

namespace akit::networktables {
    /**
     * NT chooser whose selected option is logged and is replayable
     *
     * @tparam V The value type associated with each option
     */
    template<typename V> requires std::copy_constructible<V> && std::default_initializable<V>
    class LoggedDashboardChooser : public LoggedNetworkInput {
    public:
        explicit LoggedDashboardChooser(std::string_view key)
            : key_(key) {
            frc::SmartDashboard::PutData(key_, &sendableChooser_);
            LoggedDashboardChooser<V>::Periodic();
            Logger::RegisterDashboardInput(this);
        }

        explicit LoggedDashboardChooser(std::string_view key, const frc::SendableChooser<V>& chooser)
            : LoggedDashboardChooser(key) {
            const std::string& defaultChoice = detail::StolenDefaultChoice(chooser);
            for (const auto& [optionKey, value] : detail::StolenChoices(chooser)) {
                if (optionKey == defaultChoice) AddDefaultOption(optionKey, value);
                else AddOption(optionKey, value);
            }
        }

        void AddOption(std::string_view key, V value) {
            const std::string optionKey{key};
            sendableChooser_.AddOption(optionKey, optionKey);
            options_.insert_or_assign(optionKey, std::move(value));
        }

        void AddDefaultOption(std::string_view key, V value) {
            const std::string optionKey{key};
            sendableChooser_.SetDefaultOption(optionKey, optionKey);
            options_.insert_or_assign(optionKey, std::move(value));
        }

        [[nodiscard]] V Get() const {
            auto option = options_.find(selectedValue_);
            if (option == options_.end()) return V{};
            return option->second;
        }

        void OnChange(std::function<void(V)> listener) { listener_ = std::move(listener); }

        [[nodiscard]] frc::SendableChooser<std::string>& GetSendableChooser() { return sendableChooser_; }

        void Periodic() override {
            if (!Logger::HasReplaySource()) selectedValue_ = sendableChooser_.GetSelected();
            Logger::ProcessDashboardInput(std::string(kPrefix) + "/SmartDashboard", key_, selectedValue_, selectedValue_);
            if (previousValue_ != selectedValue_) {
                if (listener_) listener_(Get());
                previousValue_ = selectedValue_;
            }
        }

    private:
        std::string key_;
        std::string selectedValue_;
        std::string previousValue_;
        frc::SendableChooser<std::string> sendableChooser_;
        std::unordered_map<std::string, V> options_;
        std::function<void(V)> listener_;
    };
} // namespace akit::networktables
