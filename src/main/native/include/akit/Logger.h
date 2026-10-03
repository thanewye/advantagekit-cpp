#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <magic_enum/magic_enum.hpp>
#include <wpi/system/Errors.hpp>
#include <wpi/units/base.hpp>
#include <wpi/units/time.hpp>
#include <wpi/util/Color.hpp>
#include <wpi/util/Color8Bit.hpp>
#include <wpi/util/struct/Struct.hpp>

#include "akit/ConsoleSource.h"
#include "akit/inputs/LoggableInputs.h"
#include "akit/log/LogDataReceiver.h"
#include "akit/log/LogReplaySource.h"
#include "akit/log/LogStorage.h"
#include "akit/log/ReceiverThread.h"

// forward declaration reduces compile time slightly
namespace akit::networktables {
    class LoggedNetworkInput;
} // namespace akit::networktables

namespace akit::mechanism {
    class LoggedMechanism2d;
} // namespace akit::mechanism

namespace akit {
    class Logger {
    public:
        static void Start();
        static void End();

        static void PeriodicBeforeUser();
        static void PeriodicAfterUser();
        static void PeriodicAfterUser(int64_t userCodeNs, int64_t periodicBeforeNs);
        static void PeriodicAfterUser(int64_t userCodeNs, int64_t periodicBeforeNs, std::string_view extraConsoleData);

        static void SetReplaySource(LogReplaySource* source);
        static bool HasReplaySource();
        static bool IsRunning() { return running_; }
        static void Clear();
        static void RegisterDashboardInput(networktables::LoggedNetworkInput* dashboardInput);
        static void UnregisterDashboardInput(networktables::LoggedNetworkInput* dashboardInput);
        static void SetConsoleSource(std::unique_ptr<ConsoleSource> console);
        static void DisableRobotBaseCheck();

        static void RecordMetadata(const std::string& key, std::string_view value);

        template<LoggableAggregate T> static void ProcessInputs(std::string_view key, T& inputs) {
            if (!running_) return;
            LogTable currentTable = LogTable(currentStorage_).GetSubtable(key);
            if (HasReplaySource()) FromLog(inputs, currentTable);
            else ToLog(inputs, currentTable);
        }

        static void ProcessInputs(std::string_view key, LoggableInputs& inputs) {
            if (!running_) return;
            LogTable currentTable = LogTable(currentStorage_).GetSubtable(key);
            if (HasReplaySource()) inputs.FromLog(currentTable);
            else inputs.ToLog(currentTable);
        }

        static void RecordOutput(const std::string& key, LogValue value);
        static void RecordOutput(const std::string& key, bool value);
        template<typename Supplier> requires std::invocable<Supplier> && std::same_as<std::remove_cvref_t<std::invoke_result_t<Supplier>>, bool>
        static void RecordOutput(const std::string& key, Supplier&& value) {
            RecordOutput(key, std::invoke(std::forward<Supplier>(value)));
        }

        static void RecordOutput(const std::string& key, int value) { RecordOutput(key, static_cast<int64_t>(value)); }
        template<typename Supplier> requires std::invocable<Supplier> && std::same_as<std::remove_cvref_t<std::invoke_result_t<Supplier>>, int>
        static void RecordOutput(const std::string& key, Supplier&& value) {
            RecordOutput(key, std::invoke(std::forward<Supplier>(value)));
        }

        static void RecordOutput(const std::string& key, int64_t value);
        template<typename Supplier> requires std::invocable<Supplier> && std::same_as<std::remove_cvref_t<std::invoke_result_t<Supplier>>, int64_t>
        static void RecordOutput(const std::string& key, Supplier&& value) {
            RecordOutput(key, std::invoke(std::forward<Supplier>(value)));
        }

        static void RecordOutput(const std::string& key, float value);

        static void RecordOutput(const std::string& key, float value, std::string_view unit) { RecordOutput(key, LogValue{value, "", std::string(unit)}); }

        static void RecordOutput(const std::string& key, double value);
        template<typename Supplier> requires std::invocable<Supplier> && std::same_as<std::remove_cvref_t<std::invoke_result_t<Supplier>>, double>
        static void RecordOutput(const std::string& key, Supplier&& value) {
            RecordOutput(key, std::invoke(std::forward<Supplier>(value)));
        }

        static void RecordOutput(const std::string& key, double value, std::string_view unit) { RecordOutput(key, LogValue{value, "", std::string(unit)}); }

        static void RecordOutput(const std::string& key, const char* value) { RecordOutput(key, std::string_view(value)); }

        static void RecordOutput(const std::string& key, std::string_view value);

        static void RecordOutput(const std::string& key, std::span<const uint8_t> value);
        static void RecordOutput(const std::string& key, std::span<const bool> value);
        static void RecordOutput(const std::string& key, const std::vector<bool>& value);
        static void RecordOutput(const std::string& key, std::span<const std::vector<bool>> value);
        static void RecordOutput(const std::string& key, std::span<const int> value);
        static void RecordOutput(const std::string& key, std::span<const int64_t> value);
        static void RecordOutput(const std::string& key, std::span<const float> value);
        static void RecordOutput(const std::string& key, std::span<const double> value);
        static void RecordOutput(const std::string& key, std::span<const std::string> value);

        template<typename T> static void ProcessDashboardInput(std::string_view tableKey, std::string_view valueKey, T& value, T defaultValue) {
            if (!running_) return;
            LogTable table = LogTable(currentStorage_).GetSubtable(tableKey);
            if (HasReplaySource()) value = table.Get(valueKey, std::move(defaultValue));
            else table.Put(std::string(valueKey), value);
        }

        static void RecordOutput(const std::string& key, std::span<const std::vector<uint8_t>> value) { RecordOutput2D<uint8_t>(key, value); }

        static void RecordOutput(const std::string& key, std::span<const std::vector<int>> value) { RecordOutput2D<int>(key, value); }

        static void RecordOutput(const std::string& key, std::span<const std::vector<int64_t>> value) { RecordOutput2D<int64_t>(key, value); }

        static void RecordOutput(const std::string& key, std::span<const std::vector<float>> value) { RecordOutput2D<float>(key, value); }

        static void RecordOutput(const std::string& key, std::span<const std::vector<double>> value) { RecordOutput2D<double>(key, value); }

        static void RecordOutput(const std::string& key, std::span<const std::vector<std::string>> value) { RecordOutput2D<std::string>(key, value); }

        template<typename E> requires std::is_enum_v<E>
        static void RecordOutput(const std::string& key, E value) {
            RecordOutput(key, std::string_view(magic_enum::enum_name(value)));
        }

        template<typename E> requires std::is_enum_v<E>
        static void RecordOutput(const std::string& key, std::span<const E> values) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs").Put(key, values);
        }

        template<typename E> requires std::is_enum_v<E>
        static void RecordOutput(const std::string& key, std::span<const std::vector<E>> values) {
            RecordOutput2D<E>(key, values);
        }

        template<typename U> static void RecordOutput(const std::string& key, wpi::units::unit_t<U> value) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs").Put(key, value);
        }

        static void RecordOutput(const std::string& key, wpi::util::Color value) { RecordOutput(key, std::string_view{value.HexString()}); }

        static void RecordOutput(const std::string& key, wpi::util::Color8Bit value) { RecordOutput(key, std::string_view{value.HexString()}); }

        static void RecordOutput(const std::string& key, const mechanism::LoggedMechanism2d& value);

        template<wpi::util::StructSerializable T> static void RecordOutput(const std::string& key, const T& value) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs").Put(key, value);
        }

        template<detail::ProtobufOnlySerializable T> static void RecordOutput(const std::string& key, const T& value) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs").Put(key, value);
        }

        template<wpi::util::StructSerializable T> static void RecordOutputStruct(const std::string& key, const T& value) {
            if (!running_) return;
            AddStructSchema<T>();
            std::vector<uint8_t> buffer(wpi::util::Struct<T>::GetSize());
            wpi::util::PackStruct(std::span{buffer}, value);
            LogTable(currentStorage_)
                .GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs")
                .Put(key, LogValue{std::move(buffer), std::string(wpi::util::GetStructTypeString<T>())});
        }

        template<detail::StructArrayElement T> static void RecordOutput(const std::string& key, const std::vector<T>& values) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs").Put(key, values);
        }

        template<wpi::util::StructSerializable T> static void RecordOutputStruct(const std::string& key, const std::vector<T>& values) {
            if (!running_) return;
            AddStructSchema<T>();
            const size_t elementSize = wpi::util::Struct<T>::GetSize();
            std::vector<uint8_t> buffer(elementSize * values.size());
            for (size_t i = 0; i < values.size(); i++) {
                wpi::util::PackStruct(std::span{buffer}.subspan(i * elementSize, elementSize), values[i]);
            }
            LogTable(currentStorage_)
                .GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs")
                .Put(key, LogValue{std::move(buffer), std::string(wpi::util::GetStructTypeString<T>()) + "[]"});
        }

        template<wpi::util::StructSerializable T> static void RecordOutput(const std::string& key, std::span<const std::vector<T>> values) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs").Put(key, values);
        }

        template<wpi::util::StructSerializable T> static void RecordOutputStruct(const std::string& key, std::span<const std::vector<T>> values) {
            if (!running_) return;
            RecordOutput(key + "/length", static_cast<int64_t>(values.size()));
            for (size_t i = 0; i < values.size(); i++) {
                RecordOutputStruct(key + "/" + std::to_string(i), values[i]);
            }
        }

        template<wpi::util::StructSerializable T> static void RecordOutput(const std::string& key, std::span<const T> values) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs").Put(key, values);
        }

        template<wpi::util::StructSerializable T> static void RecordOutputStruct(const std::string& key, std::span<const T> values) {
            if (!running_) return;
            AddStructSchema<T>();
            const size_t elementSize = wpi::util::Struct<T>::GetSize();
            std::vector<uint8_t> buffer(elementSize * values.size());
            for (size_t i = 0; i < values.size(); i++) {
                wpi::util::PackStruct(std::span{buffer}.subspan(i * elementSize, elementSize), values[i]);
            }
            LogTable(currentStorage_)
                .GetSubtable(HasReplaySource() ? "ReplayOutputs" : "RealOutputs")
                .Put(key, LogValue{std::move(buffer), std::string(wpi::util::GetStructTypeString<T>()) + "[]"});
        }

        template<typename func> static void RunEveryN(int period, func&& fn) {
            if (period < 1) {
                WPILIB_ReportError(wpi::err::Error, "[AdvantageKit] RunEveryN period must be >= 1 (got {})", period);
                return;
            }
            if (cycles_ % period == 0) std::forward<func>(fn)();
        }

        static wpi::units::second_t GetTimestamp();
        static int GetCycleCount() { return cycles_; }
        static const LogStorage& GetCurrentStorage();
        static void DumpCurrentStorage();

        static void AddDataReceiver(LogDataReceiver* receiver);
        static void ClearReceivers();
        static bool GetReceiverQueueFault();

    private:
        inline static LogStorage currentStorage_{};
        inline static std::unordered_map<std::string, std::string> metadata_{};
        inline static LogReplaySource* replaySource_ = nullptr;
        inline static std::vector<networktables::LoggedNetworkInput*> dashboardInputs_{};
        inline static std::unique_ptr<ConsoleSource> console_ = nullptr;
        inline static bool running_ = false;
        inline static bool checkRobotBase_ = true;
        inline static int cycles_ = 0;
        inline static int64_t lastTimestamp_ = 0;
        inline static std::mutex mutex_{};
        inline static ReceiverThread receiverThread_{};

        template<wpi::util::StructSerializable T> static void AddStructSchema() {
            wpi::util::ForEachStructSchema<T>([](std::string_view typeStr, std::string_view schema) {
                std::string schemaKey = "/.schema/";
                schemaKey += typeStr;
                if (currentStorage_.values.contains(schemaKey)) return;
                std::vector<uint8_t> bytes(schema.begin(), schema.end());
                currentStorage_.values.emplace(schemaKey, LogValue{std::move(bytes), "structschema"});
            });
        }

        template<typename T> static void RecordOutput2D(const std::string& key, std::span<const std::vector<T>> value) {
            RecordOutput(key + "/length", static_cast<int64_t>(value.size()));
            for (size_t i = 0; i < value.size(); i++) {
                RecordOutput(key + "/" + std::to_string(i), std::span<const T>(value[i]));
            }
        }
    };
} // namespace akit
