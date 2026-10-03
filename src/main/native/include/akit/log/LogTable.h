#pragma once

#include <concepts>
#include <ratio>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <magic_enum/magic_enum.hpp>
#include <wpi/units/base.hpp>
#include <wpi/util/Color.hpp>
#include <wpi/util/Color8Bit.hpp>
#include <wpi/util/protobuf/Protobuf.hpp>
#include <wpi/util/struct/Struct.hpp>

#include "akit/log/LogStorage.h"

namespace akit {
    namespace detail {
        template<typename BaseUnitType> constexpr std::string_view JavaUnitNameForDimension() {
            namespace category = wpi::units::category;
            if constexpr (std::same_as<BaseUnitType, category::length_unit>) return "Meter";
            else if constexpr (std::same_as<BaseUnitType, category::time_unit>) return "Second";
            else if constexpr (std::same_as<BaseUnitType, category::mass_unit>) return "Kilogram";
            else if constexpr (std::same_as<BaseUnitType, category::angle_unit>) return "Radian";
            else if constexpr (std::same_as<BaseUnitType, category::current_unit>) return "Amp";
            else if constexpr (std::same_as<BaseUnitType, category::temperature_unit>) return "Kelvin";
            else if constexpr (std::same_as<BaseUnitType, category::voltage_unit>) return "Volt";
            else if constexpr (std::same_as<BaseUnitType, category::force_unit>) return "Newton";
            else if constexpr (std::same_as<BaseUnitType, category::power_unit>) return "Watt";
            else if constexpr (std::same_as<BaseUnitType, category::frequency_unit>) return "Hertz";
            else if constexpr (std::same_as<BaseUnitType, category::energy_unit>) return "Joule";
            else if constexpr (std::same_as<BaseUnitType, category::velocity_unit>) return "Meter per Second";
            else if constexpr (std::same_as<BaseUnitType, category::acceleration_unit>) return "Meter per Second per Second";
            else if constexpr (std::same_as<BaseUnitType, category::angular_velocity_unit>) return "Radian per Second";
            else if constexpr (std::same_as<BaseUnitType, category::angular_acceleration_unit>) return "Radian per Second per Second";
            else return {};
        }

        template<typename U> inline constexpr bool kIsBaseScaleUnit =
            std::ratio_equal_v<typename wpi::units::traits::unit_traits<U>::conversion_ratio, std::ratio<1>> &&
            std::ratio_equal_v<typename wpi::units::traits::unit_traits<U>::pi_exponent_ratio, std::ratio<0>> &&
            std::ratio_equal_v<typename wpi::units::traits::unit_traits<U>::translation_ratio, std::ratio<0>>;

        template<typename T>
        concept ProtobufOnlySerializable = wpi::util::ProtobufSerializable<T> && !wpi::util::StructSerializable<T>;

        template<typename T>
        concept HasPrimitiveArrayOverload = std::same_as<T, bool> || std::same_as<T, uint8_t> || std::same_as<T, int> || std::same_as<T, int64_t> ||
                                            std::same_as<T, float> || std::same_as<T, double>;

        template<typename T>
        concept StructArrayElement = wpi::util::StructSerializable<T> && !HasPrimitiveArrayOverload<T>;
    } // namespace detail

    class LoggableInputs;

    class LogTable {
    public:
        LogTable(LogStorage& storage, std::string prefix = "/");

        /* --------------------SETTERS (or putters)-------------------- */

        // generic put for fully formed LogValue
        void Put(const std::string& key, LogValue value) const;

        void Put(const std::string& key, bool value) const;
        void Put(const std::string& key, int value) const; // bc number literals are annoying
        void Put(const std::string& key, int64_t value) const;
        void Put(const std::string& key, float value) const;
        void Put(const std::string& key, float value, std::string_view unit) const;
        void Put(const std::string& key, double value) const;
        void Put(const std::string& key, double value, std::string_view unit) const;

        // because string literals convert to bool for some reason
        void Put(const std::string& key, const char* value) const;
        void Put(const std::string& key, std::string_view value) const;

        // arrays
        void Put(const std::string& key, std::span<const uint8_t> value) const;
        void Put(const std::string& key, std::span<const bool> value) const;
        void Put(const std::string& key, const std::vector<bool>& value) const;
        void Put(const std::string& key, std::span<const int> value) const;
        void Put(const std::string& key, std::span<const int64_t> value) const;
        void Put(const std::string& key, std::span<const float> value) const;
        void Put(const std::string& key, std::span<const double> value) const;
        void Put(const std::string& key, std::span<const std::string> value) const;

        // 2d arrays
        void Put(const std::string& key, std::span<const std::vector<bool>> value) const;
        void Put(const std::string& key, std::span<const std::vector<uint8_t>> value) const;
        void Put(const std::string& key, std::span<const std::vector<int>> value) const;
        void Put(const std::string& key, std::span<const std::vector<int64_t>> value) const;
        void Put(const std::string& key, std::span<const std::vector<float>> value) const;
        void Put(const std::string& key, std::span<const std::vector<double>> value) const;
        void Put(const std::string& key, std::span<const std::vector<std::string>> value) const;

        // enums
        template<typename E> requires std::is_enum_v<E>
        void Put(const std::string& key, E value) const {
            Put(key, std::string_view(magic_enum::enum_name(value)));
        }

        template<typename E> requires std::is_enum_v<E>
        void Put(const std::string& key, std::span<const E> values) const {
            std::vector<std::string> strings;
            strings.reserve(values.size());
            for (const auto& v : values)
                strings.emplace_back(magic_enum::enum_name(v));
            Put(key, std::span<const std::string>(strings));
        }

        template<typename E> requires std::is_enum_v<E>
        void Put(const std::string& key, std::span<const std::vector<E>> values) const {
            Put(NormalizeKey(key, "length", false), static_cast<int64_t>(values.size()));
            for (size_t i = 0; i < values.size(); i++) {
                Put(NormalizeKey(key, std::to_string(i), false), std::span<const E>(values[i]));
            }
        }

        // wpilib strong units, logged in base units with java unit names
        template<typename U> void Put(const std::string& key, wpi::units::unit_t<U> value) const {
            using BaseUnitType = typename wpi::units::traits::unit_traits<U>::base_unit_type;
            using BaseUnit = wpi::units::unit<std::ratio<1>, BaseUnitType>;
            const auto baseValue = value.template convert<BaseUnit>();
            constexpr std::string_view unitName = detail::JavaUnitNameForDimension<BaseUnitType>();
            if constexpr (unitName.empty()) {
                Put(key, baseValue.value());
            } else {
                Put(key, baseValue.value(), unitName);
            }
        }

        // wpilib colors, as hex string
        void Put(const std::string& key, wpi::util::Color value) const { Put(key, std::string_view{value.HexString()}); }

        void Put(const std::string& key, wpi::util::Color8Bit value) const { Put(key, std::string_view{value.HexString()}); }

        // wpilib struct type using the type string for custom type
        template<wpi::util::StructSerializable T> void Put(const std::string& key, const T& value) const {
            AddStructSchema<T>();
            std::vector<uint8_t> buf(wpi::util::Struct<T>::GetSize());
            wpi::util::PackStruct(std::span{buf}, value);
            Put(key, LogValue{std::move(buf), std::string(wpi::util::GetStructTypeString<T>())});
        }

        // vector of aforementioned structs
        template<detail::StructArrayElement T> void Put(const std::string& key, const std::vector<T>& values) const {
            AddStructSchema<T>();
            const size_t elemSize = wpi::util::Struct<T>::GetSize();
            std::vector<uint8_t> buf(elemSize * values.size());
            for (size_t i = 0; i < values.size(); i++) {
                wpi::util::PackStruct(std::span{buf}.subspan(i * elemSize, elemSize), values[i]);
            }
            Put(key, LogValue{std::move(buf), std::string(wpi::util::GetStructTypeString<T>()) + "[]"});
        }

        // span of aforementioned structs
        template<wpi::util::StructSerializable T> void Put(const std::string& key, std::span<const T> values) const {
            AddStructSchema<T>();
            const size_t elemSize = wpi::util::Struct<T>::GetSize();
            std::vector<uint8_t> buf(elemSize * values.size());
            for (size_t i = 0; i < values.size(); i++) {
                wpi::util::PackStruct(std::span{buf}.subspan(i * elemSize, elemSize), values[i]);
            }
            Put(key, LogValue{std::move(buf), std::string(wpi::util::GetStructTypeString<T>()) + "[]"});
        }

        template<wpi::util::StructSerializable T> void Put(const std::string& key, std::span<const std::vector<T>> values) const {
            Put(NormalizeKey(key, "length", false), static_cast<int64_t>(values.size()));
            for (size_t i = 0; i < values.size(); i++) {
                Put(NormalizeKey(key, std::to_string(i), false), values[i]);
            }
        }

        template<detail::ProtobufOnlySerializable T> void Put(const std::string& key, const T& value) const {
            wpi::util::ProtobufMessage<T> message;
            AddProtobufSchema(message);
            std::vector<uint8_t> buf;
            if (!message.Pack(buf, value)) return;
            Put(key, LogValue{std::move(buf), message.GetTypeString()});
        }

        template<typename T>
        requires std::is_aggregate_v<T> && (!std::is_array_v<T>) && (!wpi::util::StructSerializable<T>) && (!wpi::util::ProtobufSerializable<T>) &&
                 (!std::derived_from<T, LoggableInputs>)
        void Put(const std::string& key, const T& value) const;

        /* --------------------GETTERS-------------------- */

        // primitives
        [[nodiscard]] bool Get(std::string_view key, bool defaultValue) const;
        [[nodiscard]] int Get(std::string_view key, int defaultValue) const;
        [[nodiscard]] int64_t Get(std::string_view key, int64_t defaultValue) const;
        [[nodiscard]] float Get(std::string_view key, float defaultValue) const;
        [[nodiscard]] double Get(std::string_view key, double defaultValue) const;
        [[nodiscard]] std::string Get(std::string_view key, std::string defaultValue) const;

        // arrays
        [[nodiscard]] std::vector<uint8_t> Get(std::string_view key, std::span<const uint8_t> defaultValue) const;
        [[nodiscard]] std::vector<bool> Get(std::string_view key, std::span<const bool> defaultValue) const;
        [[nodiscard]] std::vector<bool> Get(std::string_view key, const std::vector<bool>& defaultValue) const;
        [[nodiscard]] std::vector<int> Get(std::string_view key, std::span<const int> defaultValue) const;
        [[nodiscard]] std::vector<int64_t> Get(std::string_view key, std::span<const int64_t> defaultValue) const;
        [[nodiscard]] std::vector<float> Get(std::string_view key, std::span<const float> defaultValue) const;
        [[nodiscard]] std::vector<double> Get(std::string_view key, std::span<const double> defaultValue) const;
        [[nodiscard]] std::vector<std::string> Get(std::string_view key, std::span<const std::string> defaultValue) const;

        // 2d arrays
        [[nodiscard]] std::vector<std::vector<bool>> Get(std::string_view key, std::span<const std::vector<bool>> defaultValue) const;
        [[nodiscard]] std::vector<std::vector<uint8_t>> Get(std::string_view key, std::span<const std::vector<uint8_t>> defaultValue) const;
        [[nodiscard]] std::vector<std::vector<int>> Get(std::string_view key, std::span<const std::vector<int>> defaultValue) const;
        [[nodiscard]] std::vector<std::vector<int64_t>> Get(std::string_view key, std::span<const std::vector<int64_t>> defaultValue) const;
        [[nodiscard]] std::vector<std::vector<float>> Get(std::string_view key, std::span<const std::vector<float>> defaultValue) const;
        [[nodiscard]] std::vector<std::vector<double>> Get(std::string_view key, std::span<const std::vector<double>> defaultValue) const;
        [[nodiscard]] std::vector<std::vector<std::string>> Get(std::string_view key, std::span<const std::vector<std::string>> defaultValue) const;

        template<typename E> requires std::is_enum_v<E>
        [[nodiscard]] E Get(std::string_view key, E defaultValue) const {
            auto name = Get(key, std::string(magic_enum::enum_name(defaultValue)));
            return magic_enum::enum_cast<E>(name).value_or(defaultValue);
        }

        template<typename E> requires std::is_enum_v<E>
        [[nodiscard]] std::vector<E> Get(std::string_view key, std::span<const E> defaultValue) const {
            std::vector<std::string> defaultNames;
            defaultNames.reserve(defaultValue.size());
            for (const auto& value : defaultValue) {
                defaultNames.emplace_back(magic_enum::enum_name(value));
            }

            const auto names = Get(key, std::span<const std::string>(defaultNames));
            std::vector<E> result;
            result.reserve(names.size());
            for (const auto& name : names) {
                auto enumValue = magic_enum::enum_cast<E>(name);
                if (!enumValue.has_value()) {
                    return std::vector<E>(defaultValue.begin(), defaultValue.end());
                }
                result.push_back(*enumValue);
            }
            return result;
        }

        template<typename E> requires std::is_enum_v<E>
        [[nodiscard]] std::vector<std::vector<E>> Get(std::string_view key, std::span<const std::vector<E>> defaultValue) const {
            std::vector<std::vector<E>> defaultRows(defaultValue.begin(), defaultValue.end());
            const LogValue* lv = Get(NormalizeKey(key, "length", false));
            if (!lv) return defaultRows;
            const auto* lenPtr = std::get_if<int64_t>(&lv->value);
            if (!lenPtr) return defaultRows;

            std::vector<std::vector<E>> result;
            result.reserve(static_cast<size_t>(*lenPtr));
            for (int64_t i = 0; i < *lenPtr; i++) {
                std::vector<E> rowDefault = i < static_cast<int64_t>(defaultRows.size()) ? defaultRows[static_cast<size_t>(i)] : std::vector<E>{};
                const auto rowNames = Get(NormalizeKey(key, std::to_string(i), false), std::vector<std::string>{});
                std::vector<E> row;
                row.reserve(rowNames.size());
                bool valid = true;
                for (const auto& rowName : rowNames) {
                    auto enumValue = magic_enum::enum_cast<E>(rowName);
                    if (!enumValue.has_value()) {
                        valid = false;
                        break;
                    }
                    row.push_back(*enumValue);
                }
                result.push_back(valid ? std::move(row) : std::move(rowDefault));
            }
            return result;
        }

        template<typename U> [[nodiscard]] wpi::units::unit_t<U> Get(std::string_view key, wpi::units::unit_t<U> defaultValue) const {
            using BaseUnit = wpi::units::unit<std::ratio<1>, typename wpi::units::traits::unit_traits<U>::base_unit_type>;
            const auto baseDefault = defaultValue.template convert<BaseUnit>();
            return wpi::units::unit_t<U>{wpi::units::unit_t<BaseUnit>{Get(key, baseDefault.value())}};
        }

        [[nodiscard]] wpi::util::Color Get(std::string_view key, wpi::util::Color defaultValue) const {
            return wpi::util::Color8Bit{Get(key, std::string{defaultValue.HexString()})};
        }

        [[nodiscard]] wpi::util::Color8Bit Get(std::string_view key, wpi::util::Color8Bit defaultValue) const {
            return wpi::util::Color8Bit{Get(key, std::string{defaultValue.HexString()})};
        }

        template<wpi::util::StructSerializable T> [[nodiscard]] T Get(std::string_view key, T defaultValue) const {
            const LogValue* lv = Get(key);
            if (!lv || lv->type != LoggableType::kRaw) return defaultValue;
            if (lv->customTypeStr != wpi::util::GetStructTypeString<T>()) return defaultValue;
            const auto& raw = std::get<std::vector<uint8_t>>(lv->value);
            if (raw.size() != wpi::util::Struct<T>::GetSize()) return defaultValue;
            return wpi::util::UnpackStruct<T>(raw);
        }

        template<detail::StructArrayElement T> [[nodiscard]] std::vector<T> Get(std::string_view key, const std::vector<T>& defaultValue = {}) const {
            const LogValue* lv = Get(key);
            if (!lv || lv->type != LoggableType::kRaw) return defaultValue;
            if (lv->customTypeStr != std::string(wpi::util::GetStructTypeString<T>()) + "[]") return defaultValue;
            const auto& raw = std::get<std::vector<uint8_t>>(lv->value);
            const size_t elemSize = wpi::util::Struct<T>::GetSize();
            if (elemSize == 0 || raw.size() % elemSize != 0) return defaultValue;
            std::vector<T> result;
            result.reserve(raw.size() / elemSize);
            for (size_t i = 0; i < raw.size(); i += elemSize)
                result.push_back(wpi::util::UnpackStruct<T>(std::span{raw}.subspan(i, elemSize)));
            return result;
        }

        template<wpi::util::StructSerializable T>
        [[nodiscard]] std::vector<std::vector<T>> Get(std::string_view key, std::span<const std::vector<T>> defaultValue) const {
            std::vector<std::vector<T>> defaults(defaultValue.begin(), defaultValue.end());
            const LogValue* lv = Get(NormalizeKey(key, "length", false));
            if (!lv) return defaults;
            const auto* lenPtr = std::get_if<int64_t>(&lv->value);
            if (!lenPtr) return defaults;

            std::vector<std::vector<T>> result;
            result.reserve(static_cast<size_t>(*lenPtr));
            for (int64_t i = 0; i < *lenPtr; i++) {
                const auto& rowDefault = i < static_cast<int64_t>(defaults.size()) ? defaults[static_cast<size_t>(i)] : std::vector<T>{};
                result.push_back(Get(NormalizeKey(key, std::to_string(i), false), rowDefault));
            }
            return result;
        }

        template<detail::ProtobufOnlySerializable T> [[nodiscard]] T Get(std::string_view key, T defaultValue) const {
            const LogValue* lv = Get(key);
            if (!lv || lv->type != LoggableType::kRaw) return defaultValue;
            wpi::util::ProtobufMessage<T> message;
            if (lv->customTypeStr != message.GetTypeString()) return defaultValue;
            auto unpacked = message.Unpack(std::get<std::vector<uint8_t>>(lv->value));
            if (!unpacked.has_value()) return defaultValue;
            return std::move(*unpacked);
        }

        template<typename T>
        requires std::is_aggregate_v<T> && (!std::is_array_v<T>) && (!wpi::util::StructSerializable<T>) && (!wpi::util::ProtobufSerializable<T>) &&
                 (!std::derived_from<T, LoggableInputs>)
        T Get(std::string_view key, T defaultValue) const;

        // raw value accessor w/ no default, returns nullptr (pls don't use this)
        [[nodiscard]] const LogValue* Get(std::string_view key) const;

        /* --------------------UTIL-------------------- */

        [[nodiscard]] LogTable GetSubtable(std::string_view name) const;

        [[nodiscard]] int64_t GetTimestamp() const;
        void SetTimestamp(int64_t timestamp) const;

        [[nodiscard]] const std::unordered_map<std::string, LogValue>& GetAll() const;
        [[nodiscard]] std::unordered_map<std::string, LogValue> GetAll(bool subtableOnly) const;

        [[nodiscard]] const std::string& GetPrefix() const;
        [[nodiscard]] int GetDepth() const;

        [[nodiscard]] static LogTable Clone(const LogTable& source, LogStorage& outStorage);

        [[nodiscard]] std::string ToString() const;

        void Clear();

    private:
        LogTable(LogStorage& storage, std::string prefix, int depth);

        [[nodiscard]] std::string NormalizeKey(std::string_view key, std::string_view child = {}, bool includePrefix = true) const;
        [[nodiscard]] bool WriteAllowed(const std::string& fullKey, LoggableType type, std::string_view customTypeStr = "") const;

        LogStorage* storage_;
        std::string prefix_;
        int depth_ = 0;

        template<wpi::util::StructSerializable T> void AddStructSchema() const {
            wpi::util::ForEachStructSchema<T>([this](std::string_view typeStr, std::string_view schema) {
                std::string schemaKey = "/.schema/";
                schemaKey += typeStr;
                if (storage_->values.contains(schemaKey)) return;
                std::vector<uint8_t> bytes(schema.begin(), schema.end());
                storage_->values.emplace(schemaKey, LogValue{std::move(bytes), "structschema"});
            });
        }

        template<detail::ProtobufOnlySerializable T> void AddProtobufSchema(wpi::util::ProtobufMessage<T>& message) const {
            message.ForEachProtobufDescriptor([this](std::string_view typeStr) { return storage_->values.contains("/.schema/" + std::string(typeStr)); },
                                              [this](std::string_view typeStr, std::span<const uint8_t> descriptor) {
                                                  std::vector<uint8_t> bytes(descriptor.begin(), descriptor.end());
                                                  storage_->values.emplace("/.schema/" + std::string(typeStr),
                                                                           LogValue{std::move(bytes), "proto:FileDescriptorProto"});
                                              });
        }

        template<typename T> T GetTyped(const std::string_view key, T defaultValue) const {
            auto it = storage_->values.find(NormalizeKey(key));
            if (it == storage_->values.end()) return defaultValue;

            if (auto* v = std::get_if<T>(&it->second.value)) {
                return *v;
            }

            return defaultValue;
        }

        template<typename T> void Put2D(const std::string& key, std::span<const std::vector<T>> value) const {
            Put(NormalizeKey(key, "length", false), static_cast<int64_t>(value.size()));
            for (size_t i = 0; i < value.size(); i++) {
                Put(NormalizeKey(key, std::to_string(i), false), std::span<const T>(value[i]));
            }
        }

        template<typename T> std::vector<std::vector<T>> Get2D(std::string_view key, std::span<const std::vector<T>> defaultValue) const {
            const LogValue* lv = Get(NormalizeKey(key, "length", false));
            if (!lv) return {defaultValue.begin(), defaultValue.end()};
            const auto* lenPtr = std::get_if<int64_t>(&lv->value);
            if (!lenPtr) return {defaultValue.begin(), defaultValue.end()};
            std::vector<std::vector<T>> defaults(defaultValue.begin(), defaultValue.end());
            std::vector<std::vector<T>> result;
            result.reserve(static_cast<size_t>(*lenPtr));
            for (int64_t i = 0; i < *lenPtr; i++) {
                const auto& rowDefault = i < static_cast<int64_t>(defaults.size()) ? defaults[static_cast<size_t>(i)] : std::vector<T>{};
                result.push_back(Get(NormalizeKey(key, std::to_string(i), false), std::span<const T>(rowDefault)));
            }
            return result;
        }
    };
} // namespace akit
