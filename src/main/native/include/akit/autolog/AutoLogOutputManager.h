#pragma once

#include <concepts>
#include <cstdio>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <frc/Errors.h>
#include <wpi/struct/Struct.h>

#include "akit/Logger.h"

namespace akit {
    struct KeyTag {};

    struct DefaultKeyTag : KeyTag {};

    struct UnitTag {};

    struct NoUnitTag : UnitTag {};

    struct SerializableTag {};

    struct DefaultSerializableTag : SerializableTag {};

    struct ForceSerializableTag : SerializableTag {};

    inline constexpr DefaultKeyTag DefaultKey{};
    inline constexpr NoUnitTag NoUnit{};
    inline constexpr DefaultSerializableTag DefaultSerializable{};
    inline constexpr ForceSerializableTag ForceSerializable{};

    class AutoLogOutputManager {
    public:
        struct RegistrationId {
            size_t generation = 0;
            size_t slot = 0;
        };

        static RegistrationId Register(std::string key, std::function<void()> flush);

        static void Unregister(RegistrationId id);

        static void Periodic();

        static void Reset();
    };

    class AutoLogRegistrationHandle {
    public:
        AutoLogRegistrationHandle() = default;

        template<typename Flush> requires std::constructible_from<std::function<void()>, Flush>
        AutoLogRegistrationHandle(std::string key, Flush&& flush)
            : id_(AutoLogOutputManager::Register(std::move(key), std::forward<Flush>(flush))) {}

        ~AutoLogRegistrationHandle() { Reset(); }

        AutoLogRegistrationHandle(const AutoLogRegistrationHandle&) = delete;

        AutoLogRegistrationHandle& operator=(const AutoLogRegistrationHandle&) = delete;

        AutoLogRegistrationHandle(AutoLogRegistrationHandle&&) = delete;

        AutoLogRegistrationHandle& operator=(AutoLogRegistrationHandle&&) = delete;

        void Reset() {
            if (!id_.has_value()) return;
            AutoLogOutputManager::Unregister(*id_);
            id_.reset();
        }

    private:
        std::optional<AutoLogOutputManager::RegistrationId> id_;
    };

    namespace detail {
        [[noreturn]] inline void FailAutoLogRegistration(std::string_view message, std::string_view key) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] {}: {}", message, key);
            std::fprintf(stderr, "[AdvantageKit] %.*s: %.*s\n", static_cast<int>(message.size()), message.data(), static_cast<int>(key.size()), key.data());
            std::fflush(stderr);
            std::abort();
        }

        template<typename Unit>
        concept NamedUnit = requires(const std::remove_cvref_t<Unit>& unit) {
            { unit.name() } -> std::convertible_to<std::string_view>;
        };

        template<typename T, typename Unit>
        inline constexpr bool kSupportsExplicitUnit =
            (std::same_as<std::remove_cvref_t<T>, float> || std::same_as<std::remove_cvref_t<T>, double>) && NamedUnit<Unit>;

        template<typename T> inline constexpr bool kSupportsForceSerializable = wpi::StructSerializable<std::remove_cvref_t<T>>;

        constexpr std::string_view ExtractEnclosingClassName(const std::string_view prettyFunction, const std::string_view methodMarker) {
            const size_t classEnd = prettyFunction.find(methodMarker);
            if (classEnd == std::string_view::npos) return {};

            const size_t lastSpace = classEnd == 0 ? std::string_view::npos : prettyFunction.rfind(' ', classEnd - 1);
            const size_t qualifiedStart = lastSpace == std::string_view::npos ? 0 : lastSpace + 1;

            const size_t separatorBeforeClass = prettyFunction.rfind("::", classEnd - 1);
            const size_t classStart =
                (separatorBeforeClass == std::string_view::npos || separatorBeforeClass < qualifiedStart) ? qualifiedStart : separatorBeforeClass + 2;
            return prettyFunction.substr(classStart, classEnd - classStart);
        }

        inline std::string Capitalize(std::string_view name) {
            std::string result(name);
            if (!result.empty() && result[0] >= 'a' && result[0] <= 'z') {
                result[0] = static_cast<char>(result[0] - ('a' - 'A'));
            }
            return result;
        }

        template<typename T> std::optional<std::string> GetOverride(const T& object) {
            if constexpr (requires { object.logPrefixOverride_; }) {
                return object.logPrefixOverride_;
            } else {
                return std::nullopt;
            }
        }

        inline std::string ResolveExplicitKey(const std::string_view key) {
            return std::string(key);
        }

        inline std::string ResolveExplicitKey(const std::string& key) {
            return key;
        }

        inline std::string ResolveExplicitKey(const char* key) {
            return {key};
        }

        template<typename Key> std::string ResolveExplicitKey(Key&& key) {
            return std::string(std::forward<Key>(key));
        }

        template<typename Object>
        std::string ResolveAutoLogKey(const Object& object, const std::string_view inferredClassName, const std::string_view memberName, DefaultKeyTag) {
            const auto prefix = GetOverride(object);
            const auto resolvedPrefix = prefix ? *prefix : std::string(inferredClassName);
            return resolvedPrefix + "/" + Capitalize(memberName);
        }

        template<typename Object, typename Key> std::string ResolveAutoLogKey(const Object&, std::string_view, std::string_view, Key&& explicitKey) {
            return ResolveExplicitKey(std::forward<Key>(explicitKey));
        }

        template<typename T, typename Unit, typename SerialMode>
        void RecordAutoLoggedValue(const std::string& key, const T& value, const Unit& unit, SerialMode) {
            using ValueType = std::remove_cvref_t<T>;
            using UnitType = std::remove_cvref_t<Unit>;
            using SerialType = std::remove_cvref_t<SerialMode>;

            if constexpr (std::same_as<SerialType, ForceSerializableTag>) {
                static_assert(kSupportsForceSerializable<ValueType>, "akit::ForceSerializable requires a wpi::StructSerializable type.");
                static_assert(std::same_as<UnitType, NoUnitTag>, "Explicit unit metadata is not supported with akit::ForceSerializable.");
                Logger::RecordOutputStruct(key, value);
            } else if constexpr (std::same_as<UnitType, NoUnitTag>) {
                Logger::RecordOutput(key, value);
            } else {
                static_assert(kSupportsExplicitUnit<ValueType, UnitType>,
                              "Explicit AUTOLOG_OUTPUT unit metadata is only supported for raw float/double values.");
                Logger::RecordOutput(key, value, unit.name());
            }
        }
    } // namespace detail

    template<typename T, typename Unit = NoUnitTag, typename SerialMode = DefaultSerializableTag> class AutoLogged {
    public:
        explicit AutoLogged(std::string key, Unit unit = {}, SerialMode serialMode = {})
            : key_(std::move(key))
            , unit_(std::move(unit))
            , serialMode_(std::move(serialMode))
            , handle_(key_, [this] { detail::RecordAutoLoggedValue(key_, value_, unit_, serialMode_); }) {}

        AutoLogged(const AutoLogged&) = delete;

        AutoLogged& operator=(const AutoLogged&) = delete;

        AutoLogged(AutoLogged&&) = delete;

        AutoLogged& operator=(AutoLogged&&) = delete;

        template<typename U> requires std::assignable_from<T&, U&&>
        AutoLogged& operator=(U&& value) {
            value_ = std::forward<U>(value);
            return *this;
        }

        operator T&() { return value_; }
        operator const T&() const { return value_; }

        T& Get() { return value_; }
        const T& Get() const { return value_; }

        T* operator->() requires std::is_class_v<T>
        {
            return &value_;
        }

        const T* operator->() const requires std::is_class_v<T>
        {
            return &value_;
        }

    private:
        std::string key_;
        [[no_unique_address]] Unit unit_{};
        [[no_unique_address]] SerialMode serialMode_{};
        T value_{};
        AutoLogRegistrationHandle handle_;
    };
} // namespace akit
