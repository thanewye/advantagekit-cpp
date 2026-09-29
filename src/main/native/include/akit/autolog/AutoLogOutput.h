#pragma once

#include "akit/autolog/AutoLogOutputManager.h"

#if defined(_MSC_VER)
#define AKIT_FUNCTION_SIGNATURE __FUNCSIG__
#else
#define AKIT_FUNCTION_SIGNATURE __PRETTY_FUNCTION__
#endif

#define SET_LOG_PREFIX(expr)                                                                                                                                   \
    template<typename AutoLogPrefixOwner> friend auto ::akit::detail::GetOverride(const AutoLogPrefixOwner& object)->std::optional<std::string>;               \
    std::optional<std::string> logPrefixOverride_ {                                                                                                            \
        std::string(expr)                                                                                                                                      \
    }

#define AKIT_AUTOLOG_DECLARE_PREFIX_HELPER(name)                                                                                                               \
    static constexpr std::string_view AutoLogClassName_##name() {                                                                                              \
        return ::akit::detail::ExtractEnclosingClassName(AKIT_FUNCTION_SIGNATURE, "::AutoLogClassName_" #name);                                                \
    }

#define AKIT_AUTOLOG_DECLARE_KEY_RESOLVER(name, key_arg)                                                                                                       \
    std::string AutoLogResolvedKey_##name() const {                                                                                                            \
        return ::akit::detail::ResolveAutoLogKey(*this, AutoLogClassName_##name(), #name, key_arg);                                                            \
    }

#define AKIT_AUTOLOG_OUTPUT_5(type, name, key_arg, unit_type, serial_arg)                                                                                      \
    AKIT_AUTOLOG_DECLARE_PREFIX_HELPER(name);                                                                                                                  \
    AKIT_AUTOLOG_DECLARE_KEY_RESOLVER(name, key_arg);                                                                                                          \
    ::akit::AutoLogged<type, unit_type, std::remove_cvref_t<decltype(serial_arg)>> name {                                                                      \
        AutoLogResolvedKey_##name(), unit_type{}, serial_arg                                                                                                   \
    }

#define AKIT_AUTOLOG_OUTPUT_4(type, name, key_arg, unit_type) AKIT_AUTOLOG_OUTPUT_5(type, name, key_arg, unit_type, ::akit::DefaultSerializable)

#define AKIT_AUTOLOG_OUTPUT_3(type, name, key_arg) AKIT_AUTOLOG_OUTPUT_5(type, name, key_arg, ::akit::NoUnitTag, ::akit::DefaultSerializable)

#define AKIT_AUTOLOG_OUTPUT_2(type, name) AKIT_AUTOLOG_OUTPUT_5(type, name, ::akit::DefaultKey, ::akit::NoUnitTag, ::akit::DefaultSerializable)

#define AKIT_AUTOLOG_OUTPUT_SELECT(_1, _2, _3, _4, _5, NAME, ...) NAME
#define AUTOLOG_OUTPUT(...)                                                                                                                                    \
    AKIT_AUTOLOG_OUTPUT_SELECT(__VA_ARGS__, AKIT_AUTOLOG_OUTPUT_5, AKIT_AUTOLOG_OUTPUT_4, AKIT_AUTOLOG_OUTPUT_3, AKIT_AUTOLOG_OUTPUT_2)(__VA_ARGS__)

#define AKIT_AUTOLOG_OUTPUT_SUPPLIER_5(name, expr, key_arg, unit_type, serial_arg)                                                                             \
    AKIT_AUTOLOG_DECLARE_PREFIX_HELPER(name);                                                                                                                  \
    AKIT_AUTOLOG_DECLARE_KEY_RESOLVER(name, key_arg);                                                                                                          \
    auto AutoLogSupplierValue_##name() -> decltype((expr)) {                                                                                                   \
        return (expr);                                                                                                                                         \
    }                                                                                                                                                          \
    ::akit::AutoLogRegistrationHandle autoLogSupplierRegistration_##name = [this] {                                                                            \
        auto resolvedKey = AutoLogResolvedKey_##name();                                                                                                        \
        return ::akit::AutoLogRegistrationHandle(                                                                                                              \
            resolvedKey, [this, key = resolvedKey] { ::akit::detail::RecordAutoLoggedValue(key, AutoLogSupplierValue_##name(), unit_type{}, serial_arg); });   \
    }()

#define AKIT_AUTOLOG_OUTPUT_SUPPLIER_4(name, expr, key_arg, unit_type)                                                                                         \
    AKIT_AUTOLOG_OUTPUT_SUPPLIER_5(name, expr, key_arg, unit_type, ::akit::DefaultSerializable)

#define AKIT_AUTOLOG_OUTPUT_SUPPLIER_3(name, expr, key_arg) AKIT_AUTOLOG_OUTPUT_SUPPLIER_5(name, expr, key_arg, ::akit::NoUnitTag, ::akit::DefaultSerializable)

#define AKIT_AUTOLOG_OUTPUT_SUPPLIER_2(name, expr)                                                                                                             \
    AKIT_AUTOLOG_OUTPUT_SUPPLIER_5(name, expr, ::akit::DefaultKey, ::akit::NoUnitTag, ::akit::DefaultSerializable)

#define AKIT_AUTOLOG_OUTPUT_SUPPLIER_SELECT(_1, _2, _3, _4, _5, NAME, ...) NAME
#define AUTOLOG_OUTPUT_SUPPLIER(...)                                                                                                                           \
    AKIT_AUTOLOG_OUTPUT_SUPPLIER_SELECT(__VA_ARGS__, AKIT_AUTOLOG_OUTPUT_SUPPLIER_5, AKIT_AUTOLOG_OUTPUT_SUPPLIER_4, AKIT_AUTOLOG_OUTPUT_SUPPLIER_3,           \
                                        AKIT_AUTOLOG_OUTPUT_SUPPLIER_2)(__VA_ARGS__)
