#pragma once

#include <cctype>
#include <concepts>
#include <string>
#include <string_view>
#include <utility>

#include <boost/pfr.hpp>

#include "akit/log/LogTable.h"

namespace akit {
    // pure virtual interface for users who want to write ToLog/FromLog themselves
    // instead of relying on compile time reflection, goes through vtable lookup
    class LoggableInputs {
    public:
        virtual ~LoggableInputs() = default;

        virtual void ToLog(LogTable& table) const = 0;
        virtual void FromLog(const LogTable& table) = 0;
    };

    // LoggableInputs but using TMP and compile time reflection (Boost.pfr) for no
    // vtable lookup and you don't have to write FromLog and ToLog
    namespace detail {
        inline std::string CapitalizeFirst(std::string_view name) {
            std::string result(name);
            if (!result.empty()) {
                result[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(result[0])));
            }
            return result;
        }

        template<typename T, size_t... Is> void to_log_impl(const T& obj, LogTable& table, std::index_sequence<Is...>) {
            (table.Put(CapitalizeFirst(boost::pfr::get_name<Is, T>()), boost::pfr::get<Is>(obj)), ...);
        }

        template<typename T, size_t... Is> void from_log_impl(T& obj, const LogTable& table, std::index_sequence<Is...>) {
            T defaults{};
            (
                [&] {
                    auto& field = boost::pfr::get<Is>(obj);
                    field = table.Get(CapitalizeFirst(boost::pfr::get_name<Is, T>()), boost::pfr::get<Is>(defaults));
                }(),
                ...);
        }
    } // namespace detail

    template<typename T>
    concept LoggableAggregate = std::is_aggregate_v<T> && !std::is_array_v<T> && !std::derived_from<T, LoggableInputs>;

    template<LoggableAggregate T> void ToLog(const T& obj, LogTable& table) {
        detail::to_log_impl(obj, table, std::make_index_sequence<boost::pfr::tuple_size_v<T>>{});
    }

    template<LoggableAggregate T> void FromLog(T& obj, const LogTable& table) {
        detail::from_log_impl(obj, table, std::make_index_sequence<boost::pfr::tuple_size_v<T>>{});
    }

    template<typename T> requires std::is_aggregate_v<T> && (!std::is_array_v<T>) && (!wpi::StructSerializable<T>) && (!std::derived_from<T, LoggableInputs>)
    void LogTable::Put(const std::string& key, const T& value) const {
        auto subtable = GetSubtable(key);
        ToLog(value, subtable);
    }

    template<typename T> requires std::is_aggregate_v<T> && (!std::is_array_v<T>) && (!wpi::StructSerializable<T>) && (!std::derived_from<T, LoggableInputs>)
    T LogTable::Get(std::string_view key, T defaultValue) const {
        auto subtable = GetSubtable(key);
        FromLog(defaultValue, subtable);
        return defaultValue;
    }
} // namespace akit
