#include "pch.h"

#include "akit/log/LogTable.h"

#include <utility>

#include <frc/Errors.h>

namespace akit {
    LogTable::LogTable(LogStorage& storage, std::string prefix)
        : storage_(&storage)
        , prefix_(prefix.empty() ? "/" : std::move(prefix)) {}

    LogTable::LogTable(LogStorage& storage, std::string prefix, int depth)
        : storage_(&storage)
        , prefix_(std::move(prefix))
        , depth_(depth) {}

    void LogTable::Put(const std::string& key, LogValue value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, value.type, value.customTypeStr)) storage_->values.insert_or_assign(fk, std::move(value));
    }

    void LogTable::Put(const std::string& key, const bool value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kBoolean)) storage_->values.insert_or_assign(fk, LogValue{value});
    }

    void LogTable::Put(const std::string& key, const int value) const {
        Put(key, static_cast<int64_t>(value));
    }

    void LogTable::Put(const std::string& key, const int64_t value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kInteger)) storage_->values.insert_or_assign(fk, LogValue{value});
    }

    void LogTable::Put(const std::string& key, const float value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kFloat)) storage_->values.insert_or_assign(fk, LogValue{value});
    }

    void LogTable::Put(const std::string& key, const float value, const std::string_view unit) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kFloat)) storage_->values.insert_or_assign(fk, LogValue{value, "", std::string(unit)});
    }

    void LogTable::Put(const std::string& key, const double value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kDouble)) storage_->values.insert_or_assign(fk, LogValue{value});
    }

    void LogTable::Put(const std::string& key, const double value, const std::string_view unit) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kDouble)) storage_->values.insert_or_assign(fk, LogValue{value, "", std::string(unit)});
    }

    void LogTable::Put(const std::string& key, const char* const value) const {
        Put(key, std::string_view(value));
    }

    void LogTable::Put(const std::string& key, const std::string_view value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kString)) storage_->values.insert_or_assign(fk, LogValue{std::string(value)});
    }

    void LogTable::Put(const std::string& key, const std::span<const uint8_t> value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kRaw)) storage_->values.insert_or_assign(fk, LogValue{std::vector<uint8_t>(value.begin(), value.end())});
    }

    void LogTable::Put(const std::string& key, const std::span<const bool> value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kBooleanArray)) storage_->values.insert_or_assign(fk, LogValue{std::vector<bool>(value.begin(), value.end())});
    }

    void LogTable::Put(const std::string& key, const std::vector<bool>& value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kBooleanArray)) storage_->values.insert_or_assign(fk, LogValue{value});
    }

    void LogTable::Put(const std::string& key, const std::span<const int> value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kIntegerArray)) storage_->values.insert_or_assign(fk, LogValue{std::vector<int64_t>(value.begin(), value.end())});
    }

    void LogTable::Put(const std::string& key, const std::span<const int64_t> value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kIntegerArray)) storage_->values.insert_or_assign(fk, LogValue{std::vector<int64_t>(value.begin(), value.end())});
    }

    void LogTable::Put(const std::string& key, const std::span<const float> value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kFloatArray)) storage_->values.insert_or_assign(fk, LogValue{std::vector<float>(value.begin(), value.end())});
    }

    void LogTable::Put(const std::string& key, const std::span<const double> value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kDoubleArray)) storage_->values.insert_or_assign(fk, LogValue{std::vector<double>(value.begin(), value.end())});
    }

    void LogTable::Put(const std::string& key, const std::span<const std::string> value) const {
        const std::string fk = NormalizeKey(key);
        if (WriteAllowed(fk, LoggableType::kStringArray)) storage_->values.insert_or_assign(fk, LogValue{std::vector<std::string>(value.begin(), value.end())});
    }

    void LogTable::Put(const std::string& key, const std::span<const std::vector<uint8_t>> value) const {
        Put2D<uint8_t>(key, value);
    }

    void LogTable::Put(const std::string& key, const std::span<const std::vector<bool>> value) const {
        Put(NormalizeKey(key, "length", false), static_cast<int64_t>(value.size()));
        for (size_t i = 0; i < value.size(); i++) {
            Put(NormalizeKey(key, std::to_string(i), false), value[i]);
        }
    }

    void LogTable::Put(const std::string& key, const std::span<const std::vector<int>> value) const {
        Put2D<int>(key, value);
    }

    void LogTable::Put(const std::string& key, const std::span<const std::vector<int64_t>> value) const {
        Put2D<int64_t>(key, value);
    }

    void LogTable::Put(const std::string& key, const std::span<const std::vector<float>> value) const {
        Put2D<float>(key, value);
    }

    void LogTable::Put(const std::string& key, const std::span<const std::vector<double>> value) const {
        Put2D<double>(key, value);
    }

    void LogTable::Put(const std::string& key, const std::span<const std::vector<std::string>> value) const {
        Put2D<std::string>(key, value);
    }

    bool LogTable::Get(const std::string_view key, const bool defaultValue) const {
        return GetTyped<bool>(key, defaultValue);
    }

    int LogTable::Get(const std::string_view key, const int defaultValue) const {
        return static_cast<int>(GetTyped<int64_t>(key, static_cast<int64_t>(defaultValue)));
    }

    int64_t LogTable::Get(const std::string_view key, const int64_t defaultValue) const {
        return GetTyped<int64_t>(key, defaultValue);
    }

    float LogTable::Get(const std::string_view key, const float defaultValue) const {
        return GetTyped<float>(key, defaultValue);
    }

    double LogTable::Get(const std::string_view key, const double defaultValue) const {
        return GetTyped<double>(key, defaultValue);
    }

    std::string LogTable::Get(const std::string_view key, std::string defaultValue) const {
        return GetTyped<std::string>(key, std::move(defaultValue));
    }

    std::vector<uint8_t> LogTable::Get(const std::string_view key, const std::span<const uint8_t> defaultValue) const {
        return GetTyped<std::vector<uint8_t>>(key, std::vector<uint8_t>(defaultValue.begin(), defaultValue.end()));
    }

    std::vector<bool> LogTable::Get(const std::string_view key, const std::span<const bool> defaultValue) const {
        const LogValue* lv = Get(key);
        if (!lv) return std::vector<bool>(defaultValue.begin(), defaultValue.end());
        const auto* bools = std::get_if<std::vector<bool>>(&lv->value);
        if (!bools) return std::vector<bool>(defaultValue.begin(), defaultValue.end());
        return *bools;
    }

    std::vector<bool> LogTable::Get(const std::string_view key, const std::vector<bool>& defaultValue) const {
        const LogValue* lv = Get(key);
        if (!lv) return defaultValue;
        const auto* bools = std::get_if<std::vector<bool>>(&lv->value);
        if (!bools) return defaultValue;
        return *bools;
    }

    std::vector<int> LogTable::Get(const std::string_view key, const std::span<const int> defaultValue) const {
        const LogValue* lv = Get(key);
        if (!lv) return std::vector<int>(defaultValue.begin(), defaultValue.end());
        const auto* ints = std::get_if<std::vector<int64_t>>(&lv->value);
        if (!ints) return std::vector<int>(defaultValue.begin(), defaultValue.end());
        std::vector<int> result;
        result.reserve(ints->size());
        for (const auto value : *ints)
            result.push_back(static_cast<int>(value));
        return result;
    }

    std::vector<int64_t> LogTable::Get(const std::string_view key, const std::span<const int64_t> defaultValue) const {
        return GetTyped<std::vector<int64_t>>(key, std::vector<int64_t>(defaultValue.begin(), defaultValue.end()));
    }

    std::vector<float> LogTable::Get(const std::string_view key, const std::span<const float> defaultValue) const {
        return GetTyped<std::vector<float>>(key, std::vector<float>(defaultValue.begin(), defaultValue.end()));
    }

    std::vector<double> LogTable::Get(const std::string_view key, const std::span<const double> defaultValue) const {
        return GetTyped<std::vector<double>>(key, std::vector<double>(defaultValue.begin(), defaultValue.end()));
    }

    std::vector<std::string> LogTable::Get(const std::string_view key, const std::span<const std::string> defaultValue) const {
        return GetTyped<std::vector<std::string>>(key, std::vector<std::string>(defaultValue.begin(), defaultValue.end()));
    }

    std::vector<std::vector<uint8_t>> LogTable::Get(const std::string_view key, const std::span<const std::vector<uint8_t>> defaultValue) const {
        return Get2D<uint8_t>(key, defaultValue);
    }

    std::vector<std::vector<bool>> LogTable::Get(const std::string_view key, const std::span<const std::vector<bool>> defaultValue) const {
        std::vector<std::vector<bool>> defaults(defaultValue.begin(), defaultValue.end());
        const LogValue* lv = Get(NormalizeKey(key, "length", false));
        if (!lv) return defaults;
        const auto* lenPtr = std::get_if<int64_t>(&lv->value);
        if (!lenPtr) return defaults;

        std::vector<std::vector<bool>> result;
        result.reserve(static_cast<size_t>(*lenPtr));
        for (int64_t i = 0; i < *lenPtr; i++) {
            const auto& rowDefault = i < static_cast<int64_t>(defaults.size()) ? defaults[static_cast<size_t>(i)] : std::vector<bool>{};
            result.push_back(Get(NormalizeKey(key, std::to_string(i), false), rowDefault));
        }
        return result;
    }

    std::vector<std::vector<int>> LogTable::Get(const std::string_view key, const std::span<const std::vector<int>> defaultValue) const {
        return Get2D<int>(key, defaultValue);
    }

    std::vector<std::vector<int64_t>> LogTable::Get(const std::string_view key, const std::span<const std::vector<int64_t>> defaultValue) const {
        return Get2D<int64_t>(key, defaultValue);
    }

    std::vector<std::vector<float>> LogTable::Get(const std::string_view key, const std::span<const std::vector<float>> defaultValue) const {
        return Get2D<float>(key, defaultValue);
    }

    std::vector<std::vector<double>> LogTable::Get(const std::string_view key, const std::span<const std::vector<double>> defaultValue) const {
        return Get2D<double>(key, defaultValue);
    }

    std::vector<std::vector<std::string>> LogTable::Get(const std::string_view key, const std::span<const std::vector<std::string>> defaultValue) const {
        return Get2D<std::string>(key, defaultValue);
    }

    const LogValue* LogTable::Get(const std::string_view key) const {
        auto it = storage_->values.find(NormalizeKey(key));
        return it == storage_->values.end() ? nullptr : &it->second;
    }

    LogTable LogTable::GetSubtable(std::string_view key) const {
        std::string nextPrefix = NormalizeKey(key);
        nextPrefix += "/";
        return LogTable(*storage_, std::move(nextPrefix), depth_ + 1);
    }

    int LogTable::GetDepth() const {
        return depth_;
    }

    int64_t LogTable::GetTimestamp() const {
        return storage_->timestamp;
    }

    void LogTable::SetTimestamp(const int64_t timestamp) const {
        storage_->timestamp = timestamp;
    }

    const std::unordered_map<std::string, LogValue>& LogTable::GetAll() const {
        return storage_->values;
    }

    std::unordered_map<std::string, LogValue> LogTable::GetAll(const bool subtableOnly) const {
        if (!subtableOnly) return storage_->values;
        std::unordered_map<std::string, LogValue> result;
        for (const auto& [key, val] : storage_->values) {
            if (key.starts_with(prefix_)) result.emplace(key.substr(prefix_.size()), val);
        }
        return result;
    }

    const std::string& LogTable::GetPrefix() const {
        return prefix_;
    }

    LogTable LogTable::Clone(const LogTable& source, LogStorage& outStorage) {
        outStorage.values = source.storage_->values;
        outStorage.timestamp = source.storage_->timestamp;
        return LogTable(outStorage, source.prefix_, source.depth_);
    }

    std::string LogTable::ToString() const {
        std::string out;
        out += "Timestamp=" + std::to_string(storage_->timestamp) + "\n";
        out += "Prefix=\"" + prefix_ + "\"\n{\n";
        for (const auto& [key, val] : GetAll(true)) {
            out += "\t" + key + "[" + val.GetWPILOGType();
            if (val.unitStr.has_value()) out += "," + *val.unitStr;
            out += "]=";
            std::visit(
                [&out]<typename T>(const T& v) {
                    if constexpr (std::is_same_v<T, bool>) {
                        out += v ? "true" : "false";
                    } else if constexpr (std::is_same_v<T, std::string>) {
                        out += "\"" + v + "\"";
                    } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
                        out += "[";
                        bool first = true;
                        for (const auto elem : v) {
                            if (!first) out += ",";
                            out += elem ? "true" : "false";
                            first = false;
                        }
                        out += "]";
                    } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
                        out += "[";
                        bool first = true;
                        for (const auto& elem : v) {
                            if (!first) out += ",";
                            out += "\"" + elem + "\"";
                            first = false;
                        }
                        out += "]";
                    } else if constexpr (std::is_same_v<T, std::vector<uint8_t>> || std::is_same_v<T, std::vector<int64_t>> ||
                                         std::is_same_v<T, std::vector<float>> || std::is_same_v<T, std::vector<double>>) {
                        out += "[";
                        bool first = true;
                        for (const auto& elem : v) {
                            if (!first) out += ",";
                            out += std::to_string(elem);
                            first = false;
                        }
                        out += "]";
                    } else {
                        out += std::to_string(v);
                    }
                },
                val.value);
            out += "\n";
        }
        out += "}";
        return out;
    }

    std::string LogTable::NormalizeKey(const std::string_view key, const std::string_view child, const bool includePrefix) const {
        size_t start = 0;
        while (start < key.size() && key[start] == '/')
            start++;
        std::string normalizedKey;
        if (includePrefix) {
            normalizedKey = prefix_;
        }
        normalizedKey += key.substr(start);
        if (!child.empty()) {
            normalizedKey += "/";
            normalizedKey += child;
        }
        return normalizedKey;
    }

    bool LogTable::WriteAllowed(const std::string& fullKey, const LoggableType type, const std::string_view customTypeStr) const {
        auto it = storage_->values.find(fullKey);
        if (it == storage_->values.end()) return true;
        if (it->second.type != type) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] Failed to write to field \"{}\" — type mismatch (existing={} attempted={})", fullKey,
                            static_cast<int>(it->second.type), static_cast<int>(type));
            return false;
        }
        if (it->second.customTypeStr != customTypeStr) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] Struct type mismatch on \"{}\" (existing={} attempted={})", fullKey, it->second.customTypeStr,
                            customTypeStr);
            return false;
        }
        return true;
    }

    void LogTable::Clear() {
        if (prefix_.empty()) {
            storage_->values.clear();
            return;
        }
        std::erase_if(storage_->values, [this](const auto& pair) { return pair.first.starts_with(prefix_); });
    }
} // namespace akit
