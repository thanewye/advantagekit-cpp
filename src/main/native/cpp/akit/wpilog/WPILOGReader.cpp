#include "pch.h"

#include "akit/wpilog/WPILOGReader.h"

#include <string_view>
#include <utility>
#include <vector>

#include <frc/Errors.h>
#include <wpi/MemoryBuffer.h>

#include "akit/log/LogDataReceiver.h"
#include "akit/log/LogStorage.h"
#include "akit/wpilog/WPILOGConstants.h"

namespace akit::wpilog {
    namespace {
        std::optional<std::string> ParseUnit(std::string_view metadata) {
            constexpr std::string_view kUnitPrefix = R"("unit":")";
            const size_t prefixIndex = metadata.find(kUnitPrefix);
            if (prefixIndex == std::string_view::npos) return std::nullopt;
            const size_t startIndex = prefixIndex + kUnitPrefix.size();
            const size_t endIndex = metadata.find('"', startIndex);
            if (endIndex == std::string_view::npos) return std::nullopt;
            return std::string(metadata.substr(startIndex, endIndex - startIndex));
        }

        std::optional<LogValue> ReadValue(const wpi::log::DataLogRecord& record, const LoggableType type, std::string customType,
                                          std::optional<std::string> unit) {
            switch (type) {
            case LoggableType::kRaw: {
                const auto raw = record.GetRaw();
                return LogValue{std::vector<uint8_t>(raw.begin(), raw.end()), std::move(customType)};
            }
            case LoggableType::kBoolean: {
                bool value;
                if (!record.GetBoolean(&value)) return std::nullopt;
                return LogValue{value, std::move(customType)};
            }
            case LoggableType::kInteger: {
                int64_t value;
                if (!record.GetInteger(&value)) return std::nullopt;
                return LogValue{value, std::move(customType)};
            }
            case LoggableType::kFloat: {
                float value;
                if (!record.GetFloat(&value)) return std::nullopt;
                return LogValue{value, std::move(customType), std::move(unit)};
            }
            case LoggableType::kDouble: {
                double value;
                if (!record.GetDouble(&value)) return std::nullopt;
                return LogValue{value, std::move(customType), std::move(unit)};
            }
            case LoggableType::kString: {
                std::string_view value;
                if (!record.GetString(&value)) return std::nullopt;
                return LogValue{std::string(value), std::move(customType)};
            }
            case LoggableType::kBooleanArray: {
                std::vector<int> value;
                if (!record.GetBooleanArray(&value)) return std::nullopt;
                return LogValue{std::vector<bool>(value.begin(), value.end()), std::move(customType)};
            }
            case LoggableType::kIntegerArray: {
                std::vector<int64_t> value;
                if (!record.GetIntegerArray(&value)) return std::nullopt;
                return LogValue{std::move(value), std::move(customType)};
            }
            case LoggableType::kFloatArray: {
                std::vector<float> value;
                if (!record.GetFloatArray(&value)) return std::nullopt;
                return LogValue{std::move(value), std::move(customType)};
            }
            case LoggableType::kDoubleArray: {
                std::vector<double> value;
                if (!record.GetDoubleArray(&value)) return std::nullopt;
                return LogValue{std::move(value), std::move(customType)};
            }
            case LoggableType::kStringArray: {
                std::vector<std::string_view> value;
                if (!record.GetStringArray(&value)) return std::nullopt;
                return LogValue{std::vector<std::string>(value.begin(), value.end()), std::move(customType)};
            }
            }
            return std::nullopt;
        }
    } // namespace

    WPILOGReader::WPILOGReader(std::string filename)
        : filename_(std::move(filename)) {}

    void WPILOGReader::Start() {
        isValid_ = false;
        iterator_.reset();
        reader_.reset();
        timestamp_.reset();
        entryIDs_.clear();
        entryTypes_.clear();
        entryCustomTypes_.clear();
        entryUnits_.clear();

        auto buffer = wpi::MemoryBuffer::GetFile(filename_);
        if (!buffer) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] Failed to open replay log file.");
            return;
        }

        reader_ = std::make_unique<wpi::log::DataLogReader>(std::move(*buffer));
        if (!reader_->IsValid()) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] The replay log is not a valid WPILOG file.");
            return;
        }
        if (reader_->GetExtraHeader() != WPILOGConstants::kExtraHeader) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] The replay log was not produced by AdvantageKit.");
            return;
        }

        isValid_ = true;
        iterator_ = reader_->begin();
    }

    bool WPILOGReader::UpdateTable(LogTable& table) {
        if (!isValid_) return false;

        if (timestamp_) table.SetTimestamp(*timestamp_);

        const auto end = reader_->end();
        while (*iterator_ != end) {
            const wpi::log::DataLogRecord record = **iterator_;
            ++*iterator_;

            if (record.IsControl()) {
                if (record.IsStart()) {
                    wpi::log::StartRecordData startData;
                    if (!record.GetStartData(&startData)) continue;
                    const LoggableType type = FromWPILOGType(startData.type);
                    entryIDs_.insert_or_assign(startData.entry, std::string(startData.name));
                    entryTypes_.insert_or_assign(startData.entry, type);
                    if ((type == LoggableType::kRaw && startData.type != "raw") || startData.type == "json") {
                        entryCustomTypes_.insert_or_assign(startData.entry, std::string(startData.type));
                    }
                    if (auto unit = ParseUnit(startData.metadata)) entryUnits_.insert_or_assign(startData.entry, std::move(*unit));
                } else if (record.IsSetMetadata()) {
                    wpi::log::MetadataRecordData metadataData;
                    if (!record.GetSetMetadataData(&metadataData)) continue;
                    if (auto unit = ParseUnit(metadataData.metadata)) entryUnits_.insert_or_assign(metadataData.entry, std::move(*unit));
                    else entryUnits_.erase(metadataData.entry);
                }
                continue;
            }

            const auto entryName = entryIDs_.find(record.GetEntry());
            if (entryName == entryIDs_.end()) continue;

            if (entryName->second == LogDataReceiver::kTimestampKey) {
                int64_t newTimestamp;
                if (!record.GetInteger(&newTimestamp)) continue;
                const bool firstTimestamp = !timestamp_.has_value();
                timestamp_ = newTimestamp;
                if (!firstTimestamp) break;
                table.SetTimestamp(newTimestamp);
                continue;
            }

            if (!timestamp_ || record.GetTimestamp() != *timestamp_) continue;
            if (!entryName->second.starts_with('/')) continue;

            const std::string key = entryName->second.substr(1);
            if (key.starts_with("ReplayOutputs")) continue;

            const auto customType = entryCustomTypes_.find(record.GetEntry());
            const auto unit = entryUnits_.find(record.GetEntry());
            auto value = ReadValue(record, entryTypes_.at(record.GetEntry()), customType == entryCustomTypes_.end() ? "" : customType->second,
                                   unit == entryUnits_.end() ? std::nullopt : std::optional<std::string>(unit->second));
            if (value) table.Put(key, std::move(*value));
        }

        return *iterator_ != end;
    }
} // namespace akit::wpilog
