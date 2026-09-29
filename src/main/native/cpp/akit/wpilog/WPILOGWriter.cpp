#include "pch.h"

#include "akit/wpilog/WPILOGWriter.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <frc/Errors.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>

#include "akit/Logger.h"
#include "akit/wpilog/WPILOGConstants.h"

namespace {
    std::tm LocalTime(std::time_t timestamp) {
        std::tm localTime{};
#ifdef _WIN32
        localtime_s(&localTime, &timestamp);
#else
        localtime_r(&timestamp, &localTime);
#endif
        return localTime;
    }
} // namespace

namespace akit::wpilog {
    LoggableType WPILOGWriter::GetType(const LogValue& value) const {
        return value.type;
    }

    void WPILOGWriter::AppendValue(const int64_t entryID, const LogValue& lv, const int64_t timestamp) {
        std::visit(
            [this, entryID, timestamp]<typename T>(const T& v) {
                if constexpr (std::is_same_v<T, bool>) log_->AppendBoolean(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, int64_t>) log_->AppendInteger(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, float>) log_->AppendFloat(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, double>) log_->AppendDouble(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, std::string>) log_->AppendString(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, std::vector<uint8_t>>) log_->AppendRaw(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, std::vector<bool>>) {
                    std::vector<int> wpilibArray;
                    wpilibArray.reserve(v.size());
                    for (const auto value : v)
                        wpilibArray.push_back(value);
                    log_->AppendBooleanArray(entryID, wpilibArray, timestamp);
                } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) log_->AppendIntegerArray(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, std::vector<float>>) log_->AppendFloatArray(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, std::vector<double>>) log_->AppendDoubleArray(entryID, v, timestamp);
                else if constexpr (std::is_same_v<T, std::vector<std::string>>) log_->AppendStringArray(entryID, v, timestamp);
            },
            lv.value);
    }

    WPILOGWriter::WPILOGWriter(const std::string& path, AdvantageScopeOpenBehavior openBehavior) {
        openBehavior_ = openBehavior;
        std::mt19937 generator(std::random_device{}());
        std::uniform_int_distribution<unsigned int> distribution(0, 0xFFFF);
        for (int i = 0; i < 4; ++i) {
            std::ostringstream stream;
            stream << std::hex << std::setw(4) << std::setfill('0') << distribution(generator);
            randomIdentifier_ += stream.str();
        }
        if (path.ends_with(".wpilog")) {
            std::filesystem::path pathFile(path);
            folder_ = pathFile.parent_path().empty() ? "." : pathFile.parent_path().string();
            fileName_ = pathFile.filename().string();
            autoRename_ = false;
        } else {
            folder_ = path;
            fileName_ = "akit_" + randomIdentifier_ + ".wpilog";
            autoRename_ = true;
        }
    }

    WPILOGWriter::WPILOGWriter(const std::string& path)
        : WPILOGWriter(path, AdvantageScopeOpenBehavior::kAuto) {}

    WPILOGWriter::WPILOGWriter(AdvantageScopeOpenBehavior openBehavior)
        : WPILOGWriter(frc::RobotBase::IsSimulation() ? kDefaultPathSim : kDefaultPathRio, openBehavior) {}

    WPILOGWriter::WPILOGWriter()
        : WPILOGWriter(frc::RobotBase::IsSimulation() ? kDefaultPathSim : kDefaultPathRio, AdvantageScopeOpenBehavior::kAuto) {}

    void WPILOGWriter::Start() {
        namespace fs = std::filesystem;
        fs::create_directories(folder_);
        if (const fs::path logFile = fs::path(folder_) / fileName_; fs::exists(logFile)) fs::remove(logFile);

        const std::string logPath = (fs::path(folder_) / fileName_).string();
        std::cout << "[AdvantageKit] Logging to \"" << logPath << "\"\n";

        std::error_code ec;
        log_ = std::make_unique<wpi::log::DataLogWriter>(logPath, ec, WPILOGConstants::kExtraHeader);
        if (ec) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] Failed to open output log file.");
            return;
        }
        isOpen_ = true;
        timestampID_ = log_->Start(kTimestampKey, GetWPILOGType(LoggableType::kInteger), WPILOGConstants::kEntryMetadata, 0);
        cycle_ = 0;

        // reset data
        entries_.clear();
        dsAttachedTime_ = std::nullopt;
        logDate_ = std::nullopt;
        logMatchText_ = std::nullopt;
    }

    void WPILOGWriter::End() {
        if (!isOpen_ || !log_) return;

        log_->Stop();
        isOpen_ = false;
        bool shouldOpenAscope = false;
        switch (openBehavior_) {
        case AdvantageScopeOpenBehavior::kAlways:
            shouldOpenAscope = frc::RobotBase::IsSimulation();
            break;
        case AdvantageScopeOpenBehavior::kAuto:
            shouldOpenAscope = frc::RobotBase::IsSimulation() && Logger::HasReplaySource();
            break;
        case AdvantageScopeOpenBehavior::kNever:
            shouldOpenAscope = false;
            break;
        }
        if (shouldOpenAscope) {
            try {
                namespace fs = std::filesystem;
                std::string fullLogPath = fs::absolute(fs::path(folder_) / fileName_).lexically_normal().string();

                fs::path tempPath = fs::temp_directory_path() / kAscopeFileName;
                std::ofstream writer(tempPath);
                if (!writer) throw std::runtime_error("could not open file");
                writer << fullLogPath << "\n";

                std::cout << "[AdvantageKit] Log sent to AdvantageScope\n";
            } catch (const std::exception& e) {
                FRC_ReportError(frc::err::Error, "[AdvantageKit] Failed to send log to AdvantageScope.");
            }
        }
    }

    void WPILOGWriter::PutTable(const LogTable& table) {
        if (!isOpen_) return;
        const int64_t timestamp = table.GetTimestamp();
        const auto getMetadata = [](const std::optional<std::string>& unit) {
            if (!unit.has_value()) return std::string(WPILOGConstants::kEntryMetadata);
            std::string metadata{WPILOGConstants::kEntryMetadataUnits};
            metadata.replace(metadata.find("$UNITSTR"), 8, *unit);
            return metadata;
        };
        if (autoRename_) {
            if (!logDate_.has_value()) {
                if ((table.Get("DriverStation/DSAttached", false) && table.Get("SystemStats/SystemTimeValid", false)) || frc::RobotBase::IsSimulation()) {
                    if (!dsAttachedTime_.has_value()) {
                        dsAttachedTime_ = static_cast<double>(frc::RobotController::GetFPGATime()) / 1000000.0;
                    } else if (static_cast<double>(frc::RobotController::GetFPGATime()) / 1000000.0 - dsAttachedTime_.value() > kTimestampUpdateDelay ||
                               frc::RobotBase::IsSimulation()) {
                        const auto now = std::chrono::system_clock::now();
                        const std::time_t t = std::chrono::system_clock::to_time_t(now);
                        logDate_ = LocalTime(t);
                    }
                } else {
                    dsAttachedTime_ = std::nullopt;
                }
            }
            HAL_MatchType matchType;
            switch (table.Get("DriverStation/MatchType", static_cast<int64_t>(0))) {
            case 1:
                matchType = HAL_kMatchType_practice;
                break;
            case 2:
                matchType = HAL_kMatchType_qualification;
                break;
            case 3:
                matchType = HAL_kMatchType_elimination;
                break;
            default:
                matchType = HAL_kMatchType_none;
                break;
            }
            if (!logMatchText_.has_value() && matchType != HAL_kMatchType_none) {
                logMatchText_ = "";
                switch (matchType) {
                case HAL_kMatchType_practice:
                    logMatchText_ = "p";
                    break;
                case HAL_kMatchType_qualification:
                    logMatchText_ = "q";
                    break;
                case HAL_kMatchType_elimination:
                    logMatchText_ = "e";
                    break;
                default:
                    break;
                }
                *logMatchText_ += std::to_string(table.Get("DriverStation/MatchNumber", static_cast<int64_t>(0)));
            }

            // Update filename
            std::string newFilename = "akit_";
            if (!logDate_.has_value()) {
                newFilename += randomIdentifier_;
            } else {
                std::ostringstream timeStream;
                timeStream << std::put_time(&logDate_.value(), kTimeFormat);
                newFilename += timeStream.str();
            }

            std::string eventName = table.Get("DriverStation/EventName", std::string{});
            std::ranges::transform(eventName, eventName.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            if (!eventName.empty()) {
                newFilename += "_";
                newFilename += eventName;
            }

            if (logMatchText_.has_value() && !logMatchText_->empty()) {
                newFilename += "_";
                newFilename += *logMatchText_;
            }

            newFilename += ".wpilog";

            if (newFilename != fileName_) {
                std::filesystem::path oldPath = std::filesystem::path(folder_) / fileName_;
                std::filesystem::path newPath = std::filesystem::path(folder_) / newFilename;

                std::cout << "[AdvantageKit] Renaming log to \"" << newPath.string() << "\"\n";

                std::error_code ec;
                std::filesystem::rename(oldPath, newPath, ec);
                if (!ec) {
                    fileName_ = newFilename;
                }
            }
        }

        log_->AppendInteger(timestampID_, timestamp, timestamp);
        ++cycle_;
        for (const auto& [key, value] : table.GetAll()) {
            auto [entryIt, isNewEntry] = entries_.try_emplace(key);
            EntryState& entry = entryIt->second;
            bool appendData = false;

            if (isNewEntry) {
                entry.id = log_->Start(key, value.GetWPILOGType(), getMetadata(value.unitStr), timestamp);
                entry.unit = value.unitStr;
                appendData = true;
            } else {
                const bool presentLastCycle = entry.lastPresentCycle + 1 == cycle_;
                const auto& lastValue = entry.lastWrittenValue;
                appendData = !presentLastCycle || !lastValue.has_value() || lastValue->type != value.type ||
                             lastValue->customTypeStr != value.customTypeStr || !LogValueVariantsEqual(lastValue->value, value.value);

                if (entry.unit != value.unitStr) {
                    log_->SetMetadata(entry.id, getMetadata(value.unitStr), timestamp);
                    entry.unit = value.unitStr;
                }
            }

            if (appendData) {
                AppendValue(entry.id, value, timestamp);
                entry.lastWrittenValue = value;
            }
            entry.lastPresentCycle = cycle_;
        }

        log_->Flush();
    }
} // namespace akit::wpilog
