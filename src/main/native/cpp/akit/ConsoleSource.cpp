#include "pch.h"

#include "akit/ConsoleSource.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <utility>

#include <frc/Errors.h>

namespace akit {
    ConsoleSource::Simulator::TeeStreambuf::TeeStreambuf(std::streambuf* first, std::streambuf* second, std::mutex& captureMutex)
        : first_(first)
        , second_(second)
        , captureMutex_(captureMutex) {}

    int ConsoleSource::Simulator::TeeStreambuf::overflow(const int ch) {
        if (ch == traits_type::eof()) return traits_type::not_eof(ch);
        std::scoped_lock lock(captureMutex_);
        const auto firstResult = first_->sputc(static_cast<char>(ch));
        const auto secondResult = second_->sputc(static_cast<char>(ch));
        if (firstResult == traits_type::eof() || secondResult == traits_type::eof()) {
            return traits_type::eof();
        }
        return ch;
    }

    std::streamsize ConsoleSource::Simulator::TeeStreambuf::xsputn(const char* data, const std::streamsize count) {
        std::scoped_lock lock(captureMutex_);
        const auto firstCount = first_->sputn(data, count);
        const auto secondCount = second_->sputn(data, count);
        return std::min(firstCount, secondCount);
    }

    int ConsoleSource::Simulator::TeeStreambuf::sync() {
        std::scoped_lock lock(captureMutex_);
        const auto firstResult = first_->pubsync();
        const auto secondResult = second_->pubsync();
        return firstResult == 0 && secondResult == 0 ? 0 : -1;
    }

    ConsoleSource::Simulator::Simulator() {
        stdout_ = std::cout.rdbuf();
        stderr_ = std::cerr.rdbuf();
        stdoutTee_ = std::make_unique<TeeStreambuf>(stdout_, &capturedStdout_, mutex_);
        stderrTee_ = std::make_unique<TeeStreambuf>(stderr_, &capturedStderr_, mutex_);
        std::cout.rdbuf(stdoutTee_.get());
        std::cerr.rdbuf(stderrTee_.get());
    }

    ConsoleSource::Simulator::~Simulator() {
        std::scoped_lock lock(mutex_);
        if (stdout_ != nullptr) std::cout.rdbuf(stdout_);
        if (stderr_ != nullptr) std::cerr.rdbuf(stderr_);
    }

    std::string ConsoleSource::Simulator::GetNewData() {
        std::scoped_lock lock(mutex_);

        const std::string stdoutData = capturedStdout_.str();
        const std::string stderrData = capturedStderr_.str();

        std::string output;
        output.reserve((stdoutData.size() - stdoutPos_) + (stderrData.size() - stderrPos_));
        output.append(stdoutData.substr(stdoutPos_));
        output.append(stderrData.substr(stderrPos_));

        stdoutPos_ = stdoutData.size();
        stderrPos_ = stderrData.size();
        return output;
    }

    ConsoleSource::RoboRIO::RoboRIO()
        : thread_([this] { Run(); }) {}

    ConsoleSource::RoboRIO::~RoboRIO() {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
    }

    std::string ConsoleSource::RoboRIO::GetNewData() {
        std::vector<std::string> drainedLines;
        {
            std::scoped_lock lock(mutex_);
            drainedLines.swap(lines_);
        }

        std::string output;
        for (std::size_t i = 0; i < drainedLines.size(); ++i) {
            if (i > 0) output.push_back('\n');
            output.append(drainedLines[i]);
        }
        return output;
    }

    std::string ConsoleSource::RoboRIO::GetFilePath() {
        return "/home/lvuser/FRC_UserProgram.log";
    }

    void ConsoleSource::RoboRIO::Run() {
        std::ifstream reader(GetFilePath());
        if (!reader.is_open()) {
            FRC_ReportError(frc::err::Error, "[AdvantageKit] {}", "Failed to open console file \"" + GetFilePath() + "\", disabling console capture.");
            return;
        }

        std::string buffer;
        while (!stop_) {
            char nextChar;
            while (reader.get(nextChar)) {
                buffer.push_back(nextChar);
            }

            if (!reader.eof() && reader.fail()) {
                FRC_ReportError(frc::err::Error, "[AdvantageKit] {}", "Failed to open console file \"" + GetFilePath() + "\", disabling console capture.");
                return;
            }
            reader.clear();

            std::size_t newlinePos = 0;
            while ((newlinePos = buffer.find('\n')) != std::string::npos) {
                {
                    std::string line = buffer.substr(0, newlinePos);
                    std::scoped_lock lock(mutex_);
                    lines_.push_back(std::move(line));
                }
                buffer.erase(0, newlinePos + 1);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
} // namespace akit
