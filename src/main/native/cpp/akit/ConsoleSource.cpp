#include "pch.h"

#include "akit/ConsoleSource.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include <cerrno>

#ifndef _WIN32
#include <csignal>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

#ifdef __APPLE__
#include <crt_externs.h>
#endif

#include <wpi/system/Errors.hpp>

#ifndef _WIN32
#ifndef __APPLE__
extern char** environ;
#endif

namespace {
    char** ProcessEnvironment() {
#ifdef __APPLE__
        return *_NSGetEnviron();
#else
        return environ;
#endif
    }
} // namespace
#endif

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

    ConsoleSource::Systemcore::Systemcore() {
        thread_ = std::thread([this] { Run(); });
    }

    ConsoleSource::Systemcore::~Systemcore() {
        stop_ = true;
#ifndef _WIN32
        if (const pid_t childProcessId = childProcessId_.load(); childProcessId > 0) kill(childProcessId, SIGTERM);
#endif
        if (thread_.joinable()) thread_.join();
    }

    std::string ConsoleSource::Systemcore::GetNewData() {
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

    void ConsoleSource::Systemcore::Run() {
#ifdef _WIN32
        WPILIB_ReportError(wpi::err::Error, "[AdvantageKit] {}", "Systemcore console capture is not supported on Windows, disabling console capture.");
#else
        static constexpr const char* kJournalCommand =
            "journalctl -f -u robot.service -n all -o cat _SYSTEMD_INVOCATION_ID=$(systemctl show -p InvocationID --value robot.service)";

        int pipeFileDescriptors[2];
        if (pipe(pipeFileDescriptors) != 0) {
            WPILIB_ReportError(wpi::err::Error, "[AdvantageKit] {}", "Failed to launch console capture process, disabling.");
            return;
        }
        const int readFileDescriptor = pipeFileDescriptors[0];
        const int writeFileDescriptor = pipeFileDescriptors[1];

        posix_spawn_file_actions_t fileActions;
        posix_spawn_file_actions_init(&fileActions);
        posix_spawn_file_actions_adddup2(&fileActions, writeFileDescriptor, STDOUT_FILENO);
        posix_spawn_file_actions_addclose(&fileActions, readFileDescriptor);
        posix_spawn_file_actions_addclose(&fileActions, writeFileDescriptor);

        char* const arguments[] = {const_cast<char*>("/bin/bash"), const_cast<char*>("-c"), const_cast<char*>(kJournalCommand), nullptr};
        pid_t childProcessId = 0;
        const int spawnResult = posix_spawn(&childProcessId, "/bin/bash", &fileActions, nullptr, arguments, ProcessEnvironment());
        posix_spawn_file_actions_destroy(&fileActions);
        close(writeFileDescriptor);

        if (spawnResult != 0) {
            close(readFileDescriptor);
            WPILIB_ReportError(wpi::err::Error, "[AdvantageKit] {}", "Failed to launch console capture process, disabling.");
            return;
        }
        childProcessId_ = childProcessId;
        if (stop_) kill(childProcessId, SIGTERM);

        std::string buffer;
        char chunk[4096];
        while (true) {
            const ssize_t bytesRead = read(readFileDescriptor, chunk, sizeof(chunk));
            if (bytesRead == 0) break;
            if (bytesRead < 0) {
                if (errno == EINTR) continue;
                if (!stop_) WPILIB_ReportError(wpi::err::Error, "[AdvantageKit] {}", "Failed to read from console capture process, disabling.");
                break;
            }
            buffer.append(chunk, static_cast<std::size_t>(bytesRead));

            std::size_t newlinePos = 0;
            while ((newlinePos = buffer.find('\n')) != std::string::npos) {
                {
                    std::string line = buffer.substr(0, newlinePos);
                    std::scoped_lock lock(mutex_);
                    lines_.push_back(std::move(line));
                }
                buffer.erase(0, newlinePos + 1);
            }
        }

        close(readFileDescriptor);
        kill(childProcessId, SIGTERM);
        waitpid(childProcessId, nullptr, 0);
        childProcessId_ = 0;
#endif
    }
} // namespace akit
