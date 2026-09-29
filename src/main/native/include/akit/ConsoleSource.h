#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

namespace akit {
    class ConsoleSource {
    public:
        virtual ~ConsoleSource() = default;

        [[nodiscard]] virtual std::string GetNewData() = 0;

        class Simulator;
        class RoboRIO;
    };

    class ConsoleSource::Simulator final : public ConsoleSource {
    public:
        Simulator();
        ~Simulator() override;

        [[nodiscard]] std::string GetNewData() override;

    private:
        // directs standard output to 2 streams
        class TeeStreambuf final : public std::streambuf {
        public:
            TeeStreambuf(std::streambuf* first, std::streambuf* second, std::mutex& captureMutex);

        protected:
            int overflow(int ch) override;
            std::streamsize xsputn(const char* data, std::streamsize count) override;
            int sync() override;

        private:
            std::streambuf* first_;
            std::streambuf* second_;
            std::mutex& captureMutex_;
        };

        std::streambuf* stdout_ = nullptr;
        std::streambuf* stderr_ = nullptr;
        std::stringbuf capturedStdout_;
        std::stringbuf capturedStderr_;
        std::unique_ptr<TeeStreambuf> stdoutTee_;
        std::unique_ptr<TeeStreambuf> stderrTee_;
        std::size_t stdoutPos_ = 0;
        std::size_t stderrPos_ = 0;
        std::mutex mutex_;
    };

    class ConsoleSource::RoboRIO final : public ConsoleSource {
    public:
        RoboRIO();
        ~RoboRIO() override;

        [[nodiscard]] std::string GetNewData() override;

    protected:
        [[nodiscard]] static std::string GetFilePath();

    private:
        void Run();

        std::atomic_bool stop_{false};
        std::thread thread_;
        std::mutex mutex_;
        std::vector<std::string> lines_;
    };
} // namespace akit
