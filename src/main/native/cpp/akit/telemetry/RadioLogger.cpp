#include "pch.h"

#include "akit/telemetry/RadioLogger.h"

#include <algorithm>
#include <charconv>
#include <memory>
#include <mutex>
#include <string>

#include <wpi/framework/RobotBase.hpp>
#include <wpi/net/HttpUtil.hpp>
#include <wpi/net/TCPConnector.h>
#include <wpi/system/Notifier.hpp>
#include <wpi/units/time.hpp>
#include <wpi/util/Logger.hpp>

namespace akit {
    std::mutex RadioLogger::mutex_;
    bool RadioLogger::isConnected_ = false;
    std::string RadioLogger::statusJson_;
    std::string RadioLogger::statusURL_;
    const std::regex RadioLogger::kWhitespacePattern{"\\s+"};
    std::unique_ptr<wpi::Notifier> RadioLogger::notifier_;

    void RadioLogger::Start(const int64_t teamNumber) {
        statusURL_ = "http://10." + std::to_string(teamNumber / 100) + "." + std::to_string(teamNumber % 100) + ".1/status";
        bool error = false;
        std::string errorMsg;
        wpi::net::HttpLocation location{statusURL_, &error, &errorMsg};
        if (error) return;
        notifier_ = std::make_unique<wpi::Notifier>([location] {
            std::string response = [&] {
                wpi::util::Logger logger;
                auto stream = wpi::net::TCPConnector::connect(location.host.c_str(), location.port, logger, kConnectTimeoutSecs);
                if (!stream) {
                    return std::string{};
                }

                wpi::net::HttpConnection connection{std::move(stream), kReadTimeoutSecs};
                wpi::net::HttpRequest request{location};
                std::string warnMsg;
                if (!connection.Handshake(request, &warnMsg)) {
                    return std::string{};
                }
                std::string body;
                if (!connection.contentLength.empty()) {
                    size_t contentLen = 0;
                    const auto parseResult =
                        std::from_chars(connection.contentLength.data(), connection.contentLength.data() + connection.contentLength.size(), contentLen);
                    if (parseResult.ec != std::errc{}) {
                        return std::string{};
                    }
                    connection.is.readinto(body, contentLen);
                } else {
                    wpi::util::SmallString<256> lineBuffer;
                    while (true) {
                        const std::string_view line = connection.is.getline(lineBuffer, 4096);
                        if (!line.empty()) {
                            body.append(line);
                        }
                        if (connection.is.has_error()) {
                            connection.is.clear_error();
                            break;
                        }
                    }
                }
                return body;
            }();

            std::string responseStr = std::regex_replace(response, kWhitespacePattern, "");

            std::scoped_lock lock{mutex_};
            isConnected_ = !responseStr.empty();
            statusJson_ = responseStr;
        });
        notifier_->SetName("AdvantageKit_RadioLogger");
        notifier_->StartPeriodic(wpi::units::second_t{kRequestPeriodSecs});
    }

    void RadioLogger::Stop() {
        notifier_.reset();
    }

    void RadioLogger::Periodic(LogTable table, const int64_t teamNumber) {
        if (notifier_ == nullptr && wpi::RobotBase::IsReal() && teamNumber >= 0) {
            Start(teamNumber);
        }
        std::scoped_lock lock(mutex_);
        table.Put("Connected", isConnected_);
        table.Put("Status", LogValue{statusJson_, "json"});
    }
} // namespace akit
