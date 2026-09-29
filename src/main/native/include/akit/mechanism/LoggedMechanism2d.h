#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <frc/util/Color8Bit.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NTSendable.h>
#include <networktables/NetworkTable.h>
#include <networktables/StringTopic.h>
#include <wpi/mutex.h>
#include <wpi/sendable/SendableHelper.h>

#include "akit/log/LogTable.h"
#include "akit/mechanism/LoggedMechanismRoot2d.h"
#include "akit/util/LinkedHashMap.h"

namespace akit::mechanism {
    /** frc::Mechanism2d that can also be recorded with Logger::RecordOutput. */
    class LoggedMechanism2d : public nt::NTSendable, public wpi::SendableHelper<LoggedMechanism2d> {
    public:
        LoggedMechanism2d(double width, double height, const frc::Color8Bit& backgroundColor = {0, 0, 32});

        LoggedMechanismRoot2d* GetRoot(std::string_view name, double x, double y);

        void SetBackgroundColor(const frc::Color8Bit& color);

        void InitSendable(nt::NTSendableBuilder& builder) override;

        /** Records the current mechanism to the log. Called by Logger::RecordOutput, not user code. */
        void LogOutput(const LogTable& table) const;

        /** Converts the mechanism into poses for 3D components, roots in insertion order, each depth first. */
        std::vector<frc::Pose3d> Generate3dMechanism() const;

    private:
        double width_;
        double height_;
        std::string color_;
        mutable wpi::mutex mutex_;
        std::shared_ptr<nt::NetworkTable> table_;
        util::LinkedHashMap<std::string, std::unique_ptr<LoggedMechanismRoot2d>> roots_;
        nt::DoubleArrayPublisher dimsPub_;
        nt::StringPublisher colorPub_;
    };
} // namespace akit::mechanism
