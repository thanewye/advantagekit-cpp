#include "pch.h"

#include "akit/mechanism/LoggedMechanism2d.h"

#include <array>
#include <span>

#include <networktables/NTSendableBuilder.h>

namespace akit::mechanism {
    static constexpr std::string_view kBackgroundColor = "backgroundColor";
    static constexpr std::string_view kDims = "dims";

    LoggedMechanism2d::LoggedMechanism2d(double width, double height, const frc::Color8Bit& backgroundColor)
        : width_{width}
        , height_{height} {
        SetBackgroundColor(backgroundColor);
    }

    LoggedMechanismRoot2d* LoggedMechanism2d::GetRoot(std::string_view name, double x, double y) {
        std::scoped_lock lock(mutex_);
        const std::string key(name);
        if (auto* existing = roots_.get(key)) return existing->get();

        auto root = std::make_unique<LoggedMechanismRoot2d>(name, x, y, LoggedMechanismRoot2d::private_init{});
        LoggedMechanismRoot2d* added = root.get();
        roots_.put(key, std::move(root));
        if (table_) added->Update(table_->GetSubTable(name));
        return added;
    }

    void LoggedMechanism2d::SetBackgroundColor(const frc::Color8Bit& color) {
        std::scoped_lock lock(mutex_);
        color_ = color.HexString();
        if (colorPub_) colorPub_.Set(color_);
    }

    void LoggedMechanism2d::InitSendable(nt::NTSendableBuilder& builder) {
        builder.SetSmartDashboardType("Mechanism2d");

        std::scoped_lock lock(mutex_);
        table_ = builder.GetTable();
        dimsPub_ = table_->GetDoubleArrayTopic(kDims).Publish();
        dimsPub_.Set({{width_, height_}});
        colorPub_ = table_->GetStringTopic(kBackgroundColor).Publish();
        colorPub_.Set(color_);
        for (auto& [name, root] : roots_) {
            root->Update(table_->GetSubTable(name));
        }
    }

    void LoggedMechanism2d::LogOutput(const LogTable& table) const {
        std::scoped_lock lock(mutex_);
        const std::array<double, 2> dims{width_, height_};
        table.Put(".type", "Mechanism2d");
        table.Put(".controllable", false);
        table.Put(std::string(kDims), std::span<const double>(dims));
        table.Put(std::string(kBackgroundColor), std::string_view{color_});
        for (const auto& [name, root] : roots_) {
            root->LogOutput(table.GetSubtable(name));
        }
    }

    std::vector<frc::Pose3d> LoggedMechanism2d::Generate3dMechanism() const {
        std::scoped_lock lock(mutex_);
        std::vector<frc::Pose3d> poses;
        for (const auto& [name, root] : roots_) {
            const auto rootPoses = root->Generate3dMechanism();
            poses.insert(poses.end(), rootPoses.begin(), rootPoses.end());
        }
        return poses;
    }
} // namespace akit::mechanism
