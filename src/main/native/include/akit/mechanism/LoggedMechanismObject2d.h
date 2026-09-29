#pragma once

#include <concepts>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <frc/Errors.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/NetworkTable.h>
#include <wpi/mutex.h>

#include "akit/log/LogTable.h"
#include "akit/util/LinkedHashMap.h"

namespace akit::mechanism {
    /** Common base class for all LoggedMechanism2d node types, mirroring frc::MechanismObject2d. */
    class LoggedMechanismObject2d {
        friend class LoggedMechanism2d;

    protected:
        explicit LoggedMechanismObject2d(std::string_view name);

        virtual void UpdateEntries(std::shared_ptr<nt::NetworkTable> table) = 0;

        virtual void LogEntries(const LogTable& table) const = 0;

        mutable wpi::mutex mutex_;

    public:
        virtual ~LoggedMechanismObject2d() = default;

        const std::string& GetName() const;

        template<typename T, typename... Args> requires std::convertible_to<T*, LoggedMechanismObject2d*>
        T* Append(std::string_view name, Args&&... args) {
            std::scoped_lock lock(mutex_);
            const std::string key(name);
            if (objects_.contains(key)) {
                throw FRC_MakeError(frc::err::Error, "MechanismObject names must be unique! `{}` was inserted twice!", name);
            }
            auto object = std::make_unique<T>(name, std::forward<Args>(args)...);
            T* appended = object.get();
            objects_.put(key, std::move(object));
            if (table_) appended->Update(table_->GetSubTable(name));
            return appended;
        }

        /** Converts this node's children into poses, depth first, starting from the given seed. */
        std::vector<frc::Pose3d> Generate3dMechanism(const frc::Pose3d& seed) const;

        /** Distance in meters from this node's pivot to the pivot of its children. */
        virtual double GetObject2dRange() = 0;

        /** Angle in degrees relative to the parent node. */
        virtual double GetAngle() = 0;

    private:
        std::string name_;
        util::LinkedHashMap<std::string, std::unique_ptr<LoggedMechanismObject2d>> objects_;
        std::shared_ptr<nt::NetworkTable> table_;
        void Update(std::shared_ptr<nt::NetworkTable> table);
        void LogOutput(const LogTable& table) const;
    };
} // namespace akit::mechanism
