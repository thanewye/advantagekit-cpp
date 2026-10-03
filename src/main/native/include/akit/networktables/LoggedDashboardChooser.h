#pragma once

#include "akit/networktables/LoggedNetworkChooser.h"

namespace akit::networktables {
    template<typename V> using LoggedDashboardChooser [[deprecated("Use LoggedNetworkChooser")]] = LoggedNetworkChooser<V>;
} // namespace akit::networktables
