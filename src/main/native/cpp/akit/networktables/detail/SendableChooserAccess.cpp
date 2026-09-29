#include "akit/networktables/detail/SendableChooserAccess.h"

namespace akit::networktables::detail {
    template<typename V> const wpi::StringMap<V>& StolenChoices(const frc::SendableChooser<V>& chooser) {
        return chooser.m_choices;
    }

    const std::string& StolenDefaultChoice(const frc::SendableChooserBase& chooser) {
        return chooser.m_defaultChoice;
    }

    template const wpi::StringMap<int>& StolenChoices<int>(const frc::SendableChooser<int>&);
} // namespace akit::networktables::detail
