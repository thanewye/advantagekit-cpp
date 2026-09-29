#pragma once

#include <string>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SendableChooserBase.h>
#include <wpi/StringMap.h>

/**
 * Reaches into frc::SendableChooser's private members and steals the data to allow
 * LoggedDashboardChooser's constructor to directly accept an
 * frc::SendableChooser. Each V must be explicitly instantiated there once.
 */
namespace akit::networktables::detail {
    /**
     * Returns a specified chooser's options, keyed by name
     *
     * @tparam V The value type associated with each option.
     * @param chooser The chooser to read options from.
     * @return The chooser's options map.
     */
    template<typename V> const wpi::StringMap<V>& StolenChoices(const frc::SendableChooser<V>& chooser);

    /**
     * Returns a specified chooser's default option key
     *
     * @param chooser The chooser to read the default option from.
     * @return The default option key, or empty if none is set.
     */
    const std::string& StolenDefaultChoice(const frc::SendableChooserBase& chooser);
} // namespace akit::networktables::detail
