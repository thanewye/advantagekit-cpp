#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <wpi/tunables/Selectable.hpp>

namespace akit::networktables::detail {
    struct SelectableBaseAccess : wpi::tunables::detail::SelectableBase {
        static constexpr auto kOptions = &SelectableBaseAccess::m_options;
        static constexpr auto kDefaultChoice = &SelectableBaseAccess::m_defaultChoice;
        static constexpr auto kSelected = &SelectableBaseAccess::m_selected;
    };

    template<typename V> struct IsSharedPtr : std::false_type {};
    template<typename U> struct IsSharedPtr<std::shared_ptr<U>> : std::true_type {};

    /**
     * Reads the options of a wpi::tunables::Selectable in insertion order. Selectable keeps its values private, so
     * each value is read back through GetSelected() by temporarily selecting its option.
     */
    template<typename V> std::vector<std::pair<std::string, V>> SelectableOptions(wpi::tunables::Selectable<V>& selectable) {
        wpi::tunables::detail::SelectableBase& base = selectable;
        const std::string originalSelection = base.*SelectableBaseAccess::kSelected;
        std::vector<std::pair<std::string, V>> options;
        for (const std::string& option : base.*SelectableBaseAccess::kOptions) {
            base.*SelectableBaseAccess::kSelected = option;
            if constexpr (IsSharedPtr<V>::value) {
                options.emplace_back(option, selectable.GetSelected().lock());
            } else {
                options.emplace_back(option, selectable.GetSelected());
            }
        }
        base.*SelectableBaseAccess::kSelected = originalSelection;
        return options;
    }

    /** Returns the default option name of a wpi::tunables::Selectable, or empty if none is set. */
    inline const std::string& SelectableDefault(const wpi::tunables::detail::SelectableBase& selectable) {
        return selectable.*SelectableBaseAccess::kDefaultChoice;
    }
} // namespace akit::networktables::detail
