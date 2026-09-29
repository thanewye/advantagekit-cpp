#include "akit/networktables/detail/SendableChooserAccess.h"

namespace akit::networktables::detail {
    namespace {
        template<typename Tag, typename Tag::type Member> struct PrivateMemberAccessor {
            friend typename Tag::type GetPrivateMember(Tag) { return Member; }
        };

        template<typename V> struct ChoicesMemberTag {
            using type = wpi::StringMap<V> frc::SendableChooser<V>::*;
            friend type GetPrivateMember(ChoicesMemberTag);
        };

        struct DefaultChoiceMemberTag {
            using type = std::string frc::SendableChooserBase::*;
            friend type GetPrivateMember(DefaultChoiceMemberTag);
        };

        template struct PrivateMemberAccessor<ChoicesMemberTag<int>, &frc::SendableChooser<int>::m_choices>;
        template struct PrivateMemberAccessor<DefaultChoiceMemberTag, &frc::SendableChooserBase::m_defaultChoice>;
    } // namespace

    template<typename V> const wpi::StringMap<V>& StolenChoices(const frc::SendableChooser<V>& chooser) {
        return chooser.*GetPrivateMember(ChoicesMemberTag<V>{});
    }

    const std::string& StolenDefaultChoice(const frc::SendableChooserBase& chooser) {
        return chooser.*GetPrivateMember(DefaultChoiceMemberTag{});
    }

    template const wpi::StringMap<int>& StolenChoices<int>(const frc::SendableChooser<int>&);
} // namespace akit::networktables::detail
