#include <ruckig/block.hpp>
#include <ruckig/velocity.hpp>


namespace ruckig {

VelocityStep1::VelocityStep1(double v0, double a0, double vf, double af, double aMax, double aMin, double jMax): a0(a0), af(af), _aMax(aMax), _aMin(aMin), _jMax(jMax) {
    vd = vf - v0;
}

void VelocityStep1::time_acc0(ProfileIter& profile, double aMax, double aMin, double jMax, bool return_after_found) const {
    // UD
    {
        profile->t[0] = (-a0 + aMax)/jMax;
        profile->t[1] = (a0*a0 + af*af - 2*aMax*aMax + 2*jMax*vd)/(2*aMax*jMax);
        profile->t[2] = (-af + aMax)/jMax;
        profile->t[3] = 0;
        profile->t[4] = 0;
        profile->t[5] = 0;
        profile->t[6] = 0;

        if (profile->check_for_velocity<JerkSigns::UDDU, Limits::ACC0>(jMax, aMax, aMin)) {
            add_profile(profile);
            if (return_after_found) {
                return;
            }
        }
    }

    // UU
    {
        profile->t[0] = (-a0 + aMax)/jMax;
        profile->t[1] = (a0*a0 - af*af + 2*jMax*vd)/(2*aMax*jMax);
        profile->t[2] = 0;
        profile->t[3] = 0;
        profile->t[4] = (af - aMax)/jMax;
        profile->t[5] = 0;
        profile->t[6] = 0;

        if (profile->check_for_velocity<JerkSigns::UDUD, Limits::ACC0>(jMax, aMax, aMin)) {
            add_profile(profile);
            if (return_after_found) {
                return;
            }
        }
    }
}

void VelocityStep1::time_none(ProfileIter& profile, double aMax, double aMin, double jMax, bool return_after_found) const {
    const double h1 = std::sqrt((a0*a0 + af*af)/2 + jMax*vd);

    // Solution 1
    {
        profile->t[0] = -(a0 + h1)/jMax;
        profile->t[1] = 0;
        profile->t[2] = -(af + h1)/jMax;
        profile->t[3] = 0;
        profile->t[4] = 0;
        profile->t[5] = 0;
        profile->t[6] = 0;

        if (profile->check_for_velocity<JerkSigns::UDDU, Limits::NONE>(jMax, aMax, aMin)) {
            add_profile(profile);
            if (return_after_found) {
                return;
            }
        }
    }

    // Solution 2
    {
        profile->t[0] = (-a0 + h1)/jMax;
        profile->t[1] = 0;
        profile->t[2] = (-af + h1)/jMax;
        profile->t[3] = 0;
        profile->t[4] = 0;
        profile->t[5] = 0;
        profile->t[6] = 0;

        if (profile->check_for_velocity<JerkSigns::UDDU, Limits::NONE>(jMax, aMax, aMin)) {
            add_profile(profile);
            if (return_after_found) {
                return;
            }
        }
    }
}

bool VelocityStep1::get_profile(const Profile& input, Block& block) {
    const ProfileIter start = valid_profiles.begin();

    ProfileIter profile = start;
    profile->set_boundary(input);

    if (std::abs(af) < DBL_EPSILON) {
        // There is no blocked interval when af==0, so return after first found profile
        const double aMax = (vd >= 0) ? _aMax : _aMin;
        const double aMin = (vd >= 0) ? _aMin : _aMax;
        const double jMax = (vd >= 0) ? _jMax : -_jMax;

        time_none(profile, aMax, aMin, jMax, true);
        if (profile > start) { goto return_block; }
        time_acc0(profile, aMax, aMin, jMax, true);
        if (profile > start) { goto return_block; }

        time_none(profile, aMin, aMax, -jMax, true);
        if (profile > start) { goto return_block; }
        time_acc0(profile, aMin, aMax, -jMax, true);

    } else {
        time_none(profile, _aMax, _aMin, _jMax, false);
        time_none(profile, _aMin, _aMax, -_jMax, false);
        time_acc0(profile, _aMax, _aMin, _jMax, false);
        time_acc0(profile, _aMin, _aMax, -_jMax, false);
    }

return_block:
    return Block::calculate_block(block, valid_profiles, std::distance(start, profile));
}

} // namespace ruckig
