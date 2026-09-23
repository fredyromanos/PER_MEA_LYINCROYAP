#include "ManualController.h"
#include "../config/BoardConfig.h"
#include "../config/Calibration.h"
#include "../config/DebugConfig.h"

void ManualController::reset() {
    if (sailStateInitialized_) {
        DBG_CTRL("reset: sail→+1 (un-init)");
    }
    sailState_            = +1;
    sailStateInitialized_ = false;
}

uint16_t ManualController::clamp(uint16_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

uint16_t ManualController::mapUs(uint16_t in, uint16_t inLo, uint16_t inHi,
                                  uint16_t outLo, uint16_t outHi) {
    in = clamp(in, inLo, inHi);
    return static_cast<uint16_t>(outLo +
           static_cast<uint32_t>(in - inLo) * (outHi - outLo) / (inHi - inLo));
}

// Mode manuel unifié : voile (CH2) + safran (CH4) + propulseur (CH3 inverse).
ActuatorCommand ManualController::computeManual(const RcFrame& frame) {
    ActuatorCommand cmd;
    const uint16_t db = BoardConfig::RC_DEADBAND_US;

    // --- Voile : bascule binaire ±10° pilotée par CH2 ---
    if (!sailStateInitialized_) {
        const int8_t init = (frame.ch2 >= BoardConfig::RC_MID_US) ? +1 : -1;
        DBG_CTRL("sail state init: %+d (CH2=%u)", (int)init, (unsigned)frame.ch2);
        sailState_            = init;
        sailStateInitialized_ = true;
    }
    const int8_t prevSail = sailState_;
    if      (frame.ch2 > static_cast<uint16_t>(BoardConfig::RC_MID_US + db)) sailState_ = +1;
    else if (frame.ch2 < static_cast<uint16_t>(BoardConfig::RC_MID_US - db)) sailState_ = -1;

    if (sailState_ != prevSail) {
        DBG_CTRL("sail toggle: %+d → %+d  (CH2=%u)",
            (int)prevSail, (int)sailState_, (unsigned)frame.ch2);
    }
    cmd.sailUs = (sailState_ > 0) ? Calibration::SAIL_PLUS_US : Calibration::SAIL_MINUS_US;

    // --- Safran / REGATTA ECO II : CH4 → position cible (winch multi-tours) ---
    // Zone morte au centre → centre mécanique exact ; sinon mappage linéaire.
    const uint16_t ch4 = frame.ch4;
    if (ch4 >= static_cast<uint16_t>(BoardConfig::RC_MID_US - db) &&
        ch4 <= static_cast<uint16_t>(BoardConfig::RC_MID_US + db)) {
        cmd.rotorUs = Calibration::ROTOR_CENTER_US;
    } else {
        cmd.rotorUs = mapUs(ch4,
                            BoardConfig::CH4_MIN_US, BoardConfig::CH4_MAX_US,
                            Calibration::ROTOR_MIN_US, Calibration::ROTOR_MAX_US);
    }

    // --- Propulseur : CH3 BIDIRECTIONNEL (marche avant / arrière) ---
    // Manette à crans (ne se recentre pas) :
    //   CH3 → CH3_FULL_US (1100) : +100 % AVANT   (ESC_MAX_US, 2000 µs)
    //   CH3 → centre             :    0 % ARRÊT    (ESC_NEUTRAL_US, 1500 µs)
    //   CH3 → CH3_ZERO_US (1990) : -100 % ARRIÈRE  (ESC_REVERSE_MIN_US, 1000 µs)
    // Zone morte au centre → neutre franc. Sécurité : CH3 perdu (0) → neutre.
    // NB : la marche arrière n'agit que si l'ESC est programmé en mode
    // bidirectionnel (carte EPRG-3) ; sinon < 1500 µs reste à l'arrêt.
    if (frame.ch3 == 0) {
        cmd.esc1Us = Calibration::ESC_NEUTRAL_US;
    } else {
        const uint16_t ch3    = clamp(frame.ch3,
                                      BoardConfig::CH3_FULL_US, BoardConfig::CH3_ZERO_US);
        const int      center = static_cast<int>(BoardConfig::CH3_CENTER_US);
        const int      diff   = static_cast<int>(ch3) - center;  // <0 = avant, >0 = arrière

        if (diff >= -static_cast<int>(Calibration::ESC_CENTER_DEADBAND_US) &&
            diff <=  static_cast<int>(Calibration::ESC_CENTER_DEADBAND_US)) {
            cmd.esc1Us = Calibration::ESC_NEUTRAL_US;
        } else if (diff < 0) {
            // Côté avant : du centre (0 %) vers CH3_FULL_US (+100 %).
            float frac = static_cast<float>(center - ch3) /
                         static_cast<float>(center - BoardConfig::CH3_FULL_US);
            if (frac > 1.0f) frac = 1.0f;
            cmd.esc1Us = static_cast<uint16_t>(
                Calibration::ESC_NEUTRAL_US +
                frac * (Calibration::ESC_MAX_US - Calibration::ESC_NEUTRAL_US));
        } else {
            // Côté arrière : du centre (0 %) vers CH3_ZERO_US (-100 %).
            float frac = static_cast<float>(ch3 - center) /
                         static_cast<float>(BoardConfig::CH3_ZERO_US - center);
            if (frac > 1.0f) frac = 1.0f;
            cmd.esc1Us = static_cast<uint16_t>(
                Calibration::ESC_NEUTRAL_US -
                frac * (Calibration::ESC_NEUTRAL_US - Calibration::ESC_REVERSE_MIN_US));
        }
    }

    return cmd;
}

ActuatorCommand ManualController::update(const RcFrame& frame) {
    // Voies servo perdues → neutre sécurisé (et ré-init de la voile à la reprise).
    if (frame.ch2 == 0 || frame.ch4 == 0) {
        reset();
        return ActuatorCommand{};
    }
    return computeManual(frame);
}
