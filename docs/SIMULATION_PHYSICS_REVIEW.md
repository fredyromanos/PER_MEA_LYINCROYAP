# Simulation Physics Model — Critical Review and Upgrade Plan

This document records a detailed engineering review of the AutoBoat simulation's physical model. It assesses whether the current 3-DOF dynamics approach is appropriate for predicting whether the real boat can sail and manoeuvre, identifies which parts are solid and which parts are too naive, and lays out a staged implementation plan. The overall conclusion is that the architecture direction is correct but several of the force laws inside it are too simplistic to trust for anything beyond a navigation animation.

## The Central Distinction

The most important point established in this review is that the three-degree-of-freedom structure itself is not naive, but some of the force laws proposed inside it are. The transition from a kinematic model where turn rate equals rudder angle multiplied by a constant and speed is a fixed value, toward a dynamic model that integrates surge velocity, sway velocity, and yaw rate through forces and moments into accelerations and finally into position and heading, is the correct conceptual upgrade. A physically interpretable simulation is far more valuable than a navigation animation.

## What Is Solid

The proposal correctly identifies the essential ingredients of a sailing dynamics model. Apparent wind must be computed as the vector difference between true wind and boat velocity. The model must include sail aerodynamic forces, hull resistance, keel lateral force, rudder force, and yaw moments. Added mass should be represented, and the empirical coefficients should be explicitly uncertain rather than presented as exact. All of these elements together produce a physically interpretable simulation rather than a purely kinematic illustration.

## Where the Proposal Should Be Challenged

### The Lift Curve Is Too Simplistic

The biggest issue is the proposed relationship CL equals CLα times sin of alpha times cos of alpha. This thin-airfoil linear lift curve is only valid over a narrow operating range. A real sail is not a thin symmetric airfoil operating at arbitrary angle of attack. It exhibits stall, its lift depends on the sign of the angle of attack, it is heavily influenced by sail trim, it behaves differently upwind and downwind, and it produces significant drag at high angles. A deliberately bounded coefficient model is required, with lift and drag as functions of angle of attack that saturate and stall, rather than a single linear expression that is assumed to remain valid everywhere.

### The Keel Equation Is Also Too Simplistic

The keel lateral force expressed as one half times density times velocity squared times keel area times lift coefficient is structurally correct, but the lift coefficient should not be trusted as a linear function of leeway angle over large angles. It needs saturation and stall behavior as well. More importantly, the keel does not merely resist leeway. Its lateral force is precisely what allows the boat to generate a hydrodynamic reaction to the sail's lateral aerodynamic force. That coupling is central to whether the boat can actually sail a course, and treating the keel as a simple resisting element obscures this.

### The Hull Drag Model Is Suspiciously Crude

The quadratic drag formulation with a wave resistance term is a useful first approximation, but for a roughly one meter displacement sailboat, resistance is not necessarily well represented by a single quadratic coefficient over the entire speed range. A better approach is to expose a general hull resistance function and initially implement a simple empirical curve such as a quadratic plus linear term, or eventually a lookup table once measurements exist, so the architecture can absorb measured data without structural change.

### The Missing Element Is Sail Trim

The current model describes sail force as a function of apparent wind alone, but in reality sail force depends on both apparent wind and sail angle, and sail angle matters enormously. If the navigation system includes an auto-trim mechanism, the simulator must model that relationship, otherwise the model collapses into a magical optimal sail force hidden behind an assumption. Sail angle should be an explicit state or input feeding the aerodynamic coefficient computation along with apparent wind.

### The Biggest Missing Physics Is Heel

The model is restricted to a horizontal two-dimensional plane. This is acceptable only if the model's purpose is clearly stated. For a sailing drone, heel changes effective sail geometry, aerodynamic force, center of effort, keel effectiveness, rudder immersion and effectiveness, righting moment, and potentially steering behavior. The resulting model should therefore be described honestly as a three-degree-of-freedom horizontal-plane sailing dynamics model, not a full sailboat model.

## Validation Philosophy

The instruction that the existing six-scenario plus adversarial baseline must still converge is dangerous, because the old simulator may have been wrong. If the old simulation says a scenario converges but the physically better model says the boat cannot maintain course, forcing the new model to reproduce the old result means regression-testing the old assumption rather than the navigation system. Two separate tests are needed instead. A navigation regression test asks whether the controller behaves correctly under the same physical conditions. A physics validity test asks whether the simulated boat behaves plausibly under known physical conditions. These are different tests and must not be conflated.

## Recommended Simulator Structure

The simulator should be structured so that wind feeds into an apparent wind computation, which along with sail trim and wind angle feeds a sail aerodynamic model producing lift and drag coefficients, which produce the sail force. That sail force, together with hull resistance, keel force, and rudder force, feeds a force and moment summation, which drives the three-degree-of-freedom dynamics to produce surge, sway, and yaw accelerations and then velocities, which finally drive the kinematics to produce position and heading. This is a reasonable architecture.

## Incremental Implementation in Four Stages

The implementation should proceed through four stages rather than a single big-bang replacement. Stage zero keeps the current kinematic model frozen as the navigation baseline. Stage one implements surge physics only, adding sail thrust and hull resistance so that surge velocity changes dynamically, which immediately eliminates the hard-coded speed setter and can be tested with wind equals zero causing the boat to stop and wind greater than zero causing acceleration toward a steady speed when sail force equals drag. Stage two adds yaw physics, introducing rudder force, rudder moment, and sail yaw moment so that yaw rate becomes physical, without yet adding complex sway. Stage three introduces sway and keel, adding leeway and keel lateral force, which is where the model becomes a genuine sailing dynamics model rather than a powered boat with a sail-shaped force generator. Stage four adds heel angle only after the horizontal model behaves properly, coupling heel to sail, keel, and rudder effectiveness.

## The Uncertainty Hierarchy

Effort should not be spent on temperature-dependent air and water density at this stage. Scientifically it is true that density varies with temperature, but for this application it is almost certainly not where the dominant uncertainty lies. If the sail coefficient is wrong by twenty to fifty percent, a few percent change in air density will not save the model. The uncertainty hierarchy ranks sail coefficients first, followed by sail trim model, keel coefficients, hull resistance, rudder coefficients, added mass, and finally water and air density last. Constant values of 1.225 kilograms per cubic meter for air and 1025 for water are entirely adequate initially. The model should not be complicated before the dominant uncertainties are identified.

## Bottom Line

The proposed architecture is not naive, but the claim should be modified from a real sailboat dynamics model whose equations are textbook aerodynamics and hydrodynamics to a three-degree-of-freedom reduced-order sailing dynamics model based on standard rigid-body, aerodynamic, and hydrodynamic force relationships with explicitly uncertain empirical coefficients. The simulator should not be required to pass the old key performance indicators. It should pass physics sanity checks first, and only then should the navigation controller be evaluated. For a roughly one meter autonomous boat, getting the force, trim, and keel relationships approximately right and measurable is more valuable than adding more equations. A simple model with measured coefficients is considerably more valuable than a sophisticated model full of guessed coefficients.

## Appendix — Apparent Wind and Leeway Salvaged From the June 2026 Model

The September 2026 refactor of the digital twin deliberately simplified the physics. It replaced the true apparent-wind vector with a scalar relative wind and it removed the leeway term entirely. That was a sound choice to establish a lean kinematic baseline, and the GPS noise that used to live inside the physics step moved into the boat's GPS model where it became more faithful. But the discarded terms are exactly the building blocks that Stage two and Stage three of this plan will re-introduce, so their structure is recorded here so it is not lost. The original source was the June 2026 copy of sim_environment.cpp inside the original firmware folder, which the benchmark repository no longer carries.

The apparent wind was computed as the true vector difference between the wind and the boat's velocity. The wind direction is stored in the from convention, so the flow direction is the stored direction plus one hundred and eighty degrees. The east and north components of the true wind and of the boat velocity are computed, the boat velocity is subtracted from the wind flow to obtain the apparent flow, and the magnitude and direction of that flow feed the relative-wind angle used by the polar and the aileron-coherence logic.

```
const double trueWindFlowDeg = normalizeAngle(windDirection + 180.0);
const double trueWindEast  = windSpeed * std::sin(trueWindFlowDeg * M_PI / 180.0);
const double trueWindNorth = windSpeed * std::cos(trueWindFlowDeg * M_PI / 180.0);
const double boatEast  = speed * std::sin(heading * M_PI / 180.0);
const double boatNorth = speed * std::cos(heading * M_PI / 180.0);

const double apparentFlowEast  = trueWindEast  - boatEast;
const double apparentFlowNorth = trueWindNorth - boatNorth;
const double apparentWindSpeed = std::hypot(apparentFlowEast, apparentFlowNorth);
const double apparentFlowDeg   = normalizeAngle(std::atan2(apparentFlowEast, apparentFlowNorth) * 180.0 / M_PI);
const double apparentWindFromDeg = normalizeAngle(apparentFlowDeg + 180.0);
const double relativeWind = normalizeRelativeAngle(apparentWindFromDeg - heading);
```

The leeway term was a lateral drift velocity perpendicular to the heading, driven by the cross-wind component of the apparent flow. A unit vector pointing to the right of the heading is dotted with the apparent flow to obtain the cross-wind speed, which is multiplied by a leeway gain and clamped to a maximum. The resulting lateral drift is added to the forward velocity to form the true velocity over ground, which is what actually integrates the position.

```
const double headingRad = heading * M_PI / 180.0;
const double rightEast  = std::cos(headingRad);
const double rightNorth = -std::sin(headingRad);
const double crossWindMps = apparentFlowEast * rightEast + apparentFlowNorth * rightNorth;
const double leewaySpeedMps = clamp(crossWindMps * LEEWAY_GAIN, -MAX_LEEWAY_SPEED_MPS, MAX_LEEWAY_SPEED_MPS);

const double velocityEast  = speed * std::sin(headingRad) + leewaySpeedMps * rightEast;
const double velocityNorth = speed * std::cos(headingRad) + leewaySpeedMps * rightNorth;
```

The speed, rudder, and yaw dynamics used first-order time-constant smoothing rather than a fixed acceleration, with separate acceleration and deceleration constants for speed, a rudder constant for the mechanical linkage, and a rotation constant for the yaw rate. The mechanical rudder linkage placed the rudder at minus half the relative wind with the servo offset added on top, so the two compensations cancel and only the commanded correction turns the boat.

```
constexpr double MAX_BOAT_SPEED_MPS       = 2.5;
constexpr double ACCEL_TIME_CONSTANT_S    = 2.0;
constexpr double DECEL_TIME_CONSTANT_S    = 6.0;
constexpr double RUDDER_TIME_CONSTANT_S   = 0.7;
constexpr double ROTATION_TIME_CONSTANT_S = 1.0;
constexpr double TURN_RATE_GAIN           = 0.30;
constexpr double LEEWAY_GAIN              = 0.040;
constexpr double MAX_LEEWAY_SPEED_MPS     = 0.35;

double responseFactor(double dtS, double tauS) {
    if (tauS <= 0.0) return 1.0;
    return 1.0 - std::exp(-dtS / tauS);
}

const double mechanicalRudderDeg = -navigationRelativeWind / 2.0;
const double targetPhysicalRudderDeg = mechanicalRudderDeg + rudderOffset;
filteredRudder += (targetPhysicalRudderDeg - filteredRudder) * responseFactor(dtS, RUDDER_TIME_CONSTANT_S);

const double steeringEff = clamp(speed / 0.75, 0.22, 1.0);
const double commandedTurnRate = -filteredRudder * TURN_RATE_GAIN * steeringEff;
yawRate += (commandedTurnRate - yawRate) * responseFactor(dtS, ROTATION_TIME_CONSTANT_S);
heading = normalizeAngle(heading + yawRate * dtS);
```

These coefficients were illustrative, not measured, but several map directly onto the items the adversarial campaign flagged as unmeasured. The minus-half-relative-wind rudder compensation is the relWind-over-two magic number whose correct value depends on the real sail and rudder geometry. The leeway gain and the turn-rate gain are the empirical coefficients that Stage two and Stage three must eventually replace with measured values. When those stages are implemented, this appendix is the reference for the structure that was discarded and must be restored in proper force-and-moment form.
