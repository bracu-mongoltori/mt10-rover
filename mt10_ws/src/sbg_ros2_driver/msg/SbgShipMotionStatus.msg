# SBG Ellipse Messages
# SbgShipMotionStatus

# True after heave convergence time.
# False in following conditions:
# - Turn occurred and no velocity aiding is available
# - Heave reached higher/lower limits
# - If a step is detected and filter has to re-converge
# - If internal failure
bool heave_valid

# True if heave output is compensated for transient accelerations
bool heave_vel_aided

# True if surge and sway channels are provided in this output.
bool surge_sway_included

# True if the swell period is provided in this output.
bool period_available

# True if the period returned is assumed to be valid or not.
bool period_valid

# True if the real time heave filter is using the swell mode computations.
bool swell_mode