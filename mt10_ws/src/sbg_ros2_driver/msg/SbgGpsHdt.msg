# SBG Ellipse Messages
std_msgs/Header header

# Time since sensor is powered up [us]
uint32 time_stamp

# GPS True Heading status.
# Bit 0-5: enum:
# 0 SOL_COMPUTED		A valid solution has been computed.
# 1 INSUFFICIENT_OBS	Not enough valid SV to compute a solution.
# 2 INTERNAL_ERROR		An internal error has occurred.
# 3 HEIGHT_LIMIT		The height limit has been exceeded.
# Bit 6: mask:
# 1 BASELINE_VALID      The baseline length field is filled and valid.
uint16 status

# GPS Time of Week [ms]
uint32 tow

# True heading angle (0 to 360 deg)
# NED convention: Rotation about the down axis. Zero when the X axis is pointing North.
# ENU convention: Rotation about the up axis. Zero when the X axis is pointing East. (opposite sign compared to NED)
float32 true_heading

# 1 sigma True heading estimated accuracy
float32 true_heading_acc

# Pitch
# NED convention:
#   angle from the master to the rover
# ENU convention:
#   angle from the rover to the master
float32 pitch

# 1 sigma pitch estimated accuracy
float32 pitch_acc

# The distance between the main and aux antenna in meters.
float32 baseline

# Number of space vehicle tracked in true heading solution. 0xFF if N/A
uint8 num_sv_tracked

# Number of used space vehicle in true heading solution. 0xFF if N/A
uint8 num_sv_used