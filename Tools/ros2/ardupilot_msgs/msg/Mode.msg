# Experimental REP147 mode interface.
#
# https://ros.org/reps/rep-0147.html#flight-modes
#
# Differences to the REP147 proposal:
#   - Name message and field Mode rather than FlightMode as it applies to
#     all vehicles, not just aerial ones.
#   - Do not supply enums for the mode. These may be specific to a flight stack
#     and vehicle.

# The value of the current drive/flight mode.
uint8 mode
