# Cross-domain coordination so the robot (LOCAL, Domain 38) and the twin
# (TWINNED, Domain 1) move through the Explore -> Act -> Done phases together.
#
# Either side may trigger a phase change (whichever finishes exploring / acting
# first). The side that triggers the Act phase merges its own detected zones with
# the peer's last-reported zones and publishes the agreed plan (plan = true). On a
# simultaneous trigger, LOCAL's plan wins. Once agreed, each side actions the same
# set of points independently.

uint8 PHASE_EXPLORE = 0
uint8 PHASE_ACT = 1
uint8 PHASE_DONE = 2

std_msgs/Header header
string source                 # "LOCAL" or "TWINNED"
uint8 phase                   # PHASE_EXPLORE / PHASE_ACT / PHASE_DONE
geometry_msgs/Point[] zones   # action-zone centroids, map frame
float64[] strengths           # per-zone strength (same length as zones)
bool plan                     # true => 'zones' is the agreed plan to action
