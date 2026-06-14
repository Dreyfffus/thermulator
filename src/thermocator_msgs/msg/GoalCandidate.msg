# A navigation goal proposed by a decision node, tagged with its origin and a
# comparable quality score so the goal_arbiter can choose the best one across
# domains. Published on /thermocator/goals.

geometry_msgs/PoseStamped pose   # goal pose, always expressed in the map frame
string source                    # "LOCAL" (Domain 38) or "TWINNED" (Domain 1)
float64 score                    # higher is better (corridor gain / zone strength)
