# Status feedback of a relocalization attempt triggered via /initialpose.
# status values:
#  REQUEST_ACCEPTED  - request dispatched to the pose graph
#  SUCCESS           - scan matching against the map succeeded
#  FAILED            - no candidate submap matched, initialpose was not applied
#  FALLBACK_INITIAL_POSE - matching failed and the requested initial pose was
#                      applied directly as the new robot pose
std_msgs/Header header

string status
# Match score of the best relocalization constraint (0.0 when failed).
float64 score
# Index of the relocalization request this feedback refers to.
int32 epoch
