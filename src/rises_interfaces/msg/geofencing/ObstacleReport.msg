# Combined obstacle report from geofence node
# Contains both matched (known) and unmatched (unknown) obstacles
# with their segments already computed for direct VDA5050 state publishing

std_msgs/Header header

# Matched (known) obstacles - segments that correspond to map obstacles
Obstacle[] matched_obstacles

# Unmatched (unknown) obstacles - segments that do not match any map obstacle.
# Each carries a persistent tracker id (Obstacle.id) that is stable across scans
# for a stationary feature; the validation node uses id recurrence plus
# ground-truth spawns to distinguish a true static miss from a moving/stationary
# intruder (Static Structure Match Rate KPI).
Obstacle[] unmatched_obstacles
