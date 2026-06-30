# Mission status reported by the mission controller

# Status constants
uint8 STATUS_PENDING=0
uint8 STATUS_ACTIVE=1
uint8 STATUS_SUCCEEDED=2
uint8 STATUS_FAILED=3
uint8 STATUS_CANCELLED=4

# Header with timestamp
std_msgs/Header header

# Mission identifier
string mission_id

# Mission type (e.g., SAFE_PALLET_DELIVERY, UPDATE_GEOFENCE_MAP)
string mission_type

# Current status
uint8 status

# Current task being executed (empty if not active)
string current_task

# Total tasks in this mission
uint8 total_tasks

# Tasks completed so far
uint8 completed_tasks

# Human-readable status message
string message
