int64 id            # Unique ID of the area
uint8 operation     # Lock or unlock operation
float64 x0          # AABB min x
float64 y0          # AABB min y
float64 x1          # AABB max x
float64 y1          # AABB max y

# Define constants for operation
uint8 LOCK = 0
uint8 UNLOCK = 1
