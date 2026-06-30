# Existing types
uint8 RECTANGLE = 1
uint8 POLYGON = 2
uint8 LINE = 3         # Straight line segment (2 points)
uint8 POINT = 4        # Single isolated point (for small obstacles or noise)
uint8 CONVEX_POLYGON = 5  # Explicit convex hull polygon
uint8 FREEFORM = 6     # Arbitrary complex shape
uint8 CIRCLE = 7

# Obstacle fields
uint8 type
uint64 id

geometry_msgs/Point position  # centroid or reference point

# Optional depending on type
float32 radius        # for CIRCLE, or approximate for ELLIPSE (major axis)
# width: type-dependent semantics --
#   RECTANGLE, TRIANGLE: the shape's width dimension.
#   LINE (as emitted by ObstacleReportBuilder): the transverse extent of the
#     point group the segment was fit from -- i.e. the group's max
#     perpendicular spread off its own first->last chord, a "how round/thick
#     is this blob" proxy (used by rises_leg_filter to tell a leg-shaped blob
#     from a flat wall fragment). NOT the segment's length -- that is
#     recoverable from vertices[0]/vertices[1]. All LINE segments emitted
#     from the same point group share this same group-level value.
float32 width
float32 height        # for RECTANGLE, TRIANGLE
float32 orientation   # rotation for RECTANGLE/ELLIPSE/LINE

geometry_msgs/Point[] vertices  # for POLYGON, CONVEX_POLYGON, FREEFORM
float32 minor_axis    # optional for ELLIPSE
