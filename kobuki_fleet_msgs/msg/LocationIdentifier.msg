# Location types
uint8    SC    = 1  # Source
uint8    MA    = 2  # Machine
uint8    CB    = 3  # Conveyor belt
uint8    WP    = 4  # Way Point (additional navigation goal)
uint8    PP    = 5  # Precision Placement platform
uint8    ROBOT = 6

# The type of location
std_msgs/UInt64 type

# The instance ID of the location (e.g. if there are several shelves)
std_msgs/UInt64  instance_id

# A human-readable description of the location
std_msgs/String  description

uint8    A    = 1  # Material type A
uint8    B    = 2  # Material type B

# A human-readable description of the material type contained in the localization
std_msgs/Int16  materialType

# Pose with position and orientation
geometry_msgs/Pose pose
