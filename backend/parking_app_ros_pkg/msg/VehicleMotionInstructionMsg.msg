# Vehicle information required to register to parking management system

std_msgs/Header header

int16 id    # id of the Vehicle that this message is for


# general info of the parking garage about the vehicle
geometry_msgs/Pose2D current_pose


# Instructions for full automated Vehicle
int32[] route
int32 destination_parking_space_id

uint8 status
uint8 status_transfer = 0
uint8 status_parking = 1
uint8 status_parked = 2
uint8 status_unparking = 3
uint8 status_dropoff = 4
uint8 status_pickup = 5

# Instructions for vehicle with actuators
float32 dt
MultiPathMsg planned_path # vector of poses in constant time intervals of \dt calculated from header/stamp

Pose2DStamped[] trajectory

