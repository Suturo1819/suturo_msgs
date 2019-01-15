# Communicated over topic /suturo_perception/object_detection
string name
geometry_msgs/PoseStamped pose
uint32 shape
float64 width
float64 height
float64 depth

# Shape values
uint32 MISC=0
uint32 BOX=1
uint32 CYLINDER=2
uint32 SPHERE=3
uint32 CONE=4
uint32 PLANE=5
# Extend with other shapes/annotators