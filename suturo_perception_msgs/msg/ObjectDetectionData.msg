# Communicated over topic /suturo_perception/object_detection

# always give at least the shape, such that

string name
string obj_class # the class from the list of classes below
float64 confidence # with its confidence
uint32 shape # and the shape (please, if possible, as fallback if confidence is low). 
geometry_msgs/PoseStamped pose
float64 width
float64 height
float64 depth
std_msgs/ColorRGBA color

# Shape values
uint32 MISC=0
uint32 BOX=1
uint32 CYLINDER=2
uint32 SPHERE=3
uint32 CONE=4
uint32 PLANE=5

# classes:
# - apple
# - ball
# - banana
# - bell_pepper
# - binder
# - bowl
# - calculator
# - camera
# - cap
# - cell_phone
# - cereal_box
# - coffee_mug
# - comb
# - flashlight
# - food_bag
# - food_box
# - food_can
# - food_cup
# - food_jar
# - garlic
# - glue_stick
# - greens
# - hand_towel
# - instant_noodles
# - keyboard
# - kleenex
# - lemon
# - lightbulb
# - lime
# - marker
# - mushroom
# - notebook
# - onion
# - orange
# - peach
# - pear
# - pitcher
# - plate
# - pliers
# - potato
# - rubber_eraser
# - scissors
# - soda_can
# - shampoo
# - sponge
# - stapler
# - tomato
# - toothbrush
# - toothpaste
# - water_bottle