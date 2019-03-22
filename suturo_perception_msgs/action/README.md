# Suturo Perception Actions

There are currently two actions defined for the Suturo perception module. Each of them creates a specific [RoboSherlock](https://github.com/Suturo1819/robosherlock) pipeline. Action servers as well as a launch file is provided by the [hsr_perception package](https://github.com/Suturo1819/hsr_perception).

## PerceiveTable
This action starts a [RoboSherlock](https://github.com/Suturo1819/robosherlock) pipeline that is optimized to process the view observed from a table.

* **goal:** 
  * `visualization (bool)` Decide whether the [RoboSherlock](https://github.com/Suturo1819/robosherlock) visualization should  be displayed or not.
* **result:**
  * `detection_data (Array made from [ObjectDetectionData](https://github.com/Suturo1819/suturo_msgs/blob/master/suturo_perception_msgs/msg/ObjectDetectionData.msg)` One instance of `[ObjectDetectionData](https://github.com/Suturo1819/suturo_msgs/blob/master/suturo_perception_msgs/msg/ObjectDetectionData.msg)` for each observed and recognized object in the scene. 
* **feedback:**
  * `feedback (string)` A string containing information about whether the communcation was successfull or not.

## PerceiveShelf
This action starts a [RoboSherlock](https://github.com/Suturo1819/robosherlock) pipeline that is optimized to process the view observed from a shelf.

* **goal:** 
  * `visualization (bool)` Decide whether the [RoboSherlock](https://github.com/Suturo1819/robosherlock) visualization should  be displayed or not.
* **result:**
  * `detection_data (Array made from [ObjectDetectionData](https://github.com/Suturo1819/suturo_msgs/blob/master/suturo_perception_msgs/msg/ObjectDetectionData.msg)` One instance of `[ObjectDetectionData](https://github.com/Suturo1819/suturo_msgs/blob/master/suturo_perception_msgs/msg/ObjectDetectionData.msg)` for each observed and recognized object in the scene. 
  * `inliers_visible (bool)` True, if [RoboSherlock](https://github.com/Suturo1819/robosherlock) was able to detect the inliers of the shelf
* **feedback:**
  * `feedback (string)` A string containing information about whether the communcation was successfull or not.
