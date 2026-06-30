cmake_minimum_required(VERSION 3.22)
project(rises_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionStatus.msg"
  "msg/geofencing/AreaState.msg"
  "msg/geofencing/Contours.msg"
  "msg/geofencing/Obstacle.msg"
  "msg/geofencing/LineSegment.msg"
  "msg/geofencing/ObstacleArray.msg"
  "msg/geofencing/ObstacleUpdate.msg"
  "msg/geofencing/ObstacleUpdateArray.msg"
  "msg/geofencing/ObstacleReport.msg"
  "srv/geofencing/ValidatePath.srv"
  "srv/geofencing/SetAreaState.srv"
  "srv/geofencing/UpdateMap.srv"
  "srv/geofencing/UpdateWarehouseLayout.srv"
  "srv/geofencing/SetSafetyCircleRadius.srv"
  "srv/geofencing/GetAreaState.srv"
  "srv/geofencing/GetSafetyRadius.srv"
  "srv/geofencing/GetMapInfo.srv"
  "srv/geofencing/GetWarehouseContours.srv"
  "srv/geofencing/SetGeofenceEnabled.srv"
  "action/geofencing/ValidatePath.action"
  "action/geofencing/UpdateMap.action"
  "action/geofencing/SetAreaState.action"
  "action/geofencing/SetSafetyRadius.action"
  "action/geofencing/UpdateWarehouseLayout.action"
  "action/geofencing/GetAreaState.action"
  "action/geofencing/GetSafetyRadius.action"
  "action/geofencing/GetMapInfo.action"
  "action/geofencing/SetGeofenceEnabled.action"
  DEPENDENCIES
    geometry_msgs
    action_msgs
    nav_msgs
    std_msgs
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
