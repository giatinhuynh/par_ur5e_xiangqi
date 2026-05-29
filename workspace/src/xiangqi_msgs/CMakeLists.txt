cmake_minimum_required(VERSION 3.8)
project(xiangqi_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoardState.msg"
  "msg/PieceDetection.msg"
  "msg/GameStatus.msg"
  "msg/MoveHistory.msg"
  "msg/EngineInfo.msg"
  "msg/AiMoveCommand.msg"
  "msg/AiCommandAck.msg"
  "msg/AiExecutionResult.msg"
  "srv/GetBoardState.srv"
  "srv/GetBoardTransform.srv"
  "srv/GetBestMove.srv"
  "srv/GripperControl.srv"
  "srv/SetEngine.srv"
  "action/ExecuteMove.action"
  "action/PickAndPlace.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
