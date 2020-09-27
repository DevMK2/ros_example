# 사용법
0. ros_example 디렉토리로 이동한다.
1. catkin_make
2. ~/.bashrc 파일을 열고 다음을 추가한다
source /home/mk/ws_study/ros_example/setup.bash
3. 터미널을 새로 실행한다
4. ros_example/src/gazebo/launch 로 이동한다.
5. roslaunch simulation.launch
6. 새로운 터미널을 실행한다
7. rosrun ros_msgs cli.py
8. 7번에서 연 프로그램에서 
a를 입력하면 회색 공이 빨간공에 다가와 부착되며
d를 입력하면 탈착된다.

# 패키지 설명
1. ros_example/src/gazebo
- gazebo launch파일, world파일, model파일이 있습니다.

2. ros_example/src/model_plugin
- 예제에서 빨간공에 삽입되는 MagneticSphere 플러그인이 있습니다.
- 빌드를 위해 CMakeLists.txt 를 참고하세요
- 소스코드는 include/MagneticSphere.hh 와 src/MagneticSphere.cc 를 참고하세요

3. ros_example/src/world_plugin
- World 플러그인을 만드는 예제입니다.
- 빌드를 위해 CMakeLists.txt 를 참고하세요
- 소스코드는 include/SimulationWorld.hh 와 src/SimulationWorld.cc 를 참고하세요

4. ros_example/src/ros_msgs
- 사용자 정의 ROS service 메시지를 만드는 예시입니다.
- 빌드를 위해 CMakeLists.txt 를 참고하세요.
- 메시지 정의는 srv/Attach.srv 와 srv/Detach.srv를 참고하세요.
- 예제의 rospy 프로세스는 cli.py 를 참고하세요
