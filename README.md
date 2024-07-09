간단 사용 방법

라이다와 카메라가 publish하는 데이터를 ros의 message_filter(데이터 시간 일치)를
이용하여 이미지, 포인트클라우드를 파일로 저장한다.

최대한 간단하게 동작시키기 위해, turtlesim의 turtle_teleop_key를 이용해서 방향기 ⬆️를
누르면 저장될 수 있도록 구성


#1 src의 cpp파일을 열어 저장하고자 하는 경로로 수정

#2 ros2 run turtlesim turtle_teleop_key 를 실행해서 방향키 위를 한번씩 누르면 찍힌다.
   (QoS 설정 중요!!)

#3 패키지 선택하여 빌드
colcon build --packages-select save_jpg_pcd

#2 패키지 적용
. install/setup.bash

