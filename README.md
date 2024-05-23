



## setup
``` bash
catkin_make
source devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

## run package
``` bash
rosrun INS_Integration_MORAI gpsimu
```

`Setting yaw`를 `90 - yaw`로 설정해주세요 
아래 사진은 하나의 예시입니다
![Screenshot from 2024-05-23 17-44-49](https://github.com/mun9769/catkin_ws_1/assets/59304977/3c1c4f52-0fe7-4e10-94b9-d92845584fd5)



``` bash
roslaunch Lidar_MCduo_2023 Lidar_MCduo_2023_launch.launch
```

모라이에서 /lidar3D의 프로토콜을 UDP로 설정해주세요 
![Screenshot from 2024-05-23 17-47-28](https://github.com/mun9769/catkin_ws_1/assets/59304977/1eeeb491-edd6-4a74-b784-dcb62064829a)


다음과 같은 창이 나온다면 성공입니다.


<img src="https://github.com/mun9769/catkin_ws_1/assets/59304977/f2c8fcd0-ec38-4b72-8013-d6ecdd90719b" width=220>
