# 외부(컨트롤러 등)로 내보내는 현재 자세.
#
# 규약
#   위치      m
#   자세각    deg
#   좌표계    차체 FRD (x 전, y 우, z 하), 글로벌 = 차체 시작포즈.
#             output_enu:=true 로 실행하면 다른 출력들과 함께 ENU/FLU 로 뒤집힌다
#   회전순서  ZYX (yaw -> pitch -> roll), 항공 관례
#
# /fused_odom 과 같은 시점(매 IMU 틱)에 같은 포즈로 발행된다.
# 쿼터니언이 필요하면 /fused_odom 을 쓸 것.

std_msgs/Header header

float64 x
float64 y
float64 z

float64 roll
float64 pitch
float64 yaw
