bool jamming_signal     # 0 = 주행 시작후 재밍 전까지, 1 = 재밍 구간, 2 = 재밍 구간 빠져 나온 다음 
bool tunnel_out_signal  # 0 = 기본 신호, 1 = 재밍구간을 빠져 나온 직후 제어 측이 주행할 수 있도록 두번째 setting_yaw를 구하는 동안 보냄

float32 heading	# Yaw angle relative to true north. (degree)

float64 latitude	# INS solution position in geodetic latitude (degree)
float64 longitude	# INS solution position in geodetic longitude (degree)
float64 altitude	# Height above ellipsoid. (WGS84) (meter)