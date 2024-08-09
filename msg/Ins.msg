# 时间戳s
float64 timestamp

float64 latitude

float64 longitude

# 海拔高
float64 altitude

#椭球高 m
float64 ellip_height

# 东向速度，单位m/s
float64 east_speed

# 北向速度，单位m/s
float64 north_speed

# 天向速度，单位m/s
float64 sky_speed

# 水平地面速度，单位m/s
float64 ground_speed

# 航向 0 - 2pi 单位rad
float64 yaw

# 俯仰，单位 rad
float64 pitch

# 横滚，单位 rad
float64 roll

# X轴加速度，单位m/s²
float64 acc_x

# Y轴加速度，单位m/s²
float64 acc_y

# Z轴加速度，单位m/s²
float64 acc_z

# X轴角速度，rad/s
float64 angular_x

# Y轴角速度，单位 rad/s
float64 angular_y

#Z轴角速度，顺时针为正，单位rad/s
float64 angular_z

#星数
uint8 satelliteNum

#卫星状态
uint8 satelliteState

#导航信息不确定度
uint8 navUncertainty
