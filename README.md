# 说明

此工程为解析大陆ARS408毫米波雷达ros驱动，用于前后左右安装四个方向大陆ARS408或SRR308毫米波雷达，前，后毫米波雷达连接can0，分别配置id0和id1，左，右毫米波雷达连接can1，分别配置id0和id1，实际使用时按需修改！！

# Radar 准备工作

```bash
sudo apt-get install can-utils
candump can0 // 观察can总线是否收到毫米波雷达数据
```

配置工作模式为cluster或object模式
```bash
cansend can0 200#F8000000089C0000 // Objects detection with all extended properties
cansend can0 200#F8000000109C0000 // Clusters detection with all extended properties
```
配置id
```bash
cansend can0 200#8200000001800000 // Objects id 1
cansend can0 200#8200000002800000 // Objects id 2
cansend can0 200#8200000003800000 // Objects id 3
cansend can0 200#8200000004800000 // Objects id 4
cansend can0 200#8200000005800000 // Objects id 5
cansend can0 200#8200000006800000 // Objects id 6
cansend can0 200#8200000007800000 // Objects id 7
```

其它配置，根据协议确定
```bash
cansend can0 202#8C0000012C // Maximum distance of objects detected 30 meters
cansend can0 202#AE06800FFF // Minimum value of object RCS -10 dBm2
cansend can0 202#C600030007 // Minimum value of objects probability of existence 75%
```
# 使用方法
```bash
source devel/setup.bash
roslaunch radar_ros run.launch calib_radar:=true //if need calib Radar, both cluster and object are ok
```
# 致谢
部分源码改编自 https://github.com/Project-MANAS/ars_40X 工程
