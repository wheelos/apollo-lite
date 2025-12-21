## nano_radar

该驱动基于Apollo cyber开发，支持纳雷科技MR76毫米波雷达。

### 配置

radar的默认配置: [conf/nano_radar__front_conf.pb.txt]
radar启动时，会先根据上述配置文件，向can卡发送指令，对radar进行配置。当接收到的radar状态信息与用户配置信息一致时，才开始解析数据并发送消息。

#### 多雷达配置

为多个雷达配置不同的id时，由于消息id会因为sensor_id而不同，而初始状态所有雷达id均为1,若同时进行配置会将所有雷达配置成同一个id，没有意义，因此需要逐个配置，即先接1个，配置好之后断开再接下一个

##### 通过nanoradar驱动进行配置

配置时：

1. 将配置文件（如： `conf/nano_radar__front_conf.pb.txt`）文件中的sensor_id修改为对应的雷达id，并且设备`sensor_id_valid`为`true`
2. 然后使用启动雷达（如： `mainboard -d modules/drivers/radar/nano_radar_dag/nano_radar.dag`）
3. 通过`candump can1`（此处can1为例子，请替换为实际的can设备），确定接收到已修改后的雷达报文（如`sensor_id`为`2`，则会收到`0x221`的报文 ，若`sensor_id`为1，则会收到`0x211`的报文）
4. 若3确认无误后，关闭驱动模块（ctrl-c或者kill对应的进程id），并且修改配置文件中的`sensor_id_valid`为`false`
5. 重启雷达驱动，确认雷达配置成功

##### 通过whl-nanoradar-conftool进行配置

参考 https://github.com/wheelos-tools/whl-nanoradar-conftool

### lanuch

dag_files: "/apollo/modules/drivers/radar/nano_radar/dag/nano_radar.dag"

### Topic

**topic name**: /apollo/sensor/nanoradar/front
**data type**: apollo::drivers::NanoRadar
**channel ID**: CHANNEL_ID_ONE
