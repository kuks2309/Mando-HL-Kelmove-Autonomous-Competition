# 使用串口助手快速判断 imu 模块数据输出

波特率 921600

## 在 ubuntu 上使用串口助手

以 ubuntu16.04 为例

1. 安装 cutecom（你也可以安装其他的串口助手进行调试）。

   ```
   apt-get install cutecom -y
   ```

2. 安装成功后在终端输入 cutecom，打开串口助手，然后进行一些设置，如图中所示。  

    ![serial-set](./img/raw_serial_data/ubuntu1.png)

3. 然后我们点击`open device` ，此时下面的空白面板会有 imu 的数据打印。

4. 我们可以等待 imu 的数据打印一会儿，然后点击 `close device`来查看。

5. 如果可以找到一以下的信息，那么模块发送的数据是没有问题的。

   `55 51`、 `55 52`、 `55 53`、 `55 54` 是 b 系列 imu 模块特有的数据。

   `aa5514`、`aa552c` 是 a 系列 imu 模块特有的数据。

    ![imu-b](./img/raw_serial_data/ubuntu2.png)

    ![imu-a](./img/raw_serial_data/ubuntu3.png)

6. 提示：如果你的电脑上有多个 `/dev/ttyUSB*` 设备，请确定 imu 模块对应的设备号，然后在填到cutecom的 `device:`对应的可输入下拉框。查看所有 ttyUSB* 设备可在终端输入。

   ```
   ls /dev/ttyUSB*
   ```

## 在 windows 上使用串口助手

以 win 7 为例

1. 运行包中自带sscom5.13.1.exe（你也可以使用自己的串口助手工具）。

2. 进行一些设置，如图中所示。

    ![serial-set](./img/raw_serial_data/win1.jpg)

3. 打开串口，等数据打印一会在关闭串口，查看打印的数据。

   `55 51`、 `55 52`、 `55 53`、 `55 54` 是 b 系列 imu 模块特有的数据。

   `aa5514`、`aa552c` 是 a 系列 imu 模块特有的数据。

    ![imu-b](./img/raw_serial_data/win3.png)

    ![imu-a](./img/raw_serial_data/win2.png)

   
