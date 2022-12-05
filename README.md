# Robomaster Helios 2022
本仓库为西南交通大学Helios战队2022视觉组所有成员所建，继承了2021赛季代码，用于管理用于Robomaster赛场上哨兵、英雄、步兵的视觉代码。
***
## 硬件列表及文件树
- 硬件列表

|  硬件   | 型号  |
|  ----  | ----  |
| 视觉开发板  | NVIDIA XAVIER NX |
| 视觉开发板  | Intel NUC i5|
| 摄像头  | MV-SUA133GC-T |
| 摄像头  | MV-SUA134GC-T |
| 摄像头  | INTEL D435 |
| 摄像头  | KS1A522 |

## 必要的库
Eigen >= 3.3.9
Ceres-Solver

- 二级文件树列表

<pre><font color="#3465A4"><b>.</b></font>
├── <font color="#4E9A06"><b>CMakeLists.txt</font>
├── <font color="#4E9A06"><b>config.h</b></font>
├── <font color="#4E9A06"><b>config.h.in</b></font>
├── <font color="#4E9A06"><b>main.cpp</b></font>
├── <font color="#4E9A06"><b>RMSelfStart.sh</b></font>
├── <font color="#4E9A06"><b>README.md</b></font>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Detector</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">Armor</font></span>
│   ├── <font color="#3465A4"><b>Energy</b></font>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">resource</font></span>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Drivers</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">DAHUA</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">MindVision</font></span>
│   ├── <font color="#4E9A06"><b>Driver.h</b></font>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">RealSense</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">V4L2KAS</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">VideoDriver</font></span>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Math</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">Filter</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">Kalman</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">RMKF</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">SolveAngle</font></span>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Other</font></span>
│   ├── <font color="#4E9A06"><b>include</b></font>
│   └── <font color="#4E9A06"><b>src</b></font>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Predictor</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">EnergyPredictor.cpp</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">Predictor.cpp</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">Predictor.h</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">Predictor.yaml</font></span>
├── <span style="background-color:#4E9A06"><font color="#3465A4">Serials</font></span>
│   ├── <span style="background-color:#4E9A06"><font color="#3465A4">SerialPort.hpp</font></span>
│   └── <span style="background-color:#4E9A06"><font color="#3465A4">SerialPort.cpp</font></span>
└── <span style="background-color:#4E9A06"><font color="#3465A4">Thread</font></span>
    ├── <span style="background-color:#4E9A06"><font color="#3465A4">Fifo.h</font></span>
    ├── <span style="background-color:#4E9A06"><font color="#3465A4">MyThread.cpp</font></span>
    └── <span style="background-color:#4E9A06"><font color="#3465A4">MyThread.hpp</font></span>
</pre>

## 代码效果展示
忘记留下照片记录了，以后逛去实验室的时候有空补上:)

## 运行
- 运行：参考上交2018代码写法，通过运行可执行程序时输入参数，控制代码的执行，相比较于预编译的写法而言，在使用上更加方便，也更易于调试。可输入的参数包括：
	- 兵种（-hero -infantry -sentry -UAV -video），其中-video表示使用视频运行代码
	- 目标颜色(-blue -red)
	- 调试选项(-origin -boxes -lamp 等等)
	- 相机编号(-cameraIndex = )
	- 打开视频路径(-path = )，
	具体参数可在运行时添加 -help 参数查看。 

## 相机驱动
- 统一的相机驱动接口：我们使用了包括支持UVC协议的USB摄像头，Intel D435双目摄像头，大华A5131CU210工业摄像头在内的多种摄像头，这在一定程度上造成了我们在不同机器人上往往需要采取不同的摄像头驱动，为代码的移植更新带来了不便，为此我们提供统一的接口driver(虚函数)来根据机器人的种类选择不同的驱动，一定程度上减小了多种摄像头带来的困扰(主要是21赛季的困扰)，22赛季有钱了基本都用的迈德威视的相机。

## 装甲版自瞄
- **说明：装甲板识别中有两套识的cpp分别是ArmorDetector.cpp和armor_detetctor.cpp，后者是后来重写的也就是新的，两个都能用大体处理逻辑没变，只有细微差别，两者共用同一个头文件ArmorDetector.hpp**

- 特点：用传统算法实现了同一图像上的多目标同时识别及最优击打目标决策，识别耗时3ms左右（nuc i5）

- 弊端：由于使用的传统算法，复杂环境的鲁棒性不高；识别需要较低的相机曝光；


### 识别
- 图像预处理方案: 利用灰度图二值化后的图像信息寻找符合条件信息，轮廓提取采用的fitEllipse椭圆拟合，灯条筛选过程主要通过：**灯条点集大小、灯条倾斜角、灯条宽度、灯条长宽比**

- 灯条颜色区分：**通过灯条目标区域红色通道减去蓝色通道的值的平均值，来判断灯条的颜色**

- 打分函数：利用反指数函数的归一化特性，给5个灯条辨识参数分配不同的权重便可以人为的实现分数在某个分数上占比的侧重
```c++
	score = exp(-k_[0]*norm_angle_err) + exp(-k_[1]*norm_height_err) +
                exp(-k_[2]*norm_incline_err) + exp(-k_[3]*norm_wh_ratio) +
                exp(-k_[4]*norm_bright);
        score /= 5; // normalize
```

- 装甲板匹配策略：在装甲板匹配中采取优先级策略和二次筛选策略，每次匹配结束获得单个灯条的最优匹配子集后将子集最优与包含该灯条的历史最优结果作比较，这样可以有效排除同一灯条被两个匹配对象共有的问题，具体请看armor_detector.cpp

- 数字识别：先对于装甲板选定区域进行透视变换修正倾斜变形，在此基础上以前采用的SVM方式进行数字识别@LeonXu127，后来参考自华南师大chenjun用MLP进行数字识别并由本队大二chenzheng同学完成，效果上看后者更好，两者均在NumberClassifier.cpp中

### 预测
- 坐标解算：传统4点PnP解算。为了获得目标相对与陀螺仪坐标下的世界坐标，需要进行若干步的坐标变换，目标是使最终建立坐标系处于云台旋转中心且不会随车体旋转而变化，平移参数可以在机械图上获得并根据实际情况稍微修正，具体请移步至SolveAngle.cpp

- 弹道模型：只考虑了横向二阶空气阻力，忽略弹丸下坠过程的空气阻力，具体请看SolveAngle.cpp中的iteratePitch函数

- 目标预测：Kalman滤波，加速度模型。关于测量噪声协方差矩阵和过程噪声协方差矩阵的调整，@LeonXu127曾做过仿真用于分析其对预测系统跟踪性和稳定性的影响，具体请看RMKF

## 打符
### 识别
- 识别大体是基于华北理工20赛季开源改的，识别所需要修改的参数全部被存在.yaml的配置文件中，运行时初始化时读取该数值可避免反复修改反复编译。

### 预测
- 利用OpenCV的getTickCount()和getTickFrequency()记录程序内部走时，该时间戳在采图一并与图像打包进结构体作为该图像是时间信息。

- 速度滤波采用逐差法+Kalman的方式，值得注意的是逐差法所求的速度对应的是逐差的n张图片的时间中值。

- 在滤波速度的基础上把速度与时间用Ceres-Solver进行曲线拟合，求解角速度，幅值与相位。值得注意的是为了提高求解的精确性需要使相位初值尽可能接近，这里我们通过一次波峰或波谷的检测来计算待拟合的初始相位。

- 切换扇叶的问题要么直接丢弃差角大数据，要么记录历史角度数据反推切换后的扇叶位置相对于切换前在什么位置，这里我们采用的是后者，具体请看EnergyPredictor.cpp

***
这readme写的不大好，还请多多见谅，欢迎各路大佬指正


