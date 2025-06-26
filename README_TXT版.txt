# auto_calibration

## 怎么用

其实这一版没啥好说的了，有快捷方式直接一个个点就行，有一些注意事项提一下。

## 注意

### 本项目要放在哪？

默认放在主目录下

### 运行快捷方式

在`run_command`文件夹中，存放着所有的快捷方式

对于ubuntu18.04,GNOME版本3.28.2来说，直接在本文件夹点击即可运行。

对于ubuntu18.04,GNOME版本3.36+（也就是目前的Geo_scan）来说。需要将`run_command`里的全部文件复制到桌面，注意这里不是直接到桌面粘贴，而是到文件管理器的桌面文件夹粘贴。粘贴后到桌面去右键点击每一个新加的图标，选择允许启动。此时双击即可运行程序。

### 关于打开相机

现在默认是打开左相机，室内曝光，不需要再去关掉右边相机的rviz界面

### 关于1.3_手动删除不需要的图片

这个按了之后会自动跳到图片文件夹，去删掉不需要的图片就行了，终端等5秒自己会关

### 关于1.5_更新相机内参.desktop

> 因为程序运行很快，终端会一闪而过，这个是正常的，如果有报错，或者程序没有正常运行，去`scripts`文件夹运行以下命令即可看到终端输出

```bash
python3 1.5_process_intrinsics.py calib_result.txt
```
> [!NOTE]
>
> **注意**：新版的程序增加了等待用户输入才停止的判断，终端不会一闪而过了，终端输出的命令大家可以核对一下，比如和rosrun.txt的是否一致，和calib_result.txt的原始数据数是否一致

### 关于录制三次雷达和相机rosbag

程序是自动录三秒，自动停止，用户只需要按一次enter，等提示，然后转角度再按enter，按三次enter就行了

> 在`1.4_标定鱼眼相机`之后会输出相机内参

 **注意**：新版的程序直接输出到/scripts/calib_result.txt，不需要再复制了，不放心的话可以用以下命令跳转检查一下

 cd /home/handsfree/auto-calibration/scripts
 gedit calib_result.txt

### 关于矩阵转换网站

用程序代替了，但是还是把网址放在[这里](https://staff.aist.go.jp/k.koide/workspace/matrix_converter/matrix_converter.html)

### 关于扫把（清理终端）

每一个脚本都会跳转路径和source，所以脚本运行完，都是可以按扫把清理掉终端的。（第一个脚本录相机rosbag除外，那个要ctrl+c）



