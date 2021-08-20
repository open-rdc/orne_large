# orne_large_arduino

## Overview
Intelligent Manipulatorにおいて，台車のコントローラをアナログ入力で制御するArduinoのソフトウェアです．

## Installation
[IntelligentManipulatorの環境構築](https://github.com/citbrains/IntelligentManipulator/wiki/IntelligentManipulator-%E7%92%B0%E5%A2%83%E6%A7%8B%E7%AF%89)に含まれています．

### rosserial_arduinoの準備
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Install_ros_lib_into_the_Arduino_Environment を参照してください．

### Arduinoへの書き込み
orne_large_arduinoが対象とするArduinoはArduino MEGA 2560です．
Arduino IDE等で適切なボード・デバイスファイルを設定してビルド・書き込みを行ってください．

## Usage
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```
デバイスファイルは環境によって変化する可能性があります．適宜設定してください．
また，デバイスファイルが/dev/ttyACM0ではない場合は権限の設定が適切に行われていない可能性があります．適切な権限を設定して実行してください．

## ROS API
### Subscribed topics
Arduinoがrosserialを介して購読トピックです.

※`~`の部分は`/serial_node`などに置き換えてください．  
※`roslaunch orne_large_driver driver_with_arduino.launch`で起動した場合は，`/orne_large_arduino`になります．  

* `~cmd_vel` (std_msgs/Float32MultiArray)  
  * オリエンタルモータのコントローラに速度指令を与え，制御するためのトピックです．
  * データは２要素です
    *  `[左車輪の回転速度(rad/s), 右車輪の速度(rad/s)]`  

### Published topics
Arduinoがrosserialを介して配信するトピックです

* `~joint_state` (sensor_msgs/JointState)
  * 各関節の状態を配信するトピックです．
  * 角度のデータは2要素です．
  * データ構造は次のとおりです
    * `[右車輪, 左車輪]`

### Parameters

* `~port`
  * デバイス名　（例：/dev/ttyACM0）

* `~baud`
  * ビットレート　（例：115200）

## ソースコードに関するドキュメント
このファームウェアは[Doxygen](http://www.doxygen.jp/)の記法に基づいてコメントが記述されています．Doxygenをインストールし，`doxygen`コマンドを実行するとソースコードのドキュメントが生成されます．

