ArmISColorBall
======================
ArmISColorBallはカラーボールの座標値を出力するRTCです  
現在のバージョンでは赤、緑、青、桃色のカラーボールが追跡できます      
ロボットなどのビジョンセンサとして使用することを前提としています  

本RTCを用いたデモンストレーションです  
[アームロボットによるカラーボールのお片づけ][video]  
[video]: http://www.youtube.com/watch?v=Mtw2xgm07t8 
    
動作確認環境
------
Python:  
2.6.6  

OS:  
Windows 7 64bit / 32bit  
Ubuntu 10.04 LTS / 12.04 LTS 32bit  

対応RTC:  
[RsServoManager][servo]  
[ArmISIK][ik]  

[servo]: https://github.com/HiroakiMatsuda/RsServoManager
[ik]: https://github.com/HiroakiMatsuda/ArmISIK

外部依存Module:  
[OpenCV 2.4][cv]  

[cv]: http://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.0/

ファイル構成
------
ArmISColorBall  
│―ArmISColorBall.py  
│  
│―ini   
│　　│―camera.ini    
│  
│―rtc.conf  

* ArmISColorBal.py  
RTC本体です  
* camera.ini  
HSV値の閾値や、マーカー間距離などを設定します  
* rtc.conf  
ポートの設定や動作周期を設定できます  

注:本RTCにおいてユーザーが操作すると想定しているファイルのみ説明しています  

RTCの設定はiniファイルを通して行えるので、簡単に設定を変えられます  
iniファイルはActivate時に読み込むので、設定を変更した場合はDeactiveにしたあとActivateしてください

RTCの構成
------  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readme_01.png" width="500px" />    
[ArmISIK][ik], [RsServoManager][servo]などの対応するRTCを接続することでアームロボットを制御できます  

* pos port :OutPort  
データ型; TimedLongSeq  
データ長は最大5で、コマンドの種別によってはcommand以降のデータは無視してください  
常にデータ長5で通信する場合は、コマンド以降はどのように設定しても構わないよう受け側のRTCをコーディングしてください  
現在のバージョンでは、ロボットアームなどのRTCが接続せれることを前提とした規格となっています  
それ以外の使い方をする場合は、command = 1のPox_x, Pos_y, Pos_zの座標値を利用してください  

 ・  Command = 0 :   [0]  
イニシャルポジションに移動します  
  
 ・  'Command = 1 :   [1, Pos_x, Pos_y, Pos_z, Gripper]  
指定されたXYZ座標に移動します  

 ・  `'Pos_x` : ロボット座標系のX座標を指定します   
 ・  `'Pos_y` : ロボット座標系のY座標を指定します   
 ・  `'Pos_z` : ロボット座標系のZ座標を指定します   
 ・  `'Gripper` : ロボットのグリッパ開閉量を指定します   
このときに指定座標がロボットアームの範囲外であるときは、ロボットアームの関節値に0を発行します  
また設定した角度リミットに達した場合は、各関節の指令値はリミット値に設定されます  

 ・  Command = 2 :   [2]  
グリッパを閉じ、イニシャルポジションまで移動します  
把持対象物の付近までCommand = 1で接近し、Command = 2で初期値まで持ち帰るとを想定しています  

 ・  Command = 1000 :   [1000]  
全てのサーボモータへTorque ON司令を発行します  

 ・  Command = 1001 :   [1001]  
全てのサーボモータへTorque OFF司令を発行します  
その他のコマンドについては今後のバージョンで追加していきます  

座標系  
------  
本RTCの座標系は以下のようになっています  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISColorBall/readme_01.png" width="300px" /> 

ロボット座標の原点は画像の中心に設定されています  
ArmISCalorBallが出力するのはロボット座標系の座標です  
値は[pixel]から[mm]に変換されています   

カラーボール追跡アルゴリズム
------  
カラーボールの追跡アルゴリズムついて説明します  
以下の処理には[OpenCV 2.4][cv]を使用しています  

１. カメラ画像の取得  
2. カメラ画像にガウシアンフィルタをかけスムージング  
3. カメラ画像をHSV色空間に変換  
4. HSVの各スレッショルドで色抽出  
5. 色抽出した画像の重心を計算  
6. 求めた重心位置を画像ロボット座標に座標変換  

以上の処理を毎フレーム、各色行なっています  
なお、5の処理では追跡対象の色が見つからなかった場合、出力する座標を(0, 0)として円の表示は行わない  

以下の画像は左上から時計回りに、  
・カメラ画像に重心位置に円を合成した画像    
・緑を抽出した画像  
・赤を抽出した画像  
・青を抽出した画像  
となっている  
このように全色が抜き切れていなくても、ある程度重心を求められているのがわかる  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISColorBall/readme02.png" width="400px" /> 
 
使い方
------
###1. 追跡する色を設定する###
camera.iniをテキストエディタなどで開き編集します  

CALIB  
  ・ ``scale = X```  
キャリブレーション用十字円の初期半径 [pixel]を指定します  
 
  ・ ``robot = X```  
キャリブレーション用基準マーカー間距離[mm]を指定します    

  ・ ``flag = X```  
ON: キャリブレーション用十字円を表示します  
OFF:  キャリブレーション用十字円を非表示にします    

BALL    
  ・ ```size =  X```  
追跡するボールの直径[mm]を指定します  

  ・ ```grab =  X```  
ロボットがカラーボールへアプローチするときの、グリッパー開閉具合を設定します  
単位は[0.1deg]です       

RED  
 ・ ```h_low =  X```  
HSV色空間の色相の下限閾値を設定します  
色相は360 [°]を180 [°]に正規化した値を指定してください  
 ・ ```h_up =  X```  
HSV色空間の色相の上限閾値を設定します  
色相は360 [°]を180 [°]に正規化した値を指定してください  
 ・ ```s_low =  X```  
HSV色空間の彩度の下限閾値を設定します  
彩度は1~100 [%]を0~255に正規化した値を指定してください  
 ・ ```s_up =  X```  
HSV色空間の彩度の上限閾値を設定します  
彩度は1~100 [%]を0~255に正規化した値を指定してください  
 ・ ```v_low =  X```  
HSV色空間の明度の下限閾値を設定します  
明度は1~100 [%]を0~255に正規化した値を指定してください  
 ・ ```v_up =  X```  
HSV色空間の明度の上限閾値を設定します  
明度は1~100 [%]を0~255に正規化した値を指定してください  

これらの閾値は[Just Color Picker][jcp]などを使い、実際にカメラ画像からパラメータを取得する色を取得することで簡単に設定することができます  
閾値は多少の幅を持たせて設定してください  

[jcp]: http://www.download3k.com/Install-Anny-Just-Color-Picker.html

以下、各色同様に設定してください  
GREEN  
BLUE  
PINK  

なお、現バージョンでは色のラベルを増やしたり減らしたりなどできません  
ただし、色のラベルと内容が異なってもいいのであれば、どのような色を追跡しても構いません  
  
###2. サンプルシステム：カラーボールのお片づけ###
この項目ではArmISIK, [ArmISIK][ik], [RsServoManager][servo]を使用してカラーボールを片けるサンプルシステムを用いて本RTCの説明を行います    

なお、本システムで使用するロボットアーム'ArmIS type0'は双葉電子工業（株）のコマンド方式サーボモータである、RS405CBを４つ使用して制作された、全長280 [mm]の卓上小型ロボットアームです  
外観を以下に示します  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readme_03.jpg" width="300px" /> 

本システムの概要は以下のようになり、天井カメラ(USBカメラ)、ロボットアーム、トレー、カラーボールが必要です  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readme_04.png" width="400px" /> 

1. 環境を整える  
本システムの概要に従い、天井カメラ、ロボットアーム、トレー、カラーボールをセットします  

2. RsServoManger, ArmISIK, ArmISColorBallを起動する  
ロボットアームとPCの通信用シリアルケーブル、カメラを接続し、ロボットアームの電源を入れます    
その後各RTCを起動し、接続後にRsServoManger, ArmISIK, ArmISColorBallの順に起動します  
この起動手順は必ずしも守る必要はありませんが、下層のRTCより起動することで無用なトラブルを回避する狙いがあります  

3. ArmISColorBallのキャリブレーションを行う  
ArmISColorBallの下図のようにカメラ画面に表示されている、黄色い十字円の交点とArmIS type0台座のマーカーの位置を合わせます  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/raeadme_05.png" width="400px" />

 'u'ボタンを押すことで円の半径が1[pixel]ずつ広がり、'd'ボタンを押すことで円の半径が1[pixel]ずつ小さくなります  
十字円の位置と大きさを合わせることで、アームの台座の長さから1[pixel]がロボット座標系でx[mm]に相当するのかを測定します  
この簡易キャリブレーションシステムで、卓上ロボットアームを円滑に運用できます  
この時点でカラーボールやトレーが設置されていても構いませんが、邪魔な場合は一度除去し、キャリブレーション後に再度設置してください  

4. ロボットアームの初期化を行う  
ArmISColorBallのカメラ画面に入力のフォーカスを合わせ、'o'ボタンを押しTorque ON命令を発行します  
その後'i'ボタンを押し、イニシャルポジションにロボットアームを移動させます  
本システムではこのときのグリッパの下にトレーを設定します  

5. 認識しているカラーボールを拾う  
ArmISColorBallのカメラ画面には以下のように認識しているボールに、円が描かれ座標が保持されています  
<img src="https://github.com/downloads/HiroakiMatsuda/ArmISIK/readm_06.png" width="400px" />

 'r', 'g', 'b', 'p'のキーを押すと、それぞれ、赤、緑、青、桃のボールの付近までアームが移動します  
移動後に'c'ボタンを押すことで、アームがボールを把持しトレーまで運び格納します  
これを繰り返すことでロボットアームが卓上のカラーボールをお掃除してくれます  

注意：カラーボール検出アルゴリズムの都合で、1つの色のボールは1つまでしかカメラに映さないでください  
      
以上が本RTCの使い方のサンプルとなります  

ライセンス
----------
Copyright &copy; 2012 Hiroaki Matsuda  
Licensed under the [Apache License, Version 2.0][Apache]  
Distributed under the [MIT License][mit].  
Dual licensed under the [MIT license][MIT] and [GPL license][GPL].  
 
[Apache]: http://www.apache.org/licenses/LICENSE-2.0
[MIT]: http://www.opensource.org/licenses/mit-license.php
[GPL]: http://www.gnu.org/licenses/gpl.html