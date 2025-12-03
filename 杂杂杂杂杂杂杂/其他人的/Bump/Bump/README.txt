目前只用了pwm分了三档，考虑到要控制变量没有把follower小车的pid控速做进去，单纯用的pwm控速
也没有左右轮补偿

Leader_Bump里：
BumpIR.txt 只发射ir，主要是用手改变leader位置去看follower情况
straight.txt 发射ir，并且走直线，走到(450,0)停下
Turn.txt 发射ir，走30度曲线，走到(450,0)停下

Bump_only里：
IR检测.txt：用bump传感器做直线跟随的主程序