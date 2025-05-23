# S型轨迹插补

[参考链接1](https://blog.csdn.net/u010632165/article/details/104951091)

总体分为三段式，加加速度阶段，加速阶段，减减速阶段

加加速:$J_a$
第1~3阶段周期:$T_{1-3}$

- 第一阶段速度：
$
v_1 = v_0 + \frac{1}{2}{J_a}{t^2}
$
- 第二阶段速度
$
v_2 = v_1 + {J_a}{T_1}*t
$
- 第三阶段速度
$
v_3 = v_2 + {J_a}{T_1}*t - {\frac{1}{2}}{J_a}{t^2}
$

## 轨迹规划
- 最大加加速度$J_{a_{max}}$
- 获取目标差值diff
- 获取目标时间
- 判断是否需要匀加速阶段
- 计算$J_a,T_1,T_2,T_3$


$v_1 = v_0 + \frac{1}{2}{J_a}{T_1^2}$
$v_2 = v_1 + {J_a}{T_1}*T_2$
$v_3 = v_2 + {J_a}{T_1}*T_2 - {\frac{1}{2}}{J_a}{T_3^2}$

$
v_1+v_2+v_3 = v_0 + \frac{1}{2}{J_a}{T_1^2} + v_1 + {J_a}{T_1}*T_2 + v_2 + {J_a}{T_1}*T_2 - {\frac{1}{2}}{J_a}{T_3^2}
$
$
v_1+v_2+v_3 = v_0 + \frac{1}{2}{J_a}{T_1^2} + {(v_0 + \frac{1}{2}{J_a}{T_1^2})} + {J_a}{T_1}*T_2 + {(v_1 + {J_a}{T_1}*T_2)} + {J_a}{T_1}*T_2 - {\frac{1}{2}}{J_a}{T_3^2}
$
$
v_1+v_2+v_3 = v_0 + \frac{1}{2}{J_a}{T_1^2} + {(v_0 + \frac{1}{2}{J_a}{T_1^2})} + {J_a}{T_1}*T_2 + {({(v_0 + \frac{1}{2}{J_a}{T_1^2})} + {J_a}{T_1}*T_2)} + {J_a}{T_1}*T_2 - {\frac{1}{2}}{J_a}{T_3^2}
$

$
v_1+v_2+v_3 = {(3*v_0)} + {J_a}{T_1^2} + {(3*{J_a}{T_1}*T_2)}
$

$
式1:
diff = {(3*v_0)} + {J_a}{T_1^2} + {(3*{J_a}{T_1}*T_2)}  
$

$
式2:
T_m = 2T_1+T_2
$
$
式2:
T_2 = T_m -2T_1;
$

$
式3:
diff = {(3*v_0)} + {J_a}{T_1^2} + {(3*{J_a}{T_1}*{(T_m -2T_1)})}  
$
$
式3:
diff = {(3*v_0)} + {J_a}{T_1^2} + {(3{J_a}{T_1}*T_m-{6{J_a}{T_1}^2})}  
$
$
式3:
diff = -5J_aT_1^2 + 3J_aT_mT_1+3v_0  
$
$
式3:
-5J_aT_1^2 + 3J_aT_mT_1+3v_0 - diff = 0  
$

求根公式:
$
\frac{-b+\sqrt{b^2-4ac}}{2a}
$
