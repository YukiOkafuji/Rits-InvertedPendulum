# 立命館大学情報理工学部　実世界情報コース　実世界情報実験３　倒立振子系の制御

## プログラム内容
- `bb2_control.ipynb` Python3 (Jupyter)による倒立振子のシミュレーションプログラム（2020年以降）。
- `control.c` ビュートバランサー本体に書き込むためのプログラム
- `bb2_control_text.m` Python3 (Jupyter)による倒立振子のシミュレーションプログラム（2019年）。

## `bb2_control.ipynb`における値設定方法
- 極配置法を用いる場合は，pの値を設定する．同時に，最適制御法におけるQ, R, P, Kはコメントアウトする．
- 最適制御法を用いる場合は，Q, Rの値を設定する．同時に，極配置法におけるp, Kはコメントアウトする．
```
### ゲイン設定，どちらかを用いる ###
# ----------------------------------
# 極配置
p = [-...+...j, -...-...j, -...+...j, -...-...j]
# 左2つ・右2つがそれぞれペアの共役複素数になるように設定（例：[-1+1j, -1-1j, -2+2j, -2-2j]）
K = np.array(matlab.place(A, B, p)) # ゲイン

# 最適制御
Q = np.diag([..., ..., ..., ...])
R = np.array([...])
P = linalg.solve_continuous_are(A, B, Q, R)
K = np.dot(B.T, P)/R # ゲイン

print("Gain: ", K)
# ----------------------------------
```

## `control.c`における値設定方法
最適制御法（もしくは極配置法）のシミュレーションで求めたゲインを用いて，K[0]~K[3]の値を設定する．
```
void Control(){
	volatile double outL = 0.0,outR = 0.0;

	// MATLABで最適制御のゲインを計算し代入する
	double K[4] = {0.0, 0.0, 0.0, 0.0};
	K[0] = ...;
	K[1] = ...;
	K[2] = ...;
	K[3] = ...;

	// 状態量
	double x[4];
	x[0] = -(memmap.values.WHEEL_ANGLE_L + memmap.values.WHEEL_ANGLE_R)/2.0;
	x[1] = memmap.values.BODY_ANGLE;
	x[2] = -(memmap.values.WHEEL_ANGULAR_SPD_L + memmap.values.WHEEL_ANGULAR_SPD_R)/2.0;
	x[3] = memmap.values.BODY_ANGULAR_SPD;

	// 最適制御による状態フィードバック制御
	double u = 0.0;
	int i = 0;
	for(i=0; i<4; i++){
		u += -K[i]*x[i];
	}
	outL += u;
	outR += u;

	//最大指令値規制
	if(outL > 32767.0){
		outL = 32767.0;
	}
	if(outR > 32767.0){
		outR = 32767.0;
	}
	if(outL < -32767.0){
		outL = -32767.0;
	}
	if(outR < -32767.0){
		outR = -32767.0;
	}

	//電流指令値設定
	memmap.values.T_CURRENT_L = (short)outL*1000;
	memmap.values.T_CURRENT_R = (short)outR*1000;
}
```

## 外部リンク
- [講義動画](https://youtu.be/N5dlfMK_PqE)
- [ビュートバランサー2 取扱説明書](https://www.vstone.co.jp/products/beauto_balancer_2/download/BeautoBalancer2_Manual_1_05.pdf)
