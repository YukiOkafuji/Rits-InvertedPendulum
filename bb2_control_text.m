% 初期化
clear;clc;

% パラメータ
Rw = 0.021; % 車輪半径
l = 0.135; % 重心長さ
M = 0.21; % 本体重量
m = 0.0053; % 車輪重量
Jb = M*l^2; % 本体慣性モーメント
Jw = m*Rw^2/2.0; % 車輪慣性モーメント
fm = 4.91e-7; % モータ軸粘性摩擦係数
g = 9.81; % 重力加速度
n = 21; % ギア比
Kt = 1.15e-3; % モータトルク係数

% 状態方程式の記述
E = [(m + M)*Rw^2 + Jw  M*Rw*l;
     M*Rw*l             M*l*l + Jb];
F = [2*fm  -2*fm;
    -2*fm  2*fm];
G = [0  0;
     0  -M*g*l];
H = [-2*n*Kt; 2*Kt];
A = [zeros(2)  eye(2);
     -E\G  -E\F];
B = [zeros(2, 1); E\H];
C = eye(4);

% A行列の固有値を確認
EigA = eig(A);

% ゲイン設定(どちらかを用いる)
% ----------------------------------
% 極配置
a1=-5;
a2=-10;
b1=3;
b2=5;
p=[a1+b1*1i a1-b1*1i a2+b2*1i a2-b2*1i];
K = place(A, B, p);
% ----------------------------------
% 最適制御
Q = diag([10 10 1 1]);
R = 10;
[K, P, e] = lqr(A, B, Q, R);
% ----------------------------------

% ゲインの表示
fprintf('ゲイン: %d %d %d %d\n', K(1), K(2), K(3), K(4));

% A - BK行列の固有値を確認
EigAKB = eig(A - B*K);

% 時系列にそって状態を計算
x = [0.0;0.1;0;0];
start = 0.0;
dt = 1e-4;
tf = 5.0;

X = zeros(tf/dt + 2, 6); % 保存用変数
X(1, :) = [0, x', 0];

count = 2;
for t = start:dt:tf
    u = -K*x;
    xdot = A*x + B*u;
    x = x + xdot*dt;
    X(count, :) = [t, x', u];
    count = count + 1;
end

% データの最大値
MaxTheta = max(abs(X(:, 2)));
MaxPsi = max(abs(X(:, 3)));
MaxU = max(abs(X(:, 6)));

fprintf('タイヤ角度最大値: %d\n', MaxTheta);
fprintf('本体傾斜角最大値: %d\n', MaxPsi);
fprintf('入力最大値: %d\n', MaxU);

% データの表示
figure('DefaultAxesFontSize',18);
plot(X(:, 1), X(:, 2))
xlabel('Time [s]')
ylabel('Wheel Angle \theta [rad]')
grid on;

figure('DefaultAxesFontSize',18)
plot(X(:, 1), X(:, 3))
xlabel('Time [s]')
ylabel('Body Angle \psi [rad]')
grid on;
