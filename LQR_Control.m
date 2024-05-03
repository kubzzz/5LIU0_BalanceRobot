Mb = 1.086;   % mass body [kg]
Mw = 0.042;   % mass wheels [kg]
L = 0.085;    % length body [m]
I = 0.005712; % body inertia [kgm^2]
g = 9.81;     % gravity [m/s^2]
Tm = 0.8;     % motor torque [Nm]
d1 = 0.010;   % friction coefficient
d2 = 0.010;   % friction coefficient

den = I*(Mw+Mb)+Mw*Mb*L^2;

% State-Space Matrices 
A = [0 1 0 0; 0 (-d1*(I+Mb*L^2)/den) (-Mb^2*g*L^2/den) (-d2*(Mb*L)/den);
    0 0 0 1; 0 (-d1*Mb*L/den) (Mb*g*L*(Mw+Mb)/den) (-d2*(Mw+Mb)/den)];

B = [0; -I-Mb*L^2/den; 0; Mb*L/den];

C = [0 0 1 0]; % for example we want to control theta

D = 0;

% Calculate the controlabillity matrix 
gamma = [B A*B A^2*B A^3*B];
gamma_inv = inv([B A*B A^2*B A^3*B]);

% Continuous open-loop state-space system model
sys_cont = ss(A,B,C,D);

% LQR Control Law
Q = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1;];
R = 1;
K = lqr(A,B,Q,R)

% Continuous closed-loop system 
sys_cont_cl = ss((A-B*K), B, C, D);

eig((A-B*K))

% step(sys_cont_cl)

% Initial states
x0 = [0;  % 0 meters
      0;  % 0 m/s
      5;  % 5 degrees
      0]; % degrees/s 

% run the response 
%t = 0:0.005:30;
[y, t, x] = initial(sys_cont_cl, x0);

% plot response
p1 = plot(t, y(:, 1), 'LineWidth', 2)

% plot actuator effort
p2 = plot(t, -K*x', 'LineWidth', 2)

% plot pole placement
pzmap(sys_cont_cl);
