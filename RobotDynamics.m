Mb = 1.086;   % mass body [kg]
Mw = 0.042;   % mass wheels [kg]
L = 0.085;    % length body [m]
I = 0.005712; % body inertia [kgm^2]
g = 9.81;     % gravity [m/s^2]
Tm = 0.8;     % motor torque [Nm]
d1 = 0.010;   % friction coefficient
d2 = 0.010;   % friction coefficient

n = 4;       % order of the system
ID = eye(n); % identity matrix

den = I*(Mw+Mb)+Mw*Mb*L^2;

% State-Space Matrices 
A = [0 1 0 0; 0 (-d1*(I+Mb*L^2)/den) (-Mb^2*g*L^2/den) (-d2*(Mb*L)/den);
    0 0 0 1; 0 (-d1*Mb*L/den) (Mb*g*L*(Mw+Mb)/den) (-d2*(Mw+Mb)/den)];

B = [0; -I-Mb*L^2/den; 0; Mb*L/den];

C = [0 0 1 0]; % for example we want to control theta

D = 0;

% Continuous state-space system model
sys_cont = ss(A,B,C,D);

% Transfer function
%G = tf(sys_cont);
%[numerator,denominator] = ss2tf(A,B,C,D);
%sys_tf = tf(numerator,denominator);

% Discrete state-space system model
h = 0.002; % sampling time 
sys_disc = c2d(sys_cont, h);

% Calculate the open-loop system poles 
poles = eig(A);

%step(sys_cont);

% --------------------------CLOSED LOOP------------------------------ %
% Desired closed-loop poles
a_1 = -2;
a_2 = -2;
a_3 = -2;
a_4 = -2;

% Calculate the controlabillity matrix 
gamma = [B A*B A^2*B A^3*B];
gamma_inv = inv([B A*B A^2*B A^3*B]);

% Calculate H matrix with desired closed-loop poles
H = (A-a_1*ID)*(A-a_2*ID)*(A-a_3*ID)*(A-a_4*ID);

% Calculate the state feedback gain (K)
K = -[0 0 0 1]*gamma_inv*H;

F = 1;
r = 1;
% New closed-loop State-Space System Matrices
A_cl = (A + B*K);
B_cl = B;
C_cl = C;
D_cl = 0;

% Continuous closed-loop system 
sys_cont_cl = ss(A_cl, B_cl, C_cl, D_cl);

% Initial states
x0 = [0; % 0 meters
      0; % 0 m/s
      0.1; % radian (arround 6 degrees) radian = degree*pi/180
      0]   % radian/s 

% LQR Control Law
Q = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1;]
R = 1

step(sys_cont_cl) % Step response
