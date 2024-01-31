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

% Calculate the open-loop system poles 
poles = eig(A);

