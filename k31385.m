clear
% Constants
L=3;

%First run
%b_nom=0.252;
%x_cw=[-0.613 ;-0.696 ;-0.644 ;-0.622 ;-0.673 ];
%x_ccw=[-0.412 ;-0.510 ;-0.428 ;-0.478 ; -0.447];

%Secund run
%b_nom=0.2677;
%x_cw=[-0.150 ;-0.130 ;-0.101 ;-0.152 ;-0.194 ];
%x_ccw=[-0.350 ;-0.440 ;-0.406 ;-0.477 ;-0.415 ];

%Third run
%b_nom=0.2761;

%x_cw=[ -0.061 ; -0.130 ; -0.063 ; -0.082 ; -0.182 ];
%x_ccw=[ 0.075 ; 0.220 ; 0.132 ; 0.004 ; -0.038 ];

%y_cw=[ -0.076 ; -0.135 ; -0.126 ; -0.045 ; -0.190 ];
%y_ccw=[ -0.035 ; -0.125 ; -0.083 ; -0.023 ; 0.135 ];

%Fourth run
b_nom = 0.2761;

x_cw = [ -0.231 ; -0.167 ; -0.170 ; -0.122 ; -0.222 ];
x_ccw = [ -0.085 ; -0.135 ; -0.071 ; -0.055 ; -0.126 ];

y_cw = [ -0.212 ; -0.215 ; -0.229 ; -0.096 ; -0.218 ];
y_ccw = [ 0.050 ; 0.150 ; 0.173 ; 0.055 ; 0.075 ];

n=5;
% x calculations
x_c_g_cw = (1/n)*sum(x_cw);
x_c_g_ccw = (1/n)*sum(x_ccw);
alpha_x = (x_c_g_cw + x_c_g_ccw)/(-4*L);
beta_x = (x_c_g_cw - x_c_g_ccw)/(-4*L);
R_x = (L/2)/(sin(beta_x)/2);
% End Results
E_b_x = (pi/2)/((pi/2)-alpha_x);
b_act_x = E_b_x*b_nom
E_d_x = (R_x+(b_act_x/2))/(R_x-(b_act_x/2))


% y calculations
y_c_g_cw = (1/n)*sum(y_cw);
y_c_g_ccw = (1/n)*sum(y_ccw);
alpha_y = (y_c_g_cw - y_c_g_ccw)/(-4*L);
beta_y = (y_c_g_cw + y_c_g_ccw)/(-4*L);
R_y = (L/2)/(sin(beta_y)/2);
% End Results
E_b_y = (pi/2)/((pi/2)-alpha_y);
b_act_y = E_b_y*b_nom
E_d_y = (R_y+(b_act_y/2))/(R_y-(b_act_y/2))

% Average
E_d = (E_d_x + E_d_y)/2
b_act = (b_act_x + b_act_y)/2

