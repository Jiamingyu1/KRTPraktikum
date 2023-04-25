clear all
syms adot bdot ydot f_f f_b a b y 
L_g =0.042;
L_h =0.178;
L_p = 0.028;
L_a =0.655;
L_w =0.47;
L_m =0.152;
g =9.81;
M_w =1.918;
M_na = 0.240;
M_ha = 0.275;
M_m = 0.07;
M_j = 0.132;
M_h = 1.296;
M_b = 0.487;
M_f = 0.487;
M_g = 0.2;
M_motor = 0.287;
M_s = 0.322;

punkt_arb = deg2rad(-7.5);
punkt_start = deg2rad(-27);

%traegheitsmoment
[I_a,I_b,I_y] = traegheitsmoment(punkt_arb);
%linearizierung
% xdot = [adot; bdot; ydot; -(f_f+f_b)*sin(y)*(L_g*sin(b)+L_a*cos(b)+L_p*sin(b))/I_a; ...,
%     (M_w*g*cos(b)*L_w+M_na*g*cos(b)*1/2*L_w ...,
%     -M_ha*g*cos(b)*(1/2*L_a)-M_m*g*cos(b)*(L_a-L_m) ...,
%     -(M_h+M_j)*g*cos(b)*L_a ...,
%     -M_w*g*sin(b)*L_g-M_na*g*sin(b)*L_g ...,
%     -M_ha*g*sin(b)*L_g-M_m*g*sin(b)*L_g ...,
%     -(M_h+M_j)*g*sin(b)*(L_g+L_p)+(f_f+f_b)*cos(y)*L_a)/I_b; ...,
%     ((f_f-f_b)*L_h+(-M_f+M_b)*g*L_h*cos(y)*cos(b)-(M_f+M_b)*g*cos(b)*sin(y)*L_p)/I_y];

xdot = [adot; bdot; ydot; -(f_f+f_b)*sin(y)*(L_g*sin(b)+L_a*cos(b)+L_p*sin(b))/I_a; ...,
    (M_w*g*cos(b)*L_w+M_na*g*cos(b)*1/2*L_w ...,
    -M_ha*g*cos(b)*(1/2*L_a)-M_m*g*cos(b)*(L_a-L_m) ...,
    -(M_h+M_j)*g*cos(b)*L_a ...,
    -M_w*g*sin(b)*L_g-M_na*g*sin(b)*L_g ...,
    -M_ha*g*sin(b)*L_g-M_m*g*sin(b)*L_g ...,
    -(M_h+M_j)*g*sin(b)*(L_g+L_p)+(f_f+f_b)*cos(y)*L_a)/I_b; ...,
    ((f_f-f_b)*L_h+(-M_f+M_b)*g*L_h*cos(y)*cos(b) ...,
    +M_g*2*g*cos(b)*sin(y)*(0.069+0.027/2-L_g) ...,
    -2*M_motor*g*cos(b)*sin(y)*(L_g-0.069/2)-M_s*g*cos(b)*sin(y)*L_g)/I_y];

A = jacobian(xdot,[a,b,y,adot,bdot,ydot]);
B = jacobian(xdot,[f_f, f_b]);

%rechnen f_f und f_b bei Arbeitspunkt
gl_1 = subs(xdot(5),{a,b,y},{0,punkt_arb,0});
gl_2 = f_f-f_b;
[ff,fb] = solve(gl_1,gl_2,f_f,f_b);
double([ff,fb])
[v_1,v_2] = kennlinie(ff);
% final answer
A_ans = double(subs(A,{a,b,y,f_f,f_b},{0,punkt_arb,0,ff,fb}))
B_ans = double(subs(B,{a,b,y,f_f,f_b},{0,punkt_arb,0,ff,fb}))
C = [1 0 0 0 0 0 ; 0 1 0 0 0 0 ;0 0 1 0 0 0];

% untersuchen Steuerbarkeit, Beobachtbarkeit und Stabilitaet
STEU=ctrb(A_ans,B_ans);
BEO=obsv(A_ans,C);
rank_steu = rank(STEU)
rank_beo = rank(BEO)
eigenwert = eig(A_ans)

[a_d,b_d,mag] = trajectorygeneration1;

% lqr

%A_lqr = zeros(8);
A_lqr = A_ans;
A_lqr(7,1) = 1;
A_lqr(8,2) = 1;
A_lqr(8,8) = 0;
%B_lqr = zeros([8 2]);
B_lqr = B_ans;
B_lqr(8,2) = 0;

%%
% Q = diag([(180/(5*pi))^2 (180/(0.5*pi))^2  (180/(0.1*pi))^2 1 1 1 1 1]);
%Q = diag([25 10 100 1 1 0 2 0.1]);
Q = diag([10 0.1 1 10 15 0 15 10]); %Alternativ 1 
%Q = diag([1 1 1 1 1 1 1 1]);
R = 0.01*diag([1 1]);
Q_L = 100*diag([10 10 10 1 1 1 ]);
R_L = 0.01*diag([1 1 1 ]);
[K_lqr, R_LQR, poles_LQR] =lqr(A_lqr, B_lqr, Q, R);
[L, R_KBF, poles_KBF]     = lqr(A_ans.',C.',Q_L,R_L);
poles_KBF
poles_LQR
L = L.';
k_i = 1;
D = zeros(3,2);
%  p = [-16, -16, -24, -24, -48, -120];
p = [-2,-2,-3,-3,-6,-15];% 3  * Poles von Lueberger Beobachter
% p = [-1,-1,-1.5,-1.5,-3,-7.5]; %2*pole
% p = [-2.5,-2.5,-4,-4,-8,-22]; % 4*pole
%luenberger Beobachter
E = place(A_ans',C',p).'

K_v = K_lqr(:,1:6);
C_v = [1 0 0 0 0 0;0 1 0 0 0 0];
v = -inv((C_v*inv(A_ans - B_ans*K_v))*B_ans);

clear ym a_sim b_sim;
%% 此分割线以下部分可在simulink模拟完之后运行，可画出alpha-beta轨迹图
s_out = size(ym);
for i = 1:s_out(3)
    a_sim(i) = ym(1,1,i);
    b_sim(i) = ym(2,1,i);
end
figure
title(' Simulationsergebnis (Luenberger)','FontSize',16)
xlabel('Travel /degree','FontSize',16)
ylabel('Elevation /degree','FontSize',16)
set(gca,'FontSize',14);
hold on
plot(a_sim,b_sim,'LineWidth',2)
hold on
plot(a_d(:,2),b_d(:,2),'--','LineWidth',1)
hold on 
axis([-20,480,-28,0])
grid on
legend('Simulationdaten','Trajektorie','Location','southeast','FontSize',14)
rectangle('position',[7.5,-27,75,19.5],'FaceColor',[0.9290 0.6940 0.1250])
rectangle('position',[97.5,-27,345,19.5],'FaceColor',[0.9290 0.6940 0.1250])
rectangle('position',[-15,-27,7.5,19.5],'FaceColor',[0.9290 0.6940 0.1250])
rectangle('position',[457.5,-27,7.5,19.5],'FaceColor',[0.9290 0.6940 0.1250])
