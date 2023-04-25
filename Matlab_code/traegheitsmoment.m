
function [I_travel, I_elevation, I_pitch] = traegheitsmoment(beta)
%% Traegmoments von pitch axis
% Gehaeuse
m_ge = 0.2; 
r_ge = 0.114;
h_ge = 0.027;
I_ge = 1/12* m_ge * (3*r_ge ^2 + h_ge ^2); % als ein Cylinder

%motor
m_mo = 0.287;
r_mo = 0.0195;
h_mo = 0.069;
I_mo = 1/12* m_mo * (3*r_mo ^2 + h_mo ^2); % als ein Cylinder

%schiene
m_sch = 0.322;
d_sch = 0.495;
w_sch = 0.006;
I_sch = 1/12* m_sch * (d_sch ^2 + w_sch ^2); % als ein Wuerfel

%joint
m_jo = 0.026;
d_jo = 0.019;
w_jo = 0.025;
I_jo = 1/12* m_jo * (d_jo ^2 + w_jo ^2); % als ein Wuerfel

%gesamt 
I_pitch = 2*(I_ge+m_ge*((0.027/2+0.069-0.041)^2+(0.355/2)^2))+ ..., %Traegheitsmoment von Gehaeuse plus m*Distanz^2 (parallel axis theorem)
    2*(I_mo+m_mo*((0.041-0.069/2)^2+(0.355/2)^2))+ ...,%Traegheitsmoment von Motoren plus m*Distanz^2 (parallel axis theorem)
    I_sch+m_sch*(0.041+0.003)^2+ ...,%Traegheitsmoment von Schiene plus m*Distanz^2 (parallel axis theorem)
    I_jo+m_jo*(0.041-0.019/2)^2; %Traegheitsmoment von Glenke plus m*Distanz^2 (parallel axis theorem)

%% elevation
L_w = 0.47; % Distance between travel axis to the counterweight
L_a = 0.655; % Distance between travel axis to the helicopter
L_g = 0.042; % Distance between elevation axis and mainarm

%gegengewicht
m_co = 1.918;
d_co = 0.070;
w_co = 0.057;
I_co = 1/12* m_co * (d_co ^2 + w_co ^2); % als Wuerfel

%hauptarm
m_ha = 0.377*(655+12.5)/915;
d_ha = L_a;
w_ha = 0.019;
I_ha = 1/12* m_ha * (d_ha ^2 + w_ha ^2);% als Wuerfel

%nebenarm
m_na = 0.377-m_ha+0.138;
d_na = L_w;
w_na = 0.019;
I_na = 1/12* m_na * (d_na ^2 + w_na ^2);% als Wuerfel

I_elevation = I_co + m_co * (L_w ^ 2+L_g ^2)+ ..., %Traegheitsmoment von Gegengewicht plus m*Distanz^2 (parallel axis theorem)
    I_ha + m_ha * ((L_a/2) ^ 2+L_g ^2)+ ...,%Traegheitsmoment von Hauptarm plus m*Distanz^2 (parallel axis theorem)
    I_na + m_na * ((L_w/2) ^ 2+L_g ^2)+ ...,%Traegheitsmoment von Nebenarm plus m*Distanz^2 (parallel axis theorem)
    1.322 * (L_a^2+(L_g+0.028)^2)+ ...,%Traegheitsmoment von Helicopter plus m*Distanz^2 (parallel axis theorem)
    0.07*((L_a-0.152)^2+L_g^2);%Traegheitsmoment von Magnet plus m*Distanz^2 (parallel axis theorem)

%% travel
sym beta;
L_h = 0.178;
m_f=0.487; %beinhalten den Gewicht vom Motor und Gehaeuse
m_b=0.487;

r = 0.114;
I_travel = m_co*(L_w*cos(beta))^2 + 1/3*m_na*(L_w*cos(beta))^2+ ...,
    1/3*m_ha*(L_a*cos(beta))^2+1/2*(m_f+m_b)*r^2+ ...,
    (m_f+m_b)*((L_a*cos(beta))^2+L_h^2)+ ...,
    m_sch*(L_a*cos(beta))^2;
% halten Gegengewicht, Hauptarm und Nebenarm als Punkt und halten zwei
% Motoren als Cylinder
% beinhalten die Traegheitsmoment von Gegengewicht,Hauptarm, Nebenarm, Schiene und
% zwei Motoren

end