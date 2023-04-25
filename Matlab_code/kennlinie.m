function [v_1,v_2] = kennlinie(f)
v_1 = double(sqrt(f/0.0599));
v_2 = double(sqrt(f/0.0459));
% spannung = 0:0.5:4;
% kraft_f = 9.8e-3 .* [0 0 2 6 11 16 24 43 58];
% kraft_b = 9.8e-3 .* [0 0 2 7 16 23 36 46 58]; %% data 
% 
% 
% % plot(spannung,kraft_f,'*','markersize',10)  
% % hold on  %% plot messwert von Frontmoter
% % 
% % plot(spannung,kraft_b,'.','markersize',10) 
% % hold on %% plot messwert von Backmoter
% 
% P1 = polyfit(spannung,kraft_f,3);  
% 
% % % 画图
% % t=0:0.1:4;
% % S1=polyval(P1,t);  
% % plot(t,S1,'r');
% % hold on %%Polynomanpassung von Frontmotor und plot
% 
% P2 = polyfit(spannung,kraft_b,3);  
% 
% % % 画图
% % t=0:0.1:4;
% % S2=polyval(P2,t);  
% % plot(t,S2,'b'); %%Polynomanpassung von Backmotor und plot
% % title ('Kennlinie von jeweils Motor')
% % xlabel('Eingang Spannung /V')
% % ylabel('Messdaten /N')
% 
% g_1 = poly2sym(P1) - f;
% g_2 = poly2sym(P2) - f;
% v_1 = double(vpasolve(g_1,[0,4]));
% v_2 = double(vpasolve(g_2,[0,4]));
end