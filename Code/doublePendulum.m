% 双杆摆，无阻尼
clear; clc; close all;
N = 2; % 双摆
% 初始条件，国际单位
L1 = 1.0; L2 = 1.0;  % 杆长
m1 = 1.0; m2 = 1.0;  % 杆质量
M = 1.0;             % 摆锤质量
g = 9.81;            % 重力加速度
Input = [L1,L2,m1,m2,M,g]; % 双摆的物理模型参数,[L1,L2,m1,m2,M,g]
y0=[pi/18;pi/18;0;0]; % 这是角度和角动量初始值
h=1e-2;
x0=0:h:40;
%代入到ODE求解器中
[y1,Output]=odeRK4(x0,h,y0,Input);
%提取出角度
tN=size(y1,2);
th1=y1(1,:);
th2=y1(2,:);
%计算出关节坐标
CX1_A=zeros(1,tN);
CX1_B=CX1_A+L1*sin(th1);
CY1_A=zeros(1,tN);
CY1_B=CY1_A-L1*cos(th1);

CX2_A=CX1_B;
CX2_B=CX2_A+L2*sin(th2);
CY2_A=CY1_B;
CY2_B=CY2_A-L2*cos(th2);

figure(1) %两个角度随时间变化曲线
plot(x0,th1,x0,th2);
title('两个角度随时间变化')
legend('$\theta_1$','$\theta_2$','Location','northeast',...
    'interpreter','latex', 'FontSize', 10);
xlabel('t(s)','interpreter','latex', 'FontSize', 18);
ylabel('theta(rad)','FontSize', 18);
grid on;

figure(2) %两个角度关系
plot(th1,th2);
title('两个角度关系')
xlabel('$\theta_1$(rad)','interpreter','latex', 'FontSize', 18);
ylabel('theta_2(rad)', 'FontSize', 18);
grid on;

figure(3) %角动量随时间变化
plot(x0,y1(3,:),x0,y1(4,:));
title('两个角速度随时间变化')
legend('$\dot{\theta_1}$','$\dot{\theta_2}$','Location','northeast',...
    'interpreter','latex', 'FontSize', 10);
xlabel('t(s)','interpreter','latex', 'FontSize', 18);
ylabel('$\dot{\theta}$(rad/s)','interpreter','latex', 'FontSize', 18);
grid on;

figure(4) %摆锤位置曲线
plot(CX2_B,CY2_B);
title('摆锤运动路径')
xlabel('x','interpreter','latex', 'FontSize', 18);
ylabel('y','interpreter','latex', 'FontSize', 18);
grid on;

figure(5)%theta2角速度下的theta1
plot(y1(4,:),th1);
title('theta2角速度下的theta1')
xlabel('$\dot{\theta_2}$(rad/s)','interpreter','latex', 'FontSize', 18);
ylabel('theta_1(rad)', 'FontSize', 18);
grid on;

figure(6)%theta1角速度下的theta2
plot(y1(3,:),y1(2,:));
title('theta1角速度下的theta2')
xlabel('$\dot{\theta_1}$(rad/s)','interpreter','latex', 'FontSize', 18);
ylabel('theta_2(rad)', 'FontSize', 18);
grid on;

figure(7)%theta1角速度下的theta1
plot(y1(3,:),y1(1,:));
title('theta1角速度下的theta1')
xlabel('$\dot{\theta_1}$(rad/s)','interpreter','latex', 'FontSize', 18);
ylabel('theta_1(rad)', 'FontSize', 18);
grid on;

figure(8)%theta2角速度下的theta2
plot(y1(4,:),y1(2,:));
title('theta2角速度下的theta2')
xlabel('$\dot{\theta_2}$(rad/s)','interpreter','latex', 'FontSize', 18);
ylabel('theta_2(rad)', 'FontSize', 18);
grid on;


%绘图
% n=1;
% figure(1) %动画 %figure(*)要修改
% set(gcf,'position',[200   400   500   400])
% for k=1:4:length(x0) %这里4步一显示时间帧
%     clf
%     xlim([-(L1+L2),(L1+L2)])
%     ylim([-(L1+L2),(L1+L2)])
%     hold on
%     %绘制摆
%     plot([CX1_A(k),CX1_B(k)],[CY1_A(k),CY1_B(k)],'color','k','LineWidth',1.5)
%     plot([CX2_A(k),CX2_B(k)],[CY2_A(k),CY2_B(k)],'color','k','LineWidth',1.5)
%     grid on;
%     %绘制轨线
%     if k>200
%         n=n+1;
%     end
%     Nm=k-n+1;
%     %轨迹1
%     F_color=[1,0,0];
%     F_color=F_color*0.6+[1,1,1]*0.4*0.999;
%     cdata=[linspace(1,F_color(1),Nm+1)',linspace(1,F_color(2),Nm+1)',linspace(1,F_color(3),Nm+1)'];
%     cdata=reshape(cdata,Nm+1,1,3);
%     if k>3
%         patch([CX1_B(n:k),NaN],[CY1_B(n:k),NaN],1:Nm+1,'EdgeColor','interp','Marker','none',...
%           'MarkerFaceColor','flat','CData',cdata,'LineWidth',1.5);
%     end
%     %轨迹2
%     F_color=[0,0,1];
%     F_color=F_color*0.6+[1,1,1]*0.4*0.999;
%     cdata=[linspace(1,F_color(1),Nm+1)',linspace(1,F_color(2),Nm+1)',linspace(1,F_color(3),Nm+1)'];
%     cdata=reshape(cdata,Nm+1,1,3);
%     if k>3
%         patch([CX2_B(n:k),NaN],[CY2_B(n:k),NaN],1:Nm+1,'EdgeColor','interp','Marker','none',...
%           'MarkerFaceColor','flat','CData',cdata,'LineWidth',1.5);
%     end
%     hold off
%     pause(0.05)
%     %可以在这里添加输出动图的程序
% end


%求解器，RK4
function [y,Output]=odeRK4(x,h,y0,Input)
    %4阶RK方法
    %h间隔为常数的算法
    y=zeros(size(y0,1),size(x,2)); % 待修改，这里直接用test1的求解器
    y(:,1)=y0;
    for ii=1:length(x)-1
        yn=y(:,ii);
        xn=x(ii);
        K1=Fdydx(xn    ,yn       ,Input);
        K2=Fdydx(xn+h/2,yn+h/2*K1,Input);
        K3=Fdydx(xn+h/2,yn+h/2*K2,Input);
        K4=Fdydx(xn+h  ,yn+h*K3  ,Input);
        y(:,ii+1)=yn+h/6*(K1+2*K2+2*K3+K4);
    end
    Output=[];
end

function dydx = Fdydx(x,y,Input)
    %将原方程整理为dy/dx=F(y,x)的形式
    %将输入物理参数赋值
    L1=Input(1); L2=Input(2);
    m1=Input(3); m2=Input(4);
    M=Input(5); g=Input(6);
    th1=y(1);%角度1
    th2=y(2);%角度2
    pth1=y(3);%角动量1
    pth2=y(4);%角动量2
    %考虑阻尼的微分方程组，中间变量
    A=(m1/3 + m2 + M)*L1^2;
    B=(m2/2 + M)*L1*L2*cos(th1-th2);
    C=(m2/2 + M)*L1*L2*cos(th1-th2);
    D=(m2/3 + M)*L2^2;
    I=-(m2/2 + M)*L1*L2*sin(th1-th2)*pth2^2-(m1/2 + m2 + M)*g*L1*sin(th1);
    J=(m2/2 + M)*L1*L2*sin(th1-th2)*pth1^2-(m2/2 + M)*g*L2*sin(th2);
    %四阶微分方程
    dth1=pth1;
    dth2=pth2;
    dpth1=(B*J-D*I)/(B*C-A*D);
    dpth2=(A*J-C*I)/(A*D-B*C);
    %输出整理
    dydx=zeros(4,1);
    dydx(1)=dth1;
    dydx(2)=dth2;
    dydx(3)=dpth1;
    dydx(4)=dpth2;
end
