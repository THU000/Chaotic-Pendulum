 % %将原方程整理为dy/dx=F(y,x)的形式
 %    %将输入物理参数赋值
 %    L1=Input(1); L2=Input(2);
 %    m1=Input(3); m2=Input(4);
 %    M=Input(5); g=Input(6);
 %    th1=y(1);%角度1
 %    th2=y(2);%角度2
 %    pth1=y(3);%角动量1
 %    pth2=y(4);%角动量2
 %    %考虑阻尼的微分方程组，中间变量，见滇沅报告
 %    A=(m1/3 + m2 + M)*L1^2;
 %    B=(m2/2 + M)*L1*L2*cos(th1-th2);
 %    C=(m2/2 + M)*L1*L2*cos(th1-th2);
 %    D=(m2/3 + M)*L2^2;
 %    I=-(m2/2 + M)*L1*L2*sin(th1-th2)*pth2^2-(m1/2 + m2 + M)*g*L1*sin(th1);
 %    J=(m2/2 + M)*L1*L2*sin(th1-th2)*pth1^2-(m2/2 + M)*g*L2*sin(th2);
 %    %四阶微分方程
 %    dth1=pth1;
 %    dth2=pth2;
 %    dpth1=(B*J-D*I)/(B*C-A*D);
 %    dpth2=(A*J-C*I)/(A*D-B*C);
 %    %输出整理
 %    dydx=zeros(4,1);
 %    dydx(1)=dth1;
 %    dydx(2)=dth2;
 %    dydx(3)=dpth1;
 %    dydx(4)=dpth2;