%This program is an attempt to implement a PID control for a 2 link robot
%arm
th_init=[-pi/3 pi/3];
ths=[pi/2 -pi/4];
x0=[0 0 th_init 0 0 0 0];
Ts=[0 20];
%Specifications
L1=1;
L2=1;
M1=1;
M2=1;
spec=[L1 L2 M1 M2];
%PID PARAMETERS
Kp1=15;
Kd1=7;
Ki1=10;
Kp2=15;
Kd2=10;
Ki2=10;
Kpid=[Kp1 Kd1 Ki1 Kp2 Kd2 Ki2];
%SOLVER
[T,X]=ode45(@(t,x) r2dof(t,x,ths,spec,Kpid),Ts,x0);
%O/P
th1=X(:,3);
th2=X(:,4);
%Torque Cmp
F1=diff(X(:,7))./diff(T);
F2=diff(X(:,8))./diff(T);
tt=0:(T(end)/(length(F1)-1)):T(end);
%X and Y positions
X1=L1.*sin(th1);
y1=L1.*cos(th1);
x2=L1.*sin(th1)+L2.*sin(th1+th2);
y2=L1.*cos(th1)+L2.*cos(th1+th2);
plot(T,ths(1)-th1)
grid
title('Theta-1 error')
ylabel('theta1 error (rad)')
xlabel('time (sec)')
%theta2 error plot
figure
plot(T,ths(2)-th2)
grid
title('Theta-2 error')
ylabel('theta2 error (rad)')
xlabel('time (sec)')
%torque1 plot
figure
plot(tt,F1)
grid
title('Torque of theta 1')
ylabel('theta1 torque')
xlabel('time (sec)')
%torque2 plot
figure
plot(tt,F2)
grid
title('Torque of theta 2')
ylabel('theta2 torque')
xlabel('time (sec)')
d=2;
j=1:d:length(T);
for i=1:length(j)-1
    hold off
    plot([X1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'o',[0 X1(j(i))],[0
y1(j(i))],'k',[X1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'k')
title('Motion of 2DOF Robotic Arm')
 xlabel('x')
 ylabel('y')
 axis([-3 3 -3 3]);
 grid
 hold on
 MM(i)=getframe(gcf);
end
drawnow;


