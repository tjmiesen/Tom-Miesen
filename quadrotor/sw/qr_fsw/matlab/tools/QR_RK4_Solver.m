function [y] = QR_RK4_Solver(f,h,y,u)

for i=1:5
    k1 = h*f(y(:,1),u(:,1));
    k2 = h*(f(y(:,1),u(:,1))+k1/2);
    k3 = h*(f(y(:,1),u(:,1))+k2/2);
    k4 = h*(f(y(:,1),u(:,1))+k3);
    y = y + (k1+2*k2+2*k3+k4)/6;
end