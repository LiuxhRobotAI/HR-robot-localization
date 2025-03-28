function [Xtrue]=Route3D()
% Route of robot (moblie station, MS), Suppose uniform motion (velocity)
% Output: Xtrue=(x,y,z), route matrix: 1st column: x, 2nd column: y, 3rd column: z
% Input: None
% Route generate: 0.2m/s, 10Hz generate a route data in 5*10 m^2 area

% 1st route
y1=2.5*ones(375,1);
x1=(linspace(2.5,10,375))';

% 2nd route
n = 392;
t = linspace(-0.5*pi,0.5*pi,n+1);
rho = 2.5*ones(1,n+1);
[x2,y2] = pol2cart(t,rho);
x2=x2+10;
y2=y2+5;
y=[y1;y2'];
x=[x1;x2'];

% 3rd route
y=[y;7.5*ones(375,1)];
x=[x;linspace(10,2.5,375)'];

z=ones(length(x),1)*0.5;
x=[x; 2.5*ones(207,1)];
y=[y; linspace(7.5,3.5,207)'];
z=[z; linspace(0.5,1.5,207)'];

Xtrue=[x y z];
Xtrue=Xtrue*0.5;
end