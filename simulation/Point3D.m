function [Xtrue]=Point3D(Area)

if nargin < 1
    print("Error: required area of points");
end

n = 1000;
for i = 1:length(Area)
    a = Area(1,i);
    b = Area(2,i);
    Xtrue(:, i) = (b-a).*rand(n,1) + a;
end
end