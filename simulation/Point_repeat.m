function [Xtrue]=Point_repeat(Area)

if nargin < 1
    fprintf("Error: required area of points");
end

n = 1;
for i = 1:length(Area)
    a = Area(1,i);
    b = Area(2,i);
    Xtrue(:, i) = (b-a).*rand(n,1) + a;
end

N_repeat = 1000;
for i = 1:N_repeat-1
    Xtrue = [Xtrue;
             Xtrue(end, :)];
end
