function [AP]=hrm(A, R, k, w)
% Approximation of inverse A'A,
% output: AP=(x,y,z) % ; 1st column: x, 2nd column: y, 3rd column: z
% input: A,R,k,w

if nargin <=1
   R = 0;
   MPi = pinv(A);
end

if ~exist('k','var')
    k = 0;
end

if  k < 0
    k = 0;
    [eVe eVa] = eig(A'*A);
    miu = sqrt(2*eVa(1,1)/eVa(end,end));
    if miu >= min(diag(eVa))
        miu = min(miu,eVa(2,2));
    elseif miu <= min(diag(eVa))
        miu = max(miu, eVa(end,end));
    end

    R = zeros(size(A'*A));
    R(end,end) = miu;
    R = eVe*R*eVe';
end

if ~exist('w','var')
    w = 0;
end

if 1 == w
    w = min(abs(eig(R/(A'*A+R))));
end

if nargin <=1
   AP = MPi;
else
    X_HRM  =  inv(A'*A+R);
    HRM_k  = zeros(length(A'*A)); 

    for i = 0:k
        HRM_k = HRM_k + (R/(A'*A+R))^i;
    end

    HRM_k = HRM_k + 1/(1-w)*(R/(A'*A+R))^(k+1);
    AP = X_HRM*HRM_k; 
end