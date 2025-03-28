%%
% Description: 
% Code for high-order regularization method
%
% Author, date:
%     Xinghua Liu (xinghua.liu@rug.nl), 20250328
%

clc;
clear;

BSN = 5;
BS = anchor_3D_position(BSN);
BS(BSN,:) = [BS(BSN,1:2) BS(BSN,3)*0.20 ];
BS = [BS(1:BSN-1,:); BS(BSN,:)];

% True path
Area = [min(BS(:,1)), min(BS(:,2)), min(BS(:,3));
        max(BS(:,1)), max(BS(:,2)), max(BS(:,3))];
Xr = Point3D(Area); % Random points.
% Xr = Route3D();

rt_dx =[0,0,0];
winL = 100;
wdx = [0,0,0];
dx_wdx = zeros(winL, length(rt_dx));

rough = false;
opt_bias = true;
use_route = true;
if use_route
    Xr = Route3D();
end
use_TSVD = true;

nees_MC_point = false;
guassian_b = false;
if nees_MC_point
    Xr = Point_repeat(Area);
    guassian_b = true;
end

for j = 1:length(Xr(:,1))
    MS = [Xr(j,1),Xr(j,2),Xr(j,3)]; 

    for i  =  1: BSN
        r0(i) = sqrt((BS(i,1) - MS(1))^2 + (BS(i,2) - MS(2))^2+ (BS(i,3) - MS(3))^2); 
    end

    Noise  =  0.1;
    for i  =  1: BSN
       r(i) = r0(i) + Noise * randn(1);
       if guassian_b
           r(i) = r0(i); % NEES test
           e(i) = Noise * randn(1);
       end
    end

    A = [2*(BS(1,1)-BS(BSN,1)) 2*(BS(1,2)-BS(BSN,2)) 2*(BS(1,3)-BS(BSN,3));
       2*(BS(2,1)-BS(BSN,1)) 2*(BS(2,2)-BS(BSN,2)) 2*(BS(2,3)-BS(BSN,3))];

    B = [(BS(1,1)^2-BS(BSN,1)^2+BS(1,2)^2-BS(BSN,2)^2+BS(1,3)^2-BS(BSN,3)^2+r(BSN)^2-r(1)^2);
         (BS(2,1)^2-BS(BSN,1)^2+BS(2,2)^2-BS(BSN,2)^2+BS(2,3)^2-BS(BSN,3)^2+r(BSN)^2-r(2)^2)];

    if guassian_b
        B(1) = B(1)+e(1);
        B(2) = B(2)+e(2);
    end

   if  3<BSN
   for i = (4-1):(BSN-1)
       A = [A;
          2*(BS(i,1)-BS(BSN,1)) 2*(BS(i,2)-BS(BSN,2)) 2*(BS(i,3)-BS(BSN,3))];
       B = [B;
          (BS(i,1)^2-BS(BSN,1)^2+BS(i,2)^2-BS(BSN,2)^2+BS(i,3)^2-BS(BSN,3)^2+r(BSN)^2-r(i)^2)];
       if guassian_b
            B(i) = B(i)+e(i);
       end

   end
   end

   % LS
   XL(j,:) = inv(A'*A)*A'*B;

   In = eye(length(A'*A));

   % Regularization with SVD
   X_TSVD(j,:) = zeros(1, length(A'*A));
   [U S V] = svd(A); 
   A_k_inv = zeros(size(S'));
   kn = rank(A);

   % Mu for FTR
   if kn <= length(A)
        miu = sqrt(2*S(1,1)/S(3,3));
        miu2 = sqrt(2*S(1,1)^2/S(3,3)^2);

        if miu2 >= S(3,3)^2
            miu2 = min(miu2,S(2,2));
        end

        if miu2 <= S(2,2)^2
            miu2 = max(miu2,S(3,3));
        end
   else
       miu = S(kn-1,kn-1);
       miu2 = S(kn-1,kn-1);
   end
   % Truncated SVD
   for i = 1:kn-1
       s_i = S(i,i);
       u_i = U(:,i);
       v_i = V(:,i);
       D_m2(i,i) = max(miu2-s_i^2, 0);
       A_k_inv_i = 1/s_i*v_i*u_i';
       A_k_inv = A_k_inv + A_k_inv_i;
       x_Ti = 1/s_i*v_i*u_i' * B;
       X_TSVD(j,:) = X_TSVD(j,:) + x_Ti';
   end

   % OFTR
   X_nT(j,:)=X_TSVD(j,:);
   for i = kn : rank(A)
       s_i = S(i,i);
       u_i = U(:,i);
       v_i = V(:,i);
       D_m2(i,i) = max(miu2-s_i^2, 0);
       A_k_inv_i = 1/s_i*v_i*u_i';
       A_k_inv = A_k_inv + A_k_inv_i;
       x_Ti = (s_i^2/miu2)*1/s_i*v_i*u_i' * B;
       X_nT(j,:) = X_nT(j,:) + x_Ti';
   end
   X_nT(j,:) = V/(S'*S + D_m2)*S'*U'* B;


   % Tikhonov regularization
   miu_s = S(rank(A),rank(A));
   X_STR(j,:) = inv(A'*A + miu_s^2 *In)*A'*B;

   % % HR test w
   R = zeros(size(A'*A));
   An = length(A'*A);
   R = miu_s^2*In;
   X_HRM  =  inv(A'*A+R);
   HRM_k  = zeros(length(A'*A));
   k = 0;
   for i = 0:k-1
       HRM_k = HRM_k + (R/(A'*A+R))^i;
   end
   w = min(abs(eig(R/(A'*A+R))));
   HRM_k = HRM_k + 1/(1-w)*(R/(A'*A+R))^k;
   X_HRM_k_s(j,:) = X_HRM*HRM_k*A'*B;

   % R changes the smallest eigenvalue
   R = zeros(size(A'*A));
   evalue = eig(A'*A);
   [P Da] = eig(A'*A);
   for i = An : An
        R(i,i) = sqrt(Da(1,1)^2 + Da(1,1)*Da(3,3));
        R(i,i) = min(R(i,i),Da(2,2));
   end
   k = 1;
   w = 0;
   X_HRM_k(j,:) = hrm(A,R,k,w)* A'*B;

   X_HRM_k_bias(j,:) = inv(A'*A+R) /(In-R/(A'*A+R)) * (R/(A'*A+R))^(k+1) * A'*B;

   if use_TSVD
   R = zeros(size(A'*A));
   for i = 1 : 1
         R(i,i) = evalue(2);
   end
   R = P*R*P';
   end

   k_o = 1;
   w = 0;
   X_HRM_k_o(j,:) = hrm(A,R,k_o,w)* A'*B;

   % Sliding window
   if winL < j
       dx_wdx(1:winL-1,:) = dx_wdx(2:winL,:);
   end
   dx_wdx(min(j,winL),:) = X_HRM_k_o(j,:) - XL(j,:);
   rt_dx = sum(dx_wdx)./winL;

   if winL <= j 
       X_HRM_k_o(j,:) = X_HRM_k_o(j,:) - rt_dx;
   end

   % NEES_MC
   if winL > j && nees_MC_point
       X_HRM_k_o(j,:) = X_HRM_k_o(j,:) - (X_HRM_k_o(j,:) - XL(j,:));
   end

% TR, FTR, OFTR or TSVD, HR with different R, k and w
    if rough
        Xc(j,:) = X_STR(j,:);
        Xs(j,:) = X_HRM_k_s(j,:);        
    elseif opt_bias && use_TSVD
        Xc(j,:) = X_TSVD(j,:); 
        Xs(j,:) = X_HRM_k_o(j,:);
    elseif opt_bias && ~use_TSVD
        Xc(j,:) = X_nT(j,:);
        Xs(j,:) = X_HRM_k_o(j,:);
    else
        Xc(j,:) = X_nT(j,:);% FTR, OFTR
        Xs(j,:) = X_HRM_k(j,:);
    end

end

% Error evaluation
ErrorXLx = XL(:,1)-Xr(:,1);
ErrorXLy = XL(:,2)-Xr(:,2);
ErrorXLz = XL(:,3)-Xr(:,3);
ErrorXL = sqrt(ErrorXLx.^2+ErrorXLy.^2+ErrorXLz.^2); % error in distance
MeanVL = mean(ErrorXL);
RMSE_L = sqrt(sum((ErrorXL).^2)/length(ErrorXL));
RMSE_Lx = sqrt(sum((ErrorXLx).^2)/length(ErrorXLx));
RMSE_Ly = sqrt(sum((ErrorXLy).^2)/length(ErrorXLy));
RMSE_Lz = sqrt(sum((ErrorXLz).^2)/length(ErrorXLz));
StdVL = std(ErrorXL,0);
MaxerrorL = max(abs(ErrorXL));

ErrorXcx = Xc(:,1)-Xr(:,1);
ErrorXcy = Xc(:,2)-Xr(:,2);
ErrorXcz = Xc(:,3)-Xr(:,3);
ErrorXc = sqrt(ErrorXcx.^2+ErrorXcy.^2+ErrorXcz.^2);
MeanVc = mean(ErrorXc);
RMSE_c = sqrt(sum((ErrorXc).^2)/length(ErrorXc));
RMSE_cx = sqrt(sum((ErrorXcx).^2)/length(ErrorXcx));
RMSE_cy = sqrt(sum((ErrorXcy).^2)/length(ErrorXcy));
RMSE_cz = sqrt(sum((ErrorXcz).^2)/length(ErrorXcz));
StdVc = std(ErrorXc,0);
Maxerrorc = max(abs(ErrorXc));

ErrorXsx = Xs(:,1)-Xr(:,1);
ErrorXsy = Xs(:,2)-Xr(:,2);
ErrorXsz = Xs(:,3)-Xr(:,3);
ErrorXs = sqrt(ErrorXsx.^2+ErrorXsy.^2+ErrorXsz.^2);
MeanVs = mean(ErrorXs);
RMSE_s = sqrt(sum((ErrorXs).^2)/length(ErrorXs));
RMSE_sx = sqrt(sum((ErrorXsx).^2)/length(ErrorXsx));
RMSE_sy = sqrt(sum((ErrorXsy).^2)/length(ErrorXsy));
RMSE_sz = sqrt(sum((ErrorXsz).^2)/length(ErrorXsz));
StdVs = std(ErrorXs,0);
Maxerrors = max(abs(ErrorXs));

impL = (RMSE_L-RMSE_s)/RMSE_L;
impLz = (RMSE_Lz-RMSE_sz)/RMSE_Lz;

% Error of different methods
figure('units','normalized','position',[0.1,0.1,0.55,0.6]);
subplot(3,1,1);
pLz = plot(ErrorXLz,'b-');
hold on
pLx = plot(ErrorXLx,'r--');
hold on
pLy = plot(ErrorXLy,'k-.');
legend('error in z','error in x','error in y','Orientation','horizon',...
'location','NorthEast');
legend('boxoff');
title('LSM estimation error in each axis');
text(10,1.3,['RMSE in z axis =  ',num2str(RMSE_Lz)]);
hold off

subplot(3,1,2);
plot(ErrorXcz,'b-');
hold on
plot(ErrorXcx,'r--');
hold on
plot(ErrorXcy,'k-.');
legend('error in z','error in x','error in y','Orientation','horizon',...
'location','NorthEast');
legend('boxoff');
title('Regularized estimation error in each axis');
ylabel('Error in each axis(m)');
text(10,0.3,['RMSE in z axis =  ',num2str(RMSE_cz)]);
hold off

subplot(3,1,3);
plot(ErrorXsz,'b-');
hold on
plot(ErrorXsx,'r--');
hold on
plot(ErrorXsy,'k-.');
legend('error in z','error in x','error in y','Orientation','horizon',...
'location','NorthEast');
legend('boxoff');
title('High-order method error in each axis');
xlabel('Point number');
text(10,0.3,['RMSE in z axis =  ',num2str(RMSE_sz)]);
hold off

figure('units','normalized','position',[0.1,0.1,0.55,0.6]);
subplot(3,1,1); 
plot(ErrorXL,'-');
title('LSM estimation error');
text(10,1.5,['RMSE =  ',num2str(RMSE_L)]);
hold off;

subplot(3,1,2);
plot(ErrorXc,'-');
title('Regularized estimation error');
ylabel('Error(m)');
text(10,0.3,['RMSE =  ',num2str(RMSE_c)]);
hold off;

subplot(3,1,3); 
plot(ErrorXs,'-');
title('High-order method error');
text(10,0.3,['RMSE =  ',num2str(RMSE_s)]);
xlabel('Point number');
hold off;


% % NEES (Normalized estimation error squared) test
if nees_MC_point
ErrorXLp =[ErrorXLx,ErrorXLy,ErrorXLz];
ErrorXcp =[ErrorXcx,ErrorXcy,ErrorXcz];
ErrorXsp =[ErrorXsx,ErrorXsy,ErrorXsz];

sigma2 = Noise^2;
if ~guassian_b
P0 = 2*sigma2^2 * ones(length(B));
for i=1:length(B)
    P0(i,i)= 4*sigma2^2;
end
end

if guassian_b
P0 = sigma2*diag(ones(length(B),1));
end

P_ls =  inv(A'*A)* A'* P0 *A* inv(A'*A);
nees_ls = mean(sum((ErrorXLp) / P_ls .* (ErrorXLp), 2));

P_c = inv(A'*A + R)* A' * P0 *A* inv(A'*A + R);
nees_xc = mean(sum((ErrorXcp) / P_c .* (ErrorXcp), 2));

P_s = X_HRM*HRM_k* A'*P0 * A *HRM_k'*X_HRM';
nees_xs = mean(sum((ErrorXsp) / P_s .* (ErrorXsp), 2));

% Each point error of nees
nees_point_ls = sum((ErrorXLp) / P_ls .* (ErrorXLp), 2);
nees_point_xc = sum((ErrorXcp) / P_c .* (ErrorXcp), 2);
nees_point_xs = sum((ErrorXsp) / P_s .* (ErrorXsp), 2);
end
