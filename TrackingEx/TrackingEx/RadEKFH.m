% This function computes the H matrix as the jacobian of h = z(x)
%
% INPUT 
% - predicted state vector position elements (ENU),
% - state vector length (n)
% OUTPUT 
% - H = dh/dx     3 x n matrix
%

function H = RadEKFhJacobian(xpred,ypred,zpred,n)

H = zeros(3,n);
H(1,1) = xpred/sqrt((xpred^2)+(ypred^2)+(zpred^2));
H(1,3) = ypred/sqrt((xpred^2)+(ypred^2)+(zpred^2));
H(1,5) = zpred/sqrt((xpred^2)+(ypred^2)+(zpred^2));
H(2,1) = (-ypred)/((xpred^2)+(ypred^2));
H(2,3) = (xpred)/((xpred^2)+(ypred^2));
H(3,1) = -(zpred*xpred)/(sqrt((xpred^2)+(ypred^2))*((xpred^2)+(ypred^2)+(zpred^2)));
H(3,3) = -(zpred*ypred)/(sqrt((xpred^2)+(ypred^2))*((xpred^2)+(ypred^2)+(zpred^2)));
H(3,5) = sqrt((xpred^2)+(ypred^2))/((xpred^2)+(ypred^2)+(zpred^2));

end