% This function computes state covariance matrix (P) by following
% mathemathics from JRJ' computation.
% Remember state expression: [x, xdot, y, ydot, z, zdot]
%
% INPUTS 
% R,         radar measurement covariance matrix:
%            [sigma_range [m] 0            0; 
%                   0       sigma_az [rad] 0;
%                   0         0     sigma_el [rad]] 
% sigmavels, variances of velocities given as:
%            [sigma_zot; sigma_xdot; sigma_ydot] all in m/s
% meas,      measurement used for covariance determination,  given as:
%            [range [m]; az [rad]; el [rad]].
%

function P = RadEKFStateCov(R,sigmas_vel,meas)

    sigma_xdot = sigmas_vel(1);
    sigma_ydot = sigmas_vel(2);
    sigma_zdot = sigmas_vel(3);
    
    range = meas(1);
    az    = meas(2);
    el    = meas(3);
    
    sigma_range2 = R(1,1);
    sigma_az2    = R(2,2);
    sigma_el2    = R(3,3);
    
    % Formulas from JRJ' for position covariances
    sigmaxsq = (cos(el)*cos(az))^2*sigma_range2 + (range*sin(az)*cos(el))^2*sigma_az2 + (range*cos(az)*sin(el))^2*sigma_el2;
    
    sigmaxy = (sin(az)*cos(az)*cos(el)^2)*sigma_range2 - (range^2*sin(az)*cos(az)*cos(el)^2)*sigma_az2 + (range^2*sin(az)*cos(az)*sin(el)^2)*sigma_el2;
    
    sigmaxz = -(cos(az)*cos(el)*sin(el))*sigma_range2 + (range^2*cos(az)*cos(el)*sin(el))*sigma_el2;
    
    sigmaysq = (sin(az)*cos(el))^2*sigma_range2 + (range*cos(az)*cos(el))^2*sigma_az2 + (range*sin(az)*sin(el))^2*sigma_el2;
    
    sigmayz  = -(sin(az)*cos(el)*sin(el))*sigma_range2 + (range^2*sin(az)*cos(el)*sin(el))*sigma_el2;
    
    sigmazsq = sin(el)^2*sigma_range2 + (range*cos(el))^2*sigma_el2;
    
    % Initialize
    P = zeros(6);
    
    % Fill matrix in
    P(1,1) = sigmaxsq;
    P(1,3) = sigmaxy;
    P(1,5) = sigmaxz;
    
    P(3,1) = sigmaxy;
    P(3,3) = sigmaysq;
    P(3,5) = sigmayz;
    
    P(5,1) = sigmaxz;
    P(5,3) = sigmaxy;
    P(5,5) = sigmazsq;
    
    P(2,2) = sigma_xdot^2;
    P(4,4) = sigma_ydot^2;
    P(6,6) = sigma_zdot^2;
end

