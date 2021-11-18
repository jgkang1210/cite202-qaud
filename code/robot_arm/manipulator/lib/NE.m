function [taulist, Wrench] = NE(thetalist, dthetalist, ddthetalist, g, ...
            Mtip, Ftip, m, Ilist, pi_1_c_i, pi_c_i, pi_1_p_i)
% Takes thetalist: n-vector of joint variables,
%       dthetalist: n-vector of joint rates,
%       ddthetalist: n-vector of joint accelerations,
%       g: Gravity vector g,
%       Mtip: Moment applied by the end-effector expressed in frame 
%             {n+1},       
%       Ftip: Force applied by the end-effector expressed in frame 
%             {n+1},
%       m : mass lists of robots
%       Ilist: Inertia matrices Ii of the links,
%       pi_1_c_i : Vector that starts from origin i-1 frame to com of
%             i th link in i th frame
%       pi_c_i : Vector that starts from origin of p{i} to center of
%             i th link in i th frame
%       pi_1_c_i : Vector that starts from origin of i-1 frame to origin of
%             i th frame
% Returns taulist: The n-vector of required joint forces/torques.
%         Wrench: Final wrench in first frame
% Solve the newton euler-formulation for n-link dynamics
% with previous joint's info & next joint's info calculate force, torque
% forwardly and backwardly

    n = size(thetalist, 1);
    
    % starts from {0} cord
    wi = [0; 0; 0];
    dwi = [0; 0; 0];
    % dpi = [0; 0; 0];
    ddpi = -g;
    
    % set rotation axis for each joints
    z0 = [0; 0; 1];
    z1 = [0; 0; 1];
    z2 = [0; 1; 0];
    zi = [z1 z2];
    
    % first Fi will be F tib
    Fi = Ftip;
    Ni = Mtip;
    
    % force and moment by forward recursion
    F_hatlist = zeros(3,n);
    N_hatlist = zeros(3,n);
    
    taulist = zeros(n,1);

    % length check
    for i=1:n
        if pi_1_c_i(:,:,i) == pi_c_i(:,:,i) + pi_1_p_i(:,:,i)
            % disp("Check ok");
            % disp(i);
        else
            disp("Please correct your link vectors");
            disp(i);
        end
    end
    
    % create rotation matrix
    
    % for our robot link 1 has offset angle
    alpha1 = -1*deg2rad(48.706);
    alpha2 = deg2rad(49.706 + 90);
    
    R = zeros(3,3,n+1);
    
    R01 = [
            cos(thetalist(1)) -sin(thetalist(1)) 0;
            sin(thetalist(1)) cos(thetalist(1)) 0;
            0 0 1];
        
    R01 = R01 * [
            cos(alpha1) 0 sin(alpha1);
            0 1 0;
            -sin(alpha1) 0 cos(alpha1)];
        
    R12 = [cos(thetalist(2)) 0 sin(thetalist(2));
            0 1 0;
            -sin(thetalist(2)) 0 cos(thetalist(2))];
        
    R12 = R12 * [
            cos(alpha2) 0 sin(alpha2);
            0 1 0;
            -sin(alpha2) 0 cos(alpha2)];

    R(:,:,1) = R01;
    R(:,:,2) = R12;
    R(:,:,3) = eye(3);
    
    % Acceleration calculation
    for i = 1:n
        wi1 = wi;
        wi = R(:,:,i)'*(wi1 + dthetalist(i)*zi(:,i));
        dwi = R(:,:,i)'*(dwi + ddthetalist(i)*zi(:,i) ...
            + cross(dthetalist(i)*wi1, zi(:,i)));
        
        % dpi = dpi + cross(wi, pi_1_p_i(:,:,i));
        ddpi = R(:,:,i)'*ddpi + cross(dwi, pi_1_p_i(:,:,i)) ...
                + cross(wi, cross(wi, pi_1_p_i(:,:,i)));
            
        ddpi_c = ddpi + cross(dwi, pi_c_i(:,:,i)) ...
                + cross(wi, cross(wi, pi_c_i(:,:,i)));
            
        fi_hat = m(i) * ddpi_c;
        ni_hat = Ilist(:, :, i)*dwi + cross(wi, Ilist(:, :, i)*wi);
        
        F_hatlist(:,i) = fi_hat;
        N_hatlist(:,i) = ni_hat;
    end
    
    % Force calculation
    for i = n:-1:1
        Fi1 = Fi;
        Fi = F_hatlist(:, i) + R(:,:,i+1)*Fi1;
        Ni = N_hatlist(:, i) - cross(Fi, (pi_1_p_i(:,:,i)+pi_c_i(:,:,i))) ...
            + R(:,:,i+1)*Ni + cross(R(:,:,i+1)*Fi1, pi_c_i(:,:,i));
        taui = dot(Ni, zi(:,i));
        taulist(i) = taui;
    end
    
    % to get the final wrench exerted on the robot 1 in robot 1 cord.
    Nf = R01 * Ni;
    Ff = R01 * Fi;
    
    Wrench = -[Nf; Ff];
end