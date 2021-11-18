%% function
% input : roll of G1 and pitch of G1
% theta 2 = -(pi/2 - alpha1), theta2Dot = 0, theta2Ddot = 0
% output : Force to prevent slip

function zList = DynamicsRP(rR1List, pR1List)
    zList = zeros(length(rR1List), length(pR1List));
    % friction coefficient
    u = 0.8;

    % maximum thrust on floor (N)
    fMaxFloor = 4.4 * 9.81;

    % maximum thrust on air (N)
    fMaxAir = 1.9 * 9.81;

    %%%%%%%%%%%%%%%%%%%%%
    % Parametr of robot %
    %%%%%%%%%%%%%%%%%%%%%

    % weight of robot (kg)
    m = [0.25; 0.25];

    % lenght of link (m)
    L1 = 0.15456;
    L2 = 0.15456;

    Lc1 = 0;
    Lc2 = L1;

    % initial degree of the link (radian)
    alpha1 = deg2rad(48.706);
    alpha2 = deg2rad(360 - 2 * 48.706);

    % moment of inertia (to be updated)
    I1 = zeros(3,3);
    I2 = zeros(3,3);

    Ilist = cat(3, I1, I2);

    % vector from origin of frame {i-1} to center of mass Ci, from body cord
    p0_c1 = [Lc1; 0; 0];
    p1_c2 = [Lc2; 0; 0];
    pi_1_c_i = cat(3, p0_c1, p1_c2);

    % vector from origion of frame {i} to center of mass Ci, from body cord
    p1_c1 = [-L1+Lc1; 0; 0];
    p2_c2 = [-L2+Lc2; 0; 0];
    pi_c_i = cat(3, p1_c1, p2_c2);

    % vector from origin of frame {i-1} to origin of frame {i}, from body cord
    p0_1 = [L1; 0; 0];
    p1_2 = [L2; 0; 0];
    pi_1_p_i = cat(3, p0_1, p1_2);
    
    % configuration
    thetalist = [0; -(pi/2)];
    % angular velocity of each joints
    dthetalist = [0; 0];
    % angular acceleration of each joints
    ddthetalist = [0; 0];

    Ftip = [0; 0; 0];
    Mtip = [0; 0; 0];
    
    for i = 1:length(rR1List)
        for j = 1:length(pR1List)     
           %% Parameter of the plane

            % orientation of {R_1} frame
            % roll pitch yaw
            rR1 = rR1List(1,i);
            pR1 = pR1List(1,j);
            yR1 = 0;

            % F frame (world frame) to R_1 frame (rotation of robot1)
            R01 = RPY2SO3(rR1, pR1, yR1);

            % frame conversion matrix
            R10 = transpose(R01);

            %% Thrust force

            % SO3 orientationi matrix of robot 1
            R01_SO3 = RPY2SO3(rR1, pR1, yR1);

            % gravitanional vector (m/s^2)
            g = [0;0;-9.81];
            g = R01_SO3' * g;

            % torque to stable the robot's configuration and wrench exerted on robot 1
            % in robot 1's frame
            [taulist, Wrench] = NE(thetalist, dthetalist, ddthetalist, g, ...
                        Mtip, Ftip, m, Ilist, pi_1_c_i, pi_c_i, pi_1_p_i);

            %% calculation for robot1 not to slip

            % force exerting on the robot in global cord
            fXY = sqrt(Wrench(4,1)^2 + Wrench(5,1)^2);
            fZ = Wrench(6,1);

            % calculating the critical thrust force to prevent slip
            fCritical = fZ + fXY / u;

            if fCritical < 0
               fCritical = 0; 
            end


            %% plot the result
            zList(j,i) = fCritical;
        end
    end
end