%%-----------------------------------------------------------------------%%
% filename:         Fivelink.m
% author(s):        Niek van Rossem
% Creation date:    23-10-2024
%%-----------------------------------------------------------------------%%

function [drdt, q] = FiveLink(t, r)
    %% Documentation
    % State space analysis of a generic five link suspension system. Will
    % take as an input an array with the position vectors of all the
    % hardpoints, and calculate their velocity vectors.
    %
    % INPUTS
    % ======
    % t: scalar
    %   Analysis current time.
    % r: 40x1 matrix
    %   Input vector
    %       r(1:3)      = location of outer point rod 1
    %       r(4:6)      = location of outer point rod 2
    %       r(7:9)      = location of outer point rod 3
    %       r(10:12)    = location of outer point rod 4
    %       r(13:15)    = location of outer point rod 5 (steering rod)
    %       r(16:18)    = location of outer point rod 6 (damper)
    %       r(19:21)    = location of outer point rod 1
    %       r(22:24)    = location of outer point rod 2
    %       r(25:27)    = location of outer point rod 3
    %       r(28:30)    = location of outer point rod 4
    %       r(31:33)    = location of outer point rod 5 (steering rod)
    %       r(34:36)    = location of outer point rod 6 (damper)
    %       r(37:39)    = location of contact patch
    %       r(40)       = wheel vertical velocity
    %
    % OUTPUTS
    % =======
    % drdt: 40x1 matrix
    %   Derivative of state space vector.
    %       For order see "r" above.
    % q: 6x1 matrix
    %   Vector containing all generalised speeds.
    %       q(1) = dWheelbase/dt
    %       q(2) = dTrack/dt
    %       q(3) = dCamber/dt
    %       q(4) = dSpin/dt
    %       q(5) = dToe/dt
    %       q(6) = dTravel/dt

    %% Extract data and prepare

    r_P1o   = r(1:3);
    r_P2o   = r(4:6);
    r_P3o   = r(7:9);
    r_P4o   = r(10:12);
    r_P5o   = r(13:15);
    r_P6o   = r(16:18);
    r_P1i   = r(19:21);
    r_P2i   = r(22:24);
    r_P3i   = r(25:27);
    r_P4i   = r(28:30);
    r_P5i   = r(31:33);
    %r_P6i   = r(34:36); % unused
    r_CP_O  = r(37:39);
    dTravel = r(40);

    % 1st link (FUF)
    r_P1_CH = r_P1o - r_P1i;    % from chassis
    r_P1_CP = r_P1o - r_CP_O;   % from contact patch
    
    % 2nd link (FUR)
    r_P2_CH = r_P2o - r_P2i;
    r_P2_CP = r_P2o - r_CP_O;
    
    % 3rd link (FLF)
    r_P3_CH = r_P3o - r_P3i;
    r_P3_CP = r_P3o - r_CP_O;
    
    % 4th link (FLR)
    r_P4_CH = r_P4o - r_P4i;
    r_P4_CP = r_P4o - r_CP_O;
    
    % 5th link (TIE)
    r_P5_CH = r_P5o - r_P5i;
    r_P5_CP = r_P5o - r_CP_O;

    % 6th link (SDS)
    %r_P6_CH = r_P6o - r_P6i; % unused
    r_P6_CP = r_P6o - r_CP_O;

    %% Convert to matrix form and solve
    
    % flip contact patch velocity for sweep (temporary)
    if t > 50 && t < 150
       dTravel = -dTravel;
    end    

    % matrix entries
    Q1 = [r_P1_CP(1); r_P2_CP(1); r_P3_CP(1); r_P4_CP(1); r_P5_CP(1)];
    Q2 = [r_P1_CP(2); r_P2_CP(2); r_P3_CP(2); r_P4_CP(2); r_P5_CP(2)];
    Q3 = [r_P1_CP(3); r_P2_CP(3); r_P3_CP(3); r_P4_CP(3); r_P5_CP(3)];
    Q4 = [r_P1_CH(1); r_P2_CH(1); r_P3_CH(1); r_P4_CH(1); r_P5_CH(1)];
    Q5 = [r_P1_CH(2); r_P2_CH(2); r_P3_CH(2); r_P4_CH(2); r_P5_CH(2)];
    Q6 = [r_P1_CH(3); r_P2_CH(3); r_P3_CH(3); r_P4_CH(3); r_P5_CH(3)];
    
    % create matrix
    A = [Q4, Q5, Q2.*Q6-Q3.*Q5, Q3.*Q4-Q1.*Q6, Q1.*Q5-Q2.*Q4];
    b = -dTravel*Q6;
    
    % solve for generalised speeds
    q = A\b;
    
    % generalised speeds
    dWheelbase  = q(1);
    dTrack      = q(2);
    dCamber     = q(3);
    dSpin       = q(4);
    dToe        = q(5);
    
    %% Calculate velocity vectors
    
    % angular velocity of wheel assembly
    w_A = [dCamber; dSpin; dToe];

    % translational velocity of contact patch
    v_CP = [dWheelbase; dTrack; dTravel];
    
    % find veloctity vectors of each suspension point
    v_P1 = v_CP + cross(w_A, r_P1_CP);
    v_P2 = v_CP + cross(w_A, r_P2_CP);
    v_P3 = v_CP + cross(w_A, r_P3_CP);
    v_P4 = v_CP + cross(w_A, r_P4_CP);
    v_P5 = v_CP + cross(w_A, r_P5_CP);
    v_P6 = v_CP + cross(w_A, r_P6_CP);

    % find derivative of state vector
    drdt = vertcat(v_P1, v_P2, v_P3, v_P4, v_P5, v_P6, zeros(18,1), v_CP, 0);

    % collect all generalised speeds in vector
    q = [q; dTravel]; append()
    q = transpose(q);
end