function [drdt, q] = FiveLink(t, r)
    
    % extract PUP position vectors
    r_FUB   = r(1:3);
    r_FLB   = r(4:6);
    r_FUIF  = r(7:9);
    r_FUIR  = r(10:12);
    r_FLIF  = r(13:15);
    r_FLIR  = r(16:18);
    r_FOT   = r(19:21);
    r_FIT   = r(22:24);
    r_CP_CH = r(25:27);
    dTravel = r(28);

    % 1st link (FUF)
    r_P1_CH = r_FUB - r_FUIF;
    r_P1_CP = r_FUB - r_CP_CH;
    
    % 2nd link (FUR)
    r_P2_CH = r_FUB - r_FUIR;
    r_P2_CP = r_FUB - r_CP_CH;
    
    % 3rd link (FLF)
    r_P3_CH = r_FLB - r_FLIF;
    r_P3_CP = r_FLB - r_CP_CH;
    
    % 4th link (FLR)
    r_P4_CH = r_FLB - r_FLIR;
    r_P4_CP = r_FLB - r_CP_CH;
    
    % 5th link (TIE)
    r_P5_CH = r_FOT - r_FIT;
    r_P5_CP = r_FOT - r_CP_CH;

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
    
    % generalised DoF
    dWheelbase  = q(1);
    dTrack      = q(2);
    dCamber     = q(3);
    dSpin       = q(4);
    dToe        = q(5);
    
    disp(["Cambergain: ", -dCamber/dTravel*180/pi]);

    %% Calculate velocity vectors
    
    % angular velocity
    w_A = [dCamber; dSpin; dToe];

    % translational velocity
    v_CP = [dWheelbase; dTrack; dTravel];
    
    % find veloctity vectors of each point
    v_P1 = v_CP + cross(w_A, r_P1_CP);
    v_P2 = v_CP + cross(w_A, r_P2_CP);
    v_P3 = v_CP + cross(w_A, r_P3_CP);
    v_P4 = v_CP + cross(w_A, r_P4_CP);
    v_P5 = v_CP + cross(w_A, r_P5_CP);

    % find derivative of state vector
    drdt = vertcat(v_P1, v_P3, zeros(12,1), v_P5, zeros(3,1), v_CP, 0);

    q = [q; dTravel];
    
    q = transpose(q);
end