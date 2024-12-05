function [Car, PUP] = Suh_PUP(Settings)
    
    % set static ride height
    Car.RH = 40;  % mm

    if Settings.Axle == "Front"

        % contact patch
        PUP.r_CP_O    = [ -54.137;     718.938;   -319.000];
        
        Car.w_tr_f = PUP.r_CP_O(2);
        
        % wheel centre
        PUP.r_WC_O    = [ -54.137;     718.938;    -45.000];
        
        % suspension PUPs
        PUP.r_P1o   = [ -49.496;     603.805;     71.853];
        PUP.r_P2o   = [ -49.496;     603.805;     71.853];
        PUP.r_P3o   = [ -58.341;     624.368;   -150.141];
        PUP.r_P4o   = [ -58.341;     624.368;   -150.141];
        PUP.r_P5o   = [-174.166;     633.511;    -98.290];
        PUP.r_P6o   = [ -58.341;     624.368;   -150.141];
        PUP.r_P1i   = [-127.000;     383.250;    106.200];
        PUP.r_P2i   = [  32.000;     383.250;     95.200];
        PUP.r_P3i   = [-358.000;     314.750;    -67.600];
        PUP.r_P4i   = [ -55.650;     232.200;   -110.650];
        PUP.r_P5i   = [-158.000;     360.500;    -66.600];
        PUP.r_P6i   = [ -58.341;     604.368;   -100.141];

        % correct Z coordinate
        fn = fieldnames(PUP);
        for n = 1:numel(fn)
            PUP.(fn{n})(3) = PUP.(fn{n})(3) + 319.000;
        end


    elseif Settings.Axle == "Rear"
        disp("Rear axle not supported for this dataset");

    else
        disp("Invalid axle setting");
    end
end