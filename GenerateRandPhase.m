function phase = GenerateRandPhase(Num_Layer, Num_Trajecotry,flag)
% function: flag := true, the phase is in a linspace; 
phase_init = rand(Num_Layer, Num_Trajecotry);

if flag
    num_phase = (Num_Trajecotry - 2)*(Num_Layer - 1);
    % rewrite the data into the phase_init.
    if num_phase < 1
        disp('There is no Similiar Engagement in terms of the milling case.')
    else
        phase_list = linspace(0, pi/2, num_phase);
        n = 1;
        for i = 2:Num_Layer
            for j = 2:Num_Trajecotry-1
                phase_init(i,j) = phase_list(n);
                n = n + 1;
            end
        end
    end
end
phase = phase_init;
end