%% Get state space
function [A, B] = state_space(ref_delta, ref_yaw, v, L)
    A = [1.0, 0.0, -v * 0.1 * sin(ref_yaw);
       0.0, 1.0, v * 0.1 * cos(ref_yaw);
       0.0, 0.0, 1.0];
    B = [0.1 * cos(ref_yaw), 0;
       0.1 * sin(ref_yaw), 0;
       0.1 * tan(ref_delta) / L, v * 0.1 /L * cos(ref_delta) * cos(ref_delta)];
end

