%% Kinemetic model
function [X, Y, PSI, V] = update_state(X, Y, PSI, V, a, delta_f, L)
    X = X + V * cos(PSI) * 0.1;
    Y = Y + V * sin(PSI) * 0.1;
    PSI = PSI + V / L * tan(delta_f) * 0.1;
    V = V + a * 0.1;
end