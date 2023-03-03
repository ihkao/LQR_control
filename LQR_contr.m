%% LQR controller
function u = LQR_contr(robot_state, refer_path, A, B, Q, R)
    % x is position and heading error
    x = robot_state(1:3) - refer_path;

    P = cal_Ricatti(A, B, Q, R);
    K = -pinv(R + B.' * P * B) * B.' * P * A;
    u = K * x.';
    u = u(2);
end

%% Ricatti
function P_ = cal_Ricatti(A, B, Q, R)
    % Ricatti equation
    % Q is a semi-positive definite state weighting matrix, which is usually taken as a diagonal matrix; 
    % the larger the elements of the Q matrix, the hope that the tracking deviation can quickly approach zero;
    % R is a positive definite control weighting matrix, 
    % and the larger elements of the R matrix mean that the control input is expected to be as small as possible.
    
    % iteration initial value
    Qf = Q;
    P = Qf;
    % loop iteration
    for i = 1:1000  
        P_ = Q + A.' * P * A - A.' * P * B * pinv(R + B.' * P * B) * B.' * P * A;
        if (max(abs(P_ - P)) < 0.00005)
            break;
        end
        P = P_;
    end
end


