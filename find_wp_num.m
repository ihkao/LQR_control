%% find current waypoint number
function [cur_wp_num, wp10] = find_wp_num(cur_wp_num, path, pose)
    dis = [];
    for i = cur_wp_num:cur_wp_num + 10
        dis = [dis, norm(pose(1:2) - path(i, 1:2))];
    end
    [C, I] = min(dis);
    cur_wp_num = cur_wp_num + I - 1;
    wp10 = path(cur_wp_num:cur_wp_num + 10, 1:2);

    for i = 1:length(wp10)
        [wp10(i, 1), wp10(i, 2)] = global2local(wp10(i, 1), wp10(i, 2), pose(1), pose(2), pose(3));
    end
    ref_yaw = [];
    ref_delta = [];
    for i = 1:length(wp10)
        if i == 1
            dx = wp10(i + 1, 1) - wp10(i, 1);
            dy = wp10(i + 1, 2) - wp10(i, 2);
            ddx = wp10(3, 1) + wp10(1, 1) - 2 * wp10(2, 1);
            ddy = wp10(3, 2) + wp10(1, 2) - 2 * wp10(2, 2);
        elseif i == length(wp10)
            dx = wp10(i, 1) - wp10(i - 1, 1);
            dy = wp10(i, 2) - wp10(i - 1, 2);
            ddx = wp10(i, 1) + wp10(i - 2, 1) - 2 * wp10(i - 1, 1);
            ddy = wp10(i, 2) + wp10(i - 2, 2) - 2 * wp10(i - 1, 2);
        else
            dx = wp10(i + 1, 1) - wp10(i, 1);
            dy = wp10(i + 1, 2) - wp10(i, 2);
            ddx = wp10(i + 1, 1) + wp10(i - 1, 1) - 2 * wp10(i, 1);
            ddy = wp10(i + 1, 2) + wp10(i - 1, 2) - 2 * wp10(i, 2);
        end
        ref_yaw = [ref_yaw; atan2(dy, dx)];
        ref_delta = [ref_delta; (ddy * dx - ddx * dy) / ((dx .^ 2 + dy .^ 2) .^ (3 / 2))];
    end
    wp10 = [wp10, ref_yaw, ref_delta];
end

%% transform function
function [x_local, y_local] = global2local(x_global, y_global, ref_x, ref_y, theta)
    % Compute the distance between the reference point and the global point
    delta_x = x_global - ref_x;
    delta_y = y_global - ref_y;
    
    % Rotate the global coordinates by the orientation of the local coordinate system
    x_local = delta_x * cos(theta) + delta_y * sin(theta);
    y_local = -delta_x * sin(theta) + delta_y * cos(theta);
end
