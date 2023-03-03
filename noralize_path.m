function new_odom = noralize_path(path)
    new_odom = [];
    cur_pos = path(1, 1:2);
    new_odom = [new_odom; cur_pos];
    for i = 1:length(path)
        if norm(cur_pos - path(i, 1:2)) > 10
            cur_pos = path(i, 1:2);
            new_odom = [new_odom; cur_pos];
        end
    end