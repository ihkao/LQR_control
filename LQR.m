clear; clear all; clc;

%% QR parameter
Q = [1, 0, 0;
    0, 1, 0;
    0, 0, 1;];
R = [1, 0;
    0, 1];
V = 17;
L = 2;

%% read odom
new_odom = [];
path = csvread('odom.csv');
path = noralize_path(path);

%% kinematic model
dx = path(2, 1) - path(1, 1);
dy = path(2, 2) - path(1, 2);
pose = [path(1, 1:2) atan2(dy, dx), V];
cur_wp_num = 1;

y = [];
while(cur_wp_num ~= length(path) - 10)
    [cur_wp_num, wp10] = find_wp_num(cur_wp_num, path, pose);
    [A, B] = state_space(wp10(1, 4), wp10(1, 3), V, L);
    delta_u = LQR_contr([0, 0, 0], wp10(1, 1:3), A, B, Q, R);
    if delta_u > 0
        delta_u = min(delta_u, 0.7);
    else
        delta_u = max(delta_u, -0.7);
    end
    [pose(1), pose(2), pose(3), pose(4)] = update_state(pose(1), pose(2), pose(3), pose(4), 0, delta_u, L);
    y = [y; pose(:, 1:2)];
end

y = noralize_path(y);


%% draw road
scenario = drivingScenario('SampleTime',0.01);

roadcenters = path(:, 1:2);
lspec = lanespec(2);
road(scenario,roadcenters,'Lanes',lspec);
 
% v1 = vehicle(scenario, 'ClassID', 1, 'Velocity', V, 'PlotColor', [1, 0, 0]);
% smoothTrajectory(v1, path);

v2 = vehicle(scenario, 'ClassID', 1, 'Velocity', V, 'PlotColor', [0, 0, 1]);
smoothTrajectory(v2, y);

%plot(scenario, 'Waypoints', 'off', 'RoadCenters', 'off')
% chasePlot(v2);
% while advance(scenario)
%     pause(0.01);
% end

drivingScenarioDesigner(scenario);
