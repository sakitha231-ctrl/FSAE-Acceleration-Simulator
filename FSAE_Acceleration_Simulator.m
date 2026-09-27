%% FSAE VEHICLE ACCELERATION SIMULATOR
% Formula Student Engineering Project
% Author: Sakitha De Zoysa

clear;
clc;
close all;

%% VEHICLE PARAMETERS

m = 250;              % Vehicle mass (kg)
P = 80000;            % Motor power (W)
r = 0.23;             % Wheel radius (m)

mu = 1.2;             % Tyre-road friction coefficient

Cd = 0.9;             % Aerodynamic drag coefficient
A = 1.2;              % Frontal area (m^2)

Crr = 0.015;          % Rolling resistance coefficient

rho = 1.225;          % Air density (kg/m^3)
g = 9.81;             % Gravitational acceleration (m/s^2)


%% SIMULATION TIME

dt = 0.01;            % Time step (seconds)
t_end = 20;           % Total simulation time (seconds)

t = 0:dt:t_end;


%% CREATE ARRAYS

v = zeros(size(t));   % Velocity (m/s)
x = zeros(size(t));   % Distance (m)
a = zeros(size(t));   % Acceleration (m/s^2)


%% TYRE GRIP

% Maximum force that the tyres can transmit to the road

F_tyre_max = mu * m * g;


%% VEHICLE SIMULATION

for i = 1:length(t)-1

    % Current vehicle velocity
    v_current = v(i);

    % Prevent division by zero at the start
    v_safe = max(v_current,0.1);

    % Motor tractive force
    F_motor = P / v_safe;

    % Limit motor force according to tyre grip
    F_drive = min(F_motor,F_tyre_max);

    % Aerodynamic drag force
    F_drag = 0.5 * rho * Cd * A * v_current^2;

    % Rolling resistance force
    F_rr = Crr * m * g;

    % Total net force
    F_net = F_drive - F_drag - F_rr;

    % Newton's Second Law
    a(i) = F_net / m;

    % Update velocity
    v(i+1) = v(i) + a(i) * dt;

    % Prevent negative velocity
    v(i+1) = max(v(i+1),0);

    % Update distance
    x(i+1) = x(i) + v(i) * dt;

end


%% CONVERT SPEED TO KM/H

v_kmh = v * 3.6;


%% 0-60 KM/H TIME

idx60 = find(v_kmh >= 60,1);

if ~isempty(idx60)

    time_60 = t(idx60);

else

    time_60 = NaN;

end


%% 0-100 KM/H TIME

idx100 = find(v_kmh >= 100,1);

if ~isempty(idx100)

    time_100 = t(idx100);

else

    time_100 = NaN;

end


%% MAXIMUM SPEED

max_speed = max(v_kmh);


%% FINAL DISTANCE

final_distance = x(end);


%% DISPLAY RESULTS

fprintf('\n');
fprintf('========================================\n');
fprintf(' FSAE VEHICLE ACCELERATION SIMULATOR\n');
fprintf('========================================\n');

fprintf('Vehicle mass: %.1f kg\n',m);
fprintf('Motor power: %.1f kW\n',P/1000);
fprintf('Tyre friction coefficient: %.2f\n',mu);

fprintf('\n');

fprintf('0-60 km/h time: %.2f seconds\n',time_60);
fprintf('0-100 km/h time: %.2f seconds\n',time_100);
fprintf('Maximum speed: %.2f km/h\n',max_speed);
fprintf('Distance after %.1f seconds: %.2f m\n',t_end,final_distance);

fprintf('========================================\n');


%% GRAPH 1 - SPEED VS TIME

figure;

plot(t,v_kmh,'LineWidth',2);

xlabel('Time (s)');
ylabel('Speed (km/h)');

title('FSAE Vehicle Speed vs Time');

grid on;


%% GRAPH 2 - ACCELERATION VS TIME

figure;

plot(t,a,'LineWidth',2);

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

title('FSAE Vehicle Acceleration vs Time');

grid on;


%% GRAPH 3 - DISTANCE VS TIME

figure;

plot(t,x,'LineWidth',2);

xlabel('Time (s)');
ylabel('Distance (m)');

title('FSAE Vehicle Distance vs Time');

grid on;


%% GRAPH 4 - MOTOR FORCE VS SPEED

F_motor_plot = zeros(size(v));

for i = 1:length(v)

    v_safe = max(v(i),0.1);

    F_motor_plot(i) = P / v_safe;

end

% Apply tyre grip limit
F_drive_plot = min(F_motor_plot,F_tyre_max);

figure;

plot(v_kmh,F_drive_plot,'LineWidth',2);

xlabel('Speed (km/h)');
ylabel('Driving Force (N)');

title('Driving Force vs Vehicle Speed');

grid on;


%% GRAPH 5 - AERODYNAMIC DRAG

F_drag_plot = 0.5 * rho * Cd * A .* v.^2;

figure;

plot(v_kmh,F_drag_plot,'LineWidth',2);

xlabel('Speed (km/h)');
ylabel('Aerodynamic Drag (N)');

title('Aerodynamic Drag vs Vehicle Speed');

grid on;


%% END

fprintf('\nSimulation completed successfully.\n');