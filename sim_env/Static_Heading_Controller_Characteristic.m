%% Static Heading-Controller Characteristic

close all;
clear;
clc;

% Final controller parameters
K_theta   = 1.0;
omega_max = pi/2;     % rad/s

% Wrapped heading-error range: -pi to +pi
e_theta = linspace(-pi, pi, 1000);

% Proportional controller
omega_unsat = K_theta .* e_theta;

% Apply yaw-rate saturation
omega_cmd = min(max(omega_unsat, -omega_max), omega_max);

% Plot
figure;

plot(rad2deg(e_theta), omega_cmd, 'LineWidth', 1.6);
hold on;

% Show saturation limits
yline( omega_max, '--', '\omega_{max}');
yline(-omega_max, '--', '-\omega_{max}');

% Show zero axes
xline(0, ':');
yline(0, ':');

grid on;
box on;

xlabel('Wrapped Heading Error, e_\theta (deg)');
ylabel('Commanded Yaw Rate, \omega_{cmd} (rad/s)');
title('Static Heading-Controller Characteristic');

xlim([-180 180]);
ylim(1.15 .* [-omega_max omega_max]);

% Export for LaTeX/report
exportgraphics(gcf, ...
    'Static_Heading_Controller_Characteristic.pdf', ...
    'ContentType', 'vector');