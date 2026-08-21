% Ground Robot parameters 
%
%
% Properties:
%
% This is a structure with the following elements:
%
% X       initial position X (1x1)
% Y       initial position Y (1x1)
% Theta   initial orientation (1x1)

%
% Notes::
% - SI units are used.
%


robot.X = 0;                  
robot.Y = 0;                   
robot.Theta = 0;                
coord_lo = -100;
coord_hi = 100;
no_wps = 10;
%% Mission planner
wp_list                = wp_gen(coord_lo, coord_hi, no_wps);
[wp_ordered, tour_len] = wp_antColony(wp_list, [robot.X robot.Y]);
wp_plot                = [robot.X robot.Y; wp_ordered];   % tour incl. first leg