function [omega, v, stop] = guidance_control(x, y, theta, wpx, wpy)

    persistent wp_idx firstentry
    coder.extrinsic('wrapToPi');

    % initialise on first call
    if isempty(firstentry)
        firstentry = 0;
    end
    if isempty(wp_idx)
        wp_idx = 1;
    end
    if firstentry == 0
        wp_idx = 1;
        firstentry = 1;
    end

    N = numel(wpx);
    capture_radius = 2.0;   % metres — tune this
    k = 1.5;                % heading gain — tune this
    v_max = 2.0;            % m/s, well under the 5 m/s hard limit

    stop = 0;

    if wp_idx > N
        % mission complete: hold position, stop sim
        omega = 0;
        v = 0;
        stop = 1;
        return;
    end

    % bearing to current target waypoint
    dx = wpx(wp_idx) - x;
    dy = wpy(wp_idx) - y;
    dist = sqrt(dx^2 + dy^2);
    alpha = atan2(dy, dx);

    % heading error, wrapped to [-pi, pi]
    e = alpha - theta;
    e = wrapToPi(e);

    omega = k * e;
    omega = max(min(omega, pi/2), -pi/2);   % saturate to robot's turn-rate limit

    % speed law: slow down near the waypoint
    v = min(v_max, 0.5 * dist);
    v = max(v, 0);

    % waypoint capture / switching
    if dist < capture_radius
        wp_idx = wp_idx + 1;
    end
end