function quad_pid_lqr_research(trajType, R, v_ref, numLaps, caseLabel)
% quad_pid_lqr_research.m
% Planar quadrotor trajectory tracking: PID vs Integral-Augmented LQR
% PID & LQR weights inspired by published qxuadrotor papers.
%
% Usage:
%   quad_pid_lqr_research;
%   quad_pid_lqr_research('circle',3,1.0,3,'Slow circle (research gains)');

    if nargin < 1 || isempty(trajType),  trajType  = 'circle';  end
    if nargin < 2 || isempty(R),         R         = 3;         end
    if nargin < 3 || isempty(v_ref),     v_ref     = 1.8;       end
    if nargin < 4 || isempty(numLaps),   numLaps   = 3;         end
    if nargin < 5 || isempty(caseLabel), caseLabel = 'ResearchGains'; end

    close all; clc;

    %% Trajectory & simulation setup
    dt = 0.02;

    switch trajType
        case 'circle'
            period = 2*pi*R / v_ref;
        case 'square'
            period = 4*(2*R)/v_ref;
        otherwise
            error('Unknown trajectory type: %s', trajType);
    end

    T_total = numLaps * period;
    time    = 0:dt:T_total;
    N       = numel(time);

    %% Initial states
    x_PID   = -R;  y_PID   = -R;
    vx_PID  = 0;   vy_PID  = 0;

    x_LQR   = -R;  y_LQR   = -R;
    vx_LQR  = 0;   vy_LQR  = 0;

    %% ===== PID gains from literature (position channel) =====
    % Inspired by Baharuddin & Basri (2023): Trajectory Tracking of a
    % Quadcopter UAV using PID Controller (position x,y loop).
    % Typical reported gains (adapted to this planar model):
    Kp_xy = 35.0;
    Ki_xy = 14.5;
    Kd_xy = 9.0;

    int_err_x_PID  = 0; prev_err_x_PID = 0;
    int_err_y_PID  = 0; prev_err_y_PID = 0;
    int_limit = 5.0;

    %% ===== LQR gains: heavy position weighting, unit others =====
    A_aug = [ 0 0 1 0 0 0;
              0 0 0 1 0 0;
              0 0 0 0 0 0;
              0 0 0 0 0 0;
              1 0 0 0 0 0;
              0 1 0 0 0 0 ];
    B_aug = [ 0 0;
              0 0;
              1 0;
              0 1;
              0 0;
              0 0 ];

    % Inspired by quadrotor LQR work with extremely high weight on position
    % states and small/medium on others.
    Q_aug = diag([1e7, 1e7, 1, 1, 1, 1]);
    R_aug = diag([1, 1]);

    K_LQR_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

    %% Storage
    Xd      = zeros(1, N);  Yd      = zeros(1, N);
    X_PID   = zeros(1, N);  Y_PID   = zeros(1, N);
    X_LQR   = zeros(1, N);  Y_LQR   = zeros(1, N);

    int_ex = 0;
    int_ey = 0;

    %% ===== Main loop =====
    for k = 1:N
        t = time(k);

        % Desired trajectory
        switch trajType
            case 'circle'
                omega = v_ref / R;
                Xd(k) = R * cos(omega * t);
                Yd(k) = R * sin(omega * t);

            case 'square'
                period_sq = 4 * (2 * R) / v_ref;
                tau    = mod(t, period_sq);
                side_t = (2 * R) / v_ref;

                if tau < side_t
                    Xd(k) = -R + v_ref * tau;
                    Yd(k) = -R;
                elseif tau < 2 * side_t
                    Xd(k) =  R;
                    Yd(k) = -R + v_ref * (tau - side_t);
                elseif tau < 3 * side_t
                    Xd(k) =  R - v_ref * (tau - 2 * side_t);
                    Yd(k) =  R;
                else
                    Xd(k) = -R;
                    Yd(k) =  R - v_ref * (tau - 3 * side_t);
                end
        end

        % ----- PID -----
        err_x_PID = Xd(k) - x_PID;
        err_y_PID = Yd(k) - y_PID;

        int_err_x_PID = int_err_x_PID + err_x_PID * dt;
        int_err_y_PID = int_err_y_PID + err_y_PID * dt;

        int_err_x_PID = max(min(int_err_x_PID, int_limit), -int_limit);
        int_err_y_PID = max(min(int_err_y_PID, int_limit), -int_limit);

        der_err_x_PID = (err_x_PID - prev_err_x_PID) / dt;
        der_err_y_PID = (err_y_PID - prev_err_y_PID) / dt;

        prev_err_x_PID = err_x_PID;
        prev_err_y_PID = err_y_PID;

        ux_PID = Kp_xy * err_x_PID + Ki_xy * int_err_x_PID + Kd_xy * der_err_x_PID;
        uy_PID = Kp_xy * err_y_PID + Ki_xy * int_err_y_PID + Kd_xy * der_err_y_PID;

        x_PID  = x_PID + vx_PID * dt;
        y_PID  = y_PID + vy_PID * dt;
        vx_PID = vx_PID + ux_PID * dt;
        vy_PID = vy_PID + uy_PID * dt;

        X_PID(k) = x_PID;
        Y_PID(k) = y_PID;

        % ----- LQR -----
        ex = x_LQR - Xd(k);
        ey = y_LQR - Yd(k);

        int_ex = int_ex + ex * dt;
        int_ey = int_ey + ey * dt;

        xLQR_aug = [ex; ey; vx_LQR; vy_LQR; int_ex; int_ey];
        u_LQR_vec = -K_LQR_aug * xLQR_aug;
        ux_LQR    = u_LQR_vec(1);
        uy_LQR    = u_LQR_vec(2);

        x_LQR  = x_LQR + vx_LQR * dt;
        y_LQR  = y_LQR + vy_LQR * dt;
        vx_LQR = vx_LQR + ux_LQR * dt;
        vy_LQR = vy_LQR + uy_LQR * dt;

        X_LQR(k) = x_LQR;
        Y_LQR(k) = y_LQR;
    end

    %% ===== Metrics =====
    err_x_PID = Xd - X_PID;   err_y_PID = Yd - Y_PID;
    MAE_x_PID = mean(abs(err_x_PID));    MAE_y_PID  = mean(abs(err_y_PID));
    RMSE_x_PID = sqrt(mean(err_x_PID.^2)); RMSE_y_PID = sqrt(mean(err_y_PID.^2));

    err_x_LQR = Xd - X_LQR;   err_y_LQR = Yd - Y_LQR;
    MAE_x_LQR = mean(abs(err_x_LQR));    MAE_y_LQR  = mean(abs(err_y_LQR));
    RMSE_x_LQR = sqrt(mean(err_x_LQR.^2)); RMSE_y_LQR = sqrt(mean(err_y_LQR.^2));

    RMSE_PID_avg = mean([RMSE_x_PID,  RMSE_y_PID]);
    RMSE_LQR_avg = mean([RMSE_x_LQR,  RMSE_y_LQR]);
    MAE_PID_avg  = mean([MAE_x_PID,   MAE_y_PID]);
    MAE_LQR_avg  = mean([MAE_x_LQR,   MAE_y_LQR]);

    fprintf('\n=== Controller Metrics over %d laps (%s, research gains) ===\n', numLaps, caseLabel);
    fprintf('\n--- PID (research-based) ---\n');
    fprintf('MAE (X,Y):   %.4f, %.4f [m]\n', MAE_x_PID,  MAE_y_PID);
    fprintf('RMSE (X,Y):  %.4f, %.4f [m]\n', RMSE_x_PID, RMSE_y_PID);

    fprintf('\n--- LQR (heavy position Q_aug) ---\n');
    fprintf('MAE (X,Y):   %.4f, %.4f [m]\n', MAE_x_LQR,  MAE_y_LQR);
    fprintf('RMSE (X,Y):  %.4f, %.4f [m]\n', RMSE_x_LQR, RMSE_y_LQR);

    fprintf('\n=== Conclusion for this run (%s) ===\n', caseLabel);
    fprintf('Average PID  MAE = %.4f m, RMSE = %.4f m\n', MAE_PID_avg,  RMSE_PID_avg);
    fprintf('Average LQR  MAE = %.4f m, RMSE = %.4f m\n', MAE_LQR_avg, RMSE_LQR_avg);

    if (RMSE_LQR_avg < RMSE_PID_avg) && (MAE_LQR_avg < MAE_PID_avg)
        fprintf('LQR outperforms PID (research gains) on this scenario.\n');
    elseif (RMSE_LQR_avg <= RMSE_PID_avg) || (MAE_LQR_avg <= MAE_PID_avg)
        fprintf('LQR is comparable or slightly better than PID (research gains).\n');
    else
        fprintf('PID (research gains) outperforms this LQR tuning.\n');
    end

    %% ===== Animation (same as baseline) =====
    anim_pause = 0.005;
    step2D     = 3;

    allX = [Xd, X_PID, X_LQR];
    allY = [Yd, Y_PID, Y_LQR];
    margin = 0.5;
    x_min = min(allX) - margin; x_max = max(allX) + margin;
    y_min = min(allY) - margin; y_max = max(allY) + margin;

    figure('Name',sprintf('PID vs LQR Trajectory (%s research)', caseLabel));
    tiledlayout(1,3);

    ax1 = nexttile(1);
    hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
    xlim(ax1, [x_min, x_max]); ylim(ax1, [y_min, y_max]);
    title(ax1, 'Desired Trajectory');
    xlabel(ax1,'X (m)'); ylabel(ax1,'Y (m)');
    hTrailDes = animatedline(ax1, 'Color','k', 'LineStyle','--', 'LineWidth',1.5);
    hDroneDes = plot(ax1, Xd(1), Yd(1), 'ko', 'MarkerFaceColor','y', 'MarkerSize',7);
    legend(ax1, {'Desired path','Desired point'}, 'Location','best');

    ax2 = nexttile(2);
    hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');
    xlim(ax2, [x_min, x_max]); ylim(ax2, [y_min, y_max]);
    title(ax2, 'PID (research gains)');
    xlabel(ax2,'X (m)'); ylabel(ax2,'Y (m)');
    plot(ax2, Xd, Yd, 'Color',[0.7 0.7 0.7], 'LineStyle',':');
    hTrailPID = animatedline(ax2, 'Color',[0 0.4470 0.7410], 'LineWidth',1.5);
    hDronePID = plot(ax2, X_PID(1), Y_PID(1), 'ro', 'MarkerFaceColor','r', 'MarkerSize',7);
    legend(ax2, {'Desired path','PID path','PID drone'}, 'Location','best');

    ax3 = nexttile(3);
    hold(ax3, 'on'); grid(ax3, 'on'); axis(ax3, 'equal');
    xlim(ax3, [x_min, x_max]); ylim(ax3, [y_min, y_max]);
    title(ax3, 'LQR (research Q/R)');
    xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)');
    plot(ax3, Xd, Yd, 'Color',[0.7 0.7 0.7], 'LineStyle',':');
    hTrailLQR = animatedline(ax3, 'Color',[0.8500 0.3250 0.0980], 'LineWidth',1.5);
    hDroneLQR = plot(ax3, X_LQR(1), Y_LQR(1), 'mo', 'MarkerFaceColor','m', 'MarkerSize',7);
    legend(ax3, {'Desired path','LQR path','LQR drone'}, 'Location','best');

    for k = 1:step2D:N
        addpoints(hTrailDes, Xd(k), Yd(k));
        set(hDroneDes, 'XData', Xd(k), 'YData', Yd(k));

        addpoints(hTrailPID, X_PID(k), Y_PID(k));
        set(hDronePID, 'XData', X_PID(k), 'YData', Y_PID(k));

        addpoints(hTrailLQR, X_LQR(k), Y_LQR(k));
        set(hDroneLQR, 'XData', X_LQR(k), 'YData', Y_LQR(k));

        drawnow;
        pause(anim_pause);
    end

    %% 3D
    figure('Name',sprintf('3D Quadrotor (research) - %s', caseLabel));
    hold on; grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('3D Quadrotor (PID=blue, LQR=red) - %s', caseLabel));
    plot3(Xd, Yd, zeros(size(Xd)), 'k--', 'LineWidth', 1.5);
    axis([x_min x_max y_min y_max -1 1]);
    view(45,25);

    [pid_arm1, pid_arm2, pid_motors] = draw_quad(X_PID(1), Y_PID(1), 0.2, 0, 'b', 'r');
    [lqr_arm1, lqr_arm2, lqr_motors] = draw_quad(X_LQR(1), Y_LQR(1), -0.2, 0, 'r', 'r');

    trailPID3D = animatedline('Color','b','LineWidth',1.2);
    trailLQR3D = animatedline('Color','r','LineWidth',1.2);

    step3D  = 3;  pause3D = 0.04;

    for k = 2:step3D:N
        yawPID = atan2(Y_PID(k) - Y_PID(k-1), X_PID(k) - X_PID(k-1) + 1e-6);
        yawLQR = atan2(Y_LQR(k) - Y_LQR(k-1), X_LQR(k) - X_LQR(k-1) + 1e-6);

        update_quad(pid_arm1, pid_arm2, pid_motors, X_PID(k), Y_PID(k), 0.2, yawPID);
        update_quad(lqr_arm1, lqr_arm2, lqr_motors, X_LQR(k), Y_LQR(k), -0.2, yawLQR);

        addpoints(trailPID3D, X_PID(k), Y_PID(k), 0.2);
        addpoints(trailLQR3D, X_LQR(k), Y_LQR(k), -0.2);

        drawnow;
        pause(pause3D);
    end
end
