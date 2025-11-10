%% compare_models_no_obstacles_methods.m
% Compare Simplified vs Accurate models under Runge-Kutta4 vs Forward Euler
% Shows BOTH: Position RMSE [m] and Position Error [%] (normalized wrt reference RMS)

%% ----- Configuration -----
models = ["Simplified", "Accurate"];
methods = ["rungekutta4", "forwardeuler"];
Ts_values = [0.1,0.075,0.060, 0.05, 0.01, 0.001];
T = 15;

[~, params_simplified] = robot_model_params("Simplified");
[~, params_accurate]   = robot_model_params("Accurate");

z0 = params_accurate.z0;
goals = [0;0];
TsLabel = @(Ts) char(strrep(sprintf("Ts_%.3f", Ts),'.','_'));

%% ----- Input torque function -----
u_fun = @(t) recorded_input_fun(t, u_history, Ts); % requires u_history in workspace

%% ----- Run all simulations -----
results = struct();
for meth = 1:length(methods)
    method = methods(meth);
    fprintf('\n=== Integration method: %s ===\n', method);

    for m = 1:length(models)
        model = models(m);
        [robot_model, params] = robot_model_params(model);
        z0 = params.z0;

        fprintf('Model: %s\n', model);
        for it = 1:length(Ts_values)
            Ts = Ts_values(it);
            time = 0:Ts:T;
            N = numel(time);

            z = zeros(numel(z0), N);
            z(:,1) = z0;
            u_hist = zeros(2,N);

            for k = 1:N
                t = (k-1)*Ts;
                u_hist(:,k) = u_fun(t);
            end

            for k = 2:N
                [z(:,k), u_act] = NumCalc(robot_model, (k-1)*Ts, z(:,k-1), u_hist(:,k-1), Ts, params, method);
                u_hist(:,k-1) = u_act;
            end

            Ts_str = TsLabel(Ts);
            results.(char(method)).(char(model)).(Ts_str).time = time;
            results.(char(method)).(char(model)).(Ts_str).z = z;
            results.(char(method)).(char(model)).(Ts_str).u = u_hist;

            fprintf('  Ts = %.3f -> done (N=%d)\n', Ts, N);
        end
    end
end

%% ----- Interpolation to finest grid -----
Ts_ref = min(Ts_values);
t_fine = 0:Ts_ref:T;
nf = numel(t_fine);

interp_results = struct();
for meth = 1:length(methods)
    method = char(methods(meth));
    for m = 1:length(models)
        model = char(models(m));
        for it = 1:length(Ts_values)
            Ts = Ts_values(it);
            Ts_str = TsLabel(Ts);
            data = results.(method).(model).(Ts_str);
            z = data.z; t = data.time;
            Z_interp = zeros(size(z,1), nf);
            for si = 1:size(z,1)
                Z_interp(si,:) = interp1(t, z(si,:), t_fine, 'linear', 'extrap');
            end
            interp_results.(method).(model).(Ts_str).Z = Z_interp;
        end
    end
end

%% ----- Compute position RMSE [m] and percent error [%] -----
% We'll compute:
%  pos_RMSE(method,model,Ts_idx) = sqrt(mean( (x_ref-x_curr)^2 + (y_ref-y_curr)^2 ))
%  pos_percent = 100 * pos_RMSE / pos_ref_rms
% where pos_ref_rms = sqrt(mean( x_ref.^2 + y_ref.^2 )) for that method+model reference.

pos_rmse_vs_Ts = struct();        % absolute RMSE [m]
pos_pct_vs_Ts  = struct();        % percent [%] normalized by ref RMS
pos_rmse_between_models = struct();
pos_pct_between_models  = struct();

for meth = 1:length(methods)
    method = char(methods(meth));
    for m = 1:length(models)
        model = char(models(m));
        % reference trajectory for this method+model is the one at Ts_ref
        ref_label = TsLabel(Ts_ref);
        Z_ref = interp_results.(method).(model).(ref_label).Z(1:2,:); % x,y
        % compute normalization scalar (RMS magnitude of reference position)
        pos_ref_rms = sqrt(mean(sum(Z_ref.^2,1)));  % >0 unless robot stays at origin
        if pos_ref_rms == 0
            pos_ref_rms = 1; % avoid divide-by-zero (degenerate), percent will be meaningless then
        end

        for it = 1:length(Ts_values)
            Ts = Ts_values(it);
            Ts_str = TsLabel(Ts);
            Z_curr = interp_results.(method).(model).(Ts_str).Z(1:2,:);
            err_xy = Z_ref - Z_curr; % difference to reference
            pos_rmse = sqrt(mean(sum(err_xy.^2,1))); % [m]
            pos_pct  = 100 * (pos_rmse / pos_ref_rms); % [%]

            pos_rmse_vs_Ts.(method).(model)(it) = pos_rmse;
            pos_pct_vs_Ts.(method).(model)(it)  = pos_pct;
        end
    end
end

% Inter-model (Simplified vs Accurate) errors
for meth = 1:length(methods)
    method = char(methods(meth));
    % Use Accurate at finest Ts as normalization reference for percent-between-models
    ref_label = TsLabel(Ts_ref);
    Za_ref = interp_results.(method).Accurate.(ref_label).Z(1:2,:);
    pos_ref_rms_between = sqrt(mean(sum(Za_ref.^2,1)));
    if pos_ref_rms_between == 0, pos_ref_rms_between = 1; end

    for it = 1:length(Ts_values)
        Ts = Ts_values(it);
        Ts_str = TsLabel(Ts);
        Zs = interp_results.(method).Simplified.(Ts_str).Z(1:2,:);
        Za = interp_results.(method).Accurate.(Ts_str).Z(1:2,:);
        D = Zs - Za;
        pos_rmse_between_models.(method)(it) = sqrt(mean(sum(D.^2,1)));
        pos_pct_between_models.(method)(it)  = 100 * (pos_rmse_between_models.(method)(it) / pos_ref_rms_between);
    end
end

%% ----- Plots: replace old % plots with dual RMSE+% plots -----
% 1) Discretization sensitivity per method : two-subplot figure (top RMSE[m], bottom %)
for meth = 1:length(methods)
    method = char(methods(meth));
    fig = figure('Name', sprintf('Discretization Sensitivity (%s)', method), 'NumberTitle','off');
    % top: absolute RMSE [m] for Simplified & Accurate
    subplot(2,1,1);
    bar_data = [pos_rmse_vs_Ts.(method).Simplified(:), pos_rmse_vs_Ts.(method).Accurate(:)];
    bar(Ts_values, bar_data);
    xlabel('Sampling Time Ts [s]'); ylabel('Position RMSE [m]');
    legend('Simplified','Accurate','Location','northwest'); grid on;
    title(sprintf('%s — Position RMSE vs Ts', method));

    % bottom: percent error [%]
    subplot(2,1,2);
    bar_data_pct = [pos_pct_vs_Ts.(method).Simplified(:), pos_pct_vs_Ts.(method).Accurate(:)];
    bar(Ts_values, bar_data_pct);
    xlabel('Sampling Time Ts [s]'); ylabel('Position Error [%]');
    legend('Simplified','Accurate','Location','northwest'); grid on;
    title(sprintf('%s — Position Error (normalized) vs Ts', method));
end

% 2) Inter-model (Simplified vs Accurate) : two-subplot figure (top RMSE, bottom %)
fig2 = figure('Name','Simplified vs Accurate Errors', 'NumberTitle','off');
subplot(2,1,1); hold on;
for meth = 1:length(methods)
    method = char(methods(meth));
    plot(Ts_values, pos_rmse_between_models.(method), '-o', 'LineWidth', 1.6, 'DisplayName', method); hold on;
end
xlabel('Sampling Time Ts [s]'); ylabel('Position RMSE [m]');
title('Simplified vs Accurate — Position RMSE'); grid on; legend('Location','best');

subplot(2,1,2); hold on;
for meth = 1:length(methods)
    method = char(methods(meth));
    plot(Ts_values, pos_pct_between_models.(method), '-o', 'LineWidth', 1.6, 'DisplayName', method); hold on;
end
xlabel('Sampling Time Ts [s]'); ylabel('Position Error [%]');
title('Simplified vs Accurate — Position Error (normalized)'); grid on; legend('Location','best');

%% --- Trajectories per method (unchanged) ---
for meth = 1:length(methods)
    method = char(methods(meth));
    figure('Name', sprintf('Trajectories - %s', method)); hold on; grid on; axis equal;
    for m = 1:length(models)
        model = char(models(m));
        for it = 1:length(Ts_values)
            Ts = Ts_values(it);
            Ts_str = TsLabel(Ts);
            Z = interp_results.(method).(model).(Ts_str).Z;
            plot(Z(1,:), Z(2,:), 'LineWidth', 1.4, 'DisplayName', sprintf('%s Ts=%.3f', model, Ts));
        end
    end
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Trajectories (%s)', method));
    legend('Location','bestoutside');
end

%% ----- Per-Ts comparison plots (Simplified vs Accurate) -----
for meth = 1:length(methods)
    method = char(methods(meth));
    for it = 1:length(Ts_values)
        Ts = Ts_values(it);
        Ts_str = TsLabel(Ts);
        Zs = interp_results.(method).Simplified.(Ts_str).Z;
        Za = interp_results.(method).Accurate.(Ts_str).Z;

        figure('Name', sprintf('%s - Ts %.3f Comparison', method, Ts));
        hold on; grid on; axis equal;
        plot(Zs(1,:), Zs(2,:), 'b-', 'LineWidth', 1.6, 'DisplayName', 'Simplified');
        plot(Za(1,:), Za(2,:), 'r--', 'LineWidth', 1.6, 'DisplayName', 'Accurate');
        xlabel('x [m]'); ylabel('y [m]');
        title(sprintf('Trajectory Comparison (%s, Ts=%.3f)', method, Ts));
        legend('Location','bestoutside');
    end
end

%% ----- Summary (both RMSE [m] and % ) -----
fprintf('\n\n=== Position RMSE [m] and Position Error [%%] ===\n');
for meth = 1:length(methods)
    method = char(methods(meth));
    fprintf('\n--- %s ---\n', upper(method));
    for it = 1:length(Ts_values)
        Ts = Ts_values(it);
        fprintf('Ts=%.3f | Simplified = %.4f m (%.2f%%) | Accurate = %.4f m (%.2f%%) | Inter-model = %.4f m (%.2f%%)\n', ...
            Ts, ...
            pos_rmse_vs_Ts.(method).Simplified(it), pos_pct_vs_Ts.(method).Simplified(it), ...
            pos_rmse_vs_Ts.(method).Accurate(it),  pos_pct_vs_Ts.(method).Accurate(it), ...
            pos_rmse_between_models.(method)(it), pos_pct_between_models.(method)(it));
    end
end

%% ======== Helper Function ========
function u = recorded_input_fun(t, u_history, Ts)
    % Replays previously recorded control inputs with interpolation
    N = size(u_history,2);
    t_vec = (0:N-1)*Ts;
    u1 = interp1(t_vec, u_history(1,:), t, 'linear', 'extrap');
    u2 = interp1(t_vec, u_history(2,:), t, 'linear', 'extrap');
    u = [u1; u2];
end
