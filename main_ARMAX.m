% Trabalho_IdSys.m
clear
close all
clc

simu_Time = 0.05;
% Sanity Check
%{
wn      = 0.5;
Tau     = 0.8;
K       = 1;
Tsamp   = 0.1;
fprintf('K = %.2f, Tau = %.2f, wn = %.2f\n', K, Tau, wn)
theta   = get_theta_fromCont(wn, Tau, K, Tsamp);
alpha   = theta(3);
beta    = theta(1);
gamma   = theta(2);
cont_params = get_Cont_fromTheta(alpha, beta, gamma, Tsamp);
wn  = cont_params(1);
Tau = cont_params(2);
K   = cont_params(3);
fprintf('K = %.2f, Tau = %.2f, wn = %.2f\n', K, Tau, wn)
%}

% Allocating poles
A1          = 0.85;  % Dominant pole
A2          = 0.15;  % Dominated pole 
lambda      = A1*A2; % Initial Lambda
Tsamp       = 0.00001; 

theta       = get_theta_fromSys(100, 10, 10.^-1, 10.^-6, Tsamp); % vin, R, L, C, Tsamp
P           = eye(length(theta))*1000000.0;

theta_ARMAX = [theta;0];
P_ARMAX     = eye(length(theta_ARMAX))*1000000.0;

n_timesteps = round(simu_Time/Tsamp);
keep_y      = 3;     % How many past 'y' values must be kept doe to the model
keep_u      = 3;     % How many past 'u' values must be kept doe to the model


% How does the history logic work: saving incoming signal points in the first item 
%y_buffer = [y(k), y(k-1), y(k-2)];
%y(k)   => y_buffer(1);
%y(k-1) => y_buffer(2);
%y(k-2) => y_buffer(3);
%y(k-n) => y_buffer(n+1)
y_buffer      = [];  % initialize buffer
ref_buffer    = [];  % initialize buffer
u_buffer      = [];  % initialize buffer
res_buffer    = [];  % ARMAX 
pred_ARMAX_buffer = []; % ARMAX 

for k = 1:max(keep_y,keep_u)
    y_ref         = get_reference(k, Tsamp);
    ref_buffer    = [ref_buffer, y_ref];
    y_buffer      = [y_buffer, 0.0];
    u_buffer      = [u_buffer, 0.0];
    res_buffer    = [res_buffer, 0.0]; % ARMAX
    pred_ARMAX_buffer = [pred_ARMAX_buffer, 0.0];
end

v0 = 5; % 24
iL = 0;

% Visualization savings
y_evolution         = zeros(1, n_timesteps);
ref_evolution       = zeros(1, n_timesteps);
u_evolution         = zeros(1, n_timesteps);
y_pred_evolution    = zeros(1, n_timesteps);
y_resi_evolution    = zeros(1, n_timesteps);
theta_evolution     = zeros(length(theta), n_timesteps);
sys_param_evolution = zeros(length(theta), n_timesteps);
cov_evolution       = zeros(1, n_timesteps);
validation_start    = round(n_timesteps*0.8); % Iteration to stop updating theta and show preditions 
y_pred_evolution(1:validation_start-1)  = NaN;

for k = 1:n_timesteps
    fprintf('\nTimestep %d\n', k);
    % GET CLOSE LOOP REFERENCE SIGNAL
    y_ref       = get_reference(k,Tsamp);
    ref_buffer    = [y_ref,  ref_buffer(1:end-1)];
    fprintf('y_ref %f\n', y_ref);
    
    % UPDATE CONTROLLER BASED ON CURRENT ESTIMATION
    controller  = get_controller(theta_ARMAX, lambda);
    fprintf('controller: [%s]\n', num2str(controller, '%.4f, '))
    % GET COMMAND SIGNAL CALCULATED WITH NEW CONTROLLER
    command     = get_control_signal(controller, y_buffer, ref_buffer, u_buffer);
    %command      = 2; % Open Lopp
    fprintf('command %f\n', command);
    
    % GET PLANT OUTPUT: Inputs: iL(k-1), v0(k-1), u(k-d), Tsamp 
    [iL, vo_k] = sim_buck_linear(iL,y_buffer(1)+v0, command, Tsamp);
    y = vo_k-v0; % Removing initial condition
    fprintf('y %f\n', y);
    
    % Save signals for visualization
    y_evolution(k)          = y;
    ref_evolution(k)        = y_ref;
    u_evolution(k)          = command;
    theta_evolution(:,k)    = theta;
    
    % BUFFER PROCESSING SIGNALS: 
    y_buffer      = [y,      y_buffer(1:end-1)];
    u_buffer      = [command, u_buffer(1:end-1)];
    fprintf('ref_buffer: [%s]\n', num2str(ref_buffer, '%.4f, '))
    fprintf('y_buffer: [%s]\n', num2str(y_buffer, '%.4f, '))
    fprintf('u_buffer: [%s]\n', num2str(u_buffer, '%.4f, '))

    % DURING TRAINNING: 
    % UPDATE ESTIMATION BASED ON SIGNALS FOR NEXT ITERATION
    if k  < validation_start
        % Compute estimation
        phi         = get_Phi(y_buffer, u_buffer);
        estimation  = update_theta_P(theta, phi, y_buffer, P);
        theta       = estimation(:, 1);
        P           = estimation(:, 2:end);
        
        res              = y -theta.'*phi;
        res_buffer       = [res, res_buffer(1:end-1)];
        phi_ARMAX        = get_Phi_ARMAX(y_buffer, u_buffer, res_buffer);
        pred_ARMAX_buffer = [y-res, pred_ARMAX_buffer(1:end-1)];
        estimation_ARMAX = update_theta_P(theta_ARMAX, phi_ARMAX, pred_ARMAX_buffer, P_ARMAX); % Mudar para y_buffer
        theta_ARMAX      = estimation_ARMAX(:, 1);
        P_ARMAX          = estimation_ARMAX(:, 2:end);
    end


    % Verify estimation residual (Visualization)
    phi                 = get_Phi(y_buffer, u_buffer); 
    y_pred_evolution(k) = theta.'*phi;
    y_resi_evolution(k) = y_evolution(k) - y_pred_evolution(k);
    cov_evolution(k)    = trace(P);  
    % Translate estimation for continuos parameters (Visualization)
    beta  = theta(1);
    gamma = theta(2);
    alpha = theta(3);
    fprintf('beta = %f, gamma = %f, alpha = %f; for Tsamp = %f.\n', beta, gamma, alpha, Tsamp);
    cont_params = get_Cont_fromTheta(alpha, beta, gamma, Tsamp);
    wn  = cont_params(1);
    Tau = cont_params(2);
    K   = cont_params(3);
    fprintf('cont_params: wn = %f, Tau = %f, K = %f\n', wn, Tau, K);
    plant_params = get_plant_params(wn, Tau, K);
    Vin = plant_params(1);
    L   = plant_params(2);
    R   = plant_params(3);
    fprintf('plant_params: Vin = %f, L = %f, R = %f\n', Vin, L, R);
    sys_param_evolution(:, k) = plant_params.';

end


% Visualization:
t = Tsamp*(1:n_timesteps);

figure(1);
subplot(2,1,1);
plot(t, y_evolution, 'b-', 'LineWidth', 1.5); hold on;
plot(t, ref_evolution, 'r--', 'LineWidth', 1.5); hold on;
plot(t(1:validation_start), y_pred_evolution(1:validation_start), 'g--', 'LineWidth', 2); hold on;
plot(t(validation_start:end), y_pred_evolution(validation_start:end), 'y--', 'LineWidth', 2); hold on;
%xlabel('Time (s)');
ylabel('Output');
legend('y (output)', 'y_{ref} (reference)', 'y_{pred} (train)', 'y_{pred} (validation)');
title('System Output vs Reference');
grid on;

subplot(2,1,2);
plot(t, u_evolution, 'r-', 'LineWidth', 1.5);
%xlabel('Time (s)');
ylabel('Control input (u)');
title('Control Signal');
ylim([ ...
    min(min(u_evolution)*0.9, min(u_evolution)*1.1), ...
    max(max(u_evolution)*0.9, max(u_evolution)*1.1)] ...
    )
grid on;

figure(2);
subplot(2,1,1);
hold on;
n_theta = size(theta_evolution, 1);  % Número de parâmetros theta
styles = {'-', '--', '-.', ':'};      % Estilos de linha
colors = lines(n_theta);              % Cores distintas
legend_entries = cell(1, n_theta);
for i = 1:n_theta
    % Normaliza pelo valor final de theta(i)
    final_val = theta_evolution(i,end);
    theta_norm = theta_evolution(i,:) / final_val;

    % Plot
    plot(t, theta_norm, ...
         'LineStyle', styles{mod(i-1, length(styles)) + 1}, ...
         'Color', colors(i,:), ...
         'LineWidth', 1.5);
    
    % Prepara entrada para legenda: ex: \theta_1 (final=1.2345)
    legend_entries{i} = sprintf('\\theta_{%d} (final=%.4f)', i, final_val);
end
xlabel('Time(s)');
ylabel('$\frac{\theta}{\theta_{\mathrm{final}}}$', 'Interpreter', 'latex');
xlabel('Time (s)');
legend(legend_entries);
title('Normalized Theta Evolution');
grid on;

subplot(2,1,2);
hold on;
n_params = size(sys_param_evolution, 1);  % Número de parâmetros theta
styles = {'-', '--', '-.', ':'};      % Estilos de linha
colors = lines(n_params);              % Cores distintas
legend_entries = cell(1, n_params);
for i = 1:n_params
    % Normaliza pelo valor final de theta(i)
    final_val = sys_param_evolution(i,end);
    param_norm = sys_param_evolution(i,:) / final_val;

    % Plot
    plot(t, param_norm, ...
         'LineStyle', styles{mod(i-1, length(styles)) + 1}, ...
         'Color', colors(i,:), ...
         'LineWidth', 1.5);
    
    % Prepara entrada para legenda: ex: \theta_1 (final=1.2345)
    legend_entries{i} = sprintf('param_{%d} (final=%.6f)', i, final_val);
end
xlabel('Time(s)');
ylabel('$\frac{param}{param_{\mathrm{final}}}$', 'Interpreter', 'latex');
xlabel('Time (s)');
legend(legend_entries);
title('Normalized Evolution of System Parameters');
grid on;



figure(3);
subplot(2,1,1);
plot(t(1:validation_start), y_resi_evolution(1:validation_start), '-', 'LineWidth', 1.5); hold on;
plot(t(validation_start:end), y_resi_evolution(validation_start:end), '-', 'LineWidth', 1.5); hold on;
y_avg = mean(y_resi_evolution);
yline(y_avg, 'm:', 'LineWidth', 1.5); % Horizontal average line
legend('\xi_{train}','\xi_{vali}', sprintf('AVG =  (final=%.4f)', y_avg));
title('Estimation residual');
xlabel('Time (s)');
ylabel('Residual magnitude');
ylim([min(y_resi_evolution)*1.1 max(y_resi_evolution)*1.1])
grid on;
subplot(2,1,2);
fs      = 1 / Tsamp;
N       = length(y_resi_evolution(1:validation_start));
Y       = fft(y_resi_evolution(1:validation_start));
f_Hz    = linspace(0, fs/2, round(N/2)+1);
Y_mag   = abs(Y(1: round(N/2) +1));   % One-sided magnitude spectrum
Y_db    = 20*log10(Y_mag / max(Y_mag));  % Normalize to max magnitude
semilogx(f_Hz, Y_db, '-', 'LineWidth', 1.5); hold on;
N       = length(y_resi_evolution(validation_start:end));
Y       = fft(y_resi_evolution(validation_start:end));
f_Hz    = linspace(0, fs/2, round(N/2)+1);
Y_mag   = abs(Y(1: round(N/2) +1));   % One-sided magnitude spectrum
Y_db    = 20*log10(Y_mag / max(Y_mag));  % Normalize to max magnitude
semilogx(f_Hz, Y_db, '--', 'LineWidth', 1.5); hold on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('FFT of Validation Residual');
grid on;

figure(4);
h1 = histogram(y_resi_evolution(1:validation_start), ...
    'NumBins', 60, ...                         % Set bin width
    'FaceColor', [0, 0.4470, 0.7410], ...                        % Set color to red
    'Normalization', 'probability', ...          % Normalize to probability
    'EdgeColor', 'k', ...
    'FaceAlpha', 0.5);                           % Add black edges
hold on;
h2 = histogram(y_resi_evolution(validation_start:end), ...
    'NumBins', 60, ...                         % Set bin width
    'FaceColor', [0.8500, 0.3250, 0.0980], ...                        % Set color to red
    'Normalization', 'probability', ...          % Normalize to probability
    'EdgeColor', 'w', ...
    'FaceAlpha', 0.5);                           % Add black edges
title('Customized Histogram');
xlabel('Value');
ylabel('Probability');
legend([h1, h2], {'Train / While updating \theta', 'Validation / Not updating \theta'});



figure(5);
plot(t(1:validation_start), cov_evolution(1:validation_start), '-', 'LineWidth', 1.5); hold on;
y_avg = mean(cov_evolution);
yline(y_avg, 'm:', 'LineWidth', 1.5); % Horizontal average line
legend('trace_{train}','trace_{vali}');
title('Covariance matrix');
xlabel('Time (s)');
ylabel('Trace of covariance matrix');
ylim([min(cov_evolution)*1.1 max(cov_evolution)*1.1])
grid on;


% ---- Função vem depois do código principal ----

function ref = get_reference(k, Tsamp)
    t = (k - 1) * Tsamp;  % Convert step index to time
    if t < 0.01
        ref = 5000 * t;   % This gives ref = 50 at t = 0.01
    elseif t < 0.025
        ref = 50;
    else
        ref = 100;
    end
end
function ret = update_theta_P(theta, phi, y, P)
    ff = 0.98;
    
    [n, m] = size(phi);
    fprintf('phi size: %d x %d\n', n, m);
    [n, m] = size(P);
    fprintf('P size: %d x %d\n', n, m);
    K = P*phi/(ff +phi'*P*phi);
    [n, m] = size(K);
    fprintf('K size: %d x %d\n', n, m);
    y_k = y(1);
    theta = theta +K*(y_k -phi'*theta);
    [n, m] = size(theta);
    fprintf('theta size: %d x %d\n', n, m);
    P = P -K*phi'*P;
    ret = [theta, P]; % Return a nx(n+1) matrix
end 
function controller = get_controller(theta, lambda)
    c0 = 1;
    c1 = theta(1);
    c2 = theta(2);    
    controller  = [c0*lambda, c1*lambda, c2*lambda];
end
function command = get_control_signal(controller, y, y_ref, uc)
    c2 = controller(3);
    c1 = controller(2);
    c0 = controller(1);
    y_ref_k = y_ref(1);
    y_ref_k1 = y_ref(2);
    y_ref_k2 = y_ref(3);
    y_k = y(1);
    y_k1 = y(2);
    y_k2 = y(3);
    e   = y_ref_k - y_k;
    e_1 = y_ref_k1 - y_k1;
    e_2 = y_ref_k2 - y_k2;
    uc  = uc(1);
    command = uc + c2*e_2 + c1*e_1 + c0*e;
end 
function theta = get_theta_fromCont(wn, Tau, K, Tsamp)
    alpha = K*wn^2*Tsamp^2;
    beta  = 2*Tau*wn*Tsamp - 2;
    gamma = wn^2*Tsamp^2 - 2*Tau*wn*Tsamp + 1;
    theta = [beta; gamma; alpha];
end
function cont_params = get_Cont_fromTheta(alpha, beta, gamma, Tsamp)
    wn = sqrt((beta + gamma + 1) / Tsamp^2);
    Tau = (beta + 2) / (2 * wn * Tsamp);
    K = alpha / (wn^2 * Tsamp^2);
    cont_params = [wn, Tau, K];
end
function theta = get_theta_fromSys(vin, R, L, C, Tsamp)
    wn = 1/sqrt(L*C);
    Tau = sqrt(L*C) / (2*R*C);
    K = vin;
    theta = get_theta_fromCont(wn, Tau, K, Tsamp);
end

function plant_params = get_plant_params(wn, Tau, K)
    C   = 100*10^-6;
    Vin = K;
    L   = 1 / (C*wn^2);
    R   = sqrt(L*C)/(Tau*2*C);
    plant_params = [Vin, L, R];
end
function phi = get_Phi(y, u)
    y1 = y(2);
    y2 = y(3);
    u2 = u(3);
    phi = [-y1; -y2; u2];
end

function phi = get_Phi_ARMAX(y, u, res)
    y1   = y(2);
    y2   = y(3);
    u2   = u(3);
    res1 = res(1);
    phi  = [-y1; -y2; u2; -res1];
end