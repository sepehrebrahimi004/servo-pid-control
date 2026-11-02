%RUN_ALL  PID position control of a DC servo motor: root-locus design and
%         comparison of the PID, PI-D and I-PD structures.
%
%   Plant:  Gp(s) = 60.5634 / (s (s + 39.6422))
%   Specs:  settling time ts = 0.1 s (2 %), overshoot < 5 %, one PID zero at -10
%   Requires the Control System Toolbox (and Simulink for the cross-check).

clear; close all; clc;
here = fileparts(mfilename('fullpath'));
addpath(here);
s  = tf('s');
Gp = 60.5634 / (s*(s + 39.6422));
N  = 100;                                     % derivative filter, as in the Simulink blocks

%% 1. Desired poles
PO = 4.6;  ts = 0.1;                          % aim slightly inside the 5 % limit
zeta  = -log(PO/100) / sqrt(pi^2 + log(PO/100)^2);
sigma = 4/ts;                                 % 2 % settling: ts = 4 / (zeta wn)
wd    = sigma * tan(acos(zeta));
sd    = -sigma + 1j*wd;
fprintf('desired poles: zeta = %.3f, s_d = %.2f +/- j%.2f\n', zeta, real(sd), wd);

%% 2-5. Root-locus design: Gc = K (s + z1)(s + z2) / s
% With Gc = K/s alone the locus of Gp/s never reaches s_d (the angle condition
% fails), so two zeros are needed. z1 = 10 is given; z2 follows from the angle
% condition and K from the magnitude condition.
z1 = 10;
Gfun = @(x) 60.5634 / (x*(x + 39.6422));
phi_needed = pi - (angle(sd + z1) - angle(sd) + angle(Gfun(sd)));
phi_needed = mod(phi_needed + pi, 2*pi) - pi;              % angle of (s_d + z2)
z2 = -real(sd) + imag(sd) / tan(phi_needed);
K  = 1 / abs((sd + z1)*(sd + z2)/sd * Gfun(sd));
Kp = K*(z1 + z2);  Ti = (z1 + z2)/(z1*z2);  Td = 1/(z1 + z2);
fprintf('z2 = %.2f, K = %.4f  ->  Kp = %.3f, Ti = %.4f s, Td = %.4f s\n', z2, K, Kp, Ti, Td);
fprintf('parallel form: P = %.3f, I = Kp/Ti = %.2f, D = Kp*Td = %.4f\n', Kp, Kp/Ti, Kp*Td);

f = figure('Visible', 'off', 'Position', [100 100 700 520]);
rlocus(K*(s + z1)*(s + z2)/s * Gp / K); hold on;
plot(real(sd), [imag(sd) -imag(sd)], 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
xlim([-120 10]); ylim([-80 80]);
title(sprintf('Root locus of (s+%g)(s+%.1f)G_p(s)/s; K = %.3f places poles at s_d', z1, z2, K));
save_figure(f, 'root_locus');

%% Controller structures (2-DOF form U = Cr*R - Cy*Y, disturbance at the plant input)
D  = Td*s * N/(s + N);                                      % filtered derivative
structures = {
    'PID',  @(kp, ti, td) kp*(1 + 1/(ti*s) + td*s*N/(s+N))
    'PI-D', @(kp, ti, td) kp*(1 + 1/(ti*s))
    'I-PD', @(kp, ti, td) kp/(ti*s)
};
Cy = @(kp, ti, td) kp*(1 + 1/(ti*s) + td*s*N/(s+N));        % identical for all three

designs = {
    'corrected design',         Kp,      Ti,           Td
    'original gains (Kp only)', 52.5816, 1/8.7873,     0.0121
};

t = linspace(0, 0.5, 2001);
fprintf('\n%-26s %-5s %8s %8s %8s %10s\n', 'gains', 'type', 'PO (%)', 'tr (s)', 'ts (s)', 'dist. peak');
results = struct();
for d = 1:size(designs, 1)
    [name, kp, ti, td] = designs{d, :};
    f = figure('Visible', 'off', 'Position', [100 100 1100 420]);
    for k = 1:size(structures, 1)
        Cr = structures{k, 2}(kp, ti, td);
        T  = minreal(Gp*Cr / (1 + Gp*Cy(kp, ti, td)));
        Td_ = minreal(Gp / (1 + Gp*Cy(kp, ti, td)));         % disturbance -> output
        y  = step(T, t);
        yd = 0.1 * step(Td_, t);
        info = stepinfo(y, t, 1, 'SettlingTimeThreshold', 0.02);
        fprintf('%-26s %-5s %8.1f %8.3f %8.3f %10.4f\n', name, structures{k, 1}, ...
                info.Overshoot, info.RiseTime, info.SettlingTime, max(abs(yd)));
        subplot(1, 2, 1); plot(t, y, 'LineWidth', 1.5); hold on;
        subplot(1, 2, 2); plot(t, 1e3*yd, 'LineWidth', 1.5); hold on;
        results(d, k).name = structures{k, 1};
        results(d, k).info = info;
    end
    subplot(1, 2, 1); yline(1.05, 'k:', '+5 %'); grid on; xlabel('time (s)'); ylabel('position');
    title(['Unit step, ' name]); legend(structures(:, 1), 'Location', 'southeast');
    subplot(1, 2, 2); grid on; xlabel('time (s)'); ylabel('output deviation (x 10^{-3})');
    title('Response to a 0.1 step disturbance: identical for all three');
    legend(structures(:, 1), 'Location', 'northeast');
    save_figure(f, sprintf('step_%s', regexprep(name, '\W+', '_')));
end

%% Why the original "PID" response differed: the integral gain in parallel form
P = 52.5816;
pid_as_built = P + 8.7885/s + 0.637*s*N/(s+N);               % I entered as 1/Ti
pid_intended = P + P*8.7873/s + P*0.0121*s*N/(s+N);          % I = Kp/Ti, D = Kp*Td
f = figure('Visible', 'off', 'Position', [100 100 650 400]);
step(feedback(pid_as_built*Gp, 1), pid_intended*Gp/(1 + pid_intended*Gp), feedback(Cy(Kp, Ti, Td)*Gp, 1), 0.5);
grid on; legend('as built (I = 1/T_i = 8.79)', 'intended (I = K_p/T_i = 462)', 'corrected design (K = 0.85)');
title('Parallel PID: the I and K entries matter');
save_figure(f, 'pid_gain_check');

%% Cross-check against the original Simulink model (PID branch)
try
    mdl = 'servo_pid_original';
    load_system(fullfile(here, 'simulink', [mdl '.slx']));
    set_param(mdl, 'StopTime', '0.5');
    out = sim(mdl, 'SaveOutput', 'on');
    fprintf('\nSimulink model simulated (StopTime 0.5 s).\n');
    yref = step(feedback(pid_as_built*Gp, 1), 0.3);
    fprintf('as-built PID from transfer functions: overshoot %.1f %% (Simulink scope in the report: 14 %%)\n', ...
            100*(max(yref) - 1));
    close_system(mdl, 0);
catch err
    fprintf('Simulink cross-check skipped: %s\n', err.message);
end

%% Why no structure meets the specification with z1 = 10
fprintf('\nClosed-loop poles (corrected gains):\n');
for Nf = [100 Inf]
    if isinf(Nf), C = Kp*(1 + 1/(Ti*s) + Td*s); else, C = Kp*(1 + 1/(Ti*s) + Td*s*Nf/(s+Nf)); end
    T = minreal(feedback(C*Gp, 1));
    fprintf('  N = %-4g poles %s, zeros %s\n', Nf, mat2str(round(pole(T).', 1)), mat2str(round(zero(T).', 1)));
    if isinf(Nf), T_ideal = T; end
end
f = figure('Visible', 'off', 'Position', [100 100 650 400]);
step(T_ideal, minreal(10/(s+10)*T_ideal), minreal(10*z2/((s+10)*(s+z2))*T_ideal), 0.5);
yline(1.05, 'k:'); grid on;
legend('PID (ideal derivative)', 'PID + prefilter 10/(s+10)', 'I-PD (both zeros removed)', 'Location', 'southeast');
title('Pole at -11.2 next to the zero at -10: overshoot or a slow tail');
save_figure(f, 'zero_pole_tradeoff');
