%CHRISTINA KOUMPOGIANNI
close all;
clear;
arc_length_S = 0.1; %or 0.1*pi
% Define ranges for arc length (S), radius (R), and theta2 of tangent
d_initial_displacement = linspace(0.000, 0.1, 20); % Initial displacement in x-axis, and then starts to curve
total_arc_length = linspace(0.001, arc_length_S, 10); % Arc length values
radius_range = linspace(0.05, 10, 20); % Radius values (R ≠ 0)
theta2_range = linspace(0, 2*pi, 10); % Range of theta2

% Initialize workspace points
workspace_x = [];
workspace_y = [];
workspace_z = [];

% Initialize workspace points
actual_tip_x = [];
actual_tip_y = [];
actual_tip_z = [];


% Define trajectory parameters
radius = 0.01; % Radius of the unit circle, of the circular path
q_start = 0; % Start position
%q_end = 160*pi/180; % End position
q_end = pi; % q: angular measure, S_total, S: linear measure
s_start = radius*q_start;
S_total = radius*q_end; % mexri stigmhs perimetros 
num_points = 1000; % Number of points
qdot = 0.003; % Constant velocity
t_fin=15; 

%t_st=0;

%Generate trajectory
[path_ref, q_vectors, time_values, phi_values, theta2_values, d_values] = path_trajectory_generation(S_total, num_points, s_start, qdot, radius, t_fin, arc_length_S);

% l is the 7th argument i pass when i call path_trajectory, l=arc_length_S
 for k = 1:length(time_values)
%                 % alpha = -path_ref(1,k)/(arc_length_S * cos(theta2_values(k)));
%                 % value(k) = cos(phi_values(k)) + phi_values(k) * alpha - 1;
%                 %forward kinematics equations / s, x/s, y/s, z/s.
% 
%                 x_tip = -((arc_length_S)*((1 - cos(phi_values(k) ))/phi_values(k) )*cos(theta2_values(k)));
%                 y_tip = -((arc_length_S)*((1 - cos(phi_values(k) ))/phi_values(k) )*sin(theta2_values(k)));
%                 z_tip = d_values(k) + (arc_length_S)*(sin(phi_values(k)))/phi_values(k) ;
%                 % z_tip = (arc_length_S/(phi_values(k)))*sin(phi_values(k))+d_values(k);
%                 % x_tip = ((arc_length_S)/(phi_values(k)))*sin(theta2_values(k))-((arc_length_S)/(phi_values(k)))*sin(theta2_values(k))*cos(phi_values(k));
%                 % y_tip = ((arc_length_S)/(phi_values(k)))*cos(theta2_values(k))-((arc_length_S)/(phi_values(k)))*cos(phi_values(k))*cos(theta2_values(k));
                phi_k = phi_values(k);
                theta_k = theta2_values(k);

                if abs(phi_k) < 1e-4
                    %  not denominators so not to divide by zero
                    x_tip =  - arc_length_S * cos(theta_k);
                    y_tip = - arc_length_S  * sin(theta_k);
                    z_tip = d_values(k) + arc_length_S;  % since sin(phi)/phi → 1
                else %actual forward kinematics equations
                    x_tip = -((arc_length_S) * ((1 - cos(phi_k)) / phi_k) * cos(theta_k));
                    y_tip = -((arc_length_S) * ((1 - cos(phi_k)) / phi_k) * sin(theta_k));
                    z_tip = d_values(k) + (arc_length_S) * (sin(phi_k) / phi_k);
                 end

                % Store the point
                actual_tip_x = [actual_tip_x; x_tip];
                actual_tip_y = [actual_tip_y; y_tip];
                actual_tip_z = [actual_tip_z; z_tip];
end

% for r_len = 1:length(radius_range)
%     R = radius_range(r_len); %radius of the catheter (R), different from unit circle (radius)
% 
%     %Generate trajectory
% 
%      %size(path_ref) = 3x100, because xyz, 100 points by definition 
% 
%     for s_arc = 1:length(total_arc_length)
%         S = total_arc_length(s_arc); 
%         phi = S / R; 
% 
% 
%         for d_disp = 1:length(d_initial_displacement)
%             d = d_initial_displacement(d_disp);
% 
%             for j = 1:length(theta2_range)
%                 theta2 = theta2_range(j); %idioperistrofi
% 
%                 T0 = dh_transform(0, d, 0, 0);
%                 T1 = dh_transform(theta2, 0, 0, 0);
%                 T2 = dh_transform(0, 0, phi/2 + pi/2, 0);
%                 T3 = dh_transform(pi/2, 0, 0, 0);
%                 T4 = dh_transform(0, 0, 0, 2*S/phi*sin(phi/2));
%                 T5 = dh_transform(-pi/2-theta2, 0, 0, 0);
% 
%                 % Total transformation to the end-effector
%                 T = T0 * T1 * T2 * T3 * T4 * T5;
% 
%                 % Extract the 3D position of the end-effector
%                 x = T(1,4);
%                 y = T(2,4);
%                 z = T(3,4);
% 
%                 % Store the point
%                 workspace_x = [workspace_x; x];
%                 workspace_y = [workspace_y; y];
%                 workspace_z = [workspace_z; z];
% 
%                 % %Extract the final FK position (last computed value)
%                 %     x_fk_final = x(end);
%                 %     y_fk_final = y(end);
%                 %     z_fk_final = z(end);          
% 
%             end
%         end
%     end
% end

x_traj_final = path_ref(1, end);
y_traj_final = path_ref(2, end);
z_traj_final = path_ref(3, end);



% 
% % Plot the 3D workspace
% figure;
% plot3(workspace_x, workspace_y, workspace_z, '.', 'MarkerSize', 4);
% grid on;
% axis equal;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('3D Workspace with Dynamic S, R, theta2');

% Plot the trajectory
figure(1);
 plot3(path_ref(1,:), path_ref(2,:), path_ref(3,:), 'b-', 'LineWidth', 2);
 hold on
 plot3(actual_tip_x, actual_tip_y, actual_tip_z, 'r-.', 'LineWidth', 2);
% grid on;
% axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Circular Trajectory with Parabolic Blending - RED: ACTUAL TIP, BLUE: TRAJECTORY');
% hold on;

% % Plot the final trajectory position 
% % plot3(x_traj_final, y_traj_final, z_traj_final, 'r*', 'MarkerSize', 5, 'LineWidth', 2);
% % hold on;
% % Plot the actual FK position in red
% plot3(x_fk_final, y_fk_final, z_fk_final, 'g*', 'MarkerSize', 5, 'LineWidth', 2);
% legend('trajectory','end of trajectory', 'workspace end effector');


figure(2);
% Subplot 1: Phi vs. Time
subplot(4,1,1);
plot(time_values, phi_values, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('\phi (rad)');
title('Phi (\phi) vs. Time');

% Subplot 2: Theta2 vs. Time
subplot(4,1,2);
plot(time_values, theta2_values, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('\theta_2 (rad)');
title('Theta2 (\theta_2) vs. Time');

% Subplot 3: Displacement d vs. Time
subplot(4,1,3);
plot(time_values, d_values, 'g-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('d (m)');
title('Displacement d vs. Time');

% Plot q vs. time
subplot(4,1,4);
plot(time_values, q_vectors, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('s (Position)');
title(['Position s vs. Time (t_{fin} = ', num2str(t_fin), ' s)']);
legend('s over time in trajectory');


% figure(3)
% plot(time_values, value)
% grid

% Function to compute DH transformation matrix
function T = dh_transform(alpha, a, theta, d)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),            cos(alpha),            d;
         0,           0,                     0,                     1];
end

