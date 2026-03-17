%CHRISTINA KOUMPOGIANNI

%Path generation
function [path_ref, q_vectors, time_values, phi_values, theta2_values, d_values] = path_trajectory_generation2( S_total, num_points, s_start, qdot, radius, t_fin, l)
    path_ref = zeros(3, num_points);
    q_vectors = zeros(1, num_points);
    time_values = linspace(0, t_fin, num_points);

    phi_values = zeros(1, num_points);     % Store phi values
    theta2_values = zeros(1, num_points);  % Store theta2 values
    d_values = zeros(1, num_points);       % Store d values

    n = [0; 0; 1]; % The unit vector representing the normal to the circle passing through the center
    d_offset = [0.0; 0.0; 0.3]; 
    p_init   = [0.00; 0.02; 0.04]; 

    % delta = p_init - d_offset;
    %c = d_offset + dot(delta', n) * n;
    %c=[0.6; 0.01; 0.01]; 
    
    c = d_offset;

    R = [1, 0, 0; %rotation around the X-axis by 30 degrees (π/6 radians)
         0, sqrt(3)/2, -1/2;
         0, 1/2, sqrt(3)/2]; 


    for i = 1:num_points
        t = time_values(i);
        q = parabolic_mixed(s_start, S_total, t_fin, qdot, t); 
        q_vectors(i) = q;

        
        S = max(q, 1e-6); % Avoid zero by enforcing a small minimum value

        phi = S / radius; % phi2 is the angle of the arc of the circular path
        polar = [radius * cos(phi); radius * sin(phi); 0]; 
        p_global = c + R * polar;
        path_ref(:, i) = p_global;

        % Extract x, y, z for inverse kinematics
        x = path_ref(1, i);
        y = path_ref(2, i);
        z = path_ref(3, i);

        % l = 0.1; %the length of the catheter
        % Compute inverse kinematics
        [phi2_inv, theta2_inv, d_inv] = InverseKinematics(x, y, z, l);

        % Store values for plotting
        phi_values(i) = phi2_inv;
        theta2_values(i) = theta2_inv;
        d_values(i) = d_inv;
       

    end
end


function q = parabolic_mixed(q_start, q_end, t_fin, qdot, t)
    qdot_min = abs(q_end - q_start) / t_fin;
    qdot_max = 2 * abs(q_end - q_start) / t_fin;
    
    if qdot <= qdot_min || qdot > qdot_max
        error('qdot (%.3f) is outside the allowed range: (%.3f, %.3f)', qdot, qdot_min, qdot_max);
    end
    
    tb = (q_start - q_end + qdot * t_fin) / qdot; 
    qddot = qdot^2 / (q_start - q_end + qdot * t_fin);
    qb = q_start + (1/2) * qddot * tb^2; 
    
    if (0 <= t) && (t < tb)
        q = q_start + (1/2) * qddot * t^2;
    elseif (tb <= t) && (t <= t_fin - tb)
        q = q_start + qdot * (t - tb/2);
    else
        q = q_end - (1/2) * qddot * (t_fin - t)^2;
    end

end


% Inverse Kinematics Function
function [phi2, theta2, d] = InverseKinematics(x, y, z, l) 


    % %disp('Inverse kinematics results');
    % epsilon = 0.01;
    % % Compute theta2
     theta2 = atan2(-y, -x);
    % 
     % % Check for zero division
     if l == 0
         error('l cannot be zero');
     end
    % Compute alpha 
     alpha = -x/(cos(theta2)*l);
    %Compute beta 
     beta   = -y/(l * sin(theta2));


     epsilon = 1e-4; % smaller threshold
     
    % Solve for phi numerically using fsolve
     phi_guess = 0.6; % Initial guess for phi 
    
    
    if abs(cos(theta2)) < epsilon %if cos 0, then use sin(theta2) equation
        beta = -y / (l * sin(theta2));  
        phi2 = fsolve(@(phi2) cos(phi2) + phi2 * beta - 1, phi_guess);
    
    elseif abs(sin(theta2)) < epsilon
        alpha = -x / (cos(theta2) * l);  
        phi2 = fsolve(@(phi2) cos(phi2) + phi2 * alpha - 1, phi_guess);
    
    else
        alpha = -x / (cos(theta2) * l);
        beta  = -y / (l * sin(theta2));
    
        if abs(cos(theta2)) > abs(sin(theta2))
            phi2 = fsolve(@(phi2) cos(phi2) + phi2 * alpha - 1, phi_guess);
        else
            phi2 = fsolve(@(phi2) cos(phi2) + phi2 * beta - 1, phi_guess);
        end
    end


    % phi = lsqnonlin(@(phi) cos(phi) + phi * mi - 1, phi_guess, 0, 2*pi, options);

     % Compute 'd'
    if abs(phi2) < 1e-4 %1e-4=0.0001
        d = z - l;  % since lim φ→0 of sin(φ)/φ = 1
    else
        d = z - (l)*(sin(phi2)/phi2);
    end

    % % Display results to compare with forward kinematics
    % disp(['Phi: ', num2str(phi)]);
    % disp(['Theta2: ', num2str(theta2)]);
    % disp(['D: ', num2str(d)]);
end
