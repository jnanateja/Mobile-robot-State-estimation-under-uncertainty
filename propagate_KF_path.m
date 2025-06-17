function [Pfinal,pk] = propagate_KF_path(path,obstacles)
    A = [1 0 1 0;
         0 1 0 1;
         0 0 1 0;
         0 0 0 1];
    Q = eye(4);
    P = diag([0.01,0.01,0.01,0.01]);
    
    pk = zeros(4,4,length(path));
    pk(:,:,1) = P;

    for i = 2:size(path, 1)
        % Extract position from the path
        xk = path(i-1, :);
        [H,R]  = HR_matrix(xk, obstacles); % Check the sensors for the measurement
        p = pk(:,:,i-1);
        % Propagate Kalman filter for one time-step
        pk(:,:,i) = propagate_KF(A, H, Q, R, p);
    end
    Pfinal = pk(:,:,end);
end

function pk = propagate_KF(A, H, Q, R, p)
pk = inv(inv(A*p*transpose(A)+Q) + transpose(H)*inv(R)*H);
end

function [H,R] = HR_matrix(xk, obstacles)
num_obstacles = size(obstacles,1);
p_x = xk(1);
p_y = xk(2);
H = [0,0,1,0;
    0,0,0,1];
R = eye(2);
    for i_obs = 1:num_obstacles
        x1 = obstacles(i_obs,1); y1 = obstacles(i_obs,2);
        x2 = obstacles(i_obs,3); y2 = obstacles(i_obs,4);
        x3 = obstacles(i_obs,5); y3 = obstacles(i_obs,6);
        x4 = obstacles(i_obs,7); y4 = obstacles(i_obs,8);
        
        if (((p_x+5 >= x1) && (p_x+5 <= x2)) && ((p_y>= y1 && p_y <= y3))) 
            H(1,1) = 1;
        end
        if (((p_x-5 >= x1) && (p_x-5 <= x2)) && ((p_y>= y1 && p_y <= y3))) 
            H(1,1) = 1;
        end
        if (((p_x >= x1) && (p_x <= x2)) && ((p_y +5 >= y1 && p_y+5 <= y3)))
            H(2,2) = 1;
        end
        if (((p_x >= x1) && (p_x <= x2)) && ((p_y -5 >= y1 && p_y-5 <= y3)))
            H(2,2) = 1;
        end
    end
end