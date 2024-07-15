% function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
% %UNTITLED Summary of this function goes here
% %   Four dimensional state: position_x, position_y, velocity_x, velocity_y
% 
%     %% Place parameters like covarainces, etc. here:
%     dt = 0.033;
%    
%     % Check if the first time running this function
%     if previous_t<0
%         state = [x, y, 0, 0];
%         param.P = 0.1 * eye(4);
%         predictx = x;
%         predicty = y;
%         return;
%     end
% 
% 	% Noise covariance matrices
%     Sigma_m = 0.01 * eye(4);
%     Sigma_o = 0.01 * eye(2);
% 	
% 	% Transition matrix
%     A = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1];
%     
%     % Observation matrix
%     C = [1 0 0 0; 0 1 0 0];
% 
% 
% 
%     %% TODO: Add Kalman filter updates
%     % As an example, here is a Naive estimate without a Kalman filter
%     % You should replace this code
% %     vx = (x - state(1)) / (t - previous_t);
% %     vy = (y - state(2)) / (t - previous_t);
% %     % Predict 330ms into the future
% %     predictx = x + vx * 0.330;
% %     predicty = y + vy * 0.330;
% %     % State is a four dimensional element
% %     state = [x, y, vx, vy];
% 
%     % Predicted state & covariance matrix
%     Xkp = A * state';
%     Pkp = A * param.P * A' + Sigma_m;
%     
%     % Kalman gain
%     K = Pkp * C' /(C*Pkp*C' + Sigma_o);
%         
%     % Z measrement
%     Z = C*state';
%     
%     % Update covariance matrix  
%     param.P = Pkp - K*C*Pkp;
%     
%     % Update state
%     state = Xkp + K*(Z - C*Xkp);
%     
%     % Prediction
%     predictx = state(1);
%     predicty = state(2);
% 
%     %previous_t = t;
% end

function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
% kalmanFilter Function to implement a Kalman filter for tracking position
% 
% INPUT
% t          : Current time
% x, y       : Current positions
% state      : Previous state (1x4 vector: [pos_x, pos_y, vel_x, vel_y])
% param      : Struct containing the covariance matrix P
% previous_t : Previous time
%
% OUTPUT
% predictx, predicty : Predicted positions
% state              : Updated state
% param              : Updated parameters (including P)

    %% Place parameters like covariances, etc. here:
    dt = 0.033;
   
    % Check if the first time running this function
    if previous_t < 0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    % Noise covariance matrices
    Sigma_m = 0.01 * eye(4);
    Sigma_o = 0.01 * eye(2);
    
    % Transition matrix
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    
    % Observation matrix
    C = [1 0 0 0; 0 1 0 0];

    %% Kalman filter updates
    % Predicted state & covariance matrix
    state = state';  % Ensure state is a column vector (4x1)
    Xkp = A * state;  % (4x4) * (4x1) = (4x1)
    Pkp = A * param.P * A' + Sigma_m;  % (4x4) * (4x4) * (4x4) + (4x4) = (4x4)
    
    % Kalman gain
    K = Pkp * C' / (C * Pkp * C' + Sigma_o);  % (4x4) * (4x2) / ((2x4) * (4x4) * (4x2) + (2x2)) = (4x2)
        
    % Measurement vector
    Z = [x; y];  % (2x1)
    
    % Update state
    state = Xkp + K * (Z - C * Xkp);  % (4x1) + (4x2) * ((2x1) - (2x4) * (4x1)) = (4x1)
    
    % Update covariance matrix  
    param.P = (eye(size(K, 1)) - K * C) * Pkp;  % (4x4) - (4x2) * (2x4) = (4x4)
    
    state = state';  % Transpose back to a row vector (1x4) for consistency
    
    % Prediction
    predictx = state(1);
    predicty = state(2);
end