% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
%function myPose = particleLocalization(ranges, scanAngles, map, param)

% % Number of poses to calculate
% N = size(ranges, 2);
% % Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
% myPose = zeros(3, N);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% % Map Parameters 
% % 
% % % the number of grids for 1 meter.
% myResolution = param.resol;
% % % the origin of the map in pixels
% myOrigin = param.origin; 
% 
% % The initial pose is given
% myPose(:,1) = param.init_pose;
% % You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% % The pose(:,1) should be the pose when ranges(:,j) were measured.
% 
% 
% 
% % Decide the number of particles, M.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M = 1000;                          % Please decide a reasonable number of M, 
%                                % based on your experiment using the practice data.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Create M number of particles
% P = repmat(myPose(:,1), [1, M]);
% 
% keyboard
% 
% for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% % 
% %     % 1) Propagate the particles 
% %
% %       
% %     % 2) Measurement Update 
% %     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
% %
% %     %   2-2) For each particle, calculate the correlation scores of the particles
% %
% %     %   2-3) Update the particle weights         
% %  
% %     %   2-4) Choose the best particle to update the pose
% %     
% %     % 3) Resample if the effective number of particles is smaller than a threshold
% % 
% %     % 4) Visualize the pose on the map as needed
% %    
% % 
% end
% 
% end


function [myPose] = particleLocalization(ranges, scanAngles, map, param)
    % Number of poses to calculate
    N = size(ranges, 2);
    % Output format is [x1 x2, ...; y1, y2, ...; theta1, theta2, ...]
    myPose = zeros(3, N);

    % Map Parameters 
    myResolution = param.resol;
    myOrigin = param.origin; 

    % The initial pose is given
    myPose(:,1) = param.init_pose;

    % Number of particles
    M = 1000;

    % Effective particle threshold
    effectiveThreshold = 0.8 * M;

    % Covariance matrix for motion noise
    cov = [10e-4 0 0; 0 10e-4 0; 0 0 5.1e-5];

    % Create M number of particles
    particles = repmat(myPose(:,1), [1, M]);
    weights = ones(1, M) / M;

    % Reduce scan angles to every 10th element
    %scanAngles = -scanAngles;

    % Variance for the Gaussian model
    %measurementVar = 1; % Adjust based on sensor noise characteristics
    %flag = 1;
    % Main loop over scans
    for j = 2:N
        while true
        % 1) Propagate the particles with odometry update
        % Assuming we have odometry data (delta_x, delta_y, delta_theta)
%         if j == 2
            delta_theta = 1;
            d_radius = 0.00;
            delta_x = d_radius*cos( delta_theta + myPose(3,j-1));
            delta_y = -d_radius*sin( delta_theta + myPose(3,j-1));
            
%         else
%             delta_x = myPose(1,j-1) - myPose(1,j-2); % Replace with actual odometry data
%             delta_x = max(min(delta_x, 1), -1);
%             delta_y = myPose(2,j-1) - myPose(2,j-2); % Replace with actual odometry data
%             delta_y = max(min(delta_y, 1), -1);
%             delta_theta = myPose(3,j-1) - myPose(3,j-2);% Replace with actual odometry data
%             delta_theta = max(min(delta_theta, 0.1), -0.1);
%         end
        
            for m = 1:M
                if j>2 && myPose(1,j-1)-myPose(1,j-2) > 0
                    particles(1, m) = particles(1, m) + delta_x + normrnd(0, sqrt(cov(1, 1)));
                else
                    particles(1, m) = particles(1, m) - delta_x + normrnd(0, sqrt(cov(1, 1)));
                end

                if j>2 && myPose(2,j-1)-myPose(2,j-2) > 0
                    particles(2, m) = particles(2, m) + delta_y + normrnd(0, sqrt(cov(2, 2)));
                else
                    particles(2, m) = particles(2, m) - delta_y + normrnd(0, sqrt(cov(2, 2)));
                end
                particles(3, m) = particles(3, m) + delta_theta*normrnd(0, sqrt(cov(3, 3)));
            end
    
            % 2) Measurement Update 
            % 2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
            correlations = zeros(1, M);


            for m = 1:M
                

                particlePose = particles(:, m);
                score = 0;
                for k = 1:10:size(scanAngles,1)
                    % Convert range and angle to map coordinates
                    occX = ranges(k, j) * cos(scanAngles(k) + particlePose(3)) + particlePose(1);
                    occY = -ranges(k, j) * sin(scanAngles(k) + particlePose(3)) + particlePose(2);
                    occIdxX = ceil(occX * myResolution) + myOrigin(1);
                    occIdxY = ceil(occY * myResolution) + myOrigin(2);
    
                    % Check if within bounds
                    if occIdxX > 0 && occIdxX <= size(map, 2) && occIdxY > 0 && occIdxY <= size(map, 1)
                        if map(occIdxY, occIdxX) > 0.51  % Map Occupied
                            score = score + 10000;  % Occupied cell

                            %expectedMeasurement = 1;
                        else  % Map Free
                            score = score + 1;
                            %expectedMeasurement = 0;% Free cell
                        end
    
%                         %Calculate the Gaussian likelihood
%                         actualMeasurement = map(occIdxY, occIdxX);
%                         likelihood = exp(-0.5 * ((actualMeasurement - expectedMeasurement)^2) / measurementVar) / sqrt(2 * pi * measurementVar);
%                         score = score + log(likelihood);
                    end

%                     if occIdxX > 0 && occIdxX <= size(map, 2) && occIdxY > 0 && occIdxY <= size(map, 1) && j == 200
%                         
% 
%                         % Plot the LIDAR measurement on the map
%                         plot(occIdxX, occIdxY, 'r.', 'MarkerSize', 5);
% 
%                       
%                         %keyboard
%                     end

                end

                correlations(m) = score;
            end
            
            
            % 2-3) Update the particle weights
            weights = correlations.*weights;
            %weights = exp(correlations - max(correlations)).*weights;
            weights = weights / sum(weights);
    
            % 2-4) Choose the best particle to update the pose
            [~, bestIdx] = max(weights);
            myPose(:, j) = particles(:, bestIdx);



            if j >= 100 && mod(j,100) == 0 && j < 500
                figure;
                imagesc(map);
                colormap(gray);
                hold on;


                particlePose = myPose(:, j);

                for k = 1:10:size(scanAngles,1)
                    % Convert range and angle to map coordinates
                    occX = ranges(k, j) * cos(scanAngles(k) + particlePose(3)) + particlePose(1);
                    occY = -ranges(k, j) * sin(scanAngles(k) + particlePose(3)) + particlePose(2);
                    occIdxX = ceil(occX * myResolution) + myOrigin(1);
                    occIdxY = ceil(occY * myResolution) + myOrigin(2);

                    % Check if within bounds
                    if occIdxX > 0 && occIdxX <= size(map, 2) && occIdxY > 0 && occIdxY <= size(map, 1)
                        if map(occIdxY, occIdxX) > 0.51  % Map Occupied
                            score = score + 10000;  % Occupied cell

                            %expectedMeasurement = 1;
                        else  % Map Free
                            score = score + 1;
                            %expectedMeasurement = 0;% Free cell
                        end

                    end

                    if occIdxX > 0 && occIdxX <= size(map, 2) && occIdxY > 0 && occIdxY <= size(map, 1)


                        % Plot the LIDAR measurement on the map
                        plot(occIdxX, occIdxY, 'r.', 'MarkerSize', 5);

                    end

                end

                if j == 500
                    keyboard
                end

                title('LIDAR Measurements on the Map');
                xlabel('X');
                ylabel('Y');
                axis equal;
                %hold off;


            end
            
            % 4) Visualize the pose on the map as needed
            figure(1);
            imagesc(map); hold on;
            plot(myOrigin(1) + myPose(1, 1:j) * myResolution, myOrigin(2) + myPose(2, 1:j) * myResolution, 'r-');
            plot(myOrigin(1) + particles(1, :) * myResolution, myOrigin(2) + particles(2, :) * myResolution, 'b.');
            hold off;
            drawnow;
    
            % 3) Resample if the effective number of particles is smaller than a threshold
            effectiveN = sum(weights)^2 / sum(weights.^2);
            %keyboard
            if effectiveN < effectiveThreshold
                indices = randsample(1:M, M, true, weights);
                particles = particles(:, indices);
                weights = ones(1, M) / M;
                %keyboard
            else
                
                particles = repmat(myPose(:,j), [1, M]);
                weights = ones(1, M) / M;
                %keyboard
                break;
              
            end

        end

    end
end


