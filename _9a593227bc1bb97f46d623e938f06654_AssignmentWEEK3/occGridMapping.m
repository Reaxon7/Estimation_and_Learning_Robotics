% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;
%lo_max_min = lo_max/lo_min;

N = size(pose,2);
for j = 1:N % for each time,

      
    % Find grids hit by the rays (in the gird map coordinate)
    occ_real = [ranges(:,j).*cos(scanAngles + pose(3,j))+pose(1,j), -ranges(:,j).*sin(scanAngles + pose(3,j))+pose(2,j)];
    occ_idx = ceil([occ_real(:,1), occ_real(:,2) ]* myResol);
    occ_index = [occ_idx(:,1) + myorigin(1), occ_idx(:,2) + myorigin(2)];
    %pose_idx = ceil([pose(1,j) pose(2,j)]*myResol);

    for i = 1:size(occ_real,1)
        [freex, freey] = bresenham(pose(1,j)*myResol+myorigin(1),pose(2,j)*myResol+myorigin(2),occ_index(i,1),occ_index(i,2));
        
        valid_free_indices = freex > 0 & freex <= size(myMap, 2) & ...
                             freey > 0 & freey <= size(myMap, 1);
        freex = freex(valid_free_indices);
        freey = freey(valid_free_indices);

        free = sub2ind(size(myMap),freey,freex);
        %keyboard
        myMap(occ_index(i,2),occ_index(i,1)) = myMap(occ_index(i,2),occ_index(i,1)) + lo_occ;
        myMap(free) = myMap(free) - lo_free;

        myMap(occ_index(i,2),occ_index(i,1)) = min(myMap(occ_index(i,2),occ_index(i,1)),lo_max);
        myMap(free) = max(myMap(free),lo_min);
    end

    % Find occupied-measurement cells and free-measurement cells
    
    % Update the log-odds
  

    % Saturate the log-odd values
    

    % Visualize the map as needed
   

end

end

