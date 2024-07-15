% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerical array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = double([152.0331  145.4015   57.4567]);
sig = double([ 139.0024   95.0369 -141.3717;
   95.0369  122.0206 -152.5527;
 -141.3717 -152.5527  264.0263]);
thre = 0.5*10^-5;
    % You may consider other color space than RGB

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 

R = double(I(:,:,1));
G = double(I(:,:,2));
B = double(I(:,:,3));

% Subtract mu from each pixel
x_centered = bsxfun(@minus, double(I), reshape(mu, 1, 1, 3));

% Compute the inverse and determinant of the covariance matrix
inv_sig = inv(sig);
det_sig = det(sig);

% Initialize the probability image
pI = zeros(size(I,1), size(I,2));

% Compute the Gaussian probability for each pixel
for i = 1:size(I,1)
    for j = 1:size(I,2)
        diff = squeeze(x_centered(i, j, :));
        pI(i, j) = 1 / ((2 * pi)^(3 / 2) * sqrt(det_sig)) * exp(-0.5 * (diff' * inv_sig * diff));
    end
end

% Segment the image based on the threshold
segI = pI >= thre;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
CC = bwconncomp(segI);
numPixels = cellfun(@numel, CC.PixelIdxList);
[biggest, idx] = max(numPixels);

% create new empty binary image
segI_biggest = false(size(segI));
segI_biggest(CC.PixelIdxList{idx}) = true;


% Get the centroid of the largest connected component
S = regionprops(CC, 'Centroid');
loc = S(idx).Centroid;
