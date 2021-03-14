function [XDistMap,YDistMap] = computeScaledUndistortionGrid(params,gridSize)
%% Maps pixel grid points to undistorted pixel locations
%
% Author: B.J. Pijnacker Hordijk
% Date: 12-04-2016

if length(gridSize) == 1
    gridSize = [gridSize gridSize];
end

[X,Y] = meshgrid(1:gridSize(1),1:gridSize(2));
X = X';
Y = Y';

XDistMap = zeros(size(X));
YDistMap = zeros(size(Y));

% Perform operation column by column to limit memory usage.
% If too many points are used, undistortPoints() soaks up RAM usage beyond 
% the limits of most computers.
for i = 1:gridSize(2)
    P = undistortPoints([X(:,i) Y(:,i)],params);
    % project points onto focal plane and shift wrt principle point
    XDistMap(:,i) = (P(:,1) - params.PrincipalPoint(1)) / params.FocalLength(1);
    YDistMap(:,i) = (P(:,2) - params.PrincipalPoint(2)) / params.FocalLength(2);
    disp(['Column ' num2str(i) '...'])    
end
disp('Done')
end