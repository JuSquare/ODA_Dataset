%% Tool for calibration of event-based cameras
%
% This tool produces a blinking dot pattern with specified grid size. By
% recording the pattern produced on-screen with an event-based camera,
% 'calibration events' are produced, which can be converted to images that
% can be processed using MATLAB's Image Processing Toolbox.
%
% This setup requires a screen with a decently large viewing angle, since
% for accurate calibration, multiple recordings from different angles are
% needed. Also, high brightness/contrast settings are beneficial.
%
% Currently the GUI is set up for a 1600x900 display, such that the
% recording tool jAER can be shown alongside the window.
%
% Init --------------------------------------------------------------------
% Author: B.J. Pijnacker Hordijk
% Date: 07-03-2016
% -------------------------------------------------------------------------
% Update ------------------------------------------------------------------
% Author: J. Dupeyroux
% Date: 19-03-2020
% AEDAT4 being not supported, a conversion to .txt is introduced using the
% python code "convAedat2txt.py". 
% -------------------------------------------------------------------------

clear all
close all
clc

disp('--- EVENT-BASED CAMERA CALIBRATION TOOL ---')
disp('-> Set-up')

nBlocks = 8;
nFlashes = 16;
nReps = 10;

disp('-> Displaying GUI...')

% Set pixel size
diagon = 0.3083;

% Set up image
N = nBlocks + 1;
CB = zeros(N);
for i = 1:N
    for j = 1:N
        CB(i,j) = mod(1+i+j,2);
    end
end

% if second monitor, use its screen position
Ps = get(0,'monitorpositions');
P = Ps(end,:);
if size(Ps,1) == 1
    xMargin1 = 500; xMargin2 = 20;
    yMargin1 = 40; yMargin2 = 100;
else 
    xMargin1 = 0; xMargin2 = 0; 
    yMargin1 = 0; yMargin2 = 0;
end


fig = figure(1);
set(fig,'Position',P+[xMargin1, yMargin1, -xMargin1-xMargin2, -yMargin1-yMargin2]);
set(gcf,'Color',[0 0 0])

p=pcolor(1-CB);
colormap('gray(2)')
pAxis = gca;
set(pAxis,'Position',[0.05 0.1 0.6 0.8],'Visible','off')
axis equal
xlim([1 N])
ylim([1 N])
boardPos = getpixelposition(pAxis);

tAxis = axes('Position',[0.7 0.3 0.25 0.4], 'Visible','off');

axes(tAxis)
th = text(0,0.5,'','Color','w');

set(th,'String',{'Ready for flashing.'; 
    'Make sure the camera is in position and recording.';     
    'Click inside the window if ready.'},'Color','w');
waitforbuttonpress()

for i = 1:nReps
    set(th,'String',{['-> Starting with repetition ' num2str(i) ' of ' num2str(nReps)];
        ['Flashing ' num2str(nFlashes) ' times...']},'Color','w');
    
    pause(1)
    for j=1:nFlashes
        set(p,'Visible','off')
        drawnow()
        pause(0.05)
        set(p,'Visible','on')
        pause(0.05)
        drawnow()
    end
    if i ~= nReps
        set(th,'String',{'Stop and save recording.'; 
            'Reposition camera at a different viewing angle.';
            'Start new recording when repositioned.';
            'Click the window if ready for the next repetition.'},'Color','w');
        waitforbuttonpress()
    end
end

set(th,'String',{'All repetitions completed. Stop and save event recording.';
    'Once this window is closed, the script continues.';
    'The remainder consists of estimating camera parameters.'},'Color','w');
waitfor(fig);

%% Post-processing calibration files

tt  = atan(P(4)/P(3));
w   = cos(tt)*diagon;
PS  = w/P(3);
squareSizeMM = PS*boardPos(3)/nBlocks*1000;

Np = N-2;
imagePoints = zeros(Np^2,2,nReps);
idx = 1:nReps;
Ims = cell(nReps,1);

disp('Calculating checkerboard points from AER data...')
for i = 1:nReps
    filename = ['calib' num2str(i) '.txt'];
    disp(filename)
    if exist(filename,'file')
        calibData = dlmread(filename);
        x = calibData(:,2);
        y = calibData(:,3);
        p = calibData(:,4);
        px = 240;
        py = 180;
        S = zeros(px,py);
        x = x+1; y = y+1;
        for j = 1:length(x)
            S(x(j),y(j)) = S(x(j),y(j)) + 1;
        end
        
        % remove bad pixels
        idxs2remove = find(S > 50);
        %S(idxs2remove) = 0;
        
        % Convert to image (transpose due to coordinate system)
        Ims{i} = uint8(S'*255/max(max(S)));
        % Now find grid points
        [pp, boardSize] = detectCheckerboardPoints(Ims{i});
        imshow(Ims{i});
        if all(boardSize == nBlocks)
            imagePoints(:,:,i) = pp;
            hold on;
            plot(imagePoints(:,1,i),imagePoints(:,2,i),'go');
            hold off;
        else
            idx(i) = 0;
            warning(['For set ' num2str(i) 'points were not well detected.'])
        end
    else
        idx(i) = 0;
        warning(['AER file ' num2str(i) ' missing'])
    end
    fprintf([i '...'])
end
disp('Done')
disp('Estimating camera parameters...')

idx = idx(idx ~= 0);
if isempty(idx)
    error('No usable result');
end
imagePoints = imagePoints(:,:,idx);
worldPoints = generateCheckerboardPoints([nBlocks nBlocks],squareSizeMM);
params = estimateCameraParameters(imagePoints,worldPoints,'numradialDistortionCoefficients',2, 'ImageSize', [px, py]);
params.ImageSize = [params.ImageSize(2), params.ImageSize(1)]; 

showReprojectionErrors(params)

figure;  hold off;
showExtrinsics(params);

I = Ims{1};
used_points = 1;
figure(3);
imshow(I); hold on;
plot(imagePoints(:,1,used_points),imagePoints(:,2,used_points),'go');
plot(params.ReprojectedPoints(:,1,used_points),params.ReprojectedPoints(:,2,used_points),'r+');

[J2, newOrigin] = undistortImage(I,params,'OutputView','full');
undistortedPoints = undistortPoints(imagePoints(:,:,used_points),params);
undistortedPoints = [undistortedPoints(:,1) - newOrigin(1), ...
                    undistortedPoints(:,2) - newOrigin(2)];
figure(4);
imshow(J2); hold on;
plot(undistortedPoints(:,1),undistortedPoints(:,2),'g*');

%% Saving the results and displaying the undistortion maps

[gridx, gridy] = computeScaledUndistortionGrid(params, [px, py]);

save

printMatrixAsCArray(gridx);
pause
printMatrixAsCArray(gridy);

disp('End of program.')

