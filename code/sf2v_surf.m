close all;
clear;
clc

imageDir = fullfile("/home/fekete/Documents/DSO/test");
images = imageDatastore(imageDir);
I1 = readimage(images, 1);
I2 = readimage(images, 2);

%figure
%imshowpair(I1, I2, 'montage'); 
%title('Original Images');

data = load("/home/fekete/Documents/DSO/intrinsics_low_res.mat");
intrinsics = data.cameraParams.Intrinsics; 

% Remove Lens Distortion
I1 = undistortImage(I1, intrinsics);
I2 = undistortImage(I2, intrinsics);
%figure 
%imshowpair(I1, I2, "montage");
%title("Undistorted Images");

% Detect feature points
imagePoints1 = detectSURFFeatures(im2gray(I1), 'MetricThreshold', 400);
imagePoints2 = detectSURFFeatures(im2gray(I2), 'MetricThreshold', 400);

imagePoints1 = selectStrongest(imagePoints1, 500); % Step 2
imagePoints2 = selectStrongest(imagePoints2, 500); % Step 2


[features1, validPoints1] = extractFeatures(im2gray(I1), imagePoints1);
[features2, validPoints2] = extractFeatures(im2gray(I2), imagePoints2);
indexPairs = matchFeatures(features1, features2, 'MaxRatio', 0.8, 'Unique', true);

% Get matched points
matchedPoints1 = validPoints1(indexPairs(:, 1));
matchedPoints2 = validPoints2(indexPairs(:, 2));

% Visualize correspondences
%figure
%showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
%title("Tracked Features");

% Estimate the fundamental matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, intrinsics, Confidence = 99.99);

% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

% Display inlier matches
%figure
%showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
%title("Epipolar Inliers");
%disp('Inlier points from image 1:');
%disp(inlierPoints1(1:min(5, size(inlierPoints1, 1)), :));

disp('Inlier points from image 2:');
disp(inlierPoints2(1:min(5, size(inlierPoints2, 1)), :));

% Compute the Camera Pose
relPose = estrelpose(E, intrinsics, inlierPoints1, inlierPoints2);

% Detect dense feature points. Use an ROI to exclude points close to the
% image edges.
border = 350;
metric_threshold = 50;
roi = [border, border, size(I1, 2)- 2*border, size(I1, 1)- 2*border];
imagePoints1 = detectSURFFeatures(im2gray(I1), 'MetricThreshold', metric_threshold);
%figure
%imshow(I1);
%hold on;
%rectangle('Position', roi, 'EdgeColor', 'r' , 'LineWidth' , 2 );
%plot(imagePoints1);
%title("ROI");

imagePoints2 = detectSURFFeatures(im2gray(I2), 'MetricThreshold', metric_threshold);

[features1, validPoints1] = extractFeatures(im2gray(I1), imagePoints1);
[features2, validPoints2] = extractFeatures(im2gray(I2), imagePoints2);

indexPairs = matchFeatures(features1, features2, 'MaxRatio', 0.8, 'Unique', true);
matchedPoints1 = validPoints1(indexPairs(:, 1)).Location;
matchedPoints2 = validPoints2(indexPairs(:, 2)).Location;

%figure;
%showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
%title("Matched Features after ROI");

% Compute the camera matrices for each position of the camera
% The first camera is at the origin looking along the Z-axis. Thus, its
% transformation is identity.
camMatrix1 = cameraProjection(intrinsics, rigidtform3d);
camMatrix2 = cameraProjection(intrinsics, pose2extr(relPose));

% Compute the 3-D points
points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

% Get the color of each reconstructed point
numPixels = size(I1, 1) * size(I1, 2);
allColors = reshape(I1, [numPixels, 3]);
colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matchedPoints1(:,2)), ...
    round(matchedPoints1(:, 1)));
color = allColors(colorIdx, :);

% Create the point cloud
ptCloud = pointCloud(points3D, Color=color);

% Visualize the camera locations and orientations
cameraSize = 0.3;
figure
plotCamera(Size=cameraSize, Color="r", Label="1", Opacity=0);
hold on
grid on
plotCamera(AbsolutePose=relPose, Size=cameraSize, ...
    Color="b", Label="2", Opacity=0);

% Visualize the point cloud
pcshow(ptCloud, VerticalAxis="y", VerticalAxisDir="down", MarkerSize=45);
pcviewer(ptCloud);
rotate3d on;
% Rotate and zoom the plot
camorbit(0, -30);
camzoom(1.5);

% Label the axes
xlabel("x-axis");
ylabel("y-axis");
zlabel("z-axis")

title("Up to Scale Reconstruction of the Scene");