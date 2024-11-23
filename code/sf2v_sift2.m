close all;
clear;
clc

% Select the first image
%imageDir = fullfile("/home/fekete/Documents/DSO/images/");
%images = imageDatastore(imageDir);
%I1 = readimage(images, 70);
%I2 = readimage(images, 71);

imageDir = fullfile("/home/fekete/Documents/DSO/test");
images = imageDatastore(imageDir);
I1 = readimage(images, 1);
I2 = readimage(images, 2);

%figure
%imshowpair(I1, I2, 'montage'); 
%title('Original Images');

%data = load("/home/fekete/Documents/DSO/intrinsics.mat");
data = load("/home/fekete/Documents/DSO/intrinsics_low_res.mat");
intrinsics = data.cameraParams.Intrinsics; 

% Remove Lens Distortion
I1 = undistortImage(I1, intrinsics);
I2 = undistortImage(I2, intrinsics);
%figure 
%imshowpair(I1, I2, "montage");
%title("Undistorted Images");

% Detect SIFT features
points1 = detectSIFTFeatures(im2gray(I1), 'ContrastThreshold', 0.03);
points2 = detectSIFTFeatures(im2gray(I2), 'ContrastThreshold', 0.03);

% Extract features
[features1, validPoints1] = extractFeatures(im2gray(I1), points1);
[features2, validPoints2] = extractFeatures(im2gray(I2), points2);

% Match features
indexPairs = matchFeatures(features1, features2, 'MaxRatio', 0.8, 'Unique', true);
matchedPoints1 = validPoints1(indexPairs(:, 1));
matchedPoints2 = validPoints2(indexPairs(:, 2));

% Visualize correspondences
%figure
%showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
%title("Tracked Features");

% Estimate the fundamental matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1.Location, matchedPoints2.Location, intrinsics, Confidence = 99.99);

% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

% Display inlier matches
%figure
%showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
%title("Epipolar Inliers");
%disp('Inlier points from image 1:');
%disp(inlierPoints1(1:min(5, size(inlierPoints1, 1)), :));

%disp('Inlier points from image 2:');
%disp(inlierPoints2(1:min(5, size(inlierPoints2, 1)), :));

% Compute the Camera Pose
relPose = estrelpose(E, intrinsics, inlierPoints1, inlierPoints2);

border = 200;
roi = [border, border, size(I1, 2) - 2*border, size(I1, 1) - 2*border];

% Detect SIFT features
contrast_threshold = 0.01;
imagePoints1 = detectSIFTFeatures(im2gray(I1), 'ContrastThreshold', contrast_threshold);

% Filter SIFT features based on ROI
xmin = roi(1);
ymin = roi(2);
xmax = xmin + roi(3);
ymax = ymin + roi(4);

% Logical indexing to filter points within ROI
validIdx = imagePoints1.Location(:, 1) >= xmin & imagePoints1.Location(:, 1) <= xmax & ...
           imagePoints1.Location(:, 2) >= ymin & imagePoints1.Location(:, 2) <= ymax;
imagePoints1 = imagePoints1(validIdx);

% Detect SIFT features in the second image
imagePoints2 = detectSIFTFeatures(im2gray(I2), 'ContrastThreshold', contrast_threshold);

% Filter SIFT features based on ROI for the second image
validIdx2 = imagePoints2.Location(:, 1) >= xmin & imagePoints2.Location(:, 1) <= xmax & ...
            imagePoints2.Location(:, 2) >= ymin & imagePoints2.Location(:, 2) <= ymax;
imagePoints2 = imagePoints2(validIdx2);

% Extract SIFT descriptors for both images
[features1, validPoints1] = extractFeatures(im2gray(I1), imagePoints1);
[features2, validPoints2] = extractFeatures(im2gray(I2), imagePoints2);

% Match features using descriptors
indexPairs = matchFeatures(features1, features2, 'MaxRatio', 0.8, 'Unique', true);

% Retrieve matched points
matchedPoints1 = validPoints1(indexPairs(:, 1)).Location;
matchedPoints2 = validPoints2(indexPairs(:, 2)).Location;

% Visualize matched features after ROI filtering
%figure;
%showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
%title("Matched Features");

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
cameraSize = 1;
figure
plotCamera(Size=cameraSize, Color="r", Label="1", Opacity=0);
hold on
grid on
plotCamera(AbsolutePose=relPose, Size=cameraSize, ...
    Color="b", Label="2", Opacity=0);

% Visualize the point cloud
pcshow(ptCloud, VerticalAxis="y", VerticalAxisDir="down", MarkerSize=45);

% Rotate and zoom the plot
%camorbit(0, -30);
%camzoom(1.5);

% Label the axes
xlabel("x-axis");
ylabel("y-axis");
zlabel("z-axis")

title("Up to Scale Reconstruction of the Scene");
