clear

% Load image pair
imageDir = fullfile("/home/fekete/Documents/DSO/images/");
images = imageDatastore(imageDir);
I1 = readimage(images, 70);
I2 = readimage(images, 71);
figure
imshowpair(I1, I2, 'montage'); 
title('Original Images');

% Load camera parameters
data = load("/home/fekete/Documents/DSO/intrinsics.mat");
intrinsics = data.cameraParams.Intrinsics;

% Undistort images
I1 = undistortImage(I1, intrinsics);
I2 = undistortImage(I2, intrinsics);
figure 
imshowpair(I1, I2, "montage");
title("Undistorted Images");

% Detect SIFT features
validPoints1 = detectSIFTFeatures(im2gray(I1), ContrastThreshold=0.03);
validPoints2 = detectSIFTFeatures(im2gray(I2), ContrastThreshold=0.03);

% Match SIFT features
indexPairs = matchFeatures(features1, features2);
matchedPoints1 = validPoints1(indexPairs(:, 1), :);
matchedPoints2 = validPoints2(indexPairs(:, 2), :);

% Display matched features
figure
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
title("Matched SIFT Features");

% Estimate essential matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, intrinsics, Confidence = 99.99);

% Find inlier points
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

% Display inlier points
figure
showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
title("Epipolar Inliers");

% Compute relative camera pose
relPose = estrepose(E, intrinsics, inlierPoints1, inlierPoints2);

% Remaining code for reconstruction, point cloud creation, etc.
% (Keep this part unchanged unless specific changes are needed)

% Camera matrix calculations, triangulation, etc., remain the same
