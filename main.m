%The main project has all parts of the project, i.e. Reading data,
%projecting points from 3d to 2d and reconstructing, evaluating the error
%parameters and finally plotting the epipolar lines. All this has been done for
%the error frame number 1000.

clc;
clearvars;
profile on;
%%Read Data
%Load 3D joint data
load("Subject4-Session3-Take4_mocapJoints.mat");

%Selecting a frame and finding the joint data
%Select example frames numbers 1000 and 2000, and also the frame numbe=
mocapFnum = 17150; %mocap frame number 1000
x = mocapJoints(mocapFnum,:,1); %array of 12 X coordinates
y = mocapJoints(mocapFnum,:,2); % Y coordinates
z = mocapJoints(mocapFnum,:,3); % Z coordinates
worldPoints = [x;y;z;ones(1,12)]; %world points
conf = mocapJoints(mocapFnum,:,4); %confidence values

%Load camera parameters
load("vue2CalibInfo.mat");
load("vue4CalibInfo.mat");

%Frame Extraction
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
vue2video = VideoReader(filenamevue2mp4);
vue2video.CurrentTime = (mocapFnum-1)*(50/100)/vue2video.FrameRate;
vid2Frame = readFrame(vue2video);
figure; image(vid2Frame); title('Image 1 Frame');

filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';
vue4video = VideoReader(filenamevue4mp4);
vue4video.CurrentTime = (mocapFnum-1)*(50/100)/vue4video.FrameRate;
vid4Frame = readFrame(vue4video);
figure; image(vid4Frame); title('Image 2 Frame');


%------------------------------------------------------------------------------------------
%%Call functions
%Projecting 3d points to 2d
[img1_2d, img2_2d] = project3Dto2D(worldPoints, vue2, vue4, vid2Frame, vid4Frame); 

%Reconstructing 2d points back to 3d
points3d = triangulate(img1_2d, img2_2d, vue2, vue4, worldPoints);

%------------------------------------------------------------------------------------------
%%Error evaluation
%Error evaluation
N = size(mocapJoints,1);
error = zeros(N,12); %per frame per joint error
error2 = zeros(N,1); %error of all joint pairs

for mocapFnum = 1:N
    x = mocapJoints(mocapFnum,:,1);
    y = mocapJoints(mocapFnum,:,2);
    z = mocapJoints(mocapFnum,:,3);
    conf = mocapJoints(mocapFnum,:,4);
    if (sum(conf)~=12) 
        continue;             %skip the iteration if all confidence values are not 1
    end
    
    worldPoints = [x;y;z;ones(1,12)];
    P1 = vue2.Pmat;
    K1 = vue2.Kmat;
    P2 = vue4.Pmat;
    K2 = vue4.Kmat;
    
    %3d to 2d
    imgPointsTemp1 = K1*P1*worldPoints;
    img1_2d = imgPointsTemp1(1:2,:)./repmat(imgPointsTemp1(3,:),2,1);
    imgPointsTemp2 = K2*P2*worldPoints;
    img2_2d = imgPointsTemp2(1:2,:)./repmat(imgPointsTemp2(3,:),2,1);
   
    %2d to 3d
    A1 = [];
    A2 = [];
    points3d = [];
    projMat1 = K1*P1;
    projMat2 = K2*P2;

    for i = 1:size(img1_2d,2)
        A1 = [img1_2d(2,i)*projMat1(3,:) - projMat1(2,:); projMat1(1,:) - img1_2d(1,i)*projMat1(3,:)];
        A2 = [img2_2d(2,i)*projMat2(3,:) - projMat2(2,:); projMat2(1,:) - img2_2d(1,i)*projMat2(3,:)];
        A = [A1;A2];
        [V,~] = eigs(A'*A);
        temp = V(:,4)/V(4,4);
        points3d = [points3d, temp];
    end
    
    %error of each joint pair
    diff = points3d(1:3,:)- worldPoints(1:3,:);
    error = sqrt(sum(diff.^2,1));
    errorFrames(mocapFnum,:) = error;
    
    %error of all joints as one
    points3d_2 = sum(points3d,2);
    worldPoints_2 = sum(worldPoints(1:3,:),2);
    diff_2 = points3d_2(1:3,:) - worldPoints_2(1:3,:);
    error_2 = sqrt(sum(diff_2.^2));
    errorFrames2(mocapFnum,:) = error_2;
end

%error of each joint pair
numFrames = size(errorFrames,1); 
meanErr = sum(errorFrames,1)/numFrames;
minErr = min(errorFrames,[],1);
maxErr = max(errorFrames,[],1);
medianErr = median(errorFrames,1);
stdDevErr = std(errorFrames,0,1);

%error of all joints as one
numFrames2 = size(errorFrames2,1); 
meanErr2 = sum(errorFrames2,1)/numFrames2;
minErr2 = min(errorFrames2,[],1);
maxErr2 = max(errorFrames2,[],1);
medianErr2 = median(errorFrames2,1);
stdDevErr2 = std(errorFrames2,0,1);

%plot
sumErr = sum(errorFrames,2);
figure; 
plot(1:numFrames,sumErr(:,1));
title('Total Error');
xlabel('Frame');
ylabel('Error');

%Frame with minimum and maximum total error for all joints
[~,indMin] = min(errorFrames2);
[~,indMax] = max(errorFrames2);

%------------------------------------------------------------------------------------------
%%Plot Epipolar Lines
%Fundamental Matrix using 8 point algorithm
X1 = img1_2d(1,:);
Y1 = img1_2d(2,:);
X2 = img2_2d(1,:);
Y2 = img2_2d(2,:);

a1 = X1.*X2;
a2 = X1.*Y2;
a3 = X1;
a4 = Y1.*X2;
a5 = Y1.*Y2;
a6 = Y1;
a7 = X2;
a8 = Y2;
a9 = ones(1, 12);

A = [a1' a2' a3' a4' a5' a6' a7' a8' a9'];
A = A(1:8,:);

[~,~,V] = svd(A);
F1 = V(:, end);
F1 = reshape(F1,[3,3]);
[U,S,V] = svd(F1);
S(3,3) = 0;
F = U*S*V';

[sizey, sizex] = size(vid2Frame);
colorString = 'rgbcykrgbcyk';

figure; 
image(vid2Frame); title('Epipolar Lines for Image 1');
hold on;
for i =1:12
    line1(:,i) = F'*[X1(1,i);Y1(1,i);ones(1,1)];
    x1 = 0;
    x2 = sizex;
    y1(i) = (-x1*line1(1,i)-line1(3,i))/line1(2,i);
    y2(i) = (-x2*line1(1,i)-line1(3,i))/line1(2,i);
    plot([x1, x2],[y1(i),y2(i)], 'Color', colorString(i), 'LineWidth',2);
    hold on;
end

figure;
image(vid4Frame);title('Epipolar Lines for Image 2');
hold on;
for i = 1:12
    line2(:,i) = F*[X1(1,i);Y1(1,i);ones(1,1)];
    x1 = 0;
    x2 = sizex;
    y1(i) = (-x1*line2(1,i)-line2(3,i))/line2(2,i);
    y2(i) = (-x2*line2(1,i)-line2(3,i))/line2(2,i);
    plot([x1, x2],[y1(i),y2(i)], 'Color', colorString(i), 'LineWidth',2);
    hold on;
end

p = profile('info');
save myprofiledata p
