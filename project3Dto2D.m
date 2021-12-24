%A function to project the 3d points to 2d. This function is called in the
%mainProject2 file. It takes in the input arguments - World Points,
%Calibration information for both cameras and 2 image frames. The output is
%2d projections for both camera views.

function [img1_2d, img2_2d] = project3Dto2D(worldPoints, CalibInfo1, CalibInfo2, vidFrame1, vidFrame2)

P1 = CalibInfo1.Pmat;
K1 = CalibInfo1.Kmat;
P2 = CalibInfo2.Pmat;
K2 = CalibInfo2.Kmat;

imgPointsTemp1 = K1*P1*worldPoints;
img1_2d = imgPointsTemp1(1:2,:)./repmat(imgPointsTemp1(3,:),2,1);
imgPointsTemp2 = K2*P2*worldPoints;
img2_2d = imgPointsTemp2(1:2,:)./repmat(imgPointsTemp2(3,:),2,1);

%Plot 2d
%vue2
figure; image(vidFrame1); title('3d to 2d projection for Image 1');
for i =1:3:size(img1_2d,2)
    hold on;
    plot(img1_2d(1,i:i+2),img1_2d(2,i:i+2),'g-*'); %connecting 3 joints in both arms and legs
end
plot([img1_2d(1,1),img1_2d(1,4)],[img1_2d(2,1),img1_2d(2,4)],'g-*');  %right to left shoulder
plot([img1_2d(1,7),img1_2d(1,10)],[img1_2d(2,7),img1_2d(2,10)],'g-*'); %right to left hip
mid_shoulder2 = (img1_2d(:,1)+ img1_2d(:,4))/2;  % mid-shoulder
mid_hip2 = (img1_2d(:,7)+ img1_2d(:,10))/2; % mid-hip
mid_points2 = [mid_shoulder2 mid_hip2];
plot(mid_points2(1,:),mid_points2(2,:),'g-*'); %connecting mid point of shoulder and hip

% vue4
figure; image(vidFrame2); title('3d to 2d projection for Image 2');
for i= 1:3:size(img2_2d,2)
    hold on;
    plot(img2_2d(1,i:i+2),img2_2d(2,i:i+2),'y-*');  %connecting joints in both arms and legs
end
plot([img2_2d(1,1),img2_2d(1,4)],[img2_2d(2,1),img2_2d(2,4)],'y-*');  %right to left shoulder
plot([img2_2d(1,7),img2_2d(1,10)],[img2_2d(2,7),img2_2d(2,10)],'y-*');  %right to left shoulder
mid_shoulder4 = (img2_2d(:,1)+img2_2d(:,4))/2;  % mid-shoulder
mid_hip4 = (img2_2d(:,7)+img2_2d(:,10))/2; % mid-hip
mid_points4 = [mid_shoulder4 mid_hip4];
plot(mid_points4(1,:),mid_points4(2,:),'y-*'); %connecting mid point of shoulder and hip

end

