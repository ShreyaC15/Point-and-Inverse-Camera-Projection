%This function is to reconstruct the 3d points using the 2d point
%projections previously obtained. This fnction is called in the
%mainProject2.

function [points3d] = triangulate(img1_2d, img2_2d, CalibInfo1, CalibInfo2, worldPoints)

P1 = CalibInfo1.Pmat;
K1 = CalibInfo1.Kmat;
P2 = CalibInfo2.Pmat;
K2 = CalibInfo2.Kmat;
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

%Plot the original and reconstructed 3d skeleton
figure;
subplot(1,2,1);
title('Output 3d skeleton');
for i= 1:3:size(points3d,2)
    hold on;
    plot3(points3d(1,i:i+2),points3d(2,i:i+2),points3d(3,i:i+2),'b-*');
end
points1 = [points3d(:,1) points3d(:,4) points3d(:,7) points3d(:,10)]; 
plot3(points1(1,1:2),points1(2,1:2),points1(3,1:2),'b-*',points1(1,3:4),points1(2,3:4),points1(3,3:4),'b-*');
p1 = (points3d(:,1)+ points3d(:,4))/2;
p2 = (points3d(:,7)+ points3d(:,10))/2; 
points2 = [p1 p2];
plot3(points2(1,:),points2(2,:),points2(3,:),'b-*');
 
subplot(1,2,2);
title('Input 3d skeleton');
for i= 1:3:size(worldPoints,2)
    hold on;
    plot3(worldPoints(1,i:i+2),worldPoints(2,i:i+2),worldPoints(3,i:i+2),'g-*');
end
points3 = [worldPoints(:,1) worldPoints(:,4) worldPoints(:,7) worldPoints(:,10)];
plot3(points3(1,1:2),points3(2,1:2),points3(3,1:2),'g-*',points3(1,3:4),points3(2,3:4),points3(3,3:4),'g-*');
p1 = (worldPoints(:,1)+worldPoints(:,4))/2; 
p2 = (worldPoints(:,7)+worldPoints(:,10))/2; 
points4 = [p1 p2];
plot3(points4(1,:),points4(2,:),points4(3,:),'g-*');
end
