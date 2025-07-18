function plot_inertial_frame(magnitude)


% center point of inertial frame
% X = x;
% Y = y;
% Z = z;
X = 0;
Y = 0;
Z = 0;


% [X Y Z] axis end points of inertial frame
X_axis = [X+magnitude;Y;Z];
Y_axis = [X;Y+magnitude;Z];
Z_axis = [X;Y;Z+magnitude];


% draw inertial frame
line([X_axis(1) X],[X_axis(2) Y],[X_axis(3) Z],'Color','r','LineWidth',2)
line([Y_axis(1) X],[Y_axis(2) Y],[Y_axis(3) Z],'Color','g','LineWidth',2)
line([Z_axis(1) X],[Z_axis(2) Y],[Z_axis(3) Z],'Color','b','LineWidth',2)


end