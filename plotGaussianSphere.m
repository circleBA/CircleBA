addpath(genpath(pwd))

vector= [-0.9896 0.1405 -0.030524];
colors = ['r', 'm', 'b', 'c'];
figure;
plot_unit_sphere(1, 18, 0.8); hold on; grid on; axis equal;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
f = FigureRotator(gca());
k = 1;
for i=1:2
    for j = 1:2
        plot3([0b surface_normals{i}(1, j)],[0 surface_normals{i}(2, j)],[0 surface_normals{i}(3, j)] ,'Color' ,colors(k),'LineWidth', 4); hold on;
        k = k+1;
    end
end