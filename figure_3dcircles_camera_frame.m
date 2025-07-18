%% draw result w.r.t camera frame
function figure_3dcircles_camera_frame(circle_centers, surface_normals)
    figure;
    hold on;
    quiver3(circle_centers(1, 1), circle_centers(2, 1), circle_centers(3, 1), surface_normals(1, 1), surface_normals(2, 1), surface_normals(3, 1), 'b');
    quiver3(circle_centers(1, 2), circle_centers(2, 2), circle_centers(3, 2), surface_normals(1, 2), surface_normals(2, 2), surface_normals(3, 2), 'r');
    plot_inertial_frame(1);
    plot3(circle_centers(1, 1), circle_centers(2, 1), circle_centers(3, 1), 'bo');
    plot3(circle_centers(1, 2), circle_centers(2, 2), circle_centers(3, 2), 'ro');
    title('3D Visualization w.r.t camera frame');
    xlabel('X-axis');
    ylabel('Y-axis');
    zlabel('Z-axis');
    axis equal;
    f = FigureRotator();
    hold off;
end