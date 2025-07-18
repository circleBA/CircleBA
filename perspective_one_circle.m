%perspective one circle main function
function [circle_centers, surface_normals, C_quad] = perspective_one_circle(ellipses_params, R)
    k = ellipses_params(1); %y
    h = ellipses_params(2); %x
    a = ellipses_params(3)/2; %a
    b = ellipses_params(4)/2; %b
    angle = 90-ellipses_params(5); %angle (deg)
    % ellipse equation to quadratic form 
    [A, B, C, D, E, F] = calculate_ellipse_coefficients(h, k, a, b, angle);
    C_quad = [A, C/2, D/2; C/2, B, E/2; D/2, E/2, F]; 

    % Cone Equation
    Q = intrinsics.K'* C_quad * intrinsics.K;

    % eigen value and normalized eigen vector of Q
    [V, D] = eig(Q); % eigen value D, eigen vector V
    % Normalize the eigenvectors
    for j = 1:size(V, 2)
        V(:, j) = V(:, j) / norm(V(:, j));
    end
    
    [U, lam] = find_transform_cone_cam(V, D);

    % circle center has two solution
    circle_centers = find_circle_center_position(U, lam, R);
    % surface normal has two solution corresponding 
    surface_normals = find_circle_surface_normal(U, lam);

end