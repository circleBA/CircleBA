mu = [0 0];
Sigma = [4 1.5; 1.5 3];
hold on; axis equal;

for k = 1:3
    [X, Y] = ellipse_from_covariance(mu, Sigma, k);
    plot(X, Y, 'LineWidth', 2);
end
legend('1σ', '2σ', '3σ');
title('Confidence Ellipses from PCA (Covariance)');

function [X, Y] = ellipse_from_covariance(mu, Sigma, scale)
    % mu: [x0, y0] 중심
    % Sigma: 2x2 공분산 행렬
    % scale: 몇 σ (e.g., 1 for 68%, 2 for 95%, 3 for 99.7%)

    theta = linspace(0, 2*pi, 100);  % 타원 각도
    circle = [cos(theta); sin(theta)];  % 단위 원

    % 고유분해
    [eigvec, eigval] = eig(Sigma);

    % scale factor 적용 (1σ, 2σ, 3σ 등)
    ellipse_shape = scale * eigvec * sqrt(eigval) * circle;

    % 중심 위치로 이동
    X = ellipse_shape(1,:) + mu(1);
    Y = ellipse_shape(2,:) + mu(2);
end
