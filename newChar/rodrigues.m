% get rotation matrix from rodrigues parameters
function R = rodrigues(r1,r2,r3)
    % an alternative implementation to test/check
    ang = norm([r1 r2 r3]);
    ax = [r1 r2 r3] ./ ang;
    R = axang2rotm([ax ang]);

    %{
    omega = [r1;r2;r3]; % the vector we parameterized
    theta = norm(omega); % angle to rotate by (following right hand rule)
    n = omega ./ theta; % axis of rotation normalized
    nx = [0 -n(3) n(2); n(3) 0 -n(1); -n(2) n(1) 0]; % cross product as a matrix
    % rodrigues' formula to get rotation matrix from the parameters
    R = [1 0 0; 0 1 0; 0 0 1] + sin(theta).*nx + (1-cos(theta)).*nx^2;
    %}
end