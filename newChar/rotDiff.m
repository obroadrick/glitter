function diff = rotDiff(R1, R2)
    % computes and returns the difference (diff) in degrees of the
    % rotations described by the rotation matrices R1 and R2
    axang = rotm2axang(R1 * R2');
    diff = (180 / pi) * axang(4);
end