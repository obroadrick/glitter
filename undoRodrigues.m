function rodrig = undoRodrigues(R)
    % get rorigues parameters from rotation matrix
    axang = rotm2axang(R);
    rodrig = axang(1:3) / norm(axang(1:3)) * axang(4);
end