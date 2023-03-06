% for a position on the monitor and a given set of measurements of where
% the monitor is located in the scene relative to the canonical glitter
% coordinates, this function gives the same position in the canonical
% glitter coordinate system

function lightPos = screenPosToWorldPos(pos, M)
    % assumes the glitter plane and monitor plane are parallel
    x = (-M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - (M.FIRST_INDEX_X + (M.PX2MM_X * (pos(1)-1))))';
    y = (-M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - (M.FIRST_INDEX_Y + (M.PX2MM_Y * (pos(2)-1))))';
    z = 0 + M.GLIT_TO_MON_PLANES;
    lightPos = [x y z];
end