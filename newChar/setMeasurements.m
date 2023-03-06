% creates and saves measurements struct
% only need to run this when we update/add measurements
function M = setMeasurements(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,GLIT_TO_MON_EDGES_Y)
    % directly measured distances (by Oliver, poorly, with named tool)
    M.GLIT_TO_MON_PLANES = GLIT_TO_MON_PLANES;%MEASURED (laser)
    M.GLIT_TO_MON_EDGES_X = GLIT_TO_MON_EDGES_X;%MEASURED (jenky laser rig with calipers)
    M.GLIT_TO_MON_EDGES_Y = GLIT_TO_MON_EDGES_Y;%MEASURED (jenky laser rig with calipers)
    M.MON_WIDTH_MM = 664.32;%computed (and measured to confirm)
    M.MON_HEIGHT_MM = 373.68;%computed (and measured to confirm)
    M.CALIBRATION_SQUARE_SIZE = 146.0 / 8;%MEASURED 8 of them (calipers) = 24.5166666667
  
    % other useful constants
    M.MON_WIDTH_PXS = 3840;
    M.MON_HEIGHT_PXS = 2160;
    M.PX2MM_X = M.MON_WIDTH_MM / M.MON_WIDTH_PXS; 
    M.PX2MM_Y = M.MON_HEIGHT_MM / M.MON_HEIGHT_PXS;
    M.INDEX_TO_PX = 5; % from Addy (reconfirmed) 
    M.FIRST_INDEX_PX_X = 20; % from Addy (reconfirmed)
    M.FIRST_INDEX_X = M.FIRST_INDEX_PX_X * M.PX2MM_X;
    M.FIRST_INDEX_PX_Y = 20; % from Addy (reconfirmed)
    M.FIRST_INDEX_Y = M.FIRST_INDEX_PX_Y * M.PX2MM_Y;
    M.XRES = 8256;
    M.YRES = 5504;
end
