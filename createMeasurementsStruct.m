% creates and saves measurements struct
% only need to run this when we update/add measurements
function M = createMeasurementsStruct()
    % directly measured distances (by Oliver, poorly, with a yardstick)
    M.GLIT_TO_MON_PLANES = 508;%MEASURED
    M.GLIT_TO_MON_EDGES_X = 194;%MEASURED
    M.GLIT_TO_MON_EDGES_Y = 71;%MEASURED
    M.MON_WIDTH_MM = 670;%MEASURED
    M.MON_HEIGHT_MM = 379;%MEASURED
    M.GLIT_SIDE = 305;%MEASURED

    % other useful constants
    M.MON_WIDTH_PXS = 3840;
    M.MON_HEIGHT_PXS = 2160;
    M.PX2MM_X = M.MON_WIDTH_MM / M.MON_WIDTH_PXS; 
    M.PX2MM_Y = M.MON_HEIGHT_MM / M.MON_HEIGHT_PXS; 
    M.INDEX_TO_PX = 15; % from Addy (confirmed) 
    M.FIRST_INDEX_PX_X = 20; % from Addy (confirmed)
    M.FIRST_INDEX_X = M.FIRST_INDEX_PX_X * M.PX2MM_X;
    M.FIRST_INDEX_PX_Y = 20; % from Addy (confirmed)
    M.FIRST_INDEX_Y = M.FIRST_INDEX_PX_Y * M.PX2MM_Y;
    
    % save struct to file
    save("data/measurements","M");
end