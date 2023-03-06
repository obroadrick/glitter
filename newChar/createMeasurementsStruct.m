% creates and saves measurements struct
% only need to run this when we update/add measurements
function M = createMeasurementsStruct()
    % directly measured distances (by Oliver, poorly, with named tool)
    M.GLIT_TO_MON_PLANES = 628;%MEASURED (laser)
    M.GLIT_TO_MON_EDGES_X = 126;%MEASURED (jenky laser rig with calipers)
    M.GLIT_TO_MON_EDGES_Y = 83;%MEASURED (jenky laser rig with calipers)



    M.MON_WIDTH_MM = 664.32;%computed (confirmed with measure)
    M.MON_HEIGHT_MM = 373.68;%computed (confirmed with measure)

    M.GLIT_WIDTH = 457.2;%computed as 18in to mm (and measured to confirm)
    M.GLIT_HEIGHT = 304.8;%computed as 12in to mm (and measured to confirm)
    M.CALIBRATION_SQUARE_SIZE = 147.1 / 6;%MEASURED six of them (calipers) = 24.5166666667
    M.CALIBRATION_BOARD_THICKNESS = 3.2;%MEASURED (calipers)
    %M.FIDUCIAL_MARKER_TO_EDGE = 10.3;%MEASURED (calipers)
    %M.FIDUCIAL_MARKER_SIZE = 15.3;%MEASURED (calipers)
    M.FIDUCIAL_MARKER_TO_EDGE = 10.2305;%computed from addy
    M.FIDUCIAL_MARKER_SIZE = 15.1694;%computed from addy
    M.BOARD_LINE_WIDTH = 5.3003;%computed line width on glitter board (Addy)

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
    
    % save struct to file
    save("data/measurements","M");
    save("/Users/oliverbroadrick/Desktop/glitter-stuff/sep19characterization(new-2)/measurements.mat","M");
    %save("/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_middle/measurements.mat","M");
    save("/Users/oliverbroadrick/Desktop/glitter-stuff/oct25_nikonz7_35mm/measurements.mat","M");

end