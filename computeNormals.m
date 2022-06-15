% computes the glitter specs' surface normals given already found
% spec locations and brightness distribution gaussian means
% inputs: P, matlab struct with paths to the data (means, spec locs)
function specNormalsPath = computeNormals(P)
    means = matfile(P.means).means;
    C = matfile(P.imageCentroids).imageCentroids;
    M = matfile(P.measurements).M;
    cam = matfile(P.camPos).camera_in_glitter_coords;
    % seed the rng
    seed = 125;
    rng(seed);
    %% spec to lightsource vectors in glitter coords
    % map the gaussian means to lighting positions
    howmany = size(C,1);
    cxs = [1:size(C,1)];
    x = (-M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - (M.FIRST_INDEX_X + (M.PX2MM_X * M.INDEX_TO_PX .* (means(1,cxs)-1))))'; 
    y = (-M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - (M.FIRST_INDEX_Y + (M.PX2MM_Y * M.INDEX_TO_PX .* (means(2,cxs)-1))))'; 
    z = zeros(howmany,1) + M.GLIT_TO_MON_PLANES;
    lightPos = [x y z];
    
    % get the spec locations in canonical glitter coords
    specPos = matfile(P.canonicalCentroids).canonicalCentroids;
    
    % vector from spec to light
    spec2light = lightPos - specPos;
    
    % normalize
    spec2light = spec2light ./ vecnorm(spec2light, 2, 2);
    
    % vector from spec to camera
    spec2cam = cam - specPos;
    
    % normalize
    spec2cam = spec2cam ./ vecnorm(spec2cam, 2, 2);
    
    % spec surface normals 
    specNormals = spec2light + spec2cam; %just add since they are normalized
    specNormals = specNormals ./ vecnorm(specNormals, 2, 2);
    specNormalsPath = [P.data 'spec_normals_' datestr(now, 'mm_dd_yyyy')];
    save(specNormalsPath, "specNormals");
    
    %% pretty picture of it all
    % draw the glitter rig with these lines showing
    %number_to_draw = 10;
    %draw_idxs = randi(size(C,1),number_to_draw,1);
    %drawRig(M, lightPos(draw_idxs,:), specPos(draw_idxs,:), cam);

    %% return the spec normals path... already set
end