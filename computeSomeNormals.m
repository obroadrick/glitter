% computes the glitter specs' surface normals given already found
% spec locations and brightness distribution gaussian means
% inputs: P, matlab struct with paths to the data (means, spec locs)
function normals = computeSomeNormals(lightingMeans, specPos, camPos, charM)
    %{
    means = matfile(P.means).means;
    C = matfile(P.imageCentroids).imageCentroids;
    %M = matfile(P.measurements).M;
    %M = matfile([chardir 'measurements.mat']).M;
    M = charM;% now an argument so that re-characterizing according to new measurement sets is easy
    cam = matfile([chardir 'camPos.mat']).camPos;
    %}
    means = lightingMeans;
    cam = camPos;
    M = charM;
    specPos = specPos;

    %% spec to lightsource vectors in glitter coords
    % map the gaussian means to lighting positions
    howmany = size(specPos,1);
    x = (-M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - (M.FIRST_INDEX_X + (M.PX2MM_X * M.INDEX_TO_PX .* (means(1,:)-1))))'; 
    y = (-M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - (M.FIRST_INDEX_Y + (M.PX2MM_Y * M.INDEX_TO_PX .* (means(2,:)-1))))'; 
    z = zeros(howmany,1) + M.GLIT_TO_MON_PLANES;
    lightPos = [x y z];
    
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

    normals = specNormals;
    
    %{
    specNormals = spec2light + spec2cam; %just add since they are normalized
    specNormals = specNormals ./ vecnorm(specNormals, 2, 2);
    specNormalsPath = [P.data 'spec_normals_' datestr(now, 'mm_dd_yyyy')];
    save(specNormalsPath, "specNormals");
    specNormalsPath = [chardir 'spec_normals.mat'];
    save(specNormalsPath, "specNormals");
    %}

    %{
    % as a major sanity check:
    % draw a single incoming ray and reflect ray and the computed surface
    % normal for it
    figure;
    ix = [1];
    plot3([specPos(ix,1) specPos(ix,1)+spec2light(ix,1)], [specPos(ix,2) specPos(ix,2)+spec2light(ix,2)], [specPos(ix,3) specPos(ix,3)+spec2light(ix,3)]);
    hold on;
    plot3([specPos(ix,1) specPos(ix,1)+spec2cam(ix,1)], [specPos(ix,2) specPos(ix,2)+spec2cam(ix,2)], [specPos(ix,3) specPos(ix,3)+spec2cam(ix,3)],'color','red');
    plot3([specPos(ix,1) specPos(ix,1)+specNormals(ix,1)], [specPos(ix,2) specPos(ix,2)+specNormals(ix,2)], [specPos(ix,3) specPos(ix,3)+specNormals(ix,3)],'color','green');
    %}

    %% return the spec normals path... already set
end