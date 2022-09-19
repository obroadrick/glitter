% compareCharacterizations takes two characterizations as input in the form
% (C, N) where C is an array of the canonical (glitter coordinates) spec 
% positions and N is the surface normals. the output is visualizations
% describing the similarity of the two characterizations in order to help
% answer the question: are these two characterizations that are supposed to
% describe the same glitter sheet consistent with each other? if so, that
% gives a bit more faith in the characterizations whereas if not we know
% that there is some characterization inconsistency to address

% get the characterizations
dir1 = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep1characterization/';
C1 = matfile([dir1 'canonical_centroids.mat']).canonicalCentroids;
N1 = matfile([dir1 'spec_normals.mat']).specNormals;
dir2 = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep18characterization/';
C2 = matfile([dir2 'canonical_centroids.mat']).canonicalCentroids;
N2 = matfile([dir2 'spec_normals.mat']).specNormals;

%%
% for each spec in char 1, find the K nearest specs in char 2
K = 4;
[I, D] = knnsearch(C2, C1, 'K', K);

%% 
% for each spec in char 1 find the surface normal distance between it and
% its K nearest specs in C2
Ndiffs = [];
for ix=1:size(I,1)
    for k=1:size(I,2)%also just K
        Ndiffs(ix,k) = compareNs(N1(ix,:),N2(I(ix,k),:));
    end
end

%% 
% for each spec in char 1, find the index of the spec in char 2 which is
% nearest in surface normal degree difference among the K found
[minDs, kmins] = min(Ndiffs,[],2);
for ix=1:size(I,1)
    minIs(ix) = I(kmins(ix));
end

%%
% plot histogram of the differences in degrees
histogram(minDs(minDs < 2), 100);

%% 
% now compare the surface normals of the C1 specs to the most similar among
% the K nearest specs of C2

%% now we can also think about whether matching going the other way gives similar results
% for each spec in char 2, find the K nearest specs in char 1
K = 4;
[Ip, Dp] = knnsearch(C1, C2, 'K', K);

% for each spec



%%
function degs = compareNs(N1,N2)
    %disp(N1);
    %disp(N2);
    %disp(dot(N1,N2) / (norm(N1)*norm(N2)));
    degs = acos(dot(N1,N2) / (norm(N1)*norm(N2))) * 180 / pi;
end


