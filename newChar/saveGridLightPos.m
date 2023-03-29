% Computes the grid of 6 light positions and saves them
mmPerInch = 25.4;

lightPos1 = [18.22 (68.60-58.7+8*18.25) -187];
lightPos2 = lightPos1 + 1 * [4*mmPerInch 0 0] - 0 * [0 0 3*mmPerInch];
lightPos3 = lightPos1 + 2 * [4*mmPerInch 0 0] - 0 * [0 0 3*mmPerInch];
lightPos4 = lightPos1 + 0 * [4*mmPerInch 0 0] - 1 * [0 0 3*mmPerInch];
lightPos5 = lightPos1 + 1 * [4*mmPerInch 0 0] - 1 * [0 0 3*mmPerInch];
lightPos6 = lightPos1 + 2 * [4*mmPerInch 0 0] - 1 * [0 0 3*mmPerInch];

% experiment directory:
expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/ICCV_camPos1/';
% save :
lightPos = lightPos1; save([expir 'lightPos1.mat'], "lightPos");
lightPos = lightPos2; save([expir 'lightPos2.mat'], "lightPos");
lightPos = lightPos3; save([expir 'lightPos3.mat'], "lightPos");
lightPos = lightPos4; save([expir 'lightPos4.mat'], "lightPos");
lightPos = lightPos5; save([expir 'lightPos5.mat'], "lightPos");
lightPos = lightPos6; save([expir 'lightPos6.mat'], "lightPos");
