%saves the literal light position at the literal directory 

% light position:
lightPos = [1.6, 153.5-151.9, 491];%chemlab light
%lightPos = [457.2-24.7, 275-151.5, 497];%cubesat light%M.GLIT_WIDTH=457.2

% experiment directory:
expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_middle/';


% save :
save([expir 'chemLightPos.mat'], "lightPos");
%save([expir 'cubesatLightPos.mat'], "lightPos");