%saves the literal light position at the literal directory 

% light position:
%lightPos = [1.6, 153.5-151.9, 491];%chemlab light
%lightPos = [457.2-24.7, 275-151.5, 497];%cubesat light%M.GLIT_WIDTH=457.2
lightPos = [78.6, 128.6-73.0, 511];%first of 10 positions with screws

% experiment directory:
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_middle/';
expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/jan12data/';

% save :
save([expir 'lightPos10.mat'], "lightPos");
%save([expir 'cubesatLightPos.mat'], "lightPos");