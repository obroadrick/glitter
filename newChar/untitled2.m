%saves the literal light position at the literal directory 
mmPerInch = 25.4;

% light position:
%lightPos10 = [9.7-0.21 8.86-58.5+9*18.25 310] + (10) * [mmPerInch 0 0];%first of 10 positions with screws
lightPosOrig = [(9.7-2.1) ((88.6-58.5)+8*18.25) -310.0];%first of 10 positions with screws

lightPos = lightPosOrig + (i-1) * [mmPerInch 0 0];

% experiment directory:
expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/ICCV_camPos1/';
% save :
save([expir 'lightPos' num2str(i) '.mat'], "lightPos");
