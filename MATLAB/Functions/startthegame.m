
disp('Starting the game...');
disp(' ');


mydir    = pwd;                                 % get current path
idcs     = strfind(mydir,'MATLAB');             % find the substring 'MATLAB'
exDir    = mydir(1:idcs(end)-1);                % subtract 'MATLAB'
pathgame = strcat(exDir,'GAME\IMAVMArena.exe && exist &'); % go to the path of the game
command  = strcat( 'cmd /C', pathgame );        % build the command
status   = system( command );                    % run the game exe


