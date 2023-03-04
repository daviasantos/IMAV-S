mydir    = pwd;                                  % get current path
idcs     = strfind(mydir,'MATLAB');              % find the substring 'MATLAB'
exDir    = mydir(1:idcs(end)-1);                 % subtract 'MATLAB'
pathgame = strcat(exDir,'GAME');                 % path of the game
command  = strcat( 'cmd /C cd &', pathgame );    % go to the path of the game

status1   = system( command );                    % run the game exe


status2   = system( 'cmd /C taskkill/im IMAVMArena.exe' );                    % run the game exe


