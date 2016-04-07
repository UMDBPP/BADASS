function sendCmd(cmdtype,varargin)

    if(~evalin('base','exist(''serConn'',''var'')'))
        error('sendCmd:serConnDoesntExist','serConn doesn''t exist is base workspace, are you sure the serial connection is open?');
    end
    if(~evalin('base','exist(''logfile'',''var'')'))
        error('sendCmd:logfileDoesntExist','logfile doesn''t exist is base workspace, are you sure the serial connection is open?');
    end
    
    serConn = evalin('base','serConn');
    logfile = evalin('base','serConn');
    
    % create the command header
    arr = CreateCmdHdr(1, 1, 0, 8, cmdtype);
    
    fwrite(serConn,arr);
    
    % output it to the command line
    fprintf('S %s: ', datestr(now,'HH:MM:SS.FFF'));
    fprintf(logfile,'S %s: ', datestr(now,'HH:MM:SS.FFF'));

    for i=1:length(arr)
        fprintf('%s',dec2hex(arr(i)));
        fprintf(logfile,'%s',dec2hex(arr(i)));
        if(i~=length(arr))
            fprintf(',');
            fprintf(logfile,',');
        end
    end
    fprintf('\n');
    fprintf(logfile,'\n');
    
end