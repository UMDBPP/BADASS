function arr = sendCmd(APID,FcnCode,varargin)

    % get streams from base workspace
    if(~evalin('base','exist(''serConn'',''var'')'))
        error('sendCmd:serConnDoesntExist','serConn doesn''t exist is base workspace, are you sure the serial connection is open?');
    end
    if(~evalin('base','exist(''logfile'',''var'')'))
        error('sendCmd:logfileDoesntExist','logfile doesn''t exist is base workspace, are you sure the serial connection is open?');
    end
    
    serConn = evalin('base','serConn');
    logfile = evalin('base','logfile');
    
    % create the command header
    SeqCnt = 1;
    SegFlag = 3;
    PktLen = 8; 
    arr = CreateCmdHdr(APID, SeqCnt, SegFlag, PktLen, FcnCode);

    % command definitions
    CMD_SetTlmCtrl = hex2dec('01');
    CMD_SetCycTime = hex2dec('02');
    CMD_SetTargetNED = hex2dec('03');
    CMD_SetIMU2BODY = hex2dec('04');
    CMD_SetServoEnable = hex2dec('05');
    CMD_SetRWEnable = hex2dec('06');
    CMD_RequestTlmPt = hex2dec('07');
    CMD_SendTestPkt =  	hex2dec('08');
	CMD_SetTlmAddr =	hex2dec('09');
	CMD_SetElPolarity = hex2dec('0A');

    uint32_arg = [CMD_SetTlmCtrl CMD_RequestTlmPt];
    uint16_arg = [CMD_SetCycTime];
    uint8_arg = [CMD_SetServoEnable CMD_SetRWEnable CMD_SetElPolarity CMD_SetTlmAddr];
    no_arg = [CMD_SendTestPkt];
    three_float_args = [CMD_SetTargetNED];
    four_float_args = [CMD_SetIMU2BODY];
     
    % 1 uint32 argument
    if(ismember(FcnCode,uint32_arg))
        if(nargin ~= 3)
           error('sendCmd:IncorrectNumberOfFcnArg','Incorrect number of arguments for fcncode %d',FcnCode);
        end
        arr(9:12) = typecast(uint32(varargin{1}),'uint8');
    end
    % 1 uint16 argument
    if(ismember(FcnCode,uint16_arg))
        if(nargin ~= 3)
           error('sendCmd:IncorrectNumberOfFcnArg','Incorrect number of arguments for fcncode %d',FcnCode);
        end
        arr(9:10) = typecast(swapbytes(uint16(varargin{1})),'uint8');
    end
    % 1 uint8 argument
    if(ismember(FcnCode,uint8_arg))
        if(nargin ~= 3)
           error('sendCmd:IncorrectNumberOfFcnArg','Incorrect number of arguments for fcncode %d',FcnCode);
        end
        arr(9) = typecast(uint8(varargin{1}),'uint8');
    end
    % no arguments
    if(ismember(FcnCode,no_arg))
        if(nargin ~= 2)
           error('sendCmd:IncorrectNumberOfFcnArg','Incorrect number of arguments for fcncode %d',FcnCode);
        end
    end
    
    % update the length of the packet
    arr(5:6) = typecast(swapbytes(uint16(length(arr)-7)),'uint8');
    
    % update the packet checksum
    arr(7) = calcChecksum(arr);
    
    % write the packet
    fwrite(serConn,arr);

    % log the packet
    fprintf('S %s: ', datestr(now,'HH:MM:SS.FFF'));
    fprintf(logfile,'S %s: ', datestr(now,'HH:MM:SS.FFF'));

    for i=1:length(arr)
        fprintf('%02s',dec2hex(arr(i)));
        fprintf(logfile,'%02s',dec2hex(arr(i)));
        if(i~=length(arr))
            fprintf(',');
            fprintf(logfile,',');
        end
    end
    fprintf('\n');
    fprintf(logfile,'\n');
    
end