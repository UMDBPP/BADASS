function start_serial_monitor(varargin)
% starts the telemetry monitor
%
%   start_serial_monitor()
%       starts the telemetry monitor on COM8 at 9600 baud
%   start_serial_monitor(serPortn)
%       starts the telemetry monitor on the specified serial port at 9600
%       baud
%   start_serial_monitor(serPortn, baud)
%       starts the telemetry monitor of the specified serial port at the
%       specified baud rate
%
%   Sets up a timer callback to periodically check the serial port for new
%   input. Will attempt to identify the beginning of the packet and parse
%   the byte-stream for a CCSDS header and packet format. Will read the
%   telemetry into a database to allow processing.
%

    setupPath()

    % set default inputs
    serPortn = 8;
    baud = 9600;
    
    % check for inputs, override defaults with function inputs
    if(nargin > 0)
        serPortn = varargin{1};
    elseif(nargin > 1)
        baud = varargin{2};
    elseif(nargin > 2)
        error('start_serial_monitor:Too many arguments','Too many arguments supplied');
    end

    % define strings corresponding to serial ports
    serList = [{'COM1'} ; ...
         {'COM2'} ; ...
         {'COM3'} ; ...
         {'COM4'} ; ...
         {'COM5'} ; ...
         {'COM6'} ; ...
         {'COM7'} ; ...
         {'COM8'} ; ...
         {'COM9'} ; ...
         {'COM10'} ; ...
         {'COM11'} ; ...
         {'COM12'} ];

    % get the user-specified port
    serPort = serList{serPortn};
    fprintf('Connecting to serial %s \n',serPort);
    
    % create the serial object
    serConn = serial(serPort);

    % check that the user-entered baud rate makes sense
    valid_baudrates = [300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 250000];
    if(isnumeric(baud))
        if(~ismember(baud,valid_baudrates))
            warning('BADASSCmdTlmMonitor:connectBtnCallback:NonStandardBaud', 'The entered baud rate, ''%d'' is not a standard baud rate and may not work, connecting...',baud)
        else
            fprintf('Using baud rate %d \n',baud);
        end
        set(serConn,'BaudRate',uint32(baud));       
    else
        warning('BADASSCmdTlmMonitor:connectBtnCallback:NonIntBaud', 'The entered baud rate, ''%d'' is not an numeric value, enter a integer value and connect again.',baud)
        return
    end

    % try opening the serial port, if it doesn't work, try closing and
    % reopening
    try
        fopen(serConn);
    catch
        fclose(serConn);
        fopen(serConn);
    end
    
    % open a log file
    logfile = fopen('logs/log.txt','a');
    
    % assign the serial connection to the base workspace so the user can
    % interact with it

    % create timer object
    t=timer;
    t.StartFcn = @initTimer;
    t.TimerFcn = {@timerCallback, serConn, logfile};
    t.StopFcn = {@closeTimer, serConn, logfile};
    t.Period   = 0.5;
    t.ExecutionMode  = 'fixedRate';
    t.ErrorFcn = {@ecallback, serConn, logfile};
    start(t);
    
    % assign timer and serial ports into base workspace 
    assignin('base','timer_obj',t);
    assignin('base','serConn',serConn);
    assignin('base','logfile',logfile);
end

function ecallback(src, event, serConn, logfile)
    
    err = lasterror();
    disp(err);
    disp(err.message);
    disp(err.stack);
    disp(err.identifier);
    
    % close the serial connection
    fclose(serConn)
    delete(serConn)
    evalin('base','clear serConn');

    % close the log file
    fclose(logfile)
    evalin('base','clear logfile');

    % clear the serial and timer objects from the base workspace
    evalin('base','clear timer_obj');
    
end

function initTimer(src, event)
%   initalizes the timer callback's userdata used to store bytes read from
%   the serial port
% 

    % get userdata structure 
    UserData = get(src, 'UserData');
    
    % create the byte buffer
    UserData.ByteBuffer = 0;
    
    disp('Initialised bytebuffer')
    
    % save the userdata structure back into the callback
    set(src, 'UserData',UserData);
    
end

function closeTimer(src, event, serConn, logfile)

    % close the serial connection
    fclose(serConn)
    delete(serConn)
    evalin('base','clear serConn');

    % close the log file
    fclose(logfile)
    evalin('base','clear logfile');

    % clear the serial and timer objects from the base workspace
    evalin('base','clear timer_obj');
    
end

function timerCallback(src, event, serConn, logfile)
% called at timer frequency, reads data from serial port, processes it, and
% saves it into the telemetry database

    % get the userdata structure from the timer callback
	UserData = get(src, 'UserData');
   
    % if there are bytes to read, read them
    if(serConn.BytesAvailable > 0)
        RxText = fread(serConn,serConn.BytesAvailable);

        % convert it from char to integer
        data = uint8(RxText);

        % append the new data to what we've read previously
        UserData.ByteBuffer = [UserData.ByteBuffer; data];

        % define the length of an xbee header
        xbee_hdr_len = 6;

        % look for header bytes
        pkt_loc = strfind(UserData.ByteBuffer.', [hex2dec('08') hex2dec('02')]);
            
        % if a packet was found
        if(~isempty(pkt_loc))

            fprintf('Found pkt at %d \n',pkt_loc)
            
            % extract the packet header
            if(length(UserData.ByteBuffer)-pkt_loc > xbee_hdr_len)
                     
                pkthdr = data(pkt_loc-1:pkt_loc+xbee_hdr_len);
                
                % extract the packet length
                [~, ~, ~, ~, ~, ~, PktLen] = ExtractPriHdr(pkthdr, Endian.Little);
 
                total_pktlen = PktLen+7;
                     
                % if we've received the entire packet, process it
                if(pkt_loc+length(UserData.ByteBuffer) > total_pktlen)
 
                    % output it to the command line
                    fprintf('R %s: ', datestr(now,'HH:MM:SS.FFF'));
                    fprintf(logfile,'R %s: ', datestr(now,'HH:MM:SS.FFF'));

                    for i=pkt_loc:pkt_loc+total_pktlen
                        fprintf('%s',dec2hex(UserData.ByteBuffer(i)));
                        fprintf(logfile,'%s',dec2hex(UserData.ByteBuffer(i)));
                        if(i~=length(UserData.ByteBuffer))
                            fprintf(',');
                            fprintf(logfile,',');
                        end
                    end
                    fprintf('\n');
                    fprintf(logfile,'\n');
                    
                    % extract the packet for processing
                    pkt = UserData.ByteBuffer(pkt_loc:pkt_loc+total_pktlen);

                    % remove the pkt bytes from the buffer
                    UserData.ByteBuffer = UserData.ByteBuffer(pkt_loc+total_pktlen:end); 

                    % parse the telemetry
                    msg = parseMsg(pkt, Endian.Little);
               
                else 
                    warning('timeCallback:TooShortForHeader','Not enough bytes for valid header');
                end
                                
            end
        end
    end
    
    
    set(src, 'UserData',UserData);

    %     % if it looks like a BADASS message
%     if(all(data(1:2)== [hex2dec('08') hex2dec('2')] ))
% 
%         % parse the message
%         msg = parseMsg(data,endianness);
% 
%         % output it to the command line
%         fprintf('R %s: ', datestr(now,'HH:MM:SS.FFF'));
%         for i=1:length(data)
%             fprintf('%s',dec2hex(data(i)));
%             if(i~=length(data))
%                 fprintf(',');
%             end
% 
%         end
%         fprintf('\n');
%     % otherwise we must be out of sync
%     else
%         % print it to the command line
%         fprintf('Out of synch, waiting, got: \n');
%         for i=1:length(RxText)
%             fprintf('%s',dec2hex(uint8(RxText(i))));
%             if(i~=length(RxText))
%                 fprintf(',');
%             end
% 
%         end
%         fprintf('\n');
% 
%         % flush the input to try to resync us
%         flushinput(serConn);
%         pause(0.01);

end
