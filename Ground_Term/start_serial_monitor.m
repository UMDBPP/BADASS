function start_serial_monitor(serPortn, baud)
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

    % open the serial port
    try
        fopen(serConn);
    catch
        fclose(serConn);
        fopen(serConn);
    end
    
    % assign the serial connection to the base workspace so the user can
    % interact with it
    assignin('base','serConn',serConn);
    fprintf('\n');
    
    t=timer;
    t.StartFcn = @initTimer;
    t.TimerFcn = {@timerCallback, serConn};
    t.StopFcn = @closeTimer;
    t.Period   = 0.5;
    t.ExecutionMode  = 'fixedRate';
    start(t);
   
    assignin('base','timer_obj',t);

end

function initTimer(src, event)

   UserData = get(src, 'UserData');
   UserData.ByteBuffer = 0;
   disp('initialised')
   set(src, 'UserData',UserData);
    
end

function closeTimer(src, event)

   evalin('base','clear serConn');
   evalin('base','clear timer_obj');
    
end

function timerCallback(src, event, serConn)

	UserData = get(src, 'UserData');
   
    if(serConn.BytesAvailable > 0)
        RxText = fread(serConn,serConn.BytesAvailable);

        % convert it from char to integer
        data = uint8(RxText);

        % append the new data to what we've read previously
        UserData.ByteBuffer = [UserData.ByteBuffer; data];

%         fprintf('Read %d bytes\n',length(data));
        xbee_hdr_len = 6;

        for i = 1:length(UserData.ByteBuffer)-9

            % look for header bytes
            if(UserData.ByteBuffer(i)== hex2dec('7E') && ...
                UserData.ByteBuffer(i+8)== hex2dec('08') && ...
                UserData.ByteBuffer(i+9)== hex2dec('02'))
                
                % get rid of the pre-header stuff
                UserData.ByteBuffer = UserData.ByteBuffer(i:end);
                
                % extract the packet header
                if(length(UserData.ByteBuffer) > xbee_hdr_len)
                    pkthdr = data(1:xbee_hdr_len);
                else 
                    fprintf('hi2.7\n');
                    warning('timeCallback:TooShortForHeader','Not enough bytes for valid header');
                    break;
                end
                
                % extract the packet length
                [~, ~, ~, ~, ~, ~, PktLen] = ExtractPriHdr(pkthdr, Endian.Little)
                [~, ~, ~, ~, ~, ~, PktLen] = ExtractPriHdr(pkthdr, Endian.Big);

                total_pktlen = PktLen+7
                
                
                if(i+length(UserData.ByteBuffer) > total_pktlen)
                    fprintf('hi2.8\n');
                    % output it to the command line
                    fprintf('R %s: ', datestr(now,'HH:MM:SS.FFF'));
                    for ii=1:total_pktlen
                        fprintf('%s',dec2hex(UserData.ByteBuffer(ii)));
                        if(ii~=length(UserData.ByteBuffer))
                            fprintf(',');
                        end
                    end
                    fprintf('\n');
                    
                    
%                     fprintf('Found pkt %d bytes long \n',PktLen);
                    fprintf('hi2.9\n');
%                     fprintf('Bytes available: %d \n', length(UserData.ByteBuffer));
                    pkt = UserData.ByteBuffer(1:total_pktlen);

                    UserData.ByteBuffer = UserData.ByteBuffer(total_pktlen:end); 
                    fprintf('hi3\n');
                         
                    % parse the telemetry
                    msg = parseMsg(pkt, Endian.Little);
fprintf('hi4\n');
                else
                    warning('timeCallback:TooShortForPkt','Not enough bytes for valid pkt');
                    break
                end
                fprintf('hi4.1\n');

            end

        end
        fprintf('hi4.3\n');

    end
    fprintf('hi5\n');
    set(src, 'UserData',UserData);
    fprintf('hi6\n');
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
