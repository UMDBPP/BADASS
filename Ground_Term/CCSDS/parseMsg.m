function msg = parseMsg(byte_arr, endianness)

    % get telemetry data from base workspace (or create it if it doesn't exist)
    try
        TLM = evalin('base','TLM');
    catch
        warning('parseMsg:TLMDBmissing','Telemetry database doesn''t appear to exist, creating it...')
        evalin('base','TLM = createTlmDatabase;');
        TLM = evalin('base','TLM');
    end

    byte_arr = uint8(byte_arr);
    
    byte_arr_len = length(byte_arr);
    
    pri_hdr = byte_arr(1:6);
    
    % check if packet is long enough to 
    if(byte_arr_len < 8)
       error('parseMsg:ShortPacket','The packet is %d bytes long, which is too short to be a valid packet',byte_arr_len);
    end

    % extract primary header
    [msg.APID, msg.SecHdr, msg.PktType, msg.CCSDSVer, msg.SeqCnt, msg.SegFlag, msg.PktLen] = ExtractPriHdr(pri_hdr, endianness);
    
    if(msg.CCSDSVer)
        warning('parseMsg:CCSDSVer','The CCSDS version is expected to be 0, but is ''%d''. This may indicate a malformed packet.',msg.CCSDSVer);
    end
    
    % extract the secondary header depending on if its a command or
    % telemetry message
    if(msg.PktType)
        
        cmd_sec_hdr = byte_arr(7:8);
        
        % calculate the packet checksum
        checksum = calcChecksum(pri_hdr);
        
        % extract the secondary header
        [msg.FcnCode, msg.Checksum, msg.Reserved] = ExtractCmdSecHdr(cmd_sec_hdr, endianness);
        
        % extract the packet's payload
        pktdata = byte_arr(13:end);
        
        % inform the user
        fprintf('Received a command, fcncode = %d, ', msg.FcnCode);
        if (checksum == msg.Checksum)
            fprintf('with valid checksum. \n');
        else
            fprintf('with invalid checksum. \n');
        end
        
        msg.Time = -1;
        
        return
    else
        
        tlm_sec_hdr = byte_arr(7:12);
        
        % check that packet is long enough to extract
        if(byte_arr_len < 12)
           error('parseMsg:ShortTlmHdr','The header of the telemetry packet is %d bytes long, which is too short to be a valid telemetry header.',byte_arr_len);
        end
        
        % extract the secondary header
        msg.Time = ExtractTlmSecHdr(tlm_sec_hdr, endianness);
        
        % extract the packet's payload
        pktdata = byte_arr(13:end);

    end
    
    TLM.APID = addsample(TLM.APID, ...
            'Data', msg.APID, 'Time', msg.Time);
    TLM.SecHdr = addsample(TLM.SecHdr, ...
            'Data', msg.SecHdr, 'Time', msg.Time);
    TLM.CCSDSVer = addsample(TLM.CCSDSVer, ...
            'Data', msg.CCSDSVer, 'Time', msg.Time);
    TLM.SeqCnt = addsample(TLM.SeqCnt, ...
            'Data', msg.SeqCnt, 'Time', msg.Time);
    TLM.SegFlag = addsample(TLM.SegFlag, ...
            'Data', msg.SegFlag, 'Time', msg.Time);
    TLM.PktLen = addsample(TLM.PktLen, ...
            'Data', msg.PktLen, 'Time', msg.Time);    
        
    
    if(msg.PktType)
        TLM.FcnCode = addsample(TLM.FcnCode, ...
            'Data', msg.FcnCode, 'Time', msg.Time);
        TLM.Checksum = addsample(TLM.Checksum, ...
            'Data', msg.Checksum, 'Time', msg.Time);
        TLM.Reserved = addsample(TLM.Reserved, ...
            'Data', msg.Reserved, 'Time', msg.Time);
    else
        TLM.Time = addsample(TLM.Time, ...
            'Data', msg.Time, 'Time', msg.Time);
        
    end
    
    fprintf('%d bytes of data \n',length(pktdata));
    
    %% Defines
    % must match arduino defines
    tlmctrl_MASK =      hex2dec('00000001');
    target_ned_MASK =   hex2dec('00000002');
    bno_cal_MASK =      hex2dec('00000004');
    euler_ang_MASK =    hex2dec('00000008');
    q_imu2body_MASK =   hex2dec('00000010');
    q_ned2imu_MASK =    hex2dec('00000020');
    q_ned2body_MASK =   hex2dec('00000040');
    v_targetbody_MASK = hex2dec('00000080');
    azel_err_MASK =     hex2dec('00000100');
    el_cmd_MASK =       hex2dec('00000200');
    cycle_time_MASK =   hex2dec('00000400');
    cmd_rcvd_MASK =     hex2dec('00000800');
    desired_cyc_time_MASK = hex2dec('00001000');
    cmdecho_MASK =      hex2dec('00001000');

    % // commanding
    set_tlmctrl_CMD = hex2dec('01');
    set_cyctime_CMD = hex2dec('02');
    set_target_ned_CMD = hex2dec('03');
    set_imu2body_CMD = hex2dec('04');
    set_servoenable_CMD = hex2dec('05');
    set_rwenable_CMD = hex2dec('06');
    set_requesttlm_CMD = hex2dec('07');
    
    % initalize to beginning of packet
    data_idx = 1;

    % the first 4 bytes of the packet is expected to be a bitfield
    % indicating the telemetry it contains
    
    if(endianness == Endian.Little)
        tlmctrl = typecast(pktdata(data_idx+3:-1:data_idx),'uint32');
    else
        tlmctrl = typecast(pktdata(data_idx:data_idx+3),'uint32');
    end
    tlmctrl
    TLM.tlmctrl = addsample(TLM.tlmctrl, 'Data', tlmctrl, 'Time', msg.Time);
    data_idx = data_idx + 4;

    
    % extract target_ned
    if(bitand(tlmctrl,target_ned_MASK))
        [vec_tmp, data_idx] = extractvec(pktdata,data_idx,3, endianness);
        TLM.v_target_ned = addsample(TLM.v_target_ned, ...
            'Data', vec_tmp, 'Time', msg.Time);
    end
    
    % extrac cal values
    if(bitand(tlmctrl,bno_cal_MASK))
        tmp = pktdata(data_idx:data_idx+3).';
        TLM.bno_cal = addsample(TLM.bno_cal, ...
            'Data', tmp, 'Time', msg.Time);
        data_idx = data_idx + 4;
    end
    
    % extract euler angles
    if(bitand(tlmctrl,euler_ang_MASK))
        [vec_tmp, data_idx] = extractvec(pktdata,data_idx,3, endianness);
        TLM.euler_ang = addsample(TLM.euler_ang, ...
            'Data', vec_tmp, 'Time', msg.Time);
    end
    
    % extract q_imu2body
    if(bitand(tlmctrl,q_imu2body_MASK))
        [vec_tmp, data_idx] = extractvec(pktdata,data_idx,4, endianness);
        TLM.q_imu2body = addsample(TLM.q_imu2body, ...
            'Data', vec_tmp, 'Time', msg.Time);
    end
        
    % extract q_ned2imu
    if(bitand(tlmctrl,q_ned2imu_MASK))
        [vec_tmp, data_idx] = extractvec(pktdata,data_idx,4, endianness);
        TLM.q_ned2imu = addsample(TLM.q_ned2imu, ...
            'Data', vec_tmp, 'Time', msg.Time);
    end
    
    % extract q_ned2body
    if(bitand(tlmctrl,q_ned2body_MASK))
        [vec_tmp, data_idx] = extractvec(pktdata,data_idx,4, endianness);
        TLM.q_ned2body = addsample(TLM.q_ned2body, ...
            'Data', vec_tmp, 'Time', msg.Time);
    end
    
    % extract v_targetbody
    if(bitand(tlmctrl,v_targetbody_MASK))
        [vec_tmp, data_idx] = extractvec(pktdata,data_idx,3, endianness);
        TLM.v_targetbody = addsample(TLM.v_targetbody, ...
            'Data', vec_tmp, 'Time', msg.Time);
    end
    
    % extract azel_err
    if(bitand(tlmctrl,azel_err_MASK))
        [vec_tmp, data_idx] = extractvec(pktdata,data_idx,2, endianness);
        TLM.azel_err = addsample(TLM.azel_err, ...
            'Data', vec_tmp, 'Time', msg.Time);
    end
    
    % extract el_cmd
    if(bitand(tlmctrl,el_cmd_MASK))
        TLM.el_cmd = addsample(TLM.el_cmd, ...
            'Data', pktdata(data_idx), 'Time', msg.Time);
        data_idx = data_idx + 1;
    end
    
    % extract cycle_time
    if(bitand(tlmctrl,cycle_time_MASK))
        if(endianness ~= Endian.Little)
            tmp = typecast(pktdata(data_idx+1:-1:data_idx),'uint16');
        else
            tmp = typecast(pktdata(data_idx:data_idx+1),'uint16');
        end
        TLM.cycle_time = addsample(TLM.cycle_time, ...
            'Data', tmp, 'Time', msg.Time);
        data_idx = data_idx + 2;
    end
    
    % extract cmd_rcvd
    if(bitand(tlmctrl,cmd_rcvd_MASK))
        TLM.cmd_rcvd = addsample(TLM.cmd_rcvd, ...
            'Data', pktdata(data_idx), 'Time', msg.Time);
        data_idx = data_idx + 1;
    end
    
    % extract desired_cyc_time
    if(bitand(tlmctrl,desired_cyc_time_MASK))
        TLM.desired_cyc_time = addsample(TLM.desired_cyc_time, ...
            'Data', pktdata(data_idx), 'Time', msg.Time);
        data_idx = data_idx + 1;
    end
    
    % extract cmdecho
    if(bitand(tlmctrl,cmdecho_MASK))
        TLM.cmdecho = addsample(TLM.cmdecho, ...
            'Data', pktdata(data_idx), 'Time', msg.Time);
        data_idx = data_idx + 1;
    end


    % put the data back into the base workspace
    assignin('base','TLM',TLM);

end