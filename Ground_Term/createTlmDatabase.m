function TLM = createTlmDatabase()

% header parameters
% primary
TLM.APID = timeseries('APID');
TLM.SecHdr = timeseries('Secondary Header Flag [1=Present,0=Absent]');
TLM.PktType = timeseries('Packet Type [1=Cmd,0=Tlm]');
TLM.CCSDSVer = timeseries('CCSDS Ver #');
TLM.SeqCnt = timeseries('Sequence Counter');
TLM.SegFlag = timeseries('Segmentation Flag');
TLM.PktLen = timeseries('Packet Length [bytes]');

% command secondary
TLM.FcnCode = timeseries('Function Code');
TLM.Checksum = timeseries('Checksum');
TLM.Reserved = timeseries('Reserved');

% tlm secondary
TLM.Time = timeseries('Tlm Time');

% telemetry points
TLM.tlmctrl = timeseries('TlmCtrl');
TLM.v_target_ned = timeseries('TargetNED');
TLM.bno_cal = timeseries('BNO Cal');
TLM.euler_ang = timeseries('Euler Ang');
TLM.q_imu2body = timeseries('Q IMU2Body');
TLM.q_ned2imu = timeseries('Q NED2IMU');
TLM.q_ned2body = timeseries('Q NED2Body');
TLM.v_targetbody = timeseries('TargetBody');
TLM.azel_err = timeseries('AzEl Err [rad]');
TLM.el_cmd = timeseries('El Cmd');
TLM.cycle_time = timeseries('Cycle Time [ms]');
TLM.cmd_rcvd = timeseries('Command Received');
TLM.desired_cyc_time = timeseries('Desired Cycle Time [ms]');
TLM.cmdecho = timeseries('Cmd Echo');


end