function [arr, char_arr, str_arr] = CreatePriHdr(APID, SecHdr, PktType, CCSDSVer, SeqCnt, SegFlag, PktLen)

%    uint8   StreamId[2];  /* packet identifier word (stream ID) */
%  /*  bits  shift   ------------ description ---------------- */
%       /* 0x07FF    0  : application ID                            */
%       /* 0x0800   11  : secondary header: 0 = absent, 1 = present */
%       /* 0x1000   12  : packet type:      0 = TLM, 1 = CMD        */
%       /* 0xE000   13  : CCSDS version, always set to 0            */
% 
%    uint8   Sequence[2];  /* packet sequence word */
%       /*  bits  shift   ------------ description ---------------- */
%       /* 0x3FFF    0  : sequence count                            */
%       /* 0xC000   14  : segmentation flags:  3 = complete packet  */
% 
%    uint8  Length[2];     /* packet length word */
%       /*  bits  shift   ------------ description ---------------- */
%       /* 0xFFFF    0  : (total packet length) - 7                 */
% 

    arr = uint8(zeros(6,1));
   
    % streamID field
    streamid_tmp = uint16(0);
    streamid_tmp = bitor(bitand(bitshift(uint16(APID),0),hex2dec('07FF')),streamid_tmp);
    streamid_tmp = bitor(bitand(bitshift(uint16(SecHdr),11),hex2dec('0800')),streamid_tmp);
    streamid_tmp = bitor(bitand(bitshift(uint16(PktType),12),hex2dec('1000')),streamid_tmp);
    streamid_tmp = bitor(bitand(bitshift(uint16(CCSDSVer),13),hex2dec('E000')),streamid_tmp);

    streamid = typecast(streamid_tmp,'uint8');

    % sequence field
    sequence_tmp = uint16(0);
    sequence_tmp = bitor(bitand(bitshift(uint16(SeqCnt),0),hex2dec('3FFF')),sequence_tmp);
    sequence_tmp = bitor(bitand(bitshift(uint16(SegFlag),14),hex2dec('C000')),sequence_tmp);

    sequence = typecast(sequence_tmp,'uint8');
    
    % length field
    length_tmp = uint16(0);
    length_tmp = bitor(bitand(bitshift(uint16(PktLen-7),0),hex2dec('00FF')),length_tmp);
    length_tmp = bitor(bitand(bitshift(uint16(PktLen-7),0),hex2dec('FF00')),length_tmp);
    
    length = typecast(length_tmp,'uint8');

    % assign to array
    arr(1) = streamid(1);
    arr(2) = streamid(2);
    arr(3) = sequence(1);
    arr(4) = sequence(2);
    arr(5) = length(1);
    arr(6) = length(2);

    
    arr = arr.';
    
    % convert to characters
    char_arr = char(arr.');
    
    % convert to string of number
    str_arr = strrep(regexprep(num2str(arr),' +',' '),' ',',');


end