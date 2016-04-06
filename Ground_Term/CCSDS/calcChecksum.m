function CheckSum = calcChecksum(arr)
%
%   CheckSum = calcChecksum(arr)
%       Calculates the checksum of the array passed in
%
%   Steve Lentine
%   2016/03/26
%

    % initalize output
    CheckSum = uint8(hex2dec('FF'));
    
    % loop through elements and calculate checksum
    for i=length(arr):-1:1  
        CheckSum = bitxor(CheckSum,arr(i));
    end

end