function [vec, data_idx] = extractvec(pktdata,data_idx,veclen,endianness)
%
%   [vec, data_idx] = extractvec(pktdata,data_idx,veclen)
%       extracts a floating point VECLEN vector from a byte array
%
%   [vec, data_idx] = extractvec(pktdata,data_idx,3)
%       returns a 3 vector
%   [vec, data_idx] = extractvec(pktdata,data_idx,4)
%       returns a 4 vector
%

    vec = zeros(veclen,1);

    for i = 1:veclen
        if(endianness == Endian.Little)
%             dec2hex(pktdata(data_idx+3:-1:data_idx))
            vec(i) = typecast(pktdata(data_idx+3:-1:data_idx),'single');
        else
            vec(i) = typecast(pktdata(data_idx:data_idx+3),'single');
        end
        data_idx = data_idx + 4;
    end

    vec = vec.';
end