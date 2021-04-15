function uint16Type = doubleLe2uint16Be(doubleType)
%DOUBLELE2UINT16BE Summary of this function goes here
%   Detailed explanation goes here
 reverseOrderDouble = flip(doubleType)
 reverseOrderUint16Le = typecast(reverseOrderDouble, 'uint16');
 uint16Type = flip(reverseOrderUint16Le);
 if size(uint16Type,2)~= 1
    if size(size(uint16Type,1)) ~= 1
        disp('doubleLe2uint16Be, should recive a vector as input type');
         quit() 
    end
    uint16Type =  uint16Type';
 end
    
end

