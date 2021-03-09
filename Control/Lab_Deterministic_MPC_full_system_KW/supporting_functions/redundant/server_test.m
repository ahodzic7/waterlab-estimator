while 1
    MPCTCP = openConnectionServer('0.0.0.0', 502)

   if MPCTCP.status == 'closed'
    break
   end
   
   while ~MPCTCP.BytesAvailable
        %wait for the response to be in the buffer
   end
   
    %Insert MPC Code here
    disp();
end

function MPCTCP = openConnectionServer(ipaddress, port)
    MPCTCP=tcpip(ipaddress, port,'NetworkRole', 'Server','Timeout',10); %Create the tcpip obeject
    set(MPCTCP, 'InputBufferSize', 512); %assign the buffer
    MPCTCP.ByteOrder='bigEndian'; %specify the order in which bytes are transmitted
    try 
        if ~strcmp(MPCTCP.Status,'open') 
            fopen(MPCTCP);
            disp(['TCP/IP connection opened with host:', ipaddress]);
        end
    catch fault % display error if the channel is not opened.
        if ~strcmp(MPCTCP.Status,'open') % check if the channel is really closed
            disp(fault);
            disp(['Error: Can''t establish TCP/IP connection with: ',ipaddress,':',num2str(port)] ); 
            disp('You can check the following:');
            disp('- If the cable is plugged in correctly ');
            disp('- Whether the Codesys controller is turned on.');
            disp('- Your firewall settings');
        end
    end
end

