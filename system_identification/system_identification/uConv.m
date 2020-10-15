function y = uConv(u,mode)
% """
% unit conversion for the flow variables
% u    : input data
% mode : conversion mode vector
% help : intermediate variable
% dt   : sampling time helpsed in control/simhelplation
% """
help = u;
    for i = 1:size(mode,2)
        try
            switch mode(i)
                case 'sToh'
                    help = 3600*help;
                case 'sTo10min'
                    help = 600*help;
                case 'sTomin'
                    help = 60*help;
                case '1/sTo1/min'
                    help = 60*help;
                case '10minTos'
                    help = (1/600)*help;
                case '10minToh'
                    help = 6*help;
                case 'minToh'
                    help = 60*help;
                case 'minTo5min'
                    help = 5*help;
                case '5minTomin'
                    help = (1/5)*help;
                case 'sTo5min'
                    help = 300*help;
                case '5minTos'
                    help = (1/300)*help;
                case 'm^3ToL'
                    help = 1000*help;
                case 'mTodm'
                    help = 10*help;
                case 'mTocm'
                    help = 100*help;
                case 'mTomm'
                    help = 1000*help;
            end 
        catch 
            fprintf('Error in unit conversion');
        end
    end
    
    y = help;
end