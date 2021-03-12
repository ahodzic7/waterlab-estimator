function y = uConv(u,mode)
% """
%Unit conversion for the flow variables
% u    : input data
% mode : conversion mode
% dt   : sampling time used in control/simulation
% """
try
    switch mode
    case 'sToh'
        y = 3600.*u;
    case 'sTo10m'
        y = 600.*u;
    case 'sTom'
        y = 60.*u;
    case '10mTos'
        y = (1/600).*u;
    case '10mtoh'
        y = 6.*u;
    case 'mToh'
        y = 60.*u;
    case 'mTo5m'
        y = 5.*u;
    case '5mTom'
        y = (1/5).*u;
    case 'sTo5m'
        y = 300.*u;
    case '5mTos'
        y = (1/300).*u;
        case 'mTos'
            y = (1/60).*u;
                    case 'none'
            y = u;
    end 
catch 
    fprintf('Error in unit conversion');
end