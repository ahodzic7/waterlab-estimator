% Simple mass balance
function [y] = model(x,u,d)

    y = d(1)-u(1);

end