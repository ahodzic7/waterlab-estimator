% Simple mass balance
function [y] = model_level(x,u,d)

    y = (d(1)-u(1))/(1.25^2 * pi);

end