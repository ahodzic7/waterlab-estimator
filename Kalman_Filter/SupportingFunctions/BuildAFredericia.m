function [A] = BuildAFredericia(NumberOfStates,p,phi,DeltaT)
%BUILDB Returns the A matrix for the two tank topology with NumberOfStates
%   The first tank is not included in the A matrix
A = zeros(NumberOfStates);
A(1,:) = [1 - p(2) * DeltaT, p(3) * DeltaT, zeros(1,NumberOfStates-2)]; 

for index = 2:1:NumberOfStates-2;
A(index,:) = [zeros(1,index-2), p(2) * DeltaT, 1 - ( p(2) + p(3) ) * DeltaT,...
             p(3) * DeltaT, zeros(1,NumberOfStates-index-1)];
end
A(NumberOfStates-1,:) = [zeros(1,NumberOfStates-3), p(2)* DeltaT,...
                        1 - ( p(5) + p(3) ) * DeltaT , p(5)*DeltaT]; %Last pipe section free flow
A(NumberOfStates,:)   = [zeros(1,NumberOfStates-2), phi(2) * p(5) / p(1)...
                        * DeltaT, 1-phi(2) * (p(5) / p(1))* DeltaT]; %Tank
end
