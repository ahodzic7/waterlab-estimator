function [A] = BuildA(NumberOfStates,p,phi,DeltaT)
%BUILDB Returns the A matrix for the two tank topology with NumberOfStates
%-1 pipe sections
%   The first tank is not included in the A matrix
A = zeros(NumberOfStates);
A(1,:) = [1 - p(2) * DeltaT, p(3) * DeltaT, zeros(1,NumberOfStates-2)]; 

for index = 2:1:NumberOfStates-2;
A(index,:) = [zeros(1,index-2), p(2) * DeltaT, 1 - ( p(3) + p(3) ) * DeltaT,...
             p(3) * DeltaT, zeros(1,NumberOfStates-index-1)];
end
A(NumberOfStates-1,:) = [zeros(1,NumberOfStates-2), p(2),...
                        1 - ( p(5) + p(3) ) * DeltaT]; %Pipe In state
A(NumberOfStates,:)   = [zeros(1,NumberOfStates-2), phi(1) * p(5) / p(1)...
                        * DeltaT, 1]; %Pipe In state
end

