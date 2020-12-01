function B = BuildB(NumberOfStates,p,phi,DeltaT)
%BUILDB Returns the B matrix for the two tank topology with NumberOfStates
%-1 pipe sections
%The first tank is not included in the B matrix
B = zeros(NumberOfStates,2);
B(1,:) = [p(1) * DeltaT, 0];

for index = 2:1:NumberOfStates
    B(index,:) = [0, 0];
end
B(NumberOfStates,:) = [0, -phi(2) * DeltaT];
end

