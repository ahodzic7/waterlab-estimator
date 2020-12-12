function Delta = BuildDeltaFredericia(NumberOfStates, p,DeltaT)
%BUILDDELTA  Returns the Delta matrix for the two tank topology with
%NumberOfStates
tank_offset = 1.1;
Delta = [-p(4)*DeltaT];
Delta = [Delta; zeros(NumberOfStates-3,1)];
Delta = [Delta; (p(4)-p(5)*tank_offset)*DeltaT];
Delta = [Delta; p(6) * (p(5) / p(1))* tank_offset*DeltaT];
end
