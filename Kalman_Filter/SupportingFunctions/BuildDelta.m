function Delta = BuildDelta(NumberOfStates, p,DeltaT)
%BUILDDELTA  Returns the Delta matrix for the two tank topology with
%NumberOfStates

Delta = [-p(4)*DeltaT];
Delta = [Delta; zeros(NumberOfStates-3,1)];
Delta = [Delta; p(4)*DeltaT];
Delta = [Delta; 0];
end

