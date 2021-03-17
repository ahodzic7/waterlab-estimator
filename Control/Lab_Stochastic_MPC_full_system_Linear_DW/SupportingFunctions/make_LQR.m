    max_deviation_from_mean_x = 1;
    max_allowed_u = 0.5/60;
    sys = evalin('base','sys');
    A = full(sys.A(dT));
    B = full(sys.B(dT));
    Q = diag([1/(max_deviation_from_mean_x^2), 0.0001, 0.0001, 0.0001, 0.0001, 1/(max_deviation_from_mean_x^2)]);
    R = diag([1/max_allowed_u^2 1/max_allowed_u^2]);
    K = lqr(A,B,Q,R);