function Adjoint = Ad(T)
    Adjoint = zeros(6,6);
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    Adjoint(1:3, 1:3) = R;
    Adjoint(4:6, 4:6) = R;
    Adjoint(4:6, 1:3) = skew(p)*R;
end