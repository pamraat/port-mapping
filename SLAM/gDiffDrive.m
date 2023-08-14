function g = gDiffDrive(x, u)
    g = integrateOdom(x, u(2)*u(1), u(3)*u(1));
    g(4:end) = x(4:end);
end