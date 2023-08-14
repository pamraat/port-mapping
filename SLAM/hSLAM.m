function z = hSLAM(x)
    map = [x(4:2:end) x(5:2:end)];
    z = global2robot(x(1:3), map);
    z = reshape(z', [size(z, 1)*size(z, 2), 1]);
end