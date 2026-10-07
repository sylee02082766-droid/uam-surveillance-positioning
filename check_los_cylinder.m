function is_clear = check_los_cylinder(p1, p2, obstacles)
% obstacles = [cx cy radius height]

if isempty(obstacles)
    is_clear = true;
    return;
end

is_clear = true;
v = p2 - p1;

for i = 1:size(obstacles,1)
    cx = obstacles(i,1);
    cy = obstacles(i,2);
    r  = obstacles(i,3);
    h  = obstacles(i,4);

    a = v(1)^2 + v(2)^2;
    b = 2*((p1(1)-cx)*v(1) + (p1(2)-cy)*v(2));
    c = (p1(1)-cx)^2 + (p1(2)-cy)^2 - r^2;

    if abs(a) < 1e-12
        continue;
    end

    D = b^2 - 4*a*c;
    if D < 0
        continue;
    end

    t1 = (-b - sqrt(D)) / (2*a);
    t2 = (-b + sqrt(D)) / (2*a);

    cand = [t1, t2];
    cand = cand(cand >= 0 & cand <= 1);

    if isempty(cand)
        continue;
    end

    for t = cand
        z_line = p1(3) + t*v(3);
        if z_line <= h
            is_clear = false;
            return;
        end
    end
end
end