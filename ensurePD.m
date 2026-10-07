function A = ensurePD(A)

A = (A + A') / 2;
[V,D] = eig(A);
d = diag(D);
d(d < 1e-9) = 1e-9;
A = V * diag(d) * V';
A = (A + A') / 2;

end