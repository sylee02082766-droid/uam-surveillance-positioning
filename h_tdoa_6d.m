function [z_hat, H] = h_tdoa_6d(x, refSite, otherSites)

pos = x(1:3);
dist_ref = norm(pos' - refSite);
m = size(otherSites,1);

z_hat = zeros(m,1);
H = zeros(m,6);

for j = 1:m
    sj = otherSites(j,:);
    dist_j = norm(pos' - sj);

    z_hat(j) = dist_j - dist_ref;

    grad = (pos' - sj)/max(dist_j,eps) - (pos' - refSite)/max(dist_ref,eps);
    H(j,1:3) = grad;
end

end