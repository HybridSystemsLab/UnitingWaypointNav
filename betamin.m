function beta=betamin(gamma)
if gamma == 0
    beta=1e-6;
    return;
end
e31 = fzero(@(x) sin(x)-2*gamma*x,pi);
beta = cos(e31)+gamma*e31^2;

fzero(@(gamma) betamin(gamma),0.1)
