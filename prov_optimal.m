% Expected sample size

p0 = .5;
p1 = .55;
n = 100;

% Compute Providence expected sample size
exp_sample_size = 0;
for j=1:10
    exp_sample_size = exp_sample_size + j*tail(k,n,p1);
end



% FUNCTIONS
function t = tail(k,n,p)
    num = 0;
    for kx=k:n
        num = num + nchoosek(n,kx) * p^kx * (1-p)^(n-kx);
    end
    t = num;
end

function t = tau(k,n,p1,p0)
    % num = sum_{k}^{n} p1^{k} (1-p1)^{n-k}
    % denom = sum_{k}^{n} p0^{k} (1-p0)^{n-k}
    num = 0; denom = 0;
    for kx=k:n
        num = num + nchoosek(n,kx) * p1^kx * (1-p1)^(n-kx);
        denom = denom + nchoosek(n,kx) * p0^kx * (1-p0)^(n-kx);
    end
    t = num / denom;
end

function s = sigma(k, n, p1, p0)
    % num = p1^{k} (1-p1)^{n-k} = k log(p1) + (n-k)log(1-p1)
    % denom = p0^{k} (1-p0)^{n-k} = k log(p0) + (n-k)log(1-p0)
    s = k*log(p1) + (n-k)*log(1-p1) - (k*log(p0) + (n-k)*log(1-p0));
    s = exp(s);
end