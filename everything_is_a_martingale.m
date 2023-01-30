% Quick martingale-checker for sequential statistical tests

figure; hold on;

% Contest parameters:
p1 = .55; p0 = .5; % .1 margin
alpha = .1;

% Simulation parameters:
roundSched = [100];
k = 0; n= 0;
PROV(1) = 1;
EOR_SPRT(1) = 1;
%SO_SPRT(1) = 1;
numRounds = 10;
PROV_stop = false; EOR_SPRT_stop = false;
numTrials = 100;
for i=1:numTrials
    avg_PROV = zeros(1,numRounds);
    avg_EOR_SPRT = zeros(1,numRounds);
    for j=1:numRounds
        % Choose next round size
        roundSched(j) = chooseNextRoundSize(k,n,p1,p0);
    
        % Draw new random sample (update sample vectors accordingly)
        % (assumes p=p0, the null hypothesis)
        np = roundSched(j); ns(j) = np; n = n + np;
        kp = binornd(np,p0); ks(j) = kp; k = k + kp;
        
        % Compute test statistics
        PROV(j) = sigma(k-kp, n-np, p1, p0) * tau(kp, np, p1, p0);
        EOR_SPRT(j) = sigma(k, n, p1, p0);
    
        % Check stopping conditions
        if PROV(j) >= 1/alpha
            PROV_stop = true;
        end
        if EOR_SPRT(j) >= 1/alpha
            EOR_SPRT_stop = true;
        end
    
        % Plot the new test statistics
        plot(j, PROV(j), 'g*');
        plot(j, EOR_SPRT(j), 'ro');
        pause(.1);
    end

    % Update averages
    avg_PROV = avg_PROV + PROV;
    avg_EOR_SPRT = avg_EOR_SPRT + EOR_SPRT;

    % Plot the average test statistics
    hold off;
    plot(1:numRounds, avg_PROV ./ i, 'b*'); hold on;
    plot(1:numRounds, avg_EOR_SPRT ./ i, 'bo');
    hold on;
    pause(.01);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function n = chooseNextRoundSize(k, n, p1, p0)
    n = 100;
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