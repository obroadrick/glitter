function [R,Q] = rq(A)
    % computes the RQ decomposition of matrix A
    % R is an right upper triangular matrix and
    % Q is ano orthogonal matrix
    % (matlab has QR decomposition but not RQ...)
    % uses method described in accepted answer at:
    % https://math.stackexchange.com/questions/1640695/rq-decomposition
    P_ = [0 0 1; 0 1 0; 1 0 0];
    A_ = P_ * A;
    [Q_, R_] = qr(A_');
    Q = P_ * Q_';
    R = P_ * R_' * P_;
end