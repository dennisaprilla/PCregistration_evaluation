%======================================================================
% function to perform CPD Rigid registration
%======================================================================
function tform = cpdnormal(moving, movingnormal, fixed, fixednormal, maxIterations, ...
    w, tolerance, verbose)

if(isSimMode())
    printer = vision.internal.MessagePrinter.configure(verbose);
end
X = fixed;
X_hat = fixednormal;
Y = moving;
Y_hat = movingnormal;

D = size(X, 2);
M = size(Y, 1);
N = size(X, 1);

% normalize to zero mean
xmean = mean(X);
ymean = mean(Y);

if(isSimMode())
    X = X - xmean;
    Y = Y - ymean;
else
    for i = 1 : D
        X(:,i) = X(:,i) - xmean(i);
        Y(:,i) = Y(:,i) - ymean(i);
    end
end
% Initialize rotation matrix to identity, translation vector to 0 and
% scaling parameter to 1, i.e. R = I, t = 0
R = cast(eye(3), 'like', Y);
t = cast([0 0 0]', 'like', Y);

sigma2    = cast(0, 'like', Y);

for col = 1 : D
    if(isSimMode())
        sigma2 = sigma2 + sum(sum((X(:,col) - Y(:,col)').^2 ));
    else
        for j = 1 : N
            sigma2 = sigma2 + sum((X(j, col) - Y(:,col)).^2);
        end
    end
end
sigma2 = sigma2 / (D*N*M);

nIter  = 0;
negativeLogLikelihood      = cast(0, 'like', X);

transformedY      = Y ;
transformedY_hat  = Y_hat;
% EM optimization, repeat until convergence
while (nIter < maxIterations)
    negativeLogLikelihoodPrev = negativeLogLikelihood;
    
    % E-step: Compute P
    X_prime = [X, X_hat * 0.05];
    transformedY_prime = [transformedY, transformedY_hat * 0.05];
    [ P1, Pt1, Px, negativeLogLikelihood ] = computeEStep(X_prime, transformedY_prime, sigma2, w);
    
    ntol            = abs((negativeLogLikelihood-negativeLogLikelihoodPrev)/negativeLogLikelihood);
    
    % M-step : Solve R, t, sigma2
    Np  = sum(P1);
    mux = X'*Pt1 / Np;
    muy = Y'*P1 / Np;
    
    A   = Px(:,1:3)'*Y - Np*(mux*muy');
    rcondVal = rcond(A);
    if(isnan(rcondVal) || rcondVal<eps)
        if(isSimMode())
            printer.printMessage('vision:pointcloud:cpdStopCondIllMatrix');
        end
        break;
    end
    [U,~,V]    = svd(A);
    C          = eye(size(U,2));
    C(end,end) = sign(det(U*V'));
    R = U*C*V';
    
    t = mux - R*muy;
    if(isSimMode())
        X1  = X - mux';
        Y1  = Y - muy';
        sigma2 = abs(( sum(sum((X1.^2).*Pt1 )) - (trace(A'*R)^2)/sum(sum((Y1.^2).*P1)) )/(Np*D));
        transformedY      = Y*R' + t';
        % test
        transformedY_hat  = (R * transformedY_hat')';
    else
        sum1 = cast(0, 'like', X);
        sum2 = cast(0, 'like', X);
        for c = 1 : D
            sum1 = sum1 + sum(((X(:, c)-mux(c)).^2).*Pt1);
            sum2 = sum2 + sum(((Y(:, c)-muy(c)).^2).*P1);
        end
        sigma2 = abs(( sum1 - (trace(A'*R)^2)/sum2 )/(Np*D));
        transformedY      = Y*R';
        
        for col = 1 : D
            transformedY(:, col) = transformedY(:, col) + t(col);
        end
    end
    nIter  = nIter + 1;
    if(isSimMode())
        printer.linebreak;
        printer.print('--------------------------------------------\n');
        printer.printMessage('vision:pointcloud:cpdIteration','Rigid', nIter);
        printer.printMessage('vision:pointcloud:cpdCurrentCovariance', mat2str(sigma2));
        printer.printMessage('vision:pointcloud:cpdCurrentVar', mat2str(t, 6), mat2str(R', 6));
        printer.printMessage('vision:pointcloud:cpdCurrentFcn', mat2str(ntol));
    end
    if(ntol <= tolerance)
        break;
    end
    
end
if(isSimMode())
    if(nIter>0)
        printer.print('--------------------------------------------\n');
        printer.printMessage('vision:pointcloud:cpdTotalIterations', nIter);
    end
end
t = t+xmean' - (R*ymean');


tform = rigid3d(R', t');
end

%======================================================================
% function to compute P, i.e, E-step of CPD registration
%======================================================================
function [ P1, Pt1, Px, negativeLogLikelihood ] = computeEStep( X, Y, sigma2, w )
% The elements m, n of the matrix P are defined as:
%
%                      exp(C1*(Xn-Ym)^2)
% P(m,n) = ----------------------------------------,
%            sum-over-k(exp(C1*(Xn-Yk)^2)) + C2
%
% where C1 = -1/(2*sigma2), M is the number of points in Y, N is the number
% of points in X and D is the dimensionality of points (ex D=3 in case of 3
% dimensional points),
% and   C2 = ( (M*w)/(N*(1-w)) )*((2*pi*sigma2)^(D/2))
%
% The outputs are: P1 = P*1, Pt1 = P'*1, Px = P*X,
%  where 1 is a column vector of all ones.


D = size(X, 2);
M = size(Y, 1);
N = size(X, 1);

% Compute C2 as given in above equation
c   = power(2*pi*sigma2, D/2) *( w /(1-w) *M/N );
if(isSimMode())
    % Compute elements of P matrix
    pMatrix = zeros(M, N, 'like', X);
    for col = 1 : D
        pMatrix = pMatrix + (X(:,col)' - Y(:,col)).^2;
    end
    pMatrix     = exp(pMatrix*(-1/(2*sigma2)));
    
    % Compute Pt1, P1, Px
    pMatrixColSum  = sum(pMatrix);
    Pt1         = (pMatrixColSum./(pMatrixColSum +c))';
    pMatrix     = pMatrix./(pMatrixColSum +c);
    P1          = sum(pMatrix, 2);
    
    Px  = zeros(M, D, 'like', X);
    for col = 1 : D
        Px(:, col)  = pMatrix*X(:, col);
    end
    negativeLogLikelihood  = -sum(log(pMatrixColSum +c)) + D*N*log(sigma2)/2;
else
    % Codegen
    P1  = zeros(M,1, 'like', X);
    Px  = zeros(M, D, 'like', X);
    Pt1 = zeros(N, 1, 'like', X);
    
    for j = 1 : N
        pMatrixCol = (X(j,1) - Y(:,1)).^2;
        for col = 2 : D
            pMatrixCol = pMatrixCol + (X(j,col) - Y(:,col)).^2;
        end
        
        pMatrixCol = exp(pMatrixCol.*(-1/(2*sigma2)));
        pMatrixColSum =  sum(pMatrixCol);
        
        pMatrixCol = pMatrixCol./(c+pMatrixColSum);
        P1 = P1 + pMatrixCol;
        Px = Px + pMatrixCol*X(j, :);
        Pt1(j) = pMatrixColSum;
    end
    
    negativeLogLikelihood  = cast(-sum(log(Pt1 + c)) + D*N*log(sigma2)/2, 'like', X);
    Pt1 = Pt1./(Pt1+c);
end
end

function flag = isSimMode()
flag = isempty(coder.target);
end