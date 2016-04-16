function [ SSE ] = SumOfSquaredErrors( a, b )
%SumOfSquaredErrors finds the sum of the squared differences bewteen
%   input vectors a & b (must be same size)

SE = (a-b).^2;
SSE = 0;
for k = 1:length(a)
    SSE = SSE+SE(k);
end

end

% Marciano C Preciado, u0963168, ME EN 1010, HW5