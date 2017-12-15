function [feas, xOpt, uOpt, predErr] = MPC(x0, M, N, xT, T)



xOpt = zeros(15, N+1);

xOpt(1:12, 1) = x0;
feas = false(1, M);
predErr = 0;

for i = 1 : (M-1)
    
    fprintf('%d\n', i)
    
    [feas(i), xOptOL, uOptOL, JOptOL] = solveOPTOriginal(x0, N, T, xT);
    
    if feas(i) == false
        xOpt = [];
        uOpt = [];
        predErr = [];
        break
    end
    
    xOpt(:, (5*i-3):(5*i+1)) = xOptOL(:, 2:6);
    uOpt((5*i-4):(5*i)) = uOptOL(1:5);
    x0 = xOpt(1:12, 5*1+1);
    
%     if i>N
%         predErr(1, i-N) = norm(xOpt(1, i-N:i)-xOptOL{i-N}(1, :), 2);
%         predErr(2, i-N) = norm(xOpt(2, i-N:i)-xOptOL{i-N}(2, :), 2);
%     end
    
    
    
end

% if isempty(xOpt) == 0
%     [feas(i), xOptOL{M+1}, uOptOL{i}, JOpt] = solve_cftoc(A, B, P, Q, R, N, xOpt(:, i), xL, xU, uL, uU, bf, Af);
%     xOpt(:, i+1) = xOptOL{i}(:, 2);
%     predErr(1, M-N+1) = norm(xOpt(1, M-N+1:M+1)-xOptOL{M-N+1}(1, :), 2);
%     predErr(2, M-N+1) = norm(xOpt(2, M-N+1:M+1)-xOptOL{M-N+1}(2, :), 2);
% end

% plot(xOpt(1,:), xOpt(2,:), '-', xOptOL{1}(1,:), xOptOL{1}(2,:), '--')
% legend('Close Loop', 'Open Loop')
% xlabel('x1')
% ylabel('x2')
% title(['Trajectory when N is ' num2str(N) '.'])
% 



end