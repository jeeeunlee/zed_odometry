function [o_tau dV]= SerialImpulseInverseDynamics(i_q,i_ddq)
% Lie Group Inverse Dynamics Algorithm for Serial Open Chain Systems
% tau = SerialInverseDynamics(q, dq, ddq)
% n : # of links
% q, dq, ddq, tau : n*1
% Syungkwon Ra (dearLenin@gmail.com)

% Initialize Parameters of Systems
global g_n g_V0 g_dV0 g_S g_M g_J g_ft g_Fext;
%SystemSpecification;
                                                                                                                                                                                           
% variables
i = 0;                      % iterator
invf = zeros(4, 4, g_n);    % SE(3), invf(:, :, i) = inv(f_i-1_i)
V = zeros(6, 1, g_n);       % se(3),  V(:, :, i) = V_i
dV = zeros(6, 1, g_n);      % se(3), dV(:, :, i) = dV_i
F = zeros(6, 1, g_n);       % dse(3), F(:, :, i) = F_i
tau = zeros(g_n-1,1);         % torque

%%%%%%%%%%%%%%% Lie Algorithms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
% V(:, :, 1) = g_V0;
dV(:, :, 1) = zeros(6,1);

% Forward recursion : for i = 1 to n
for i = 2 : g_n
    invf(:, :, i) = InverseSE3(g_M(:, :, i-1) * LargeSE3(g_S(:, :, i-1) * i_q(i-1)));
%    invf = inv(f(:, :, i));
%     V(:, :, i) = Adj( invf(:, :, i), V(:, :, i-1)) + g_S(:, :, i-1) * i_dq(i-1);
    dV(:, :, i) = g_S(:, :, i-1) * i_ddq(i-1) + Adj(invf(:, :, i), dV(:, :, i-1)) ;%+ ad(V(:, :, i), g_S(:, :, i-1) * i_dq(i-1));
end

% Backward recursion : for i = n to 1
for i = g_n : -1 : 2
    F(:, :, i) = g_J(:, :, i) * dV(:, :, i);% - dad(V(:, :, i), g_J(:, :, i) * V(:, :, i));
    if i ~= g_n
        F(:, :, i) = F(:, :, i) + dAdj(invf(:, :, i+1), F(:, :, i+1));
    end
    if g_ft(4, 4) == 1
        F(:, :, i) = F(:, :, i) - dAdj(InverseSE3(g_ft(:, :, i)), g_Fext(:, :, i));
    end
%    g_S(:, :, i-1);
%   F(:, :, i);
    tau(i-1) = g_S(:, :, i-1)' * F(:, :, i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

o_tau = tau;
 
