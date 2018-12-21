function [o_ddq dV0] = FloatingSerialForwardDynamics(i_q, i_dq,v,i_tau,gravity)
% Lie Group Forward Dynamics Algorithm for Serial Open Chain Systems
% ddq = SerialForwardDynamics(q, dq, tau)
% n : # of links
% q, dq, tau, ddq : n*1
% Syungkwon Ra (dearLenin@gmail.com)

% Initialize Parameters of Systems
global g_n g_V0 g_dV0 g_S g_M g_J g_ft g_Fext;

% variables
i = 0;                      % iterator
aJ = zeros(6, 6, g_n);      % artigulated inertia
b = zeros(6, 1, g_n);       % dse(3), bias force
V = zeros(6, 1, g_n);       % se(3), V(:, :, i) = V_i
dV = zeros(6, 1, g_n);      % se(3), dV(:, :, i) = dV_i
invf = zeros(4, 4, g_n);    % SE(3), invf(:, :, i) = inv(f_i-1_i)
omega = zeros(1, g_n);      % S' * aJ * S;
c = zeros(6, 1, g_n);       % b + aJ * ad_V_Sq
ddq = zeros(g_n-1);         % angular acceleration

%%%%%%%%%%%%%%% Lie Algorithms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
% V(:, :, 1) = g_V0;
% dV(:, :, 1) = g_dV0;
aJ(:, :, g_n) = g_J(:, :, g_n);
%% new Initialization for floating body
% Initialization
V(:, :, 1) = v;
gravity_vector=[0 0 0 0 gravity 0]';
% Forward recursion : for i = 1 to n
for i = 2 : g_n
    invf(:, :, i) = inv(g_M(:, :, i-1) * expm(se3(g_S(:, :, i-1) * i_q(i-1))));
    V(:, :, i) = Adj( invf(:, :, i), V(:, :, i-1)) + g_S(:, :, i-1) * i_dq(i-1);
end
b(:, :, g_n) = - dad(V(:, :, g_n), g_J(:, :, g_n) * V(:, :, g_n));
if g_ft(4, 4, g_n) == 1
    b(:, :, g_n) = b(:, :, g_n) - dAdj(inv(g_ft(:, :, g_n)), g_Fext(:, :, g_n));
end
c(:, :, g_n) = b(:, :, g_n) + aJ(:, :, g_n) * ad(V(:, :, g_n), g_S(:, :, g_n-1) * i_dq(g_n-1));
omega(:, g_n) = g_S(:, :, g_n-1)' * aJ(:, :, g_n) * g_S(:, :, g_n-1);

% Backward recursion : for i = n-1 to 1
for i = g_n-1 : -1 : 2
    aJ(:, :, i) = g_J(:, :, i) + Transform(invf(:, :, i+1), aJ(:, :, i+1) - (aJ(:, :, i+1) * g_S(:, :, i) * g_S(:, :, i)' * aJ(:, :, i+1)) / omega(:, i+1));
    b(:, :, i) = - dad(V(:, :, i), g_J(:, :, i) * V(:, :, i)) + dAdj(invf(:, :, i+1), c(:, :, i+1)) + dAdj(invf(:, :, i+1), aJ(:, :, i+1) * g_S(:, :, i)) / omega(:, i+1) * (i_tau(i) - g_S(:, :, i)'*c(:, :, i+1));
    if g_ft(4, 4, i) == 1
        b(:, :, i) = b(:, :, i) - dAd(inv(g_ft(:, :, i)), g_Fext(:, :, i));
    end
    c(:, :, i) = b(:, :, i) + aJ(:, :, i) * ad(V(:, :, i), g_S(:, :, i-1) * i_dq(i-1));
    omega(:, i) = g_S(:, :, i-1)' * aJ(:, :, i) * g_S(:, :, i-1);
end

%% Evaluate Accle. of base body in floating body
aJ(:, :, 1) = g_J(:, :, 1) + Transform(invf(:, :, 2), aJ(:, :, 2));
b(:, :, 1) = - dad(V(:, :, 1), g_J(:, :, 1) * V(:, :, 1)) + dAdj(invf(:, :, 2), c(:, :, 2));
if g_ft(4, 4, 1) == 1
        b(:, :, 1) = b(:, :, 1) - dAd(inv(g_ft(:, :, 1)), g_Fext(:, :, 1));
end
dV(:,:,1)=-pinv(aJ(:,:,1))*b(:,:,1);

% Forward recursion : for i = 1 to n
for i = 2 : g_n
    ddq(i-1) = (i_tau(i-1) - g_S(:, :, i-1)' * (aJ(:, :, i) * Adj(invf(:, :, i), dV(:, :, i-1)) + c(:, :, i)))/ omega(:, i);
    dV(:, :, i) = g_S(:, :, i-1) * ddq(i-1) + Adj(invf(:, :, i), dV(:, :, i-1)) + ad(V(:, :, i), g_S(:, :, i-1) * i_dq(i-1));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

o_ddq =  ddq(1);
for i = 2:g_n-1
    o_ddq = [o_ddq; ddq(i)];
end
dV0=dV(:,:,1);
