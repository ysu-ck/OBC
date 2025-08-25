function [sys,x0,str,ts,simStateCompliance] = ft_bucket(t,x,u,flag)

%
% The following outlines the general structure of an S-function.
%
switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys = mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=[];

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=[];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 330;
sizes.NumDiscStates  = 1;
sizes.NumOutputs     = 37;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0 = zeros(sizes.NumContStates + sizes.NumDiscStates, 1);
% x0 = [0, -1.0635, -2.1229, -3.1943,0.5, 0, 0, 0...
x0(1:330) = [0.45, 0, 0, ...
    0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,...
    0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,...
    0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,...
    0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,...
    0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,...
    0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,...
    0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,...
    0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,...
    0.45, 0, 0, 10^5, 10^5, 0, 0];
x0(331)=0;


%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

ue = x(331);
% 系统参数-挖掘机铲斗液压缸

m = 10;
L = 1.5;
A_1 = 0.002;
A_2 = 0.0015;
% V_1 = (0.25+0.45)*A_1;
% V_2 =(1.25-0.45)*A_2;
V_1 = 0.25*A_1;
V_2 =1.25*A_2;
K_t = 10^(-7);
beta_e = 8*10^8;
p_s = 10^9;
p_r = 0;
h_r = 1;
h_l = 1;
b_r = 0.01;
b_l = 0.01;
 % K_s = 100;
K_s = 0;
B_c = 80;
% F_l = 200+100*sin(t);
F_l = 0;
F_f = 0;
q_1 = 0;
q_2 = 0;
if ue>=0
    h = h_r;
else
    h = h_l;
end

if ue>=b_r
    b = -h_r*b_r;
elseif ue<=-b_l
    b = -h_l*b_l;
else 
    b = -h*ue;
end

a_2 = K_s/m;
b_2 = B_c/m;
g_2 = 1/m;
F_2 = (F_l+F_f)/m;
a_3 = (A_1^2)*beta_e/(V_1+A_1*x(1))+(A_2^2)*beta_e/(V_2-A_2*x(1));
% g_3 = h*A_1*beta_e*K_t*sqrt(p_s-x(327))/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(x(328)-p_r)/(V_2-A_2*x(1));
% if ue>=0
%     g_3 = h*A_1*beta_e*K_t*sqrt(p_s-x(327))/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(x(328)-p_r)/(V_2-A_2*x(1));
% else
%     g_3 = h*A_1*beta_e*K_t*sqrt(x(327)-p_r)/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(p_s-x(328))/(V_2-A_2*x(1));
% end
if ue>0
    g_3 = h*A_1*beta_e*K_t*sqrt(p_s-x(327))/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(x(328)-p_r)/(V_2-A_2*x(1));
elseif ue<0
    g_3 = h*A_1*beta_e*K_t*sqrt(x(327)-p_r)/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(p_s-x(328))/(V_2-A_2*x(1));
else 
    g_3 =0;
end
F_3 = A_1*beta_e*q_1/(V_1+A_1*x(1))+A_2*beta_e*q_2/(V_2-A_2*x(1));
% m = 10;
% A_1 = 0.002;
% A_2 = 0.0015;
% V_1 = 0.06;
% V_2 =0.045;
% C_i = 2.4*10^(-13);
% C_e = 2.4*10^(-14);
% beta_e = 8*10^8;
% C_t = 0.00074;
% % k_u = 0.15;
% k_u = 1;
% p_s = 10^7;
% p_r = 10^5;
% p_r = 0;
% h_r = 1;
% h_l = 1;
% b_r = 0;
% b_l = 0;
% % K_s = 100;
% K_s = 0;
% B_c = 80;
% F_l = 200+100*sin(t);
% F_l = 0;
% 
% n = A_2/A_1;
% a_2 = -K_s/m;
% b_2 = -B_c/m;
% F_2 = -F_l/m;
% 
% if ue>0
%     h = h_r;
% else
%     h = h_l;
% end
% 
% if ue>=b_r
%     b = -h_r*b_r;
% elseif ue<=-b_l
%     b = -h_l*b_l;
% else 
%     b = -h*ue;
% end
% 
% 
% % if ue>=0
%     g_2 = A_1/m;
%     a_3 = -((1+n^3)*beta_e*A_1)/V_1;
%     b_3 = -((C_i*(1+n^2)+C_e)*beta_e)/V_1;
%     g_3 = (1+n^3)*beta_e*C_t*k_u*h*sqrt((p_s+n*p_r+x(3))/(1+n^3))/V_1;
%     % F_3 = -beta_e*(C_i*(n^3-n^2)+C_e*n^3)*p_s/V_1-beta_e*(C_i*(n-1)+C_e*n)*p_r/V_1+(1+n^3)*beta_e*C_t*k_u*b*sqrt((p_s+n*p_r+x(3))/(1+n^3))/V_1;
% % else
%     % g_2 = -A_2/m;
%     % a_3 = ((1+n^3)*beta_e*A_2)/(V_2*n^3);
%     % b_3 = -((C_i*(1+n^2)+C_e*n^2)*beta_e)/(V_2*n^2);
%     % g_3 = -(1+n^3)*beta_e*C_t*k_u*h*sqrt((n^3*p_s-n^2*p_r-n^3*x(3))/(1+n^3))/(V_2*n^3);
%     % % F_3 = beta_e*(C_i*(n-1)+C_e)*p_s/(V_2*n^3)-beta_e*(C_i*(n-1)+C_e)*p_r/(V_2*n)-(1+n^3)*beta_e*C_t*k_u*b*sqrt((n^3*p_s-n^2*p_r-n^3*x(3))/(1+n^3))/(V_2*n^3);
% % end
%     % g_2 = A_1/m;
%     % a_3 = -((1+n^3)*beta_e*A_1)/V_1;
%     % b_3 = -((C_i*(1+n^2)+C_e)*beta_e)/V_1;
%     % g_3 = (1+n^3)*beta_e*C_t*k_u*h*sqrt((p_s+n*p_r+x(3))/(1+n^3))/V_1;
%     F_3 = 0;
% 跟踪目标
x_1d=0.5+0.3*sin(0.5*t);
x_1dd=0.3*cos(0.5*t);
% x_1d=0.5;
% x_1dd=0;
x_2d=x(329);
x_3d=x(330);

% 控制参数
global nd_j1
global nd_j2
global nd_j3
nd_j1=40;
nd_j2=40;
nd_j3=40;

gamma=1;
scal=2;
mu=3;
k=0.7;
k1=7;
k2=7;
k3=7;
k_o1=100;
k_o2=100;
k_o3=100;
Ki=0.2;
kappa_2=0.5;
kappa_3=0.5;

r_c1=4*scal;
r_a1=6*scal;
r_c2=4*scal;
r_a2=6*scal;
r_c3=4*scal;
r_a3=6*scal;
r_c4=10;
% %状态观测器
S_f2=zeros(nd_j2,1);
for i = 1:1:nd_j2
    S_f2(i)=exp(-norm([x(324),x(325),x(326)]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end 
S_f3=zeros(nd_j3,1);
for i = 1:1:nd_j3
    S_f3(i)=exp(-norm([x(324),x(325),x(326),ue]'-[10,10,10,10]'+(1/2)*[i,i,i,i]')^2/mu^2);
end 

S_f2_2=S_f2*S_f2';
S_f3_2=S_f3*S_f3';

W_f2=x(4+nd_j2*6:3+nd_j2*7);
W_f3=x(4+nd_j2*7:3+nd_j2*8);

hatxd(1)=x(325)-k_o1*(x(324)-x(1));
hatxd(2)=x(326)-k_o2*x(325)+W_f2'*S_f2;
hatxd(3)=ue-k_o3*x(326)+W_f3'*S_f3;

% hatxd(1)=x(325)-k_o1*(x(324)-x(1));
% hatxd(2)=x(326)-k_o2*x(325)+W_f2'*S_f2-100*(x(324)-x(1));
% hatxd(3)=ue-k_o3*x(326)+W_f3'*S_f3+100*(x(324)-x(1));

% hatxd(1)=x(325)-k_o1*(x(324)-x(1));
% hatxd(2)=x(326)-100*(x(324)-x(1));
% hatxd(3)=ue+100*(x(324)-x(1));

tildex(1)=x(324)-x(1);
tildex(2)=x(325)-x(2);
tildex(3)=x(326)-x(3);
% X1-子系统

% z1=x(324)-x_1d;
z1=x(1)-x_1d;
S_j1=zeros(nd_j1,1);
for i = 1:1:nd_j1
    S_j1(i)=exp(-norm([z1,tildex(1)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);
end 
S_j1_2=S_j1*S_j1';
W_c1=x(4:3+nd_j1);
W_a1=x(4+nd_j1:3+nd_j1*2);

% alpha_1=-k1*sign(z1)*abs(z1)^(2*k-1)-((2*k-1)/(2*k))*sign(z1)*abs(z1)^(1/(2*k-1))-0.75*z1+k_o1*(x(324)-x(1))-0.5*W_a1'*S_j1;
alpha_1=-k1*sign(z1)*abs(z1)^(2*k-1)-((2*k-1)/(2*k))*sign(z1)*abs(z1)^(1/(2*k-1))-0.75*z1+k_o1*tildex(1)+x_1dd-0.5*W_a1'*S_j1;


% X2-子系统
% z2=x(325)-x_2d;
% z2=x(2)-alpha_1;
z2=x(2)-x_2d;
S_j2=zeros(nd_j2,1);
for i=1:1:nd_j2
    S_j2(i)=exp(-norm([z2,x(2)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);
end
S_j2_2=S_j2*S_j2';
W_c2=x(4+nd_j1*2:3+nd_j1*2+nd_j2);
W_a2=x(4+nd_j1*2+nd_j2:3+nd_j1*2+nd_j2*2);


% alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2+k_o2*x(325)-W_f2'*S_f2-0.5*W_a2'*S_j2;
alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2...
        +k_o2*x(325)-sign(x_2d-alpha_1)*abs(x_2d-alpha_1)^(2*k-1)/Ki-W_f2'*S_f2-0.5*W_a2'*S_j2;

% X3-子系统
% z3=x(326)-x_3d;
% z3=x(3)-alpha_2;
z3=x(3)-x_3d;
S_j3=zeros(nd_j3,1);
for i=1:1:nd_j3
    S_j3(i)=exp(-norm([z3,x(1),x(3)]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end

S_j3_2=S_j3*S_j3';
W_c3=x(4+nd_j1*2+nd_j2*2:3+nd_j1*2+nd_j2*2+nd_j3);
W_a3=x(4+nd_j1*2+nd_j2*2+nd_j3:3+nd_j1*2+nd_j2*2+nd_j3*2);

% ue=-k3*sign(z3)*abs(z3)^(2*k-1)-0.75*z3+k_o3*x(326)-W_f3'*S_f3-0.5*W_a3'*S_j3;
ue=-k3*sign(z3)*abs(z3)^(2*k-1)-0.75*z3+k_o3*x(326)-sign(x_3d-alpha_2)*abs(x_3d-alpha_2)^(2*k-1)/Ki-W_f3'*S_f3-0.5*W_a3'*S_j3;

%***********************

% 系统模型-挖掘机液压缸
sys(1) = x(2);
sys(2) = -a_2*x(1)-b_2*x(2)+g_2*x(3)-F_2;
sys(3) = -a_3*x(2)+g_3*ue-(F_3-g_3*b);

% X1-子系统
for i=1:1:nd_j1
    sys(3+i) = -r_c1*S_j1_2(i,:)*W_c1; % W_c1
end

for i=1:1:nd_j1
    sys(3+nd_j1+i) = -S_j1_2(i,:)*(r_a1*(W_a1-W_c1)+r_c1*W_c1); % W_a1
end

% X2-子系统
for i=1:1:nd_j2
    sys(3+nd_j1*2+i) = -r_c2*S_j2_2(i,:)*W_c2; % W_c2
end

for i=1:1:nd_j2
    sys(3+nd_j1*2+nd_j2+i) = -S_j2_2(i,:)*(r_a2*(W_a2-W_c2)+r_c2*W_c2); % W_a2
end

% X3-子系统
for i=1:1:nd_j3
    sys(3+nd_j1*2+nd_j2*2+i) = -r_c3*S_j3_2(i,:)*W_c3; % W_c3
end
for i=1:1:nd_j3
    sys(3+nd_j1*2+nd_j2*2+nd_j3+i) = -S_j3_2(i,:)*(r_a3*(W_a3-W_c3)+r_c3*W_c3); % W_a3
end

% 观测器-子系统

for i=1:1:nd_j2
    sys(3+nd_j2*6+i) = -r_c4*S_f2_2(i,:)*W_f2; % W_f2
end

for i=1:1:nd_j3
    sys(3+nd_j3*7+i) = -r_c4*S_f3_2(i,:)*W_f3; % W_f3
end

sys(324)=hatxd(1);
sys(325)=hatxd(2);
sys(326)=hatxd(3);
%p1\p2
    % Q_1=K_t*x(327)*sqrt(p_s-x(327));
    % Q_2=K_t*x(327)*sqrt(x(328)-p_r);
% if ue>=0
%     Q_1=K_t*x(329)*sqrt(p_s-x(327));
%     Q_2=K_t*x(329)*sqrt(x(328)-p_r);
% else
%     Q_1=K_t*x(329)*sqrt(x(327)-p_r);
%     Q_2=K_t*x(329)*sqrt(p_s-x(328));
% end

if ue>0
    Q_1=K_t*x(331)*sqrt(p_s-x(327));
    Q_2=K_t*x(331)*sqrt(x(328)-p_r);
elseif ue<0
    Q_1=K_t*x(331)*sqrt(x(327)-p_r);
    Q_2=K_t*x(331)*sqrt(p_s-x(328));
else 
    Q_1=0;
    Q_2=0;
end

sys(327)=beta_e*(Q_1-A_1*x(2)-q_1)/(V_1+A_1*x(1));
sys(328)=beta_e*(-Q_2+A_2*x(2)+q_2)/(V_2-A_2*x(1));
sys(329)=-sign(x_2d-alpha_1)*abs(x_2d-alpha_1)^(2*k-1)/Ki;
sys(330)=-sign(x_3d-alpha_2)*abs(x_3d-alpha_2)^(2*k-1)/Ki;
% sys(329)=ue;
% fprintf('*** t = %.4f, size(sys) = [%d×%d]\n', t, size(sys,1), size(sys,2));
% end mdlDerivatives

    % 查找所有非实数（NaN、Inf 或带虚部）的下标
bad = find(~isreal(sys) | isnan(sys) | isinf(sys));
if ~isempty(bad)
    % 直接用 MATLAB error
    error('ft_bucket:BadDerivative', ...
          'ft_bucket: 导数在索引 %s 上出现了 NaN/Inf/复数！', ...
          mat2str(bad));
end


function sys = mdlUpdate(t,x,u)

     ue = x(331);
     
% 跟踪目标
x_1d=0.5+0.3*sin(0.5*t);
x_1dd=0.3*cos(0.5*t);
% x_1d=0.5;
% x_1dd=0;
x_2d=x(329);
x_3d=x(330);

% 控制参数
nd_j1=40;
nd_j2=40;
nd_j3=40;

gamma=1;
scal=2;
mu=3;
k=0.7;
k1=7;
k2=7;
k3=7;
k_o1=100;
k_o2=100;
k_o3=100;
Ki=0.2;
kappa_2=0.5;
kappa_3=0.5;

r_c1=4*scal;
r_a1=6*scal;
r_c2=4*scal;
r_a2=6*scal;
r_c3=4*scal;
r_a3=6*scal;
r_c4=10;
% %状态观测器
S_f2=zeros(nd_j2,1);
for i = 1:1:nd_j2
    S_f2(i)=exp(-norm([x(324),x(325),x(326)]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end 
S_f3=zeros(nd_j3,1);
for i = 1:1:nd_j3
    S_f3(i)=exp(-norm([x(324),x(325),x(326),ue]'-[10,10,10,10]'+(1/2)*[i,i,i,i]')^2/mu^2);
end 

S_f2_2=S_f2*S_f2';
S_f3_2=S_f3*S_f3';

W_f2=x(4+nd_j2*6:3+nd_j2*7);
W_f3=x(4+nd_j2*7:3+nd_j2*8);
hatxd(1)=x(325)-k_o1*(x(324)-x(1));
hatxd(2)=x(326)-k_o2*x(325)+W_f2'*S_f2;
hatxd(3)=ue-k_o3*x(326)+W_f3'*S_f3;

% hatxd(1)=x(325)-k_o1*(x(324)-x(1));
% hatxd(2)=x(326)-k_o2*x(325)+W_f2'*S_f2-100*(x(324)-x(1));
% hatxd(3)=ue-k_o3*x(326)+W_f3'*S_f3+100*(x(324)-x(1));

% hatxd(1)=x(325)-k_o1*(x(324)-x(1));
% hatxd(2)=x(326)-100*(x(324)-x(1));
% hatxd(3)=ue+100*(x(324)-x(1));

tildex(1)=x(324)-x(1);
tildex(2)=x(325)-x(2);
tildex(3)=x(326)-x(3);
% X1-子系统

% z1=x(324)-x_1d;
z1=x(1)-x_1d;
S_j1=zeros(nd_j1,1);
for i = 1:1:nd_j1
    S_j1(i)=exp(-norm([z1,tildex(1)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);
end 
S_j1_2=S_j1*S_j1';
W_c1=x(4:3+nd_j1);
W_a1=x(4+nd_j1:3+nd_j1*2);

% alpha_1=-k1*sign(z1)*abs(z1)^(2*k-1)-((2*k-1)/(2*k))*sign(z1)*abs(z1)^(1/(2*k-1))-0.75*z1+k_o1*(x(324)-x(1))-0.5*W_a1'*S_j1;
alpha_1=-k1*sign(z1)*abs(z1)^(2*k-1)-((2*k-1)/(2*k))*sign(z1)*abs(z1)^(1/(2*k-1))-0.75*z1+k_o1*tildex(1)+x_1dd-0.5*W_a1'*S_j1;


% X2-子系统
% z2=x(325)-x_2d;
% z2=x(2)-alpha_1;
z2=x(2)-x_2d;
S_j2=zeros(nd_j2,1);
for i=1:1:nd_j2
    S_j2(i)=exp(-norm([z2,x(2)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);
end
S_j2_2=S_j2*S_j2';
W_c2=x(4+nd_j1*2:3+nd_j1*2+nd_j2);
W_a2=x(4+nd_j1*2+nd_j2:3+nd_j1*2+nd_j2*2);


% alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2+k_o2*x(325)-W_f2'*S_f2-0.5*W_a2'*S_j2;
alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2...
        +k_o2*x(325)-sign(x_2d-alpha_1)*abs(x_2d-alpha_1)^(2*k-1)/Ki-W_f2'*S_f2-0.5*W_a2'*S_j2;

% X3-子系统
% z3=x(326)-x_3d;
% z3=x(3)-alpha_2;
z3=x(3)-x_3d;
S_j3=zeros(nd_j3,1);
for i=1:1:nd_j3
    S_j3(i)=exp(-norm([z3,x(1),x(3)]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end

S_j3_2=S_j3*S_j3';
W_c3=x(4+nd_j1*2+nd_j2*2:3+nd_j1*2+nd_j2*2+nd_j3);
W_a3=x(4+nd_j1*2+nd_j2*2+nd_j3:3+nd_j1*2+nd_j2*2+nd_j3*2);

% ue=-k3*sign(z3)*abs(z3)^(2*k-1)-0.75*z3+k_o3*x(326)-W_f3'*S_f3-0.5*W_a3'*S_j3;
ue=-k3*sign(z3)*abs(z3)^(2*k-1)-0.75*z3+k_o3*x(326)-sign(x_3d-alpha_2)*abs(x_3d-alpha_2)^(2*k-1)/Ki-W_f3'*S_f3-0.5*W_a3'*S_j3;


  sys = ue;   % 这是唯一的离散状态更新
% end


%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
% 系统参数-挖掘机铲斗液压缸
 ue = x(331);
% 系统参数-挖掘机铲斗液压缸

m = 10;
L = 1.5;
A_1 = 0.002;
A_2 = 0.0015;
% V_1 = (0.25+0.45)*A_1;
% V_2 =(1.25-0.45)*A_2;
V_1 = 0.25*A_1;
V_2 =1.25*A_2;
K_t = 10^(-7);
beta_e = 8*10^8;
p_s = 10^9;
p_r = 0;
h_r = 1;
h_l = 1;
b_r = 0.01;
b_l = 0.01;
 % K_s = 100;
K_s = 0;
B_c = 80;
% F_l = 200+100*sin(t);
F_l = 0;
F_f = 0;
q_1 = 0;
q_2 = 0;
if ue>=0
    h = h_r;
else
    h = h_l;
end

if ue>=b_r
    b = -h_r*b_r;
elseif ue<=-b_l
    b = -h_l*b_l;
else 
    b = -h*ue;
end

a_2 = K_s/m;
b_2 = B_c/m;
g_2 = 1/m;
F_2 = (F_l+F_f)/m;
a_3 = (A_1^2)*beta_e/(V_1+A_1*x(1))+(A_2^2)*beta_e/(V_2-A_2*x(1));
% g_3 = h*A_1*beta_e*K_t*sqrt(p_s-x(327))/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(x(328)-p_r)/(V_2-A_2*x(1));
% if ue>=0
%     g_3 = h*A_1*beta_e*K_t*sqrt(p_s-x(327))/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(x(328)-p_r)/(V_2-A_2*x(1));
% else
%     g_3 = h*A_1*beta_e*K_t*sqrt(x(327)-p_r)/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(p_s-x(328))/(V_2-A_2*x(1));
% end
if ue>0
    g_3 = h*A_1*beta_e*K_t*sqrt(p_s-x(327))/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(x(328)-p_r)/(V_2-A_2*x(1));
elseif ue<0
    g_3 = h*A_1*beta_e*K_t*sqrt(x(327)-p_r)/(V_1+A_1*x(1))+h*A_2*beta_e*K_t*sqrt(p_s-x(328))/(V_2-A_2*x(1));
else 
    g_3 =0;
end
F_3 = A_1*beta_e*q_1/(V_1+A_1*x(1))+A_2*beta_e*q_2/(V_2-A_2*x(1));
% 跟踪目标
x_1d=0.5+0.3*sin(0.5*t);
x_1dd=0.3*cos(0.5*t);
% x_1d=0.5;
% x_1dd=0;
x_2d=x(329);
x_3d=x(330);
% 控制参数

nd_j1=40;
nd_j2=40;
nd_j3=40;

gamma=1;
scal=2;
mu=3;
k=0.7;
k1=7;
k2=7;
k3=7;
k_o1=100;
k_o2=100;
k_o3=100;
Ki=0.2;
% %状态观测器
S_f2=zeros(nd_j2,1);
for i = 1:1:nd_j2
    S_f2(i)=exp(-norm([x(324),x(325),x(326)]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end 
S_f3=zeros(nd_j3,1);
for i = 1:1:nd_j3
    S_f3(i)=exp(-norm([x(324),x(325),x(326),ue]'-[10,10,10,10]'+(1/2)*[i,i,i,i]')^2/mu^2);
end 

W_f2=x(4+nd_j2*6:3+nd_j2*7);
W_f3=x(4+nd_j2*7:3+nd_j2*8);

tildex(1)=x(324)-x(1);
tildex(2)=x(325)-x(2);
tildex(3)=x(326)-x(3);

% X1-子系统

% z1=x(324)-x_1d;
z1=x(1)-x_1d;
S_j1=zeros(nd_j1,1);
for i = 1:1:nd_j1
    S_j1(i)=exp(-norm([z1,tildex(1)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);
end 
S_j1_2=S_j1*S_j1';
W_c1=x(4:3+nd_j1);
W_a1=x(4+nd_j1:3+nd_j1*2);

% alpha_1=-k1*sign(z1)*abs(z1)^(2*k-1)-((2*k-1)/(2*k))*sign(z1)*abs(z1)^(1/(2*k-1))-0.75*z1+k_o1*(x(324)-x(1))-0.5*W_a1'*S_j1;
alpha_1=-k1*sign(z1)*abs(z1)^(2*k-1)-((2*k-1)/(2*k))*sign(z1)*abs(z1)^(1/(2*k-1))-0.75*z1+k_o1*tildex(1)+x_1dd-0.5*W_a1'*S_j1;


% X2-子系统
% z2=x(325)-x_2d;
% z2=x(2)-alpha_1;
z2=x(2)-x_2d;
S_j2=zeros(nd_j2,1);
for i=1:1:nd_j2
    S_j2(i)=exp(-norm([z2,x(2)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);
end
S_j2_2=S_j2*S_j2';
W_c2=x(4+nd_j1*2:3+nd_j1*2+nd_j2);
W_a2=x(4+nd_j1*2+nd_j2:3+nd_j1*2+nd_j2*2);


% alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2+k_o2*x(325)-W_f2'*S_f2-0.5*W_a2'*S_j2;
alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2...
        -sign(x_2d-alpha_1)*abs(x_2d-alpha_1)^(2*k-1)/Ki-W_f2'*S_f2-0.5*W_a2'*S_j2;

% X3-子系统
% z3=x(326)-x_3d;
% z3=x(3)-alpha_2;
z3=x(3)-x_3d;
S_j3=zeros(nd_j3,1);
for i=1:1:nd_j3
    S_j3(i)=exp(-norm([z3,x(1),x(3)]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end

S_j3_2=S_j3*S_j3';
W_c3=x(4+nd_j1*2+nd_j2*2:3+nd_j1*2+nd_j2*2+nd_j3);
W_a3=x(4+nd_j1*2+nd_j2*2+nd_j3:3+nd_j1*2+nd_j2*2+nd_j3*2);

% ue=-k3*sign(z3)*abs(z3)^(2*k-1)-0.75*z3+k_o3*x(326)-W_f3'*S_f3-0.5*W_a3'*S_j3;
ue=-k3*sign(z3)*abs(z3)^(2*k-1)-0.75*z3+k_o3*x(326)-sign(x_3d-alpha_2)*abs(x_3d-alpha_2)^(2*k-1)/Ki-W_f3'*S_f3-0.5*W_a3'*S_j3;

%***********************

% if ue>=0
%     Q_1=K_t*x(329)*sqrt(p_s-x(327));
%     Q_2=K_t*x(329)*sqrt(x(328)-p_r);
% else
%     Q_1=K_t*x(329)*sqrt(x(327)-p_r);
%     Q_2=K_t*x(329)*sqrt(p_s-x(328));
% end
if ue>0
    Q_1=K_t*x(331)*sqrt(p_s-x(327));
    Q_2=K_t*x(331)*sqrt(x(328)-p_r);
elseif ue<0
    Q_1=K_t*x(331)*sqrt(x(327)-p_r);
    Q_2=K_t*x(331)*sqrt(p_s-x(328));
else 
    Q_1=0;
    Q_2=0;
end
dotp1=beta_e*(Q_1-A_1*x(2)-q_1)/(V_1+A_1*x(1));
dotp2=beta_e*(-Q_2+A_2*x(2)+q_2)/(V_2-A_2*x(1));
% 代价函数计算
% J1=W_c1'*S_j1+2*k1*z1-2*x(4)*x(1);
% J2=W_c2'*S_j2+2*k2*z2;
% J3=W_c3'*S_j3+2*k3*z3-2*x(4+nd_j1*2+nd_j2*2+1)*x(2);

% 代价函数计算
h1=z1^2+alpha_1^2;
h2=z2^2+alpha_2^2;
h3=z3^2+ue^2;

% 输出状态
sys(1) = x(1);
sys(2) = x(2);
sys(3) = x(3);
% 观测状态
sys(4) = x(324);
sys(5) = x(325);
sys(6) = x(326);
%观测误差
sys(7) = tildex(1);
sys(8) = tildex(2);
sys(9) = tildex(3);
% 输出误差
sys(10) = z1;
sys(11) = z2;
sys(12) = z3;

% 权值
sys(13) = norm(W_c1);
sys(14) = norm(W_a1);

% 权值
sys(15) = norm(W_c2);
sys(16) = norm(W_a2);

% 权值
sys(17) = norm(W_c3);
sys(18) = norm(W_a3);

% 权值
sys(19) = norm(W_f2);
sys(20) = norm(W_f3);

% 代价函数
sys(21) = h1;
sys(22) = h2;
sys(23) = h3;

% 期望信号
sys(24) = x_1d;
sys(25) = ue;
% 
sys(26) = alpha_1;
sys(27) = alpha_2;
sys(28) = x_2d-alpha_1;
sys(29) = x_3d-alpha_2;
sys(30) = dotp1;
sys(31) = dotp2;
% sys(32) = Q_1;
% sys(33) = Q_2;
sys(32) = W_f2'*S_f2+a_2*x(1)+b_2*x(2)-(g_2-1)*x(3)-F_2-k_o2*x(2);
sys(33) = W_f3'*S_f3+a_3*x(2)-(g_3-1)*ue-F_3-k_o3*x(3);
%p1\p2
sys(34) = x(327);
sys(35) = x(328);
sys(36) = x_2d;
sys(37) = x_3d;
    % 查找所有非实数（NaN、Inf 或带虚部）的下标
bad = find(~isreal(sys) | isnan(sys) | isinf(sys));
if ~isempty(bad)
    % 直接用 MATLAB error
    error('ft_bucket:BadDerivative', ...
          'ft_bucket: 输出在索引 %s 上出现了 NaN/Inf/复数！', ...
          mat2str(bad));
end
 % end mdlOutputs
