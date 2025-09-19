function [sys,x0,str,ts,simStateCompliance] = ft(t,x,u,flag)

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

sizes.NumContStates  = 205;
sizes.NumDiscStates  = 1;
sizes.NumOutputs     = 20;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0 = zeros(sizes.NumContStates + sizes.NumDiscStates, 1);

x0(1:205) = [0.45, 0,...
    0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,...
    0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,...
    0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,...
    0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,...
    0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,...
    0.45, 0, 0];
x0(206)=0;


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

if length(x) < 206
    error('状态向量长度不足: %d < 206', length(x));
end

alpha_2 = x(206);
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

a_2 = K_s/m;
b_2 = B_c/m;
g_2 = 1/m;
F_2 = (F_l+F_f)/m;
a_3 = (A_1^2)*beta_e/(V_1+A_1*x(1))+(A_2^2)*beta_e/(V_2-A_2*x(1));

% 跟踪目标
x_1d=0.5+0.3*sin(0.5*t);
x_1dd=0.3*cos(0.5*t);
x_2d=x(205);

% 控制参数
global nd_j1
global nd_j2
nd_j1=40;
nd_j2=40;

gamma=1;
scal=2;
mu=3;
k=0.8;
k1=0.7;
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
    S_f2(i)=exp(-norm([x(203),x(204),alpha_2]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end 

S_f2_2=S_f2*S_f2';

W_f2=x(3+nd_j2*4:2+nd_j2*5);

hatxd(1)=x(204)-k_o1*(x(203)-x(1));
hatxd(2)=x(206)-k_o2*x(204)+W_f2'*S_f2;

tildex(1)=x(203)-x(1);
tildex(2)=x(204)-x(2);
% X1-子系统

% z1=x(324)-x_1d;
z1=x(1)-x_1d;
S_j1=zeros(nd_j1,1);
for i = 1:1:nd_j1
    S_j1(i)=exp(-norm([z1,tildex(1)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);                                                                                                                                                                                                                                                                                                                                                                                                                                                   
end 
S_j1_2=S_j1*S_j1';
W_c1=x(3:2+nd_j1);
W_a1=x(3+nd_j1:2+nd_j1*2);

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
W_c2=x(3+nd_j1*2:2+nd_j1*2+nd_j2);
W_a2=x(3+nd_j1*2+nd_j2:2+nd_j1*2+nd_j2*2);


alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2...
        +k_o2*x(204)-sign(x_2d-alpha_1)*abs(x_2d-alpha_1)^(2*k-1)/Ki-W_f2'*S_f2-0.5*W_a2'*S_j2;


%***********************

% 系统模型-挖掘机液压缸
sys(1) = x(2);
sys(2) = -a_2*x(1)-b_2*x(2)+g_2*alpha_2-F_2;


% X1-子系统
for i=1:1:nd_j1
    sys(2+i) = -r_c1*S_j1_2(i,:)*W_c1; % W_c1
end

for i=1:1:nd_j1
    sys(2+nd_j1+i) = -S_j1_2(i,:)*(r_a1*(W_a1-W_c1)+r_c1*W_c1); % W_a1
end

% X2-子系统
for i=1:1:nd_j2
    sys(2+nd_j1*2+i) = -r_c2*S_j2_2(i,:)*W_c2; % W_c2
end

for i=1:1:nd_j2
    sys(2+nd_j1*2+nd_j2+i) = -S_j2_2(i,:)*(r_a2*(W_a2-W_c2)+r_c2*W_c2); % W_a2
end


% 观测器-子系统

for i=1:1:nd_j2
    sys(2+nd_j2*4+i) = -r_c4*S_f2_2(i,:)*W_f2; % W_f2
end

sys(203)=hatxd(1);
sys(204)=hatxd(2);
sys(205)=-sign(x_2d-alpha_1)*abs(x_2d-alpha_1)^(2*k-1)/Ki;

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

     alpha_2 = x(206);
     
% 跟踪目标
x_1d=0.5+0.3*sin(0.5*t);
x_1dd=0.3*cos(0.5*t);
x_2d=x(205);

% 控制参数
global nd_j1
global nd_j2
nd_j1=40;
nd_j2=40;

gamma=1;
scal=2;
mu=3;
k=0.8;
k1=0.7;
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
    S_f2(i)=exp(-norm([x(203),x(204),alpha_2]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end 

S_f2_2=S_f2*S_f2';

W_f2=x(3+nd_j2*4:2+nd_j2*5);

hatxd(1)=x(204)-k_o1*(x(203)-x(1));
hatxd(2)=x(206)-k_o2*x(204)+W_f2'*S_f2;

tildex(1)=x(203)-x(1);
tildex(2)=x(204)-x(2);
% X1-子系统

% z1=x(324)-x_1d;
z1=x(1)-x_1d;
S_j1=zeros(nd_j1,1);
for i = 1:1:nd_j1
    S_j1(i)=exp(-norm([z1,tildex(1)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);                                                                                                                                                                                                                                                                                                                                                                                                                                                   
end 
S_j1_2=S_j1*S_j1';
W_c1=x(3:2+nd_j1);
W_a1=x(3+nd_j1:2+nd_j1*2);

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
W_c2=x(3+nd_j1*2:2+nd_j1*2+nd_j2);
W_a2=x(3+nd_j1*2+nd_j2:2+nd_j1*2+nd_j2*2);


alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2...
        +k_o2*x(204)-sign(x_2d-alpha_1)*abs(x_2d-alpha_1)^(2*k-1)/Ki-W_f2'*S_f2-0.5*W_a2'*S_j2;


  sys = alpha_2;   % 这是唯一的离散状态更新
% end


%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

if length(x) < 206
    error('状态向量长度不足: %d < 206', length(x));
end

% 系统参数-挖掘机铲斗液压缸
alpha_2 = x(206);
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


a_2 = K_s/m;
b_2 = B_c/m;
g_2 = 1/m;
F_2 = (F_l+F_f)/m;
a_3 = (A_1^2)*beta_e/(V_1+A_1*x(1))+(A_2^2)*beta_e/(V_2-A_2*x(1));

% 跟踪目标
x_1d=0.5+0.3*sin(0.5*t);
x_1dd=0.3*cos(0.5*t);
x_2d=x(205);

% 控制参数
global nd_j1
global nd_j2
nd_j1=40;
nd_j2=40;

gamma=1;
scal=2;
mu=3;
k=0.8;
k1=0.7;
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
    S_f2(i)=exp(-norm([x(203),x(204),alpha_2]'-[10,10,10]'+(1/2)*[i,i,i]')^2/mu^2);
end 

S_f2_2=S_f2*S_f2';

W_f2=x(3+nd_j2*4:2+nd_j2*5);

hatxd(1)=x(204)-k_o1*(x(203)-x(1));
hatxd(2)=x(206)-k_o2*x(204)+W_f2'*S_f2;

tildex(1)=x(203)-x(1);
tildex(2)=x(204)-x(2);
% X1-子系统

% z1=x(324)-x_1d;
z1=x(1)-x_1d;
S_j1=zeros(nd_j1,1);
for i = 1:1:nd_j1
    S_j1(i)=exp(-norm([z1,tildex(1)]'-[10,10]'+(1/2)*[i,i]')^2/mu^2);                                                                                                                                                                                                                                                                                                                                                                                                                                                   
end 
S_j1_2=S_j1*S_j1';
W_c1=x(3:2+nd_j1);
W_a1=x(3+nd_j1:2+nd_j1*2);

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
W_c2=x(3+nd_j1*2:2+nd_j1*2+nd_j2);
W_a2=x(3+nd_j1*2+nd_j2:2+nd_j1*2+nd_j2*2);


alpha_2=-k2*sign(z2)*abs(z2)^(2*k-1)-((2*k-1)/(2*k))*sign(z2)*abs(z2)^(1/(2*k-1))-1.25*z2...
        +k_o2*x(204)-sign(x_2d-alpha_1)*abs(x_2d-alpha_1)^(2*k-1)/Ki-W_f2'*S_f2-0.5*W_a2'*S_j2;



% 代价函数计算
h1=z1^2+alpha_1^2;
h2=z2^2+alpha_2^2;


% 输出状态
sys(1) = x(1);
sys(2) = x(2);

% 观测状态
sys(3) = x(203);
sys(4) = x(204);

%观测误差
sys(5) = tildex(1);
sys(6) = tildex(2);

% 输出误差
sys(7) = z1;
sys(8) = z2;


% 权值
sys(9) = norm(W_c1);
sys(10) = norm(W_a1);

% 权值
sys(11) = norm(W_c2);
sys(12) = norm(W_a2);

% 权值
sys(13) = norm(W_f2);

% 代价函数
sys(14) = h1;
sys(15) = h2;

% 期望信号
sys(16) = x_1d;
sys(17) = x_2d;
% 
sys(18) = alpha_1;
sys(19) = alpha_2;
sys(20) = x_2d-alpha_1;


    % 查找所有非实数（NaN、Inf 或带虚部）的下标
bad = find(~isreal(sys) | isnan(sys) | isinf(sys));
if ~isempty(bad)
    % 直接用 MATLAB error
    error('ft_bucket:BadDerivative', ...
          'ft_bucket: 输出在索引 %s 上出现了 NaN/Inf/复数！', ...
          mat2str(bad));
end
 % end mdlOutputs
