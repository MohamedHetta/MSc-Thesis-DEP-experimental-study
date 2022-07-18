function [Ts_100,I_max,Throttle,I_cruise,RPM] = Combo_Estimator_3(Motor,Prop,Battery,ESC,Req,Vc,nm,np,nn)

%% Renaming Variables

kv = Motor.kv(nm);
rm = Motor.rm(nm);
io = Motor.io(nm);
nmotor = Motor.n(nn);
s = Battery.s;
rb = Battery.rb;
resc = ESC.r;
B = Prop.Blades(np);
D = Prop.D(np);
P = Prop.P(np);
Pc = Prop.Pc(np);
Tc = Prop.Tc(np);

%% Calculating Some Data

v = s*3.7;
kt = 9.5684/kv;
rbt = rb * s;
R = rbt + 1/nmotor*(resc+rm);

%% Solving for RPM @ 100% Throttle

syms x

Propeller_Power = sqrt(B-1) * Pc * 4.0188e-15 * D^4 * P * (x)^3;
System_Power = 2*pi*x/60*kt*(((v-x/kv)*(1/(R*nmotor)))-io);

s = vpasolve( Propeller_Power == System_Power ,x);

if s(1)>0 && s(1)<kv*v
    RPM = s(1);
elseif s(2)>0 && s(1)<kv*v
     RPM = s(2);
else
     RPM = s(3);  
end

RPM = double(RPM);              % Full Throttle RPM

%% Calculating System Performance - Single Motor @ 100% Throttle

% Propeller efficiency due to Blade Number

if B == 2
   eta_B = 1;
elseif B == 3
   eta_B = 0.9;
end

% Static Thrust (100%)

Ts_100 = eta_B * sqrt(B-1) * Tc * 2.6908e-9 * D^3 * P * RPM^2;             

% Dynamic Thrust (100%) @ Required Cruise Velocity

Vp = P * 0.0254 * RPM/60;      % Pitch Speed
C1 = -31/130 * Ts_100/(Vp^2);    % Constant 1    
C2 = -0.4534 * Ts_100/Vp;        % Constant 2

Td_100 = Ts_100 + C1*Vc^2 + C2*Vc; 

% Current Draw

Ps = Pc * sqrt(B-1) * 4.0188e-15 * D^4 * P * RPM^3;
Tau = Ps/(2*pi*RPM/60);
I_max = (Tau/kt + io);

%% Calculating System Performance - Single Motor @ Required Cruise Velocity

if Td_100 >= Req.Tdynamic/nmotor;

% Calculate Throttle

k1 = eta_B * sqrt(B-1) * Tc * 2.6908e-9 * D^3 * P * (RPM/100)^2;
k2 = P * 0.0254 * RPM/(100*60);

poly = [k1 -0.4543*k1/k2*Vc -31/130*k1/k2*Vc^2-Req.Tdynamic/nmotor];
R = roots(poly);

    if R(1)>0 && R(1)<100
        Throttle = R(1);
    else
        Throttle = R(2);  
    end

RPM_T = RPM/100 * Throttle;

% Current Draw @ Cruise Velocity

Ps_c = Pc * sqrt(B-1) * 4.0188e-15 * D^4 * P * RPM_T^3;
Tau_c = Ps_c/(2*pi*RPM_T/60);
I_cruise = (Tau_c/kt + io);

else

Throttle = nan;
I_cruise = nan;

end

end