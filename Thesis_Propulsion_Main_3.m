clc
clear all
close all

%% Aircraft Data

w_empty = 750;  % Empty Weight (g)
Vc = 18.5;        % Cruise Speed (m/s)

%% Propeller Data

[Prop.Manu,Prop.D,Prop.P,Prop.Tc,Prop.Pc,Prop.Blades]=readvars('Mini_Propeller_Database.xlsx');

w_prop = 10;

%% Battery and ESC Data

Battery.s = 3;
Battery.rb = 0.0033;
ESC.r = 0.006;

w_esc = 10;
w_batt = 220;

%% Motor Data

Motor.n = [4,6];
[Motor.manu,Motor.model,Motor.kv,Motor.rm,Motor.io,Motor.w]=readvars('Mini_Motor_Database.xlsx');

%% Requirements

Req.TWratio = 1.5;         % Min Thrust-Weight Ratio
Req.Tdynamic = 150;         % Min Cruise Required Thrust (g)
Req.maxDW = 450;            % Max Drive Weight (g)
Req.maxCurrent = 100;       % Max Current Draw (A)

%% Combination Performance Estimator

Accepted_Motors_arr = [];

m = 1;
for nn = 1:length(Motor.n)
    
    for np = 1:length(Prop.D)
    
        for nm = 1:length(Motor.kv)
        
        [Ts_100,I_max,Throttle,I_cruise,RPM] = Combo_Estimator_3(Motor,Prop,Battery,ESC,Req,Vc,nm,np,nn);
        
            if isnan(Throttle) == 0

            % Full System Performance

            Tstatic = Ts_100 * Motor.n(nn);
            Imax = I_max * Motor.n(nn);
            Icruise = I_cruise * Motor.n(nn);            
            DW = Motor.w(nm) * Motor.n(nn) + w_esc * Motor.n(nn) + w_batt + w_prop * Motor.n(nn);
            MTOW = w_empty + DW;

                if (Tstatic/MTOW >= Req.TWratio) && (DW <= Req.maxDW) && (Imax <= Req.maxCurrent)

                    Accepted.motornumber(m,1) = Motor.n(nn);
                    Accepted.Name(m,1) = Motor.manu(nm) + " " + Motor.model(nm);
                    Accepted.PropD(m,1) = Prop.D(np);
                    Accepted.DW(m,1) = DW;
                    Accepted.TW(m,1) = Tstatic/MTOW;
                    Accepted.I_max(m,1) = Imax;
                    Accepted.Throttle(m,1) = Throttle;
                    Accepted.I_cruise(m,1) = Icruise;
                    Accepted.RPM(m,1) = RPM;

                    m = m+1;
                end % IF T/W Ratio Check 
                
            end % IF nan Check 
            
        end % Motors
                           
    end % Props
    
end % No. of Motors
      

        [status,cmdout] = system("del" + " " + "Accepted_Motors_3.xlsx");
        Headers = {'No. of Props';'Prop Diameter (in)';'Motor Model';'Drive Weight (g)';'RPM';'Static Thrust-Weight Ratio';'Max Current (A)';'Cruise Throttle Setting %';'Cruise Current (A)'};
        Results = table(Accepted.motornumber,Accepted.PropD,Accepted.Name,round(Accepted.DW,0),Accepted.RPM,round(Accepted.TW,2),round(Accepted.I_max,1),round(Accepted.Throttle,0),round(Accepted.I_cruise,1),'VariableNames',Headers)
        writetable(Results,'Accepted_Motors_3.xlsx')

