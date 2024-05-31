if exist('f1')==0 || ~isvalid(f1)
    f1 = figure;
end
clf(f1);

tabledata = readtable("T200_data_16v.csv");

pwm = tabledata{:,1};
percent = (pwm - 1500)/4;
power = tabledata{:,5};
force = tabledata{:,6};
force = force .* 9.80665; % Convertere fra Kgf til newton
force_abs = abs(force);

% Få verdier for estimert kraft fra prosent pådrag
% hold on;
% koeffisienter = polyfit(percent,power,4);
% f = polyval(koeffisienter,percent);
% plot(percent,f,'-')
% plot(percent,power);
% xlabel("percent[%]");
% ylabel("power[W]");
% hold off;
% format longE;
% koeffisienter
% format short;
% fmt=['static const float thurster_map_percent_to_power[]={' repmat('%d,',1,numel(koeffisienter)-1) '%d}'];
% c_code=sprintf(fmt,koeffisienter)
% % Verifisering av pwr
% P(-80,koeffisienter)
% function [pwr] = P(u,koeffisienter)
%     pwr = 0;
%     l = numel(koeffisienter) - 1
%     for i=0:l
%         px = koeffisienter(i+1);
%         pwr = pwr + px*u^(l-i);
%     end
% end

% Få verdier for u(prosent) gitt U(newton)
%force = force(1:93)
%percent = percent(1:93);
% remove indicies where force == 0
idxs_force_0 = find(force == 0);
force(idxs_force_0) = [];
percent(idxs_force_0) = [];
% offset force
percent(percent > 0) = percent(percent > 0) - 8;
percent(percent < 0) = percent(percent < 0) + 8;
force(force > 0) = force(force > 0) - 0.392266;
force(force < 0) = force(force < 0) + 0.392266;


hold on;
koeffisienter = polyfit(force,percent,20);
f = polyval(koeffisienter,force);
plot(force,f,'-')
plot(force,percent);
xlabel("force[N]");
ylabel("percent[%]");
hold off;
format longE;
koeffisienter
format short;
fmt=['static const float thurster_map_force_to_percent_negative[]={' repmat('%d,',1,numel(koeffisienter)-1) '%d}\nforce<%.2f'];
c_code=sprintf(fmt,koeffisienter,force(1))
% Verifisering av pwr
P(-80,koeffisienter)
function [pwr] = P(u,koeffisienter)
    pwr = 0;
    l = numel(koeffisienter) - 1
    for i=0:l
        px = koeffisienter(i+1);
        pwr = pwr + px*u^(l-i);
    end
end

% 
% subplot(2,2,1);
% plot(percent,power);
% ylabel("power[W]");
% 
% subplot(2,2,3);
% plot(percent,force);
% ylabel("force[N]");
% xlabel("percent %");
% 
% 
% subplot(2,2,2);
% plot(force,power);
% xlabel("force[N]");
% ylabel("power[W]");
% 
% subplot(2,2,4);
