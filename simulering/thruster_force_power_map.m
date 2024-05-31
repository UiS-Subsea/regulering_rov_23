clear all; close all;

tabledata = readtable("T200_data_16v.csv");

pwm = tabledata{:,1};
percent = (pwm - 1500)/4;
power = tabledata{:,5};
force = tabledata{:,6};
force = force .* 9.80665; % Convertere fra Kgf til newton
force_abs = abs(force);

figure;
hold on;
plot(force, power);
hold off;
xlabel("Kraft[N]")
ylabel("Effekt[W]")

figure;
% Få verdier for pwr(watt) gitt U(newton)
power2 = power(101:end);
positive = force(101:end);
negative = abs(flip(force(1:101)));
percent = 0:1:100;

% idxs_force_0 = find(positive == 0.0);
% positive(idxs_force_0) = [];
% negative(idxs_force_0) = [];
% power(idxs_force_0) = [];

% 51.4849, ... er endeverdi +-100% i kraft
kneg = 51.4849/39.9131;
negative = negative.*kneg;

hold on;
plot(positive, power2, "r");
plot(negative, power2, "b");
legend("Kraft$>$0", "Kraft$<$0")
hold off;
xlabel("$|Kraft[N]|$")
ylabel("Effekt[W]")
clf;

% use average as map
meanforce = (positive + negative)/2;

koeffisienter = polyfit(meanforce,power2,2);
f = polyval(koeffisienter,meanforce);
hold on;
plot(meanforce,f)
plot(meanforce,power2)
hold off;
xlabel("Kraft[N]");
ylabel("Effekt[W]");
legend("Original", "Poly2")
format longE;
kneg
format short;
fmt=['static const float thurster_map_force_to_percent[]={' repmat('%d,',1,numel(koeffisienter)-1) '%d}'];
c_code=sprintf(fmt,koeffisienter)
% Verifisering av pwr
mn35 = mp(-35,koeffisienter)% skal gi 300 ca
mp35 = mp(35,koeffisienter)% skal gi 200 ca


figure;
hold on;
xx = min(force):0.01:max(force);
yy = []
for i=1:numel(xx)
    yy(i) = mp(xx(i), koeffisienter);
end
plot(force,power);
plot(xx,yy);
xlim([min(force), max(force)]);
ylim([min(power), 400]);
xlabel("Kraft[N]");
ylabel("Effekt[W]");
legend("Original", "Approksimasjon");
print("T200_16V_kraft_til_effekt_apr_cmp",'-depsc2');


hold off;

function [prc] = mp(u,koeffisienter)
    prc = 0;
    isneg = u < 0;

    if isneg
        % if u > -0.449
        %     return;
        % end

        u = u * (51.4849/39.9131); % kneg
    else
        % if u < 0.449
        %     return;
        % end
    end

    u = abs(u);
    
    l = numel(koeffisienter) - 1;
    for i=0:l
        px = koeffisienter(i+1);
        prc = prc + px*u.^(l-i);
    end

    if prc < 0
        prc = 0;
    end
end