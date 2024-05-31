clear all; close all;

tabledata = readtable("T200_data_16v.csv");

pwm = tabledata{:,1};
percent = (pwm - 1500)/4;
power = tabledata{:,5};
force = tabledata{:,6};
force = force .* 9.80665; % Convertere fra Kgf til newton
force_abs = abs(force);

% lagre figurer som .eps?
bprinteps = false;

figure;
plot(force, percent);
xlabel("Kraft[N]");
ylabel("Prosent[$\%$]");
if bprinteps
    print("T200_16V_kraft_til_prosent_org",'-depsc2');
end

figure;
koeffisienter = polyfit(force,percent,30);
[P, S] = polyfit(force,percent,30);
R_squared_p30 = 1 - (S.normr/norm(percent - mean(percent)))^2;
f = polyval(koeffisienter,force);
hold on;
xlabel("Kraft[N]");
ylabel("Prosent[$\%$]");
plot(force, percent);
plot(force, f);
legend("Original", "Poly30",Location="northwest")
if bprinteps
   print("T200_16V_kraft_til_prosent_poly",'-depsc2');
end

% Få verdier for u(prosent) gitt U(newton)
percent2 = 0:100;
positive = force(101:end);
negative = abs(flip(force(1:101)));

figure;
hold on;
plot(positive, percent2, "r");
plot(negative, percent2, "b");
posx = positive(end);
negx = negative(end);
plot([posx,posx],[0,100],Color=[0.25, 0.25, 0.25], LineStyle="--");
plot([negx,negx],[0,100],Color=[0.25, 0.25, 0.25], LineStyle="--");
hold off;
xlabel("Kraft[N]");
ylabel("Prosent[$\%$]");
xlim([0 60]);
ylim([0 100]);
legend("Kraft $>$ 0", "Kraft $<$ 0",Location="northwest");
if bprinteps
   print("T200_16V_kraft_til_prosent_abs",'-depsc2');
end


idxs_force_0 = find(positive == 0);
positive(idxs_force_0) = [];
negative(idxs_force_0) = [];
percent2(idxs_force_0) = [];

% 51.4849, ... er endeverdi +-100% i kraft
kneg = 51.4849/39.9131;
scalednegative = negative.*kneg;
meanforce = (positive + scalednegative)/2;

figure;
hold on;
plot(positive, percent2, "r");
plot(scalednegative, percent2, "b");
posx = positive(end);
negx = scalednegative(end);
hold off;
xlabel("Kraft[N]");
ylabel("Prosent[$\%$]");
legend("Kraft $>$ 0", "Kraft $<$ 0",Location="northwest");
if bprinteps
   print("T200_16V_kraft_til_prosent_abs_scaled",'-depsc2');
end

figure;
hold on;
koeffisienter = polyfit(meanforce,percent2,3);
[P, S] = polyfit(meanforce,percent2,3);
R_squared_p3 = 1 - (S.normr/norm(percent2 - mean(percent2)))^2;
f = polyval(koeffisienter,force);
f = polyval(koeffisienter,meanforce);
plot(meanforce,f)
plot(meanforce,percent2);
xlabel("Kraft[N]");
ylabel("Prosent[$\%$]");
hold off;
legend("Original", "Poly3",Location="northwest")
if bprinteps
   print("T200_16V_kraft_til_prosent_apr",'-depsc2');
end

% Plotting coefficients
format longE;
kneg
format short;
fmt=['static const float thurster_map_force_to_percent[]={' repmat('%d,',1,numel(koeffisienter)-1) '%d}'];
c_code=sprintf(fmt,koeffisienter)
% Verifisering av pwr
mn35 = mp(-35,koeffisienter)% skal gi -90 ca
mp35 = mp(35,koeffisienter)% skal gi 77 ca
mn = mp(-0.01, koeffisienter)


figure;
hold on;
xx = min(force):0.01:max(force);
yy = []
for i=1:numel(xx)
    yy(i) = mp(xx(i), koeffisienter);
end
plot(force, percent);
plot(xx,yy);
hold off;
xlim([min(force), max(force)]);
ylim([-100, 100]);
legend("Original", "Approksimasjon", Location="northwest")
xlabel("Kraft[N]");
ylabel("Prosent[\%]");
if bprinteps
    print("T200_16V_kraft_til_prosent_apr_cmp",'-depsc2');
end

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
    
    l = numel(koeffisienter) - 1
    for i=0:l
        px = koeffisienter(i+1);
        prc = prc + px*u^(l-i);
    end

    if isneg
        prc = -prc;
    end

end