motor_calculations;


variables = ["Fx" "Fy" "Fz" "Tx" "Ty" "Tz"];
for i=1:6
    Uformax = (B(:,i) < 0) * umin + (B(:,i) > 0) * umax;
    Uformin = (B(:,i) < 0) * umax + (B(:,i) > 0) * umin;
    
    Fmin = Uformin' * B(:,i);
    Fmax = Uformax' * B(:,i);
    
    %fmt = '%s [%.1f, %.1f]\n'; fprintf(fmt, variables(i),Fmin, Fmax);
    fmt = '%.1f, %.1f\n'; fprintf(fmt, Fmin, Fmax);
end