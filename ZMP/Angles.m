
l1 = 0.361     ; l2 = 0.3811;
x  = 0.15053; y = 0.6336;
xtf = x*ones(length(X_com),1);
xtb = x*ones(length(X_com),1);
num = 70 ; change = -1; delay = 39; cycle = 0; n = 0;

for i = 1: length(X_com)
if (i == 1)

else
    if(ZMP(i) - ZMP(i-1) > 0.2)
        change = change + 2    
        n = n+1;
    end

    if(ZMP(i) - ZMP(i-1) < -0.2)
        change = change - 2
        n = n+1;
    end
end

% back leg support
if (change == -1)
    if (i > 69)
        xtb(i) =  xtb(delay+n*num) + X_com(i) - X_com(delay+n*num);
    else
        xtb(i) =  x + X_com(i);
    end    
    [alpha3, alpha4] = inverse2R(xtb(i),y,l1,l2,0);
    theta3(i) =  alpha3 + alpha4;
    theta4(i) = -alpha4;
end

% front leg starting support
if (ZMP(i) == 0)
      xtf(i) =  -x + X_com(i);
     [alpha1, alpha2] = inverse2R(xtf(i),y,l1,l2,1);
     theta1(i) = 180 - (alpha1+alpha2);
     theta2(i) = alpha2;
end

% front swing leg
if (change == -1 & abs(ZMP(i)) > 0)
    r = stepLength/(2*pi);
    angle = 2*pi*(i-delay-n*num)/num ;
    
    xtf(i) = xtf(delay+n*num) - r*(angle - sin(angle)) + (X_com(i) - X_com(delay + n*num));
    ytf(i) = y - r*(1 - cos(angle));
    
     [alpha1, alpha2] = inverse2R(xtf(i),ytf(i),l2,l1,0);
     theta1(i) = 180-alpha1;
     theta2(i) = -alpha2;  
end
if (change == 1)
     % front leg support
      xtf(i) = xtf(delay+n*num) + X_com(i)- X_com(delay+n*num);
      [alpha1, alpha2] = inverse2R(xtf(i),y,l1,l2,1);
      theta1(i) = 180 - (alpha1+alpha2);
      theta2(i) = alpha2;
      
     % back leg swing
     r = stepLength/(2*pi);
     angle = 2*pi*(i-delay-n*num)/num ;
    
     xtb(i) = xtb(delay+n*num) - r*(angle - sin(angle)) + X_com(i) - X_com(delay + n*num);
     ytb(i) = y - r*(1 - cos(angle));
    
     [alpha3, alpha4] = inverse2R(xtb(i),ytb(i),l2,l1,1);
     theta3(i) = alpha3;
     theta4(i) = alpha4;  
end
end
% %  theta2 = deg2rad(theta2 +theta1 - 90) ;
% theta1 = deg2rad(90-theta1) ;
%  theta4 = deg2rad(theta3 + theta4 - 90);
% theta3 = deg2rad(90 - theta3);

% writematrix(theta1,'theta1.csv')
% writematrix(theta2,'theta2.csv')
% writematrix(theta3,'theta3.csv')
% writematrix(theta4,'theta4.csv')
% writematrix(X_com,'X_com.csv')

figure(2)
plot(theta1)
ylabel('theta1(deg)')
xlabel('time in 0.01sec')

figure(3)
plot(theta3)
ylabel('theta2(deg)')
xlabel('time in 0.01sec')

figure(4)
plot(theta1)
ylabel('theta3(deg)')
xlabel('time in 0.01sec')

figure(5)
plot(theta4)
ylabel('theta4(deg)')
xlabel('time in 0.01sec')





