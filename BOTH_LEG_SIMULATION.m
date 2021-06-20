clc
clear all

M = 1.40983/2;      %  Mass of base link
mT = 0.23236;     %  Mass of tibia
mF = 0.49995;     %  Mass of femur
m = 0.10898;      %  Mass of foot

Ix = 0.00475204;   %  Inertia of base link about x axis
IxF = 0.0080336;   %  Inertia of femur about x axis
IxT = 0.0044393;   %  Inertia of tibia about x axis

lT = 0.39857;      %  Length of tibia
lF = 0.39152;      %  Length of femur

d = 0.1;           %  Seperation at the hip joint
h = 0.0558;        %  height of base link COM from hip joint
g = 9.84;          %  acceleration due to gravity

%% JOINT CONTROLLER PARAMETERS

Kp2 = 300 ;
Kd2 = 20 ;
Ki2 = 0.25 ;
errsum2 = 0 ;
maxlimit2 = 10000 ;
minlimit2 = 10000 ;

Kp4 = 400 ;
Kd4 = 30 ;
Ki4 = 0.25 ;
errsum4 = 0;
maxlimit4 = 10000 ;
minlimit4 = 10000 ;

Kp1 = 300 ;
Kd1 = 20 ;
Ki1 = 0.25 ;
errsum1 = 0 ;
maxlimit1 = 10000 ;
minlimit1 = 10000 ;

Kp3 = 400 ;
Kd3 = 30 ;
Ki3 = 0.25 ;
errsum3 = 0;
maxlimit3 = 10000 ;
minlimit3 = 10000 ;
             
%% ODE SOLVER

tstep = 0.0001;
tfinal = 40;
Duration = 0:tstep:tfinal;
iter = size(Duration,2);

th1desired = -xlsread('C:\Users\Home\Desktop\Robot Walking\theta1')*pi/180;
th2desired = xlsread('C:\Users\Home\Desktop\Robot Walking\theta3')*pi/180;
th3desired = -xlsread('C:\Users\Home\Desktop\Robot Walking\theta2')*pi/180;
th4desired = xlsread('C:\Users\Home\Desktop\Robot Walking\theta4')*pi/180;

th1d = -(pi/6)*ones(iter,1);   
th2d = (pi/6)*ones(iter,1);      
th3d = -(pi/4)*ones(iter,1);    
th4d = (pi/4)*ones(iter,1);

dth1d = -(pi/6)*zeros(iter,1);   
dth2d = (pi/6)*zeros(iter,1);      
dth3d = -(pi/4)*zeros(iter,1);    
dth4d = (pi/4)*zeros(iter,1);

count = 1;
for n = 1:1:iter
    
    if( n >= 0 && n < iter/4 )
        
        th1d(n,1) = -(pi/4);   
        th2d(n,1) = (pi/4);      
        th3d(n,1) = -(pi/4);    
        th4d(n,1) = (pi/4);
        
    elseif( n >= iter/4 && n < iter/2 )
        
        th1d(n,1) = -(pi/12) + ((th1desired(1,1) + pi/12)/(iter*tstep/4))*(n-iter/4)*tstep;    
        th2d(n,1) = (pi/12) + ((th2desired(1,1) - pi/12)/(iter*tstep/4))*(n-iter/4)*tstep;      
        th3d(n,1) = -(pi/12) + ((th3desired(1,1) + pi/12)/(iter*tstep/4))*(n-iter/4)*tstep;     
        th4d(n,1) = (pi/12) + ((th4desired(1,1) - pi/12)/(iter*tstep/4))*(n-iter/4)*tstep; 
        
    else
        
        if (rem(n-floor(iter/2),size(th1desired,2)) == 0)
           th1d(n,1) = th1desired(1,count) ;
           th2d(n,1) = th2desired(1,count) ;
           th3d(n,1) = th3desired(1,count) ;
           th4d(n,1) = th4desired(1,count) ;
           count = count + 1;
        else
           th1d(n,1) = th1d(n-1,1); 
           th2d(n,1) = th2d(n-1,1); 
           th3d(n,1) = th3d(n-1,1); 
           th4d(n,1) = th4d(n-1,1); 
        end
    end
end

phi = zeros(iter,1);    phi(1,1) = 0 ;        
th1 = zeros(iter,1);    th1(1,1) = -pi/6;
th2 = zeros(iter,1);    th2(1,1) =  pi/6;   
th3 = zeros(iter,1);    th3(1,1) = - pi/6;
th4 = zeros(iter,1);    th4(1,1) = pi/6;
yCOM = zeros(iter,1);   yCOM(1,1) = 0; 
zCOM = zeros(iter,1);   zCOM(1,1) = 0.9;
yEL = zeros(iter,1);    yEL(1,1) = yCOM(1,1) - 0.5*d*cos(phi(1,1)) + h*sin(phi(1,1)) ;
zEL = zeros(iter,1);    zEL(1,1) = zCOM(1,1) - 0.5*d*sin(phi(1,1)) - h*cos(phi(1,1)) ;
yER = zeros(iter,1);    yER(1,1) = yCOM(1,1) + 0.5*d*cos(phi(1,1)) + h*sin(phi(1,1)) ;
zER = zeros(iter,1);    zER(1,1) = zCOM(1,1) + 0.5*d*sin(phi(1,1)) - h*cos(phi(1,1)) ;
yL = zeros(iter,1);     yL(1,1) = yEL(1,1) - lT*cos(phi(1,1) + th2(1,1) + th4(1,1)) - lF*cos(phi(1,1) + th2(1,1)) ;
zL = zeros(iter,1);     zL(1,1) = zEL(1,1) - lT*sin(phi(1,1) + th2(1,1) + th4(1,1)) - lF*sin(phi(1,1) + th2(1,1)) ;
yR = zeros(iter,1);     yR(1,1) = yER(1,1) + lT*cos(phi(1,1) + th1(1,1) + th3(1,1)) + lF*cos(phi(1,1) + th1(1,1)) ;
zR = zeros(iter,1);     zR(1,1) = zER(1,1) + lT*sin(phi(1,1) + th1(1,1) + th3(1,1)) + lF*sin(phi(1,1) + th1(1,1)) ;

dphi = zeros(iter,1);   ddphi = zeros(iter,1);
dth1 = zeros(iter,1);   ddth1 = zeros(iter,1);
dth2 = zeros(iter,1);   ddth2 = zeros(iter,1);
dth3 = zeros(iter,1);   ddth3 = zeros(iter,1);
dth4 = zeros(iter,1);   ddth4 = zeros(iter,1);
dyCOM = zeros(iter,1);  ddyCOM = zeros(iter,1);
dzCOM = zeros(iter,1);  ddzCOM = zeros(iter,1);
dyEL = zeros(iter,1);   ddyEL = zeros(iter,1);
dzEL = zeros(iter,1);   ddzEL = zeros(iter,1);
dyER = zeros(iter,1);   ddyER = zeros(iter,1);
dzER = zeros(iter,1);   ddzER = zeros(iter,1);
dyL = zeros(iter,1);    ddyL = zeros(iter,1);
dzL = zeros(iter,1);    ddzL = zeros(iter,1);
dyR = zeros(iter,1);    ddyR = zeros(iter,1);
dzR = zeros(iter,1);    ddzR = zeros(iter,1);

s = [ th2(1,1); th4(1,1); yL(1,1); zL(1,1); phi(1,1); dth2(1,1); dth4(1,1); dyL(1,1); dzL(1,1); dphi(1,1);...
      th1(1,1); th3(1,1); yR(1,1); zR(1,1); dth1(1,1); dth3(1,1); dyR(1,1); dzR(1,1) ] ;

for n = 1:1:iter
 
    th2(n,1) = s(1); th4(n,1) = s(2); yL(n,1) = s(3); zL(n,1) = s(4);           phi(n,1) = s(5);
    dth2(n,1) = s(6); dth4(n,1) = s(7); dyL(n,1) = s(8); dzL(n,1) = s(9);         dphi(n,1) = s(10);
    th1(n,1) = s(11); th3(n,1) = s(12); yR(n,1) = s(13); zR(n,1) = s(14);
    dth1(n,1) = s(15); dth3(n,1) = s(16); dyR(n,1) = s(17); dzR(n,1) = s(18);
    
    %% LEFT LEG MASS MATRIX

    D11 = IxF + IxT + (M + 0.25*mF)*lF^2 + (M + mF + 0.25*mT)*lT^2 + 2*(M + 0.5*mF)*cos(th4(n,1)) ;
    D12 = IxT + (M + mF + 0.25*mT)*lT^2 + (M + 0.5*mF)*lF*lT*cos(th4(n,1)) ;
    D13 = - (M + 0.5*mF)*lF*sin(phi(n,1) + th2(n,1)) - (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) ;
    D14 = (M + 0.5*mF)*lF*cos(phi(n,1) + th2(n,1)) + (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) ;

    C11 = 0 ;
    C12 = - 2*(M + 0.5*mF)*(dphi(n,1) + dth2(n,1) + 0.5*dth4(n,1))*lT*lF*sin(th4(n,1)) ;
    C13 = 0 ;
    C14 = 0 ;

    G1 = ( (M + 0.5*mF)*lF*cos(phi(n,1) + th2(n,1)) + (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) )*g ...
         + ddphi(n,1)*( IxF + IxT + (M + 0.25*mF)*lF^2 + (M + mF + 0.25*mT)*lT^2 + 2*(M + 0.5*mF)*lF*lT*cos(th4(n,1)) ) ;

    D21 = IxT + (M + mF + 0.25*mT)*lT^2 + (M + 0.5*mF)*lF*lT*cos(th4(n,1)) ;
    D22 = IxT + (M + mF + 0.25*mT)*lT^2 ;
    D23 = - (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) ;
    D24 = (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) ;

    C21 = (M + 0.5*mF)*lF*lT*sin(th4(n,1))*(dphi(n,1) + dth2(n,1)) ;
    C22 = 0 ;
    C23 = 0 ;
    C24 = 0 ;

    G2 = ( (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) )*g ...
          + ddphi(n,1)*( IxT + (M + mF + 0.25*mT)*lT^2 + (M + 0.5*mF)*lF*lT*cos(th4(n,1)) ) ...
          + dphi(n,1)^2*( (M + 0.5*mF)*lF*lT*sin(th4(n,1)) );

    D31 = -(M + mF + 0.5*mT)*lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) - (M + 0.5*mF)*lF*sin(phi(n,1) + th2(n,1)) ;
    D32 = -(M + mF + 0.5*mT)*lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) ;
    D33 = M + m + mT + mF ;
    D34 = 0 ;

    C31 = -(M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1)) - (M + 0.5*mF)*lF*cos(phi(n,1) + th2(n,1))*(dphi(n,1) + dth2(n,1)) ; 
    C32 = -(M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1)) ;
    C33 = 0 ;
    C34 = 0 ;

    G3 = 0 ...
         + ddphi(n,1)*( -(M + mF + 0.5*mT)*lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) - (M + 0.5*mF)*lF*sin(phi(n,1) + th2(n,1)) ) ...
         + dphi(n,1)*( -(M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1)) - (M + 0.5*mF)*lF*cos(phi(n,1) + th2(n,1))*(dphi(n,1) + dth2(n,1)) ) ;

    D41 = (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) + (M + 0.5*mF)*lF*cos(phi(n,1) + th2(n,1)) ;
    D42 = (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) ;
    D43 = 0 ;
    D44 = M + m + mT + mF ;

    C41 = -(M + mF + 0.5*mT)*lT*sin(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1)) - (M + 0.5*mF)*lF*sin(phi(n,1) + th2(n,1))*(dphi(n,1) + dth2(n,1)) ; 
    C42 = -(M + mF + 0.5*mT)*lT*sin(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1)) ;
    C43 = 0 ;
    C44 = 0 ;

    G4 = (M + m + mT + mF)*g ...
         + ddphi(n,1)*( (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) + (M + 0.5*mF)*lF*cos(phi(n,1) + th2(n,1)) )...
         + dphi(n,1)*( -(M + mF + 0.5*mT)*lT*sin(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1)) - (M + 0.5*mF)*lF*sin(phi(n,1) + th2(n,1))*(dphi(n,1) + dth2(n,1)) ) ;

    D_left = [ D11 D12 D13 D14 ; ...
               D21 D22 D23 D24 ; ...
               D31 D32 D33 D34 ; ...
               D41 D42 D43 D44 ] ;

    C_left = [ C11 C12 C13 C14 ; ...
               C21 C22 C23 C24 ; ...
               C31 C32 C33 C34 ; ...
               C41 C42 C43 C44 ] ;

    G_left =  [ G1 ; G2 ; G3 ; G4 ] ; 

   
    %% RIGHT LEG MASS MATRIX
    
    D11 = IxF + IxT + (M + 0.25*mF)*lF^2 + (M + mF + 0.25*mT)*lT^2 + 2*(M + 0.5*mF)*lF*lT*cos(th3(n,1)) ;
    D12 = IxT + (M + mF + 0.25*mT)*lT^2 + (M+ 0.5*mF)*lF*lT*cos(th3(n,1)) ;
    D13 = (M + 0.5*mF)*lF*sin(phi(n,1) + th1(n,1)) + (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) ;
    D14 = - (M + 0.5*mF)*lF*cos(phi(n,1) + th1(n,1)) - (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) ;

    C11 = 0 ;
    C12 = - 2*(M + 0.5*mF)*(dphi(n,1) + dth1(n,1) + 0.5*dth3(n,1))*lT*lF*sin(th3(n,1)) ;
    C13 = 0 ;
    C14 = 0 ;

    G1 = ( - (M + 0.5*mF)*lF*cos(phi(n,1) + th1(n,1)) - (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) )*g ...
         + ddphi(n,1)*( IxF + IxT + (M + 0.25*mF)*lF^2 + (M + mF + 0.25*mT)*lT^2 + 2*(M + 0.5*mF)*lF*lT*cos(th3(n,1)) ) ;

    D21 = IxT + (M + mF + 0.25*mT)*lT^2 + (M+ 0.5*mF)*lF*lT*cos(th3(n,1)) ;
    D22 = IxT + (M + mF + 0.25*mT)*lT^2 ;
    D23 = (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) ;
    D24 = - (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) ;

    C21 = 2*(M + 0.5*mF)*lF*lT*sin(th3(n,1))*(dphi(n,1) + 0.5*dth1(n,1)) ;
    C22 = 0 ;
    C23 = 0 ;
    C24 = 0 ;

    G2 = ( - (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) )*g ...
          + ddphi(n,1)*( IxT + (M + mF + 0.25*mT)*lT^2 + (M + 0.5*mF)*lF*lT*cos(th3(n,1)) ) ...
          + dphi(n,1)^2*( (M + 0.5*mF)*lF*lT*sin(th3(n,1)) );

    D31 = (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) + (M + 0.5*mF)*lF*sin(phi(n,1) + th1(n,1)) ;
    D32 = (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) ;
    D33 = M + m + mT + mF ;
    D34 = 0 ;

    C31 = (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1)) + (M + 0.5*mF)*lF*cos(phi(n,1) + th1(n,1))*(dphi(n,1) + dth1(n,1)) ; 
    C32 = (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1)) ;
    C33 = 0 ;
    C34 = 0 ;

    G3 = 0 ...
         + ddphi(n,1)*( (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) + (M + 0.5*mF)*lF*sin(phi(n,1) + th1(n,1)) ) ...
         + dphi(n,1)*( (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1)) + (M + 0.5*mF)*lF*cos(phi(n,1) + th1(n,1))*(dphi(n,1) + dth1(n,1)) ) ;

    D41 = - (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) - (M + 0.5*mF)*lF*cos(phi(n,1) + th1(n,1)) ;
    D42 = - (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) ;
    D43 = 0 ;
    D44 = M + m + mT + mF ;

    C41 = (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1)) + (M + 0.5*mF)*lF*sin(phi(n,1) + th1(n,1))*(dphi(n,1) + dth1(n,1)) ; 
    C42 = (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1)) ;
    C43 = 0 ;
    C44 = 0 ;

    G4 = (M + m + mT + mF)*g ...
         + ddphi(n,1)*( - (M + mF + 0.5*mT)*lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) - (M + 0.5*mF)*lF*cos(phi(n,1) + th1(n,1)) )...
         + dphi(n,1)*( (M + mF + 0.5*mT)*lT*sin(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1)) + (M + 0.5*mF)*lF*sin(phi(n,1) + th1(n,1))*(dphi(n,1) + dth1(n,1)) ) ;

    D_right = [ D11 D12 D13 D14 ; ...
                D21 D22 D23 D24 ; ...
                D31 D32 D33 D34 ; ...
                D41 D42 D43 D44 ] ;

    C_right = [ C11 C12 C13 C14 ; ...
                C21 C22 C23 C24 ; ...
                C31 C32 C33 C34 ; ...
                C41 C42 C43 C44 ];

    G_right = [ G1 ; G2 ; G3 ; G4 ] ; 

    %% REACTION TORQUE
    
    JL = [ -( lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) + lF*sin(phi(n,1) + th2(n,1)) ) ,  - lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) ;...
            ( lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) + lF*cos(phi(n,1) + th2(n,1)) ) ,    lT*cos(phi(n,1) + th2(n,1) + th4(n,1))  ] ;

    ddth2s = ddyEL(n,1) - ddyL(n,1) ...
            + ( lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) + lF*sin(phi(n,1) + th2(n,1)) )*ddphi(n,1) ...
            + lT*cos(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1))^2  ...
            + lT*cos(phi(n,1) + th2(n,1))*(dphi(n,1) + dth2(n,1))^2 ;

    ddth4s = ddzEL(n,1) - ddzL(n,1) ...
            - ( lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) + lF*cos(phi(n,1) + th2(n,1)) )*ddphi(n,1) ...
            + lT*sin(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1))^2  ...
            + lT*sin(phi(n,1) + th2(n,1))*(dphi(n,1) + dth2(n,1))^2 ;

    TBL =  JL\[ddth2s ; ddth4s] ;
    %disp(TBL);
    
    JR = [   ( lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) + lF*sin(phi(n,1) + th1(n,1)) ) ,    lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) ;...
           - ( lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) + lF*cos(phi(n,1) + th1(n,1)) ) ,  - lT*cos(phi(n,1) + th1(n,1) + th3(n,1))  ] ;
    
    ddth1s = ddyER(n,1) - ddyR(n,1) ...
            - ( lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) + lF*sin(phi(n,1) + th1(n,1)) )*ddphi(n,1) ...
            - lT*cos(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1))^2  ...
            - lT*cos(phi(n,1) + th1(n,1))*(dphi(n,1) + dth1(n,1))^2 ;

    ddth3s = ddzER(n,1) - ddzR(n,1) ...
            + ( lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) + lF*cos(phi(n,1) + th1(n,1)) )*ddphi(n,1) ...
            - lT*sin(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1))^2  ...
            - lT*sin(phi(n,1) + th1(n,1))*(dphi(n,1) + dth1(n,1))^2 ;

    TBR = ( JR\[ddth1s ; ddth3s] );
    %disp(TBR)
  
   %% NORMAL REACTION FRICTION FORCE BALANCE

    errsum2 = max( ( min( (errsum2 + ( th2d(n,1) - th2(n,1)) ), maxlimit2 ) ) , minlimit2 );
    errsum4 = max( ( min( (errsum4 + ( th4d(n,1) - th4(n,1)) ), maxlimit4 ) ) , minlimit4 );
    errsum1 = max( ( min( (errsum1 + ( th1d(n,1) - th1(n,1)) ), maxlimit1 ) ) , minlimit1 );
    errsum3 = max( ( min( (errsum3 + ( th3d(n,1) - th3(n,1)) ), maxlimit3 ) ) , minlimit3 );
    
    if( n < iter/4)
        T2 = 0;
        T4 = 0;
        T1 = 0;
        T3 = 0;
    else
        T2 =  Kp2*( th2d(n,1) - th2(n,1) ) + Kd2*( dth2d(n,1) - dth2(n,1) ) + Ki2*errsum2*tstep ;
        T4 =  Kp4*( th4d(n,1) - th4(n,1) ) + Kd4*( dth4d(n,1) - dth4(n,1) ) + Ki4*errsum4*tstep ;
        T1 =  Kp1*( th1d(n,1) - th1(n,1) ) + Kd1*( dth1d(n,1) - dth1(n,1) ) + Ki1*errsum1*tstep ;
        T3 =  Kp3*( th3d(n,1) - th3(n,1) ) + Kd4*( dth3d(n,1) - dth3(n,1) ) + Ki3*errsum3*tstep ;
    end
    
    if ( th2(n,1) <= pi/12 )   
        dth2(n,1) = max(0, dth2(n,1)) ;
        Tth2 = max(0, T2 - IxF*TBL(1)) ;    
    elseif ( th2(n,1) >= 5*pi/12 )    
        dth2(n,1) = min(0, dth2(n,1));
        Tth2 = min(0, T2 - IxF*TBL(1)) ;   
    else
        dth2(n,1) = s(6);
        Tth2 = T2 - IxF*TBL(1) ; 
    end

    if ( th4(n,1) <= pi/12 )   
        dth4(n,1) = max(0, dth4(n,1)) ;
        Tth4 = max(0, T4 - IxT*TBL(2)) ;    
    elseif ( th4(n,1) >= 5*pi/12 )    
        dth4(n,1) = min(0, dth4(n,1));
        Tth4 = min(0, T4 - IxT*TBL(2)) ;   
    else
        dth4(n,1) = s(7);
        Tth4 = T4 - IxT*TBL(2) ; 
    end
    
    if ( th1(n,1) >= -pi/12 )   
        dth1(n,1) = min(0, dth1(n,1)) ;
        Tth1 = min(0, T1 - IxF*TBR(1)) ;    
    elseif ( th1(n,1) <= - 5*pi/12 )    
        dth1(n,1) = max(0, dth1(n,1));
        Tth1 = max(0, T1 - IxF*TBR(1)) ;   
    else
        dth1(n,1) = s(15);
        Tth1 = T1 - IxF*TBR(1) ; 
    end

    if ( th3(n,1) >= - pi/12 )   
        dth3(n,1) = min(0, dth3(n,1)) ;
        Tth3 = min(0, T3 - IxT*TBR(2)) ;    
    elseif ( th3(n,1) <= - 5*pi/12 )    
        dth3(n,1) = max(0, dth3(n,1));
        Tth3 = max(0, T3 - IxT*TBR(2)) ;   
    else
        dth3(n,1) = s(16);
        Tth3 = T3 - IxT*TBR(2) ; 
    end
    
    if( zL(n,1) <= 0 )
        if (dzL(n,1) < 0)
            NL = (M + mT + mF + m)*g + m*(-2*dzL(n,1) - dzL(n,1))/tstep;
            dzL(n,1) = zL(n,1)*dzL(n,1);
            dyL(n,1) = 0;
        else
            dzL(n,1) = 0;
            NL = (M + mT + mF + m)*g ;
        end
         FkL = - m*ddyL(n,1) + m*(-2*dyL(n,1) - dyL(n,1))/tstep ;
    else       
        FkL = 0 ;
        NL = 0 ;
    end
    
    if( zR(n,1) <= 0 )  
        if (dzR(n,1) < 0)
            NR = (M + mT + mF + m)*g + m*(-2*dzR(n,1) - dzR(n,1))/tstep;
            dzR(n,1) = zR(n,1)*dzR(n,1);
            dyR(n,1) = 0;
        else
            dzR(n,1) = 0;
            NR = (M + mT + mF + m)*g ;
        end
         FkR = - m*ddyR(n,1) + m*(-2*dyR(n,1) - dyR(n,1))/tstep ;
    else       
        FkR = 0 ;
        NR = 0 ;
    end
       
    dq = [dth2(n,1); dth4(n,1); dyL(n,1); dzL(n,1)] ;
    T_left = [ Tth2 ; Tth4 ; FkL ; NL  ]; 

    ddq = (D_left)\(T_left - C_left*dq - G_left);
    ddth2(n,1) = ddq(1);
    ddth4(n,1) = ddq(2);
    ddyL(n,1) = ddq(3);
    ddzL(n,1) = ddq(4);
    
    dp = [dth1(n,1); dth3(n,1); dyR(n,1); dzR(n,1)] ;
    T_right = [ Tth1 ; Tth3 ; FkR ; NR  ];
    
    ddp = (D_right)\(T_right - C_right*dp - G_right);
    ddth1(n,1) = ddp(1);
    ddth3(n,1) = ddp(2);
    ddyR(n,1) = ddp(3);
    ddzR(n,1) = ddp(4);
    
%% CENTER OF MASS DYNAMICS
  
    ddyEL(n,1) = ddyL(n,1) ...
            - ( lT*sin(phi(n,1) + th2(n,1) + th4(n,1)) + lF*sin(phi(n,1) + th2(n,1)) )*(ddphi(n,1) + ddth2(n,1)) ...
            - lT*sin(phi(n,1) + th2(n,1) + th4(n,1))*ddth4(n,1) ...
            - lT*cos(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1))^2  ...
            - lT*cos(phi(n,1) + th2(n,1))*(dphi(n,1) + dth2(n,1))^2 ;

    ddzEL(n,1) = ddzL(n,1) ...
            + ( lT*cos(phi(n,1) + th2(n,1) + th4(n,1)) + lF*cos(phi(n,1) + th2(n,1)) )*(ddphi(n,1) + ddth2(n,1)) ...
            + lT*cos(phi(n,1) + th2(n,1) + th4(n,1))*ddth4(n,1) ...
            - lT*sin(phi(n,1) + th2(n,1) + th4(n,1))*(dphi(n,1) + dth2(n,1) + dth4(n,1))^2  ...
            - lT*sin(phi(n,1) + th2(n,1))*(dphi(n,1) + dth2(n,1))^2 ;
        
    ddyER(n,1) = ddyR(n,1) ...
            + ( lT*sin(phi(n,1) + th1(n,1) + th3(n,1)) + lF*sin(phi(n,1) + th1(n,1)) )*(ddphi(n,1) + ddth1(n,1)) ...
            + lT*sin(phi(n,1) + th1(n,1) + th3(n,1))*ddth3(n,1) ...
            + lT*cos(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1))^2  ...
            + lT*cos(phi(n,1) + th1(n,1))*(dphi(n,1) + dth1(n,1))^2 ;

    ddzER(n,1) = ddzR(n,1) ...
            - ( lT*cos(phi(n,1) + th1(n,1) + th3(n,1)) + lF*cos(phi(n,1) + th1(n,1)) )*(ddphi(n,1) + ddth3(n,1)) ...
            - lT*cos(phi(n,1) + th1(n,1) + th3(n,1))*ddth3(n,1) ...
            + lT*sin(phi(n,1) + th1(n,1) + th3(n,1))*(dphi(n,1) + dth1(n,1) + dth3(n,1))^2  ...
            + lT*sin(phi(n,1) + th1(n,1))*(dphi(n,1) + dth1(n,1))^2 ;

    ddyCOM(n,1) = 0.5*ddyEL(n,1) + 0.5*ddyER(n,1) ;
    ddzCOM(n,1) = 0.5*ddzEL(n,1) + 0.5*ddzER(n,1) ;
    ddphi(n,1) = (ddyEL(n,1)*sin(phi(n,1)) - ddzEL(n,1)*cos(phi(n,1))) - (ddyER(n,1)*sin(phi(n,1)) - ddzER(n,1)*cos(phi(n,1))) ; 
    ddphi(n,1) = 0 ;
    dphi(n,1) = 0 ;
    ddyEL(n,1) = ddyCOM(n,1) + 0.5*d*ddphi(n,1)*sin(phi(n,1)) ;
    ddzEL(n,1) = ddzCOM(n,1) - 0.5*d*ddphi(n,1)*cos(phi(n,1)) ;
    ddyER(n,1) = ddyCOM(n,1) - 0.5*d*ddphi(n,1)*sin(phi(n,1)) ;
    ddzER(n,1) = ddzCOM(n,1) + 0.5*d*ddphi(n,1)*cos(phi(n,1)) ;

%% SYSTEM STATE SPACE EULER RUNGE KUTTA SOLVER

    s0 = [ th2(n,1); th4(n,1); yL(n,1); zL(n,1); phi(n,1); dth2(n,1); dth4(n,1); dyL(n,1); dzL(n,1); dphi(n,1); th1(n,1); th3(n,1); yR(n,1); zR(n,1); dth1(n,1); dth3(n,1); dyR(n,1); dzR(n,1) ] ;
    sdot = [ dth2(n,1); dth4(n,1); dyL(n,1); dzL(n,1); dphi(n,1); ddth2(n,1); ddth4(n,1); ddyL(n,1); ddzL(n,1); ddphi(n,1); dth1(n,1); dth3(n,1); dyR(n,1); dzR(n,1); ddth1(n,1); ddth3(n,1); ddyR(n,1); ddzR(n,1) ] ; 
        
    s = s0 + sdot*tstep ; 
    
end

%%

Timestamp = datetime(now,'ConvertFrom','datenum');
foldername = ['C:\Users\Home\Desktop\Robot Walking\',strrep(char(Timestamp),':','-')]; 
RobotVideo = VideoWriter([foldername,'Walking','.avi']);
RobotVideo.FrameRate = 10; %can adjust this 5 - 10 works well
open(RobotVideo);

yEL = yL + lT*cos(phi + th2 + th4) + lF*cos(phi + th2) ;
zEL = zL + lT*sin(phi + th2 + th4) + lF*sin(phi + th2) ;
yCOM = yEL + 0.5*d*cos(phi) - h*sin(phi) ;
zCOM = zEL + 0.5*d*sin(phi) + h*cos(phi) ;

mode = 1;
figure(mode)
for i = 1:200:size(Duration,2)
    Rphi = [ 1 0 0 ; 0 cos(phi(i,1)) -sin(phi(i,1)) ; 0 sin(phi(i,1)) cos(phi(i,1)) ] ;
    NABIRoS_model(mode,[0; yCOM(i,1); zCOM(i,1)],Rphi,- th1(i,1),th2(i,1),-th3(i,1),th4(i,1))
        text(0,0.5,['Time :',num2str((i-1)*tstep),' sec'])
    frame = getframe(gcf);
    writeVideo(RobotVideo, frame);
    delete(gcf);
end

close(RobotVideo)

%%

figure(2)
subplot(2,2,1)
plot((1:100:size(zL,1))*tstep,zL(1:100:size(zL,1)),'linewidth',2)
grid on
legend('Left Feet Z','Fontsize',12,'location','best')
subplot(2,2,2)
plot((1:100:size(zL,1))*tstep,yL(1:100:size(zL,1)),'linewidth',2)
grid on
legend('Left Feet Y','Fontsize',12,'linewidth',2,'location','best')
subplot(2,2,3)
plot((1:100:size(zL,1))*tstep,th2(1:100:size(zL,1))*180/pi,(1:100:size(zL,1))*tstep,th2d(1:100:size(zL,1))*180/pi,'linewidth',2)
grid on
legend('Hip joint angle theta2','desired','Fontsize',12,'linewidth',2,'location','best')
subplot(2,2,4)
plot((1:100:size(zL,1))*tstep,th4(1:100:size(zL,1))*180/pi,(1:100:size(zL,1))*tstep,th4d(1:100:size(zL,1))*180/pi,'linewidth',2)
grid on
legend('knee joint angle theta4','desired','Fontsize',12,'linewidth',2,'location','best')
set(gcf, 'Position', get(0, 'Screensize'));
print(figure(2),[foldername,'Left Position.png'],'-dpng','-r300');

figure(3)
subplot(2,2,1)
plot((1:size(zL,1))*tstep,dzL,'linewidth',2)
grid on
legend('Left Feet Z Velocity','Fontsize',12)
subplot(2,2,2)
plot((1:size(zL,1))*tstep,dyL,'linewidth',2)
grid on
legend('Left Feet Y Velocity','Fontsize',12,'linewidth',2)
subplot(2,2,3)
plot((1:size(zL,1))*tstep,dth2*180/pi,'linewidth',2)
grid on
legend('Hip joint angle theta2 Velocity','Fontsize',12,'linewidth',2)
subplot(2,2,4)
plot((1:size(zL,1))*tstep,dth4*180/pi,'linewidth',2)
grid on
legend('knee joint angle theta4 Velocity','Fontsize',12,'linewidth',2)
set(gcf, 'Position', get(0, 'Screensize'));
print(figure(3),[foldername,'Left Velocity.png'],'-dpng','-r300');

figure(4)
subplot(2,2,1)
plot((1:100:size(zR,1))*tstep,zR(1:100:size(zR,1)),'linewidth',2)
grid on
legend('Right Feet Z','Fontsize',12,'location','best')
subplot(2,2,2)
plot((1:100:size(zR,1))*tstep,yL(1:100:size(zR,1)),'linewidth',2)
grid on
legend('Right Feet Y','Fontsize',12,'linewidth',2,'location','best')
subplot(2,2,3)
plot((1:100:size(zR,1))*tstep,th1(1:100:size(zR,1))*180/pi,(1:100:size(zR,1))*tstep,th1d(1:100:size(zR,1))*180/pi,'linewidth',2)
grid on
legend('Hip joint angle theta1','desired','Fontsize',12,'linewidth',2,'location','best')
subplot(2,2,4)
plot((1:100:size(zL,1))*tstep,th3(1:100:size(zL,1))*180/pi,(1:100:size(zL,1))*tstep,th3d(1:100:size(zL,1))*180/pi,'linewidth',2)
grid on
legend('knee joint angle theta3','desired','Fontsize',12,'linewidth',2,'location','best')
set(gcf, 'Position', get(0, 'Screensize'));
print(figure(4),[foldername,'Right Position.png'],'-dpng','-r300');

figure(5)
subplot(2,2,1)
plot((1:size(zL,1))*tstep,dzR,'linewidth',2)
grid on
legend('Right Feet Z Velocity','Fontsize',12)
subplot(2,2,2)
plot((1:size(zL,1))*tstep,dyR,'linewidth',2)
grid on
legend('Right Feet Y Velocity','Fontsize',12,'linewidth',2)
subplot(2,2,3)
plot((1:size(zR,1))*tstep,dth1*180/pi,'linewidth',2)
grid on
legend('Hip joint angle theta1 Velocity','Fontsize',12,'linewidth',2)
subplot(2,2,4)
plot((1:size(zR,1))*tstep,dth3*180/pi,'linewidth',2)
grid on
legend('knee joint angle theta3 Velocity','Fontsize',12,'linewidth',2)
set(gcf, 'Position', get(0, 'Screensize'));
print(figure(5),[foldername,'Right Velocity.png'],'-dpng','-r300');


