function NABIRoS_model(mode,r,R,theta1,theta2,theta3,theta4)

Rtheta1 = [ 1 0 0; 0 cos(theta1)  sin(theta1); 0 -sin(theta1) cos(theta1) ];
Rtheta2 = [ 1 0 0; 0 cos(theta2) -sin(theta2); 0  sin(theta2) cos(theta2) ];
Rtheta3 = [ 1 0 0; 0 cos(theta3)  sin(theta3); 0 -sin(theta3) cos(theta3) ];
Rtheta4 = [ 1 0 0; 0 cos(theta4) -sin(theta4); 0  sin(theta4) cos(theta4) ];

base_link(mode,r,R);

joint(mode,r,R,[0;-0.05;-0.0558]);
femur(mode,r,R,Rtheta2,-1,[0;-0.05;-0.0558]);
joint(mode,r,R,([0;-0.05;-0.0558]+Rtheta2*[0; -0.39152; 0]));
tibia(mode,r,R,Rtheta2,Rtheta4,-1,[0;-0.05;-0.0558]);
foot(mode,r,R,([0;-0.05;-0.0558]+Rtheta2*([0; -0.39152; 0] + Rtheta4*[0; -0.39857; 0])));

joint(mode,r,R,[0;0.05;-0.0558]);
femur(mode,r,R,Rtheta1,1,[0;0.05;-0.0558]);
joint(mode,r,R,([0;0.05;-0.0558]+Rtheta1*[0; 0.39152; 0]));
tibia(mode,r,R,Rtheta1,Rtheta3,1,[0;0.05;-0.0558]);
foot(mode,r,R,([0;0.05;-0.0558]+Rtheta1*([0; 0.39152; 0] + Rtheta3*[0; 0.39857; 0])));

%axis equal
xlim([-1 1])
ylim([-2 1])
zlim([-1 1])
view(90,0)
grid on

end

function base_link(mode,r,R)

figure(mode);

scale = 1 ;

vertices = [-0.02  ,  -0.10   ,  -0.01   ;
            -0.02  ,  -0.10   ,   0.01   ;
            -0.02  ,   0.10   ,   0.01   ;
            -0.02  ,   0.10   ,  -0.01   ;
            -0.02  ,   0.06   ,  -0.01   ;
            -0.02  ,   0.06   ,  -0.0558 ;
            -0.02  ,   0.04   ,  -0.0558 ;
            -0.02  ,   0.04   ,  -0.01   ;
            -0.02  ,  -0.04   ,  -0.01   ;
            -0.02  ,  -0.04   ,  -0.0558 ;
            -0.02  ,  -0.06   ,  -0.0558 ;
            -0.02  ,  -0.06   ,  -0.01   ;          
             0.02  ,  -0.10   ,  -0.01   ;
             0.02  ,  -0.10   ,   0.01   ;
             0.02  ,   0.10   ,   0.01   ;
             0.02  ,   0.10   ,  -0.01   ;
             0.02  ,   0.06   ,  -0.01   ;
             0.02  ,   0.06   ,  -0.0558 ;
             0.02  ,   0.04   ,  -0.0558 ;
             0.02  ,   0.04   ,  -0.01   ;
             0.02  ,  -0.04   ,  -0.01   ;
             0.02  ,  -0.04   ,  -0.0558 ;
             0.02  ,  -0.06   ,  -0.0558 ;
             0.02  ,  -0.06   ,  -0.01   ]'*scale;

n_ver = 24;
VertexData = zeros(24,3);

for i_ver=1:n_ver
    VertexData(i_ver,:) = r + R*vertices(:,i_ver);
end
 
frontback = [1 2 3 4 5 6 7 8 9 10 11 12; 13 14 15 16 17 18 19 20 21 22 23 24 ];
n_pat = 2;
PatchData_X = zeros(12,n_pat);
PatchData_Y = zeros(12,n_pat);
PatchData_Z = zeros(12,n_pat);

for i_pat=1:n_pat
    PatchData_X(:,i_pat) = VertexData(frontback(i_pat,:),1);
    PatchData_Y(:,i_pat) = VertexData(frontback(i_pat,:),2);
    PatchData_Z(:,i_pat) = VertexData(frontback(i_pat,:),3);
end

patch(PatchData_X,PatchData_Y,PatchData_Z,'y');

sidepatch = [1 2 14 13; 2 3 15 14; 3 4 16 15; 4 5 17 16; 1 12 24 13; 12 11 23 24; 9 10 22 21; 8 7 19 20; 5 6 18 17; 9 8 20 21; 11 10 22 23; 7 6 18 19 ];   
n_pat = 12;
PatchData_X = zeros(4,n_pat);
PatchData_Y = zeros(4,n_pat);
PatchData_Z = zeros(4,n_pat);

for i_pat=1:n_pat
    PatchData_X(:,i_pat) = VertexData(sidepatch(i_pat,:),1);
    PatchData_Y(:,i_pat) = VertexData(sidepatch(i_pat,:),2);
    PatchData_Z(:,i_pat) = VertexData(sidepatch(i_pat,:),3);
end
patch(PatchData_X,PatchData_Y,PatchData_Z,'y');

end

function joint(mode,r,R,pos)

figure(mode);

scale = 1;
Radius = 0.015*scale;
Width = 0.012*scale;
SideCount = 10;

n_side = SideCount;
VertexData_0 = zeros(3,SideCount);
VertexData = zeros(SideCount,3);

for i_ver=1:n_side
    VertexData_0(:,i_ver) = [(-Width/2); pos(2)+(Radius*cos(2*pi/n_side*i_ver)); pos(3)+(Radius*sin(2*pi/n_side*i_ver))];
    VertexData_0(:,n_side+i_ver) = [(Width/2); pos(2)+(Radius*cos(2*pi/n_side*i_ver)); pos(3)+(Radius*sin(2*pi/n_side*i_ver))];
end

n_ver = 2*n_side;

for i_ver=1:n_ver
    VertexData(i_ver,:) = r + R*VertexData_0(:,i_ver);
end

Index_Patch1 = zeros(SideCount,4);
for i_pat=1:n_side-1
    Index_Patch1(i_pat,:) = [i_pat,i_pat+1,i_pat+1+n_side,i_pat+n_side];
end
Index_Patch1(n_side,:) = [n_side,1,1+n_side,2*n_side];

PatchData1_X = zeros(4,6);
PatchData1_Y = zeros(4,6);
PatchData1_Z = zeros(4,6);

for i_pat = 1:(n_side)
    PatchData1_X(:,i_pat) = VertexData(Index_Patch1(i_pat,:),1);
    PatchData1_Y(:,i_pat) = VertexData(Index_Patch1(i_pat,:),2);
    PatchData1_Z(:,i_pat) = VertexData(Index_Patch1(i_pat,:),3);
end

patch(PatchData1_X,PatchData1_Y,PatchData1_Z,'b');

Index_Patch2(1,:) = [1:n_side];
Index_Patch2(2,:) = [n_side+1:2*n_side];

PatchData2_X = zeros(n_side,2);
PatchData2_Y = zeros(n_side,2);
PatchData2_Z = zeros(n_side,2);

for i_pat=1:2
    PatchData2_X(:,i_pat) = VertexData(Index_Patch2(i_pat,:),1);
    PatchData2_Y(:,i_pat) = VertexData(Index_Patch2(i_pat,:),2);
    PatchData2_Z(:,i_pat) = VertexData(Index_Patch2(i_pat,:),3);
end

patch(PatchData2_X,PatchData2_Y,PatchData2_Z,'b');

end

function femur(mode,r,R,Rtheta1,F,pos)

figure(mode)

scale = 1;
L = [0.01 0.39152 0.01];

VertexData_0 = ([L(1)*ones(8,1),L(2)*ones(8,1),L(3)*ones(8,1)]...
    .*[ -0.5 0 -0.5 
         0.5 0 -0.5 
         0.5 0  0.5 
        -0.5 0  0.5 
        -0.5 F -0.5 
         0.5 F -0.5 
         0.5 F  0.5 
        -0.5 F  0.5 ])'*scale;

n_ver = 8;
VertexData = zeros(8,3);

for i_ver=1:n_ver
    VertexData(i_ver,:) = r + R*(pos+Rtheta1*VertexData_0(:,i_ver));
end

% Patches
Index_Patch = ...
    [1,2,3,4;
     5,6,7,8;
     1,5,6,2;
     1,5,8,4;
     4,8,7,3;
     2,3,7,6];

n_pat = 6;
PatchData_X = zeros(4,6);
PatchData_Y = zeros(4,6);
PatchData_Z = zeros(4,6);

for i_pat=1:n_pat
    
    % Patches data
    PatchData_X(:,i_pat) = VertexData(Index_Patch(i_pat,:),1);
    PatchData_Y(:,i_pat) = VertexData(Index_Patch(i_pat,:),2);
    PatchData_Z(:,i_pat) = VertexData(Index_Patch(i_pat,:),3);
end

patch(PatchData_X,PatchData_Y,PatchData_Z,'y');

end

function tibia(mode,r,R,Rtheta1,Rtheta2,T,pos)

figure(mode)

scale = 1;
L = [0.01 0.39857 0.01 ];
F = [0; T*0.39152; 0];

VertexData_0 = ([L(1)*ones(8,1),L(2)*ones(8,1),L(3)*ones(8,1)]...
    .*[ -0.5 0 -0.5  
         0.5 0 -0.5  
         0.5 0  0.5  
        -0.5 0  0.5  
        -0.5 T -0.5  
         0.5 T -0.5   
         0.5 T  0.5  
        -0.5 T  0.5  ])'*scale;

n_ver = 8;
VertexData = zeros(8,3);

for i_ver=1:n_ver
    VertexData(i_ver,:) = r + R*(pos+ Rtheta1*(F+Rtheta2*VertexData_0(:,i_ver)));
end

% Patches
Index_Patch = ...
    [1,2,3,4;
     5,6,7,8;
     1,5,6,2;
     1,5,8,4;
     4,8,7,3;
     2,3,7,6];

n_pat = 6;
PatchData_X = zeros(4,6);
PatchData_Y = zeros(4,6);
PatchData_Z = zeros(4,6);

for i_pat=1:n_pat
    
    % Patches data
    PatchData_X(:,i_pat) = VertexData(Index_Patch(i_pat,:),1);
    PatchData_Y(:,i_pat) = VertexData(Index_Patch(i_pat,:),2);
    PatchData_Z(:,i_pat) = VertexData(Index_Patch(i_pat,:),3);
end

patch(PatchData_X,PatchData_Y,PatchData_Z,'y');

end

function foot(mode,r,R,pos)

figure(mode);

scale = 1;
Radius = 0.015*scale;
Width = 0.05*scale;
SideCount = 10;

n_side = SideCount;
VertexData_0 = zeros(3,SideCount);
VertexData = zeros(SideCount,3);

for i_ver=1:n_side
    VertexData_0(:,i_ver) = [(-Width/2); pos(2)+(Radius*cos(2*pi/n_side*i_ver)); pos(3)+(Radius*sin(2*pi/n_side*i_ver))];
    VertexData_0(:,n_side+i_ver) = [(Width/2); pos(2)+(Radius*cos(2*pi/n_side*i_ver)); pos(3)+(Radius*sin(2*pi/n_side*i_ver))];
end

n_ver = 2*n_side;

for i_ver=1:n_ver
    VertexData(i_ver,:) = r + R*VertexData_0(:,i_ver);
end

Index_Patch1 = zeros(SideCount,4);
for i_pat=1:n_side-1
    Index_Patch1(i_pat,:) = [i_pat,i_pat+1,i_pat+1+n_side,i_pat+n_side];
end
Index_Patch1(n_side,:) = [n_side,1,1+n_side,2*n_side];

PatchData1_X = zeros(4,6);
PatchData1_Y = zeros(4,6);
PatchData1_Z = zeros(4,6);

for i_pat = 1:(n_side)
    PatchData1_X(:,i_pat) = VertexData(Index_Patch1(i_pat,:),1);
    PatchData1_Y(:,i_pat) = VertexData(Index_Patch1(i_pat,:),2);
    PatchData1_Z(:,i_pat) = VertexData(Index_Patch1(i_pat,:),3);
end

patch(PatchData1_X,PatchData1_Y,PatchData1_Z,'b');

Index_Patch2(1,:) = [1:n_side];
Index_Patch2(2,:) = [n_side+1:2*n_side];

PatchData2_X = zeros(n_side,2);
PatchData2_Y = zeros(n_side,2);
PatchData2_Z = zeros(n_side,2);

for i_pat=1:2
    PatchData2_X(:,i_pat) = VertexData(Index_Patch2(i_pat,:),1);
    PatchData2_Y(:,i_pat) = VertexData(Index_Patch2(i_pat,:),2);
    PatchData2_Z(:,i_pat) = VertexData(Index_Patch2(i_pat,:),3);
end

patch(PatchData2_X,PatchData2_Y,PatchData2_Z,'b');

end