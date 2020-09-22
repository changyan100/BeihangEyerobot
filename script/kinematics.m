% matlab code and Results
clear
%calculate Beihang Eyerobot kinematics
syms q1 q2 q3 q4 q5 q6 r1 r234 lx ly lz
% r1 = 200mm
% r234 = 499mm

% Rotx =[1     0        0
%        0   cos(q)   -sin(q)
%        0   sin(q)   cos(q) ]
% Roty =[cos(q)    0    sin(q)
%           0      1     0
%        -sin(q)   0    cos(q)]
%    
% Rotz =[cos((q))    -sin(q)   0
%        sin(q)     cos(q)     0
%          0           0       1]
     
E01 = [cos(q1)    -sin(q1)    0   0
       sin(q1)     cos(q1)    0   0
         0            0       1   0
         0            0       0   1];
E12 = [cos(q2)    -sin(q2)    0   0
       sin(q2)     cos(q2)    0   -r1
         0            0       1   0
         0            0       0   1];
E23 = [1 0 0 0
       0 1 0 -r234
       0 0 1 q3
       0 0 0 1];
E34 = [cos(q4)    0    sin(q4)   0
          0       1      0       0
       -sin(q4)   0    cos(q4)   0
          0       0      0       1];
E45 = [1     0          0       0
       0   cos(q5)   -sin(q5)   0
       0   sin(q5)   cos(q5)    0
       0    0          0        1];
E56 = [1 0 0 0
       0 1 0 0
       0 0 1 q6
       0 0 0 1];
E6tip = [1 0 0 lx
         0 1 0 ly
         0 0 1 -lz
         0 0 0 1];
% E0tip = E01*E12*E23*E34*E45*E56*E6tip;
% E3tip = E34*E45*E56*E6tip;
% E03 = E01*E12*E23;
% 
% E0tip_sim = simplify(E0tip)
% E3tip_sim = simplify(E3tip)
% E03_sim = simplify(E03)


%Results:
syms tx_0tip ty_0tip tz_0tip
% tx_0tip = r234*sin(q1 + q2) + r1*sin(q1) - (ly*cos(q1 + q2)*cos(q4 + q5))/2 - (lz*cos(q1 + q2)*sin(q4 + q5))/2 + (q6*cos(q1 + q2)*sin(q4 + q5))/2 + lx*cos(q1 + q2)*cos(q4) - ly*sin(q1 + q2)*cos(q5) - lz*sin(q1 + q2)*sin(q5) + q6*sin(q1 + q2)*sin(q5) + (ly*cos(q4 - q5)*cos(q1 + q2))/2 - (lz*sin(q4 - q5)*cos(q1 + q2))/2 + (q6*sin(q4 - q5)*cos(q1 + q2))/2;
% ty_0tip =  (q6*sin(q1 + q2)*sin(q4 + q5))/2 - r1*cos(q1) - (ly*cos(q4 + q5)*sin(q1 + q2))/2 - (lz*sin(q1 + q2)*sin(q4 + q5))/2 - r234*cos(q1 + q2) + ly*cos(q1 + q2)*cos(q5) + lx*sin(q1 + q2)*cos(q4) + lz*cos(q1 + q2)*sin(q5) - q6*cos(q1 + q2)*sin(q5) + (ly*cos(q4 - q5)*sin(q1 + q2))/2 - (lz*sin(q4 - q5)*sin(q1 + q2))/2 + (q6*sin(q4 - q5)*sin(q1 + q2))/2;
% tz_0tip = q3 - lx*sin(q4) - lz*cos(q4)*cos(q5) + q6*cos(q4)*cos(q5) + ly*cos(q4)*sin(q5);
E0tip_ans =[ cos(q1 + q2)*cos(q4), cos(q1 + q2)*sin(q4)*sin(q5) - sin(q1 + q2)*cos(q5), sin(q1 + q2)*sin(q5) + cos(q1 + q2)*cos(q5)*sin(q4), tx_0tip
             sin(q1 + q2)*cos(q4), cos(q1 + q2)*cos(q5) + sin(q1 + q2)*sin(q4)*sin(q5), sin(q1 + q2)*cos(q5)*sin(q4) - cos(q1 + q2)*sin(q5), ty_0tip
             -sin(q4),                                     cos(q4)*sin(q5),                                     cos(q4)*cos(q5),              tz_0tip                                                                                                                                                                                                                                                            
                    0,                                                   0,                                                   0,              1];
 
syms tx_3tip ty_3tip tz_3tip
% tx_3tip =  lx*cos(q4) - lz*cos(q5)*sin(q4) + q6*cos(q5)*sin(q4) + ly*sin(q4)*sin(q5);
% ty_3tip =  ly*cos(q5) + lz*sin(q5) - q6*sin(q5);
% tz_3tip = q6*cos(q4)*cos(q5) - lz*cos(q4)*cos(q5) - lx*sin(q4) + ly*cos(q4)*sin(q5);
E3tip_ans =[  cos(q4), sin(q4)*sin(q5), cos(q5)*sin(q4),    tx_3tip
              0,         cos(q5),        -sin(q5),          ty_3tip
             -sin(q4), cos(q4)*sin(q5), cos(q4)*cos(q5),    tz_3tip
               0,               0,               0,          1];

syms tx_03 ty_03 tz_03
% tx_03 =  r234*sin(q1 + q2) + r1*sin(q1);
% ty_03 =  - r234*cos(q1 + q2) - r1*cos(q1);
% tz_03 = q3;
E03_ans =[ cos(q1 + q2), -sin(q1 + q2), 0,     tx_03
           sin(q1 + q2),  cos(q1 + q2), 0,     ty_03  
            0,             0,           1,     tz_03             
            0,             0,           0,     1];
      
        
% Jacobian for 0-6 joints
%for jabobian calculation -using the method from Youtube
% d0n = E0tip_ans(1:3,4);
E06 = E01*E12*E23*E34*E45*E56;
E06_sim = simplify(E06);
d06 = E06_sim(1:3,4);

d01 = E01(1:3,4);
R01 = E01(1:3,1:3);
R01_z = R01*[0 0 1]';
c1_up = cross(R01_z,(d06 - d01));
c1_low = R01_z;
c1 = [ c1_up
       c1_low];
% c1_ans = [ -ty_0tip
%            tx_0tip
%             0
%             0
%             0
%             1];

E02 = simplify(E01*E12);
d02 = E02(1:3,4);
R02 = E02(1:3,1:3);
R02_z = R02*[0 0 1]';
c2_up = cross(R02_z, (d06 - d02));
c2_low = R02_z;
c2 = [c2_up
      c2_low];
% c2_ans = [ - ty_0tip - r1*cos(q1)
%           tx_0tip - r1*sin(q1)
%                   0
%                   0
%                   0
%                   1]; 
              
E03 = simplify(E01*E12*E23);
d03 = E03(1:3,4);
R03 = E03(1:3,1:3);
R03_z = R03*[0 0 1]';
c3_up = R03_z;
c3_low = [0 0 0]'; %prismatic joint
c3 = [c3_up
      c3_low];
% c3_ans = [0 0 1 0 0 0]';

E04 = simplify(E01*E12*E23*E34);
d04 = E04(1:3,4);
R04 = E04(1:3,1:3);
R04_z = R04*[0 1 0 ]';  % rotational axis for joint 4 is along y axis
c4_up = cross(R04_z, (d06 - d04));
c4_low = R04_z;
c4 = [c4_up
      c4_low];
% c4_ans = [-cos(q1 + q2)*(q3 - tz_0tip)
%           -sin(q1 + q2)*(q3 - tz_0tip)
%           cos(q1 + q2)*(r234*sin(q1 + q2) - tx_0tip + r1*sin(q1)) - sin(q1 + q2)*(ty_0tip + r234*cos(q1 + q2) + r1*cos(q1))
%           -sin(q1 + q2)
%           cos(q1 + q2)
%           0];
%   
      
E05 = simplify(E01*E12*E23*E34*E45);
d05 = E05(1:3,4);
R05 = E05(1:3,1:3);
R05_z = R05*[1 0 0 ]';  % rotational axis for joint 5 is along x axis
c5_up = cross(R05_z, (d06 - d05));
c5_low = R05_z;
c5 = [c5_up
      c5_low];
% c5_ans = [ sin(q4)*(ty_0tip + r234*cos(q1 + q2) + r1*cos(q1)) - sin(q1 + q2)*cos(q4)*(q3 - tz_0tip)
%            sin(q4)*(r234*sin(q1 + q2) - tx_0tip + r1*sin(q1)) + cos(q1 + q2)*cos(q4)*(q3 - tz_0tip)
%            sin(q1 + q2)*cos(q4)*(r234*sin(q1 + q2) - tx_0tip + r1*sin(q1)) + cos(q1 + q2)*cos(q4)*(ty_0tip + r234*cos(q1 + q2) + r1*cos(q1))
%            cos(q1 + q2)*cos(q4)
%            sin(q1 + q2)*cos(q4)
%            -sin(q4)];
  
E06 = simplify(E01*E12*E23*E34*E45*E56);
d06 = E06(1:3,4);
R06 = E06(1:3,1:3);
R06_z = R06*[0 0 1]';
c6_up = R06_z;
c6_low = [0 0 0]'; %prismatic joint
c6 = [c6_up
      c6_low];
% c6_ans = [ sin(q1 + q2)*sin(q5) + cos(q1 + q2)*cos(q5)*sin(q4)
%            sin(q1 + q2)*cos(q5)*sin(q4) - cos(q1 + q2)*sin(q5)
%            cos(q4)*cos(q5)
%            0
%            0
%            0]';
       

% for 0-6 joints Jacobian matrix:
J06 = [c1, c2, c3, c4, c5, c6];
j11 = r234*cos(q1 + q2) + r1*cos(q1) - (q6*sin(q1 + q2)*sin(q4 + q5))/2 + q6*cos(q1 + q2)*sin(q5) - (q6*sin(q4 - q5)*sin(q1 + q2))/2;
j12 = r234*cos(q1 + q2) - (q6*sin(q1 + q2)*sin(q4 + q5))/2 + q6*cos(q1 + q2)*sin(q5) - (q6*sin(q4 - q5)*sin(q1 + q2))/2;
j15 = sin(q4)*((q6*sin(q1 + q2)*sin(q4 + q5))/2 - q6*cos(q1 + q2)*sin(q5) + (q6*sin(q4 - q5)*sin(q1 + q2))/2) + q6*sin(q1 + q2)*cos(q4)^2*cos(q5);
j21 =  r234*sin(q1 + q2) + r1*sin(q1) + (q6*cos(q1 + q2)*sin(q4 + q5))/2 + q6*sin(q1 + q2)*sin(q5) + (q6*sin(q4 - q5)*cos(q1 + q2))/2;
j22 =  r234*sin(q1 + q2) + (q6*cos(q1 + q2)*sin(q4 + q5))/2 + q6*sin(q1 + q2)*sin(q5) + (q6*sin(q4 - q5)*cos(q1 + q2))/2;
j25 =  - sin(q4)*((q6*cos(q1 + q2)*sin(q4 + q5))/2 + q6*sin(q1 + q2)*sin(q5) + (q6*sin(q4 - q5)*cos(q1 + q2))/2) - q6*cos(q1 + q2)*cos(q4)^2*cos(q5);
j34 =  - cos(q1 + q2)*((q6*cos(q1 + q2)*sin(q4 + q5))/2 + q6*sin(q1 + q2)*sin(q5) + (q6*sin(q4 - q5)*cos(q1 + q2))/2) - sin(q1 + q2)*((q6*sin(q1 + q2)*sin(q4 + q5))/2 - q6*cos(q1 + q2)*sin(q5) + (q6*sin(q4 - q5)*sin(q1 + q2))/2);
j35 = cos(q1 + q2)*cos(q4)*((q6*sin(q1 + q2)*sin(q4 + q5))/2 - q6*cos(q1 + q2)*sin(q5) + (q6*sin(q4 - q5)*sin(q1 + q2))/2) - sin(q1 + q2)*cos(q4)*((q6*cos(q1 + q2)*sin(q4 + q5))/2 + q6*sin(q1 + q2)*sin(q5) + (q6*sin(q4 - q5)*cos(q1 + q2))/2);

J06_ans =  [ j11, j12, 0, q6*cos(q1 + q2)*cos(q4)*cos(q5), j15,                  sin(q1 + q2)*sin(q5) + cos(q1 + q2)*cos(q5)*sin(q4)
             j21, j22, 0, q6*sin(q1 + q2)*cos(q4)*cos(q5), j25,                  sin(q1 + q2)*cos(q5)*sin(q4) - cos(q1 + q2)*sin(q5)
             0,   0,   1, j34,                             j35,                  cos(q4)*cos(q5)
             0,   0,   0,  -sin(q1 + q2),                  cos(q1 + q2)*cos(q4), 0
             0,   0,   0,  cos(q1 + q2),                   sin(q1 + q2)*cos(q4), 0
             1,   1,   0,  0,                              -sin(q4),             0];
 
     
% for 0-3 joints Jacobian matrix:
J03= [-r1*sin(q1)-r234*sin(q1+q2)    -r234*sin(q1+q2) 0
      r1*cos(q1)+r234*cos(q1+q2)     r234*cos(q1+q2)  0
      0                              0                1];
% [vx, vy, vz]' = J03*[q1_dot, q2_dot, q3_dot]';

% Jacobian for 3-6 joints
% [vz, wx, wy]' = [q6_dot, q5_dot, q4_dot]';

 
   
 
  
  
     