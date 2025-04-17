function [ROTX, ROTY, ROTZ] = Quat2Euler(q_full, RotOrdVec, U1, DirCrx)

[Qa2b,~,t] = qcheck4(q_full,4);


% Output: phi, theta, psi
irt1 = RotOrdVec(1);
irt2 = RotOrdVec(2);
irt3 = RotOrdVec(3);

Qb2a = [Qa2b(:,1) -Qa2b(:,2:4)]; %Quaternion conjugate

qstar0_1 = Qb2a(:,1) + 1;


U1_irt2_0 = Qtrans(Qb2a,(U1(:,irt2) * qstar0_1')');
U1_irt3_0 = Qtrans(Qb2a,(U1(:,irt3) * qstar0_1')');

%Rotation 1
if irt3 == DirCrx(irt1,1)
    i_ROT1 = eye(2);
else
    i_ROT1 = [ 0, -1; 1 0];
end

tan_ROT1 = (i_ROT1 * U1_irt3_0(:,[DirCrx(irt1,2) DirCrx(irt1,1)])')';
ROT1 = atan2(tan_ROT1(:,1), tan_ROT1(:,2));

%Rotation 2
Q0to1 = [cos(0.5*ROT1), (U1(:,irt1)*sin(0.5*ROT1)')']; % Note the concatenation direction is reversed compared to the simulink

U1_irt3_1 = Qtrans(Q0to1, U1_irt3_0);
if irt3 == DirCrx(irt2,1)
    i_ROT2 = eye(2);
else
    i_ROT2 = [0 -1; 1 0];
end

tan_ROT2 = (i_ROT2 * U1_irt3_1(:,[DirCrx(irt2,2) DirCrx(irt2,1)])')';
ROT2 = atan2(tan_ROT2(:,1), tan_ROT2(:,2));

% Rotation 3
Q1to2 = [cos(0.5*ROT2), (U1(:,irt2)*sin(0.5*ROT2)')'];

Q0to2 = Qmult(Q0to1, Q1to2);

U1_irt2_1 = Qtrans(Q0to2, U1_irt2_0);
if irt2 == DirCrx(irt3,1)
    i_ROT3 = eye(2);
else
    i_ROT3 = [0 -1; 1 0];
end

tan_ROT3 = (i_ROT3 *  U1_irt2_1(:,[DirCrx(irt3,2) DirCrx(irt3,1)])')';
ROT3 = atan2(tan_ROT3(:,1), tan_ROT3(:,2));


ROT_XYZ = ([U1(:,irt1) U1(:,irt2) U1(:,irt3)] * [ROT1, ROT2, ROT3]')';

ROTX = ROT_XYZ(:,1);
ROTY = ROT_XYZ(:,2);
ROTZ = ROT_XYZ(:,3);

if t
    ROTX = ROTX';
    ROTY = ROTY';
    ROTZ = ROTZ';
else
    % Do nothing
end

end