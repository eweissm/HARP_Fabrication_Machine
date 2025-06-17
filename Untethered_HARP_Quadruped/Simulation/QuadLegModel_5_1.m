%% Long-Leg Quad Model

%% Notes
% modeled on the rear leg


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear 

global L1 L2 L3 L4 L5 L6 L7 L8 L9 L11 L12 RobotMass R qlim TorqueHistory IndividualTorques cycleTime counter
 
a1 = 100/1000;
a2 = 100/1000;

LinkMass = [10/1000, 4.5/1000];
DH = [a1 0 0 0; a2 0 0 0];
LinkInertia = [(1/2)*LinkMass(1)*.0015^2, (1/12)*LinkMass(1)*a1^2, (1/12)*LinkMass(1)*a1^2; 
               (1/2)*LinkMass(2)*.0015^2, (1/12)*LinkMass(2)*a2^2, (1/12)*LinkMass(2)*a2^2]; %[Ixx_1, Iyy_1, Izz_1; Ixx_2, Iyy_2, Izz_2]
LinkCOM = [0, 0, 0; 0, 0,0];

ViscousFriction = [0.02,0.02];
CoulumbFriction = [0.0,0.0];
qlim = [70,80; 30,50]*pi/180;

RobotMass = .7;

L1 =77/1000;
L2 = 125/1000;
L3 = 75/1000;
L4 = 35/1000;
L5 = 80/1000;
L6 = 57/1000;
L7 = 30/1000;
L8 = 130/1000;
L9 = 60/1000;
L10 = 30/1000;
L11 = 10/1000;
L12 = 130/1000;

TorqueHistory=[];
IndividualTorques=[];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate Robot Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L = cell(size(DH,1), 1);

for i = 1:size(DH,1)

    L{i} = Revolute('a', DH(i,1), 'd', DH(i,3), 'alpha', DH(i,4), 'I', LinkInertia(i,:), 'r', LinkCOM(i,:), 'm', LinkMass(i), 'G', 1, 'B', ViscousFriction(i), 'Jm', 0, 'Tc', CoulumbFriction, 'qlim',qlim(i,:));
   
end

    % Create Serial Robot
    R = SerialLink(horzcat(L{:}), 'base', transl(0, 0, 0.1)*trotx(pi/2)*trotz(pi));
    R.display()
    R.dyn()
    % 
    % figure;
    % test_q = zeros([90,2]);
    % test_q(:,1)= linspace(75*pi/180, 105*pi/180,90);
    % test_q(:,2)= linspace(-pi/2,pi/2,90);
    % q = enforceJointLimits(test_q)
    % R.plot(q,'jvec','delay',.05,'nobase')
    
    % test_q=[74,91]*pi/180;
    % R.islimit(test_q)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulate Gait Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp("Simulating...")

cycleTime = .4;
NumberOfCycles = 2;
InitialJointPose = [qlim(1,2),qlim(2,1)];
InitialJointVel = [0, 0];

JointPoseValues = [];
JointVelValues = [];
timeValues = [];

JointPoseValues(1,:) = InitialJointPose;
JointVelValues(1,:) = InitialJointVel;
timeValues(1) =0;

for i = 1:NumberOfCycles
    %start next gait cycle where we left off
    counter = 0;
    q0 = JointPoseValues(end,:);
    qd0 = JointVelValues(end,:);

    %perform integrated forward dynamics
    [T,q,qd] = R.fdyn(cycleTime, @torqfun,q0,qd0);
    
    %store gait cycle values
    JointPoseValues(end+1:end+size(q,1),:) = q;
    JointVelValues(end+1:end+size(q,1),:) = qd;
    timeValues(end+1:end+size(q,1)) = T+timeValues(end);
    
    disp('    Completed Gait #' + string(i))
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp("Joint Dynamics Calculated... Animating...")

% figure;
% plot(timeValues,JointPoseValues(:,1))
% hold on
% plot(timeValues,JointPoseValues(:,2))
% legend({'Joint1','Joint2'})
% title('Joint Angles')
% 
% figure;
% plot(timeValues,JointVelValues(:,1))
% hold on
% plot(timeValues,JointVelValues(:,2))
% legend({'Joint1','Joint2'})
% title('Joint Vels')
% 
% 
% figure;
% plot(TorqueHistory(:,1))
% hold on
% plot(TorqueHistory(:,2))
% legend({'Joint1','Joint2'})
% title('Torque History')
% 
% figure;
% plot(IndividualTorques(:,1,1))
% hold on
% plot(IndividualTorques(:,2,1))
% plot(IndividualTorques(:,3,1))
% plot(IndividualTorques(:,4,1))
% plot(IndividualTorques(:,5,1))
% legend({'GFR','HE', 'HF', 'KE', 'KF'})
% title('Torque History Joint 1')
% 
% figure;
% plot(IndividualTorques(:,1,2))
% hold on
% plot(IndividualTorques(:,2,2))
% plot(IndividualTorques(:,3,2))
% plot(IndividualTorques(:,4,2))
% plot(IndividualTorques(:,5,2))
% legend({'GFR','HE', 'HF', 'KE', 'KF'})
% title('Torque History Joint 2')













D= importdata("Quad LegTraj.xlsx");
xF= D.data(70:180,11);
yF=D.data(70:180,12);
xR=D.data(35:95,7);
yR=D.data(35:95,8);

theta= 10*pi/180;

R = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];

for i = 1:length(xF)
    x = R*[xF(i);yF(i)];
    xF(i) = x(1);
    yF(i) = x(2);
end

xF= xF-mean(xF);
yF= yF-mean(yF);

xR= xR-mean(xR);
yR= yR-mean(yR);

xF = smoothdata(xF, "movmean",5);
yF = smoothdata(yF, "movmean",5);
xR = smoothdata(xR, "movmean",5);
yR = smoothdata(yR, "movmean",5);


TrajX=-a1*cos(JointPoseValues(:,1))-a2*cos(JointPoseValues(:,1)+JointPoseValues(:,2));
TrajY = zeros(size(JointPoseValues,1),1);
TrajZ = .1 -a1*sin(JointPoseValues(:,1))-a2*sin(JointPoseValues(:,1)+JointPoseValues(:,2));

TrajX = smoothdata(TrajX, 'movmedian',400);
TrajZ = smoothdata(TrajZ, 'movmedian',400);
Z_Disp = abs(max(TrajZ)-min(TrajZ));

figure;



plot(TrajX-mean(TrajX), TrajZ-mean(TrajZ), 'k--')
text(-.1, 0,-.1,"Lift: "+ string(Z_Disp))
hold on
xlim([-.15,.15])
zlim([-.15,.15])
plot(-xF-mean(xF),yF-mean(yF))
axis equal

% R.plot(JointPoseValues,'delay',0.001,'nobase', 'view', 'x', 'noname','loop')

disp("Finished Animating")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Functions        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [q_Fixed, isLimit] =  enforceJointLimits(q)
     global qlim
    isLimit = zeros(size(q,1),2);

    epsilon = .01;

    for i = 1:size(q,1)
        if(q(i,1)<qlim(1,1)-epsilon)
            q(i,1) = qlim(1,1);
            isLimit(i,1) =1;

        elseif(q(i,1)>qlim(1,2)+epsilon)
            q(i,1) = qlim(1,2) ;
            isLimit(i,1) =1;
            
        end

        if(q(i,2)<qlim(2,1)-epsilon)
            q(i,2) = qlim(2,1);
            isLimit(i,2) =1;
        elseif(q(i,2)>qlim(2,2)+epsilon)
            q(i,2) = qlim(2,2);
            isLimit(i,2) =1;
        end

    end
    
    q_Fixed = q;
end

function tau = torque_HipExtensor(Q,isActuated)
    global L1 L2 L3 L4 L5 L6 L7 L8 RobotMass qlim

    Q = enforceJointLimits(Q);

    NumMuscles =3;

    MuscleLength = sqrt(L1^2+L2^2-2*L1*L2*cos(Q(1)));
    Leg_Muscle_Angle = asin(L2*(sin(Q(1))/MuscleLength));

   if (isActuated == 1)
        %if p=35
        MuscleRestLength = .07;
        F = @(x) (x*1000)^3*(3.17057866794312e-05)+(x*1000)^2*(-0.00265565636784081)+0.0932934422155938*(x*1000)+0.143221072151513;
        MuscleForce = F(MuscleLength-MuscleRestLength);
            
    else
        %if p=0
        MuscleRestLength = .12;
        F = @(x) 0.0215960828716410*(x*1000)+0.0379106595281091;
        MuscleForce = F(MuscleLength-MuscleRestLength);
   
    end

    tauMuscle = NumMuscles*MuscleForce * L1 * sin(Leg_Muscle_Angle);

    tau = [-tauMuscle,0];
end

function tau = torque_HipFlexor(Q, isActuated)
    global L1 L2 L3 L4 L5 L6 L7 L8 RobotMass qlim

    Q = enforceJointLimits(Q);
    NumMuscles =2;

    MuscleLength = sqrt(L3^2+L2^2-2*L3*L2*cos(pi - Q(1)));
    Leg_Muscle_Angle = asin(L2*(sin(pi-Q(1))/MuscleLength));

    if (isActuated == 1)
        %if p=35
        MuscleRestLength = .07;
        F = @(x) (x*1000)^3*(3.17057866794312e-05)+(x*1000)^2*(-0.00265565636784081)+0.0932934422155938*(x*1000)+0.143221072151513;
        MuscleForce = F(MuscleLength-MuscleRestLength);
            
    else
        %if p=0
        MuscleRestLength = .12;
        F = @(x) 0.0215960828716410*(x*1000)+0.0379106595281091;
        MuscleForce = F(MuscleLength-MuscleRestLength);
   
    end

    tauMuscle = NumMuscles* MuscleForce * L3 * sin(Leg_Muscle_Angle);

    tau = [tauMuscle,0];
end

function tau = torque_KneeExtensor(Q,isActuated)
    global L1 L2 L3 L4 L5 L6 L7 L8 L9 L11 RobotMass qlim

    Q = enforceJointLimits(Q);
    NumMuscles =3;

    phi = atan2(L11, L5);
    zeta = sqrt(L11^2+L5^2);

    MuscleLength = sqrt(L8^2+zeta^2-2*L8*zeta*cos(Q(2) + phi ));
    Leg_Muscle_Angle = acos((MuscleLength^2+zeta^2-L8^2)/(2*MuscleLength*zeta));

    if (isActuated == 1)
        %if p=35
        MuscleRestLength = .07;
        F = @(x) (x*1000)^3*(3.17057866794312e-05)+(x*1000)^2*(-0.00265565636784081)+0.0932934422155938*(x*1000)+0.143221072151513;
        MuscleForce = F(MuscleLength-MuscleRestLength);
            
    else
        %if p=0
        MuscleRestLength = .12;
        F = @(x) 0.0215960828716410*(x*1000)+0.0379106595281091;
        MuscleForce = F(MuscleLength-MuscleRestLength);
   
    end

    tauMuscle = NumMuscles * MuscleForce * zeta *sin(Leg_Muscle_Angle);
    
    % if -tauMuscle>0
    %     tauMuscle = 0;
    % end
    tau = [0,-tauMuscle];

end

function tau = torque_KneeFlexor(Q,isActuated)
    global L1 L2 L3 L4 L5 L6 L7 L8 L9 L11 L12 RobotMass qlim
    
    Q = enforceJointLimits(Q);
    NumMuscles =2;
    
    phi = atan2(L9, L4);
    zeta = sqrt(L9^2+L4^2);

    MuscleLength = sqrt(L12^2+zeta^2-2*L12*zeta*cos(pi-Q(2)-phi));
    Leg_Muscle_Angle = acos((MuscleLength^2+zeta^2-L12^2)/(2*MuscleLength*zeta));

    if (isActuated == 1)
        %if p=35
        MuscleRestLength = .07;
        F = @(x) (x*1000)^3*(3.17057866794312e-05)+(x*1000)^2*(-0.00265565636784081)+0.0932934422155938*(x*1000)+0.143221072151513;
        MuscleForce = F(MuscleLength-MuscleRestLength);
            
    else
        %if p=0
        MuscleRestLength = .12;
        F = @(x) 0.0215960828716410*(x*1000)+0.0379106595281091;
        MuscleForce = F(MuscleLength-MuscleRestLength);
   
    end
    tauMuscle = NumMuscles* MuscleForce* zeta * sin(Leg_Muscle_Angle);
    
    % if tauMuscle<0
    % tauMuscle = 0;
    % end

    tau = [0,tauMuscle];
end

function tau = torque_GRF(Q, isActive)
    global L1 L2 L3 L4 L5 L6 L7 L8 RobotMass R

    if(isActive==1)
        w = [0 0, -9.81*RobotMass/2, 0 ,0 ,0];
        tau = R.pay(Q,w','0');
    else
      tau=[0, 0];
    end
end

function tau = torqfun(R, T,Q,DQ)
     global TorqueHistory IndividualTorques cycleTime qlim counter

     k = 10; % joint limit stiffness

     %debugging counter
     if mod(counter,1000)==0
         disp('counter: ' + string(counter) + ' T: ' + string(T) + ' dQ1: ' + string(DQ(1))+ ' Q1: ' + string(Q(1)*180/pi));
     end
    
     %apply muscle torques depending on position in gait cycle:

    phasePlan = [1 0 0 1;
                 1 1 0 0;
                 0 0 1 1;
                 1 0 0 1;
                 0 1 1 0]; % GRF; HE; HF; KE; KF

    numPhases = size(phasePlan,2);
    for i = 1: numPhases
        
        CurrentPhase = floor(mod(T,cycleTime) / (cycleTime/numPhases))+1;
         IndividualTorques(end+1, :, 1:2) = [torque_GRF(Q,phasePlan(1,CurrentPhase));
                                             torque_HipExtensor(Q,phasePlan(2,CurrentPhase));
                                             torque_HipFlexor(Q,phasePlan(3,CurrentPhase));
                                             torque_KneeExtensor(Q,phasePlan(4,CurrentPhase));
                                             torque_KneeFlexor(Q,phasePlan(5,CurrentPhase))];
    end

     %sum up all the muscle/GRF torques
     tau = [sum(IndividualTorques(end, :, 1)), sum(IndividualTorques(end, :, 2))];
      
    %see if we violate a joint limit. if we do lets apply a torque to
    %counter the motion
     isLimitBool = R.islimit(Q);
    
    for i = 1:R.n %for each robot link
        if isLimitBool(i,1) ==1 % upper lim
            JointLimitViolation = Q(i)-qlim(i,2); %makes violation positive
            tau(i) = tau(i)-k*JointLimitViolation; %at the upper limit we want to apply a negative torque
             
        elseif isLimitBool(i,1) ==-1 % lower lim

            JointLimitViolation = qlim(i,1)-Q(i); %makes violation positive
            tau(i) = tau(i)+k*JointLimitViolation; %at the lower limit we want to apply a positive torque

        end
    end

     TorqueHistory(end+1,:) = tau;
     counter = counter+1;
end