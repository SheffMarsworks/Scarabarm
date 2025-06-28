

clc; clear; close all;
cd("C:\Users\spes0\OneDrive\Desktop\scarab_matlab");

%% initial Import

arm = importrobot("full_scarab_arm.urdf","DataFormat","row");

%% freeze End Efector

bodiesToFreeze = ["inner_link_x" "inner_link_y" "outer_link_x" "finger_x" "outer_link_y" "finger_y"];

for b = bodiesToFreeze
    body  = arm.getBody(b);
    rj    = body.Joint; % current revolute

    % overall child-to-parent transform
    T = rj.JointToParentTransform * inv(rj.ChildToJointTransform);

    % new fixed joint with SAME name
    fj = rigidBodyJoint(rj.Name,"fixed");
    setFixedTransform(fj, T);

    % replace on that body
    replaceJoint(arm, b, fj);
end

%% Random Configuration Test

% test: random arm pose (fingers locked, geometry preserved)
randomConfig = arm.randomConfiguration;
figure; show(arm,randomConfig,"Visuals","on","Frames","on");
axis equal; view(120,25); grid on;
title("Random");


%% Inverse Kinematics


ik = inverseKinematics('RigidBodyTree',arm);
ik.SolverParameters

tform = getTransform(arm, randomConfig, 'flange', 'base_link');

weights = [0.25 0.25 0.25 1 1 1];

[configSol,solIn] = ik('flange',tform,weights,randomConfig);


randomConfig

configSol

tformik = getTransform(arm, configSol, 'flange', 'base_link');

figure; show(arm,configSol,"Visuals","on","Frames","on");
axis equal; view(120,25); grid on;
title("Ik");






