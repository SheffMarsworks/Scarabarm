clc; clear; close all;
cd("C:\Users\spes0\OneDrive\Desktop\scarab_matlab");

arm = importrobot("full_scarab_arm.urdf","DataFormat","row");

bodiesToFreeze = ["inner_link_x" "inner_link_y" ...
                  "outer_link_x" "finger_x" ...
                  "outer_link_y" "finger_y"];

for b = bodiesToFreeze
    body  = arm.getBody(b);
    rj    = body.Joint;                                   % current revolute

    % overall child-to-parent transform
    T = rj.JointToParentTransform * inv(rj.ChildToJointTransform);

    % new fixed joint with SAME name
    fj = rigidBodyJoint(rj.Name,"fixed");
    setFixedTransform(fj, T);

    % replace on that body
    replaceJoint(arm, b, fj);
end

% test: random arm pose (fingers locked, geometry preserved)
q = arm.randomConfiguration;
figure; show(arm,q,"Visuals","on","Frames","on");
axis equal; view(120,25); grid on;
title("Arm moves – finger joints locked, geometry intact");
