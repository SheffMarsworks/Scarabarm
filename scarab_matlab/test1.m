clc; clear; close all;

cd('C:\Users\spes0\OneDrive\Desktop\scarab_matlab');
scarab_arm = importrobot('full_scarab_arm.urdf')
scarab_arm.Gravity = [0 0 -9.81];
showdetails(scarab_arm);

q_home = scarab_arm.homeConfiguration;      % or your own joint positions

freezeNames = ["base_to_inner_x"  "base_to_inner_y" ...
               "base_to_outer_x" "outer_x_to_finger_x" ...
               "base_to_outer_y" "outer_y_to_finger_y"];

for j = freezeNames
    % create a new fixed joint with the same name
    newJ = rigidBodyJoint(j, "fixed");
    % zero transform (keep current origin)
    setFixedTransform(newJ, eye(4));
    % replace it in the model
    replaceJoint(scarab_arm, j, newJ);
end


% transform from base to flange
T_flange = getTransform(scarab_arm, q_home, 'flange', 'base_link')

% transform from base to the body just after joint 6
T_link6  = getTransform(scarab_arm, q_home, 'joint_6', 'base_link');

randConfig = scarab_arm.randomConfiguration

tfrom = getTransform(scarab_arm, randConfig, 'flange', 'base_link')

ik=inverseKinematics('RigidBodyTree',scarab_arm)


show(scarab_arm, randConfig,"PreservePlot",false);

