%% ros2_jointstate_publisher.m
% -------------------------------------------------------------
% 1) start MATLAB from a Humble-sourced terminal
% 2) edit the URDF path below if needed
% 3) run this file
% -------------------------------------------------------------

clc; clear; close all;

rosshutdown
rosinit

%% 1. Load robot (row format)
armURDF = "C:\Users\spes0\OneDrive\Desktop\scarab_matlab\urdf\full_scarab_arm.urdf";
arm     = importrobot(armURDF,"DataFormat","row");

%% 2. Freeze six finger joints (preserve geometry)
freezeBodies = ["inner_link_x","inner_link_y", ...
                "outer_link_x","finger_x", ...
                "outer_link_y","finger_y"];

for b = freezeBodies
    body = arm.getBody(b);
    rj   = body.Joint;                                   % original revolute
    T    = rj.JointToParentTransform / rj.ChildToJointTransform;
    fj   = rigidBodyJoint(rj.Name,"fixed");
    setFixedTransform(fj,T);
    replaceJoint(arm,b,fj);
end

%% 3. Collect active joint names (non-fixed)  ⇦  USE j.Type
active = {};
for k = 1:numel(arm.Bodies)
    j = arm.Bodies{k}.Joint;
    if j.Type ~= "fixed"                % <-- changed
        active{end+1} = j.Name;         %#ok<SAGROW>
    end
end
jointNames = active(:);                 % 6×1 cell array

%% 4. ROS 2 node + publisher
node = ros2node("/matlab_arm");
pub  = ros2publisher(node,"/joint_states","sensor_msgs/JointState");

tmpl              = ros2message("sensor_msgs/JointState");
tmpl.name         = jointNames;
tmpl.velocity     = zeros(numel(jointNames),1,"double");
tmpl.effort       = zeros(numel(jointNames),1,"double");

r = ros2rate(node,100);                 % 100 Hz

disp("Streaming /joint_states   (Ctrl-C to stop)");
while true
    qRow         = arm.randomConfiguration;
    msg          = ros2message("sensor_msgs/JointState");
    msg.name     = jointNames;
    msg.position = qRow(:);

    t            = rostime("now");     % ROS time object
    msg.header.stamp.sec     = int32(t.Sec);   % note capital S
    msg.header.stamp.nanosec = uint32(t.Nsec);

    send(pub,msg);
    waitfor(r);
end


