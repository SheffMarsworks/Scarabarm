function gui_joint_control
% Six-slider MATLAB GUI for Scarab arm + ROS 2 /joint_states publisher
% ------------------------------------------------------------------

clc;

%% 1. Load robot & freeze finger joints
arm = importrobot("full_scarab_arm.urdf","DataFormat","row");

freezeBodies = ["inner_link_x","inner_link_y", ...
                "outer_link_x","finger_x", ...
                "outer_link_y","finger_y"];
for b = freezeBodies
    rj = arm.getBody(b).Joint;
    T  = rj.JointToParentTransform / rj.ChildToJointTransform;
    fj = rigidBodyJoint(rj.Name,"fixed");
    setFixedTransform(fj,T);
    replaceJoint(arm,b,fj);
end

active = {};
for k = 1:numel(arm.Bodies)
    if arm.Bodies{k}.Joint.Type ~= "fixed"
        active{end+1} = arm.Bodies{k}.Joint.Name; %#ok<SAGROW>
    end
end
nJ   = numel(active);
jNm  = active(:);

%% 2. ROS 2 node + publisher
node = ros2node("/matlab_gui");
pub  = ros2publisher(node,"/joint_states","sensor_msgs/JointState");
rate = ros2rate(node,100);

%% 3. Graphics ---------------------------------------------------------
% 3-a  regular figure for robot view  (must exist before callbacks)
vf = figure("Name","Scarab View","Position",[360 60 600 600]);
ax = axes(vf); view(ax,120,25); axis(ax,"equal"); grid(ax,"on");
show(arm, zeros(1,nJ), "Parent",ax, "Visuals","on","Frames","off");

% 3-b  uifigure for sliders + numeric fields
uf = uifigure("Name","Scarab Controls","Position",[80 80 260 400]);

sliderH   = gobjects(nJ,1);
editH     = gobjects(nJ,1);

for k = 1:nJ
    baseY = 360 - 55*k;

    % joint label
    uilabel(uf,"Text",jNm{k}, ...
            "Position",[20 baseY+10 80 15], ...
            "HorizontalAlignment","left");

    % numeric edit field  (−π … π)
    e = uieditfield(uf,"numeric", ...
           "Limits",[-pi pi], ...
           "LowerLimitInclusive","on", ...
           "UpperLimitInclusive","on", ...
           "Value",0, ...
           "Position",[110 baseY+6 60 22], ...
           "ValueChangedFcn",@(src,evt) editChanged(k));
    editH(k) = e;

    % slider
    s = uislider(uf, ...
           "Limits",[-pi pi], ...
           "Value",0, ...
           "MajorTicks",[], ...
           "Position",[20 baseY 200 3], ...
           "ValueChangedFcn",@(src,evt) sliderChanged(k));
    sliderH(k) = s;
end

%% 4. Callback helpers -------------------------------------------------
    function sliderChanged(idx)
        % sync edit field → slider
        editH(idx).Value = sliderH(idx).Value;
        pushUpdate();
    end

    function editChanged(idx)
        % sync slider → edit field
        sliderH(idx).Value = editH(idx).Value;
        pushUpdate();
    end

    function pushUpdate
        % gather all values
        q = zeros(1,nJ);
        for i = 1:nJ
            q(i) = sliderH(i).Value;
        end

        % redraw robot
        cla(ax,'reset');
        show(arm,q,"Parent",ax,"Visuals","on","Frames","off");
        drawnow;

        % publish JointState
        msg          = ros2message("sensor_msgs/JointState");
        msg.name     = jNm;
        msg.position = q(:);

        send(pub,msg);
        waitfor(rate);
    end

end
