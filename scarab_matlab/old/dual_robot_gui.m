function dual_robot_gui
% Two-robot GUI with ROS 2 joint_state publisher
% Live robot (gray, left) moves with sliders / edit boxes.
% Snapshot robot (red, right) updates only on button press.

%% ---------- 1. Load robot & lock fingers ----------------------------
arm = importrobot("full_scarab_arm.urdf","DataFormat","row");

lockBodies = ["inner_link_x","inner_link_y", ...
              "outer_link_x","finger_x", ...
              "outer_link_y","finger_y"];
for b = lockBodies
    j = arm.getBody(b).Joint;
    T = j.JointToParentTransform / j.ChildToJointTransform;
    fj = rigidBodyJoint(j.Name,"fixed");
    setFixedTransform(fj,T);
    replaceJoint(arm,b,fj);
end

jNames = {};
for k = 1:numel(arm.Bodies)
    if arm.Bodies{k}.Joint.Type ~= "fixed"
        jNames{end+1} = arm.Bodies{k}.Joint.Name; %#ok<SAGROW>
    end
end
nJ = numel(jNames);

%% ---------- 2. ROS 2 node & publisher -------------------------------
% If ROS 2 environment is not sourced, the try/catch prevents a hard error.
try
    node = ros2node("/matlab_gui");
    pub  = ros2publisher(node,"/joint_states","sensor_msgs/JointState");
    canPublish = true;
catch ME
    warning("ROS 2 not available, publishing disabled:\n%s", ME.message);
    canPublish = false;
end

%% ---------- 3. Figures ----------------------------------------------
% 3-a: two axes for robots
fView = figure("Name","Scarab — live (gray)  |  snapshot (red)", ...
               "Position",[340 60 920 540]);

axLive = subplot(1,2,1,parent=fView); title(axLive,"LIVE   (gray)");
view(axLive,125,25); axis(axLive,"equal"); grid(axLive,"on");

axSnap = subplot(1,2,2,parent=fView); title(axSnap,"SNAPSHOT   (red)");
view(axSnap,125,25); axis(axSnap,"equal"); grid(axSnap,"on");

show(arm,zeros(1,nJ),"Parent",axLive,"Visuals","on","Frames","off");
show(arm,zeros(1,nJ),"Parent",axSnap,"Visuals","on","Frames","off");
recolorSnapshot();                           % paint right robot red

% 3-b: sliders + edits in a UIFigure
ufH   = 60 + 55*nJ;                          % dynamic height
uf    = uifigure("Name","Joint Controls","Position",[80 80 300 ufH]);

sliderH = gobjects(nJ,1);
editH   = gobjects(nJ,1);

for k = 1:nJ
    baseY = ufH - 60 - 55*(k-1);

    uilabel(uf,"Text",jNames{k}, ...
            "Position",[20 baseY+18 160 15]);

    editH(k) = uieditfield(uf,"numeric", ...
        "Limits",[-pi pi], ...
        "Value",0, ...
        "Position",[215 baseY+12 65 22], ...
        "ValueChangedFcn",@(src,evt) editChanged(k));

    sliderH(k) = uislider(uf, ...
        "Limits",[-pi pi], ...
        "Value",0, ...
        "MajorTicks",[], ...
        "Position",[20 baseY 255 3], ...
        "ValueChangingFcn",@(src,evt) sliderChanging(k), ...
        "ValueChangedFcn", @(src,evt) sliderChanged(k));
end

uibutton(uf,"Text","Apply ➜ Snapshot", ...
         "Position",[80 5 140 30], ...
         "ButtonPushedFcn",@(~,~) copyPose);

%% ---------- 4. Callbacks --------------------------------------------
    function q = currentPose        % read all sliders
        q = arrayfun(@(h) h.Value, sliderH)';
    end

    function publishJointState(q)
        % send /joint_states if ROS 2 works
        if ~canPublish, return; end
        msg          = ros2message("sensor_msgs/JointState");
        msg.name     = jNames(:);
        msg.position = q(:);
        now          = rostime("now");
        msg.header.stamp.sec     = int32(now.Sec);
        msg.header.stamp.nanosec = uint32(now.Nsec);
        send(pub,msg);
    end

    function syncEdits             % keep edits == sliders
        for i = 1:nJ
            editH(i).Value = sliderH(i).Value;
        end
    end

    function drawLive(q)
        cla(axLive,'reset');
        show(arm,q,"Parent",axLive,"Visuals","on","Frames","off");
        drawnow limitrate
        publishJointState(q);
    end

% ---- slider events ---------------------------------------------------
    function sliderChanging(~), syncEdits(); drawLive(currentPose()); end
    function sliderChanged(~),  syncEdits(); drawLive(currentPose()); end

% ---- edit event ------------------------------------------------------
    function editChanged(idx)
        sliderH(idx).Value = editH(idx).Value;
        drawLive(currentPose());
    end

% ---- button event ----------------------------------------------------
    function copyPose
        q = currentPose();
        drawLive(q);                       % refresh left, publish once
        cla(axSnap,'reset');
        show(arm,q,"Parent",axSnap,"Visuals","on","Frames","off");
        recolorSnapshot();
    end

%% helper: paint snapshot robot red
    function recolorSnapshot
        p = findobj(axSnap,"Type","patch");
        set(p,"FaceColor",[0.8 0 0],"EdgeColor","none");
    end
end
