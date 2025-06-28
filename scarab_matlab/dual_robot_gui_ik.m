function dual_robot_gui_ik
% Live / Snapshot GUI + IK jog with user-set step size
%  – Sliders / edits move the LIVE robot (gray, left)
%  – 4×4 flange transform table
%  – Jog buttons (XYZαβγ) shift goal pose by stepT (mm) / 5°
%  – Apply ➜ Snapshot copies pose; future jogs start from that pose
%  – Streams /joint_states if ROS 2 is available

%% ---------- 1. Load robot & lock fingers ----------------------------
arm = importrobot("full_scarab_arm.urdf","DataFormat","row");

lockBodies = ["inner_link_x","inner_link_y", ...
              "outer_link_x","finger_x", ...
              "outer_link_y","finger_y"];
for b = lockBodies
    j  = arm.getBody(b).Joint;
    T  = j.JointToParentTransform / j.ChildToJointTransform;
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

%% ---------- 2. Optional ROS 2 publisher -----------------------------
canPub = true;
try
    node = ros2node("/matlab_gui");
    pub  = ros2publisher(node,"/joint_states","sensor_msgs/JointState");
catch
    warning("ROS 2 unavailable – publishing disabled.");
    canPub = false;
end

%% ---------- 3. IK solver & initial goal pose ------------------------
ik       = inverseKinematics("RigidBodyTree",arm);
weights  = [0.25 0.25 0.25  1 1 1];      % xyz | rpy
qZero    = zeros(1,nJ);                  % home
goalT    = getTransform(arm,qZero,"flange","base_link");   % NEW

%% ---------- 4. Figures ----------------------------------------------
fView = figure("Name","LIVE (gray)  |  SNAPSHOT (red)", ...
               "Position",[330 50 930 550]);

axLive = subplot(1,2,1,parent=fView); title(axLive,"LIVE  (gray)");
view(axLive,125,25); axis(axLive,"equal"); grid(axLive,"on");

axSnap = subplot(1,2,2,parent=fView); title(axSnap,"SNAPSHOT  (red)");
view(axSnap,125,25); axis(axSnap,"equal"); grid(axSnap,"on");

show(arm,qZero,"Parent",axLive,"Visuals","on","Frames","off");
show(arm,qZero,"Parent",axSnap,"Visuals","on","Frames","off");
recolorSnapshot();

%% ---------- 5. Control window ---------------------------------------
ufH = 60 + 55*nJ + 260;                  % extra room for step box
uf  = uifigure("Name","Joint Controls + IK Jog", ...
               "Position",[60 80 360 ufH]);

sliderH = gobjects(nJ,1);   editH = gobjects(nJ,1);
limit = 2*pi;                              % ±2π rad

for k = 1:nJ
    baseY = ufH - 60 - 55*(k-1);

    uilabel(uf,"Text",jNames{k},...
            "Position",[20 baseY+18 150 15]);

    editH(k) = uieditfield(uf,"numeric",...
        "Limits",[-limit limit],"Value",0,...
        "Position",[270 baseY+12 75 22],...
        "ValueChangedFcn",@(s,e) editChanged(k));

    sliderH(k) = uislider(uf,...
        "Limits",[-limit limit],"Value",0,"MajorTicks",[],...
        "Position",[20 baseY 325 3],...
        "ValueChangingFcn",@(s,e) sliderChanging(k),...
        "ValueChangedFcn", @(s,e) sliderChanged(k));
end

% flange transform table
uilabel(uf,"Text","Flange T (base frame)", ...
        "Position",[20 ufH-55*nJ-10 200 15],"FontWeight","bold");
tbl = uitable(uf,"Position",[20 ufH-55*nJ-150 325 125], ...
              "Data",goalT,"RowName",[],"ColumnName",[]);

%% ---------- 6. Jog panel with editable step size --------------------
jogBox = uipanel(uf,"Title","Jog EE pose", ...
                 "Position",[20 80 325 150]);

uilabel(jogBox,"Text","Step (mm):", ...
        "Position",[10 100 65 20],"HorizontalAlignment","left");
stepField = uieditfield(jogBox,"numeric", ...      % NEW
        "Limits",[0.1 100],"Value",5,...
        "Position",[80 100 60 22]);

stepR = deg2rad(5);  % 5° rotation always

% helper to read current step in metres
stepT = @() stepField.Value/1000;

% add jog buttons
btnCfg = {...
    "X−",1,-1, 5,60;"X+",1,1,55,60; ...
    "Y−",2,-1,105,60;"Y+",2,1,155,60; ...
    "Z−",3,-1,205,60;"Z+",3,1,255,60; ...
    "α−",4,-1, 5,20;"α+",4,1,55,20; ...
    "β−",5,-1,105,20;"β+",5,1,155,20; ...
    "γ−",6,-1,205,20;"γ+",6,1,255,20};
for i = 1:size(btnCfg,1)
    txt = btnCfg{i,1}; axisID = btnCfg{i,2}; dir = btnCfg{i,3};
    xpos = btnCfg{i,4}; ypos = btnCfg{i,5};
    uibutton(jogBox,"Text",txt,...
        "Position",[xpos ypos 45 25],...
        "ButtonPushedFcn",@(~,~) jog(axisID,dir));
end

% apply button
uibutton(uf,"Text","Apply ➜ Snapshot",...
         "Position",[110 25 140 32],...
         "ButtonPushedFcn",@(~,~) copyPose);

%% ---------- 7. Callback helpers -------------------------------------
    function q = poseVec, q = arrayfun(@(s)s.Value,sliderH)'; end

    function publishJS(q)
        if ~canPub, return; end
        msg          = ros2message("sensor_msgs/JointState");
        msg.name     = jNames(:); msg.position = q(:);
        t = rostime("now");
        msg.header.stamp.sec = int32(t.Sec);
        msg.header.stamp.nanosec = uint32(t.Nsec);
        send(pub,msg);
    end

    function updateFlange(q)
        tbl.Data = round(getTransform(arm,q,"flange","base_link"),4);
    end

    function syncEdits
        for i = 1:nJ, editH(i).Value = sliderH(i).Value; end
    end

    function drawLive(q)
        cla(axLive,'reset');
        show(arm,q,"Parent",axLive,"Visuals","on","Frames","off");
        updateFlange(q); drawnow limitrate; publishJS(q);
    end

%   ----- slider / edit callbacks -----
    function sliderChanging(~), syncEdits(); drawLive(poseVec()); end
    function sliderChanged(~),  syncEdits(); drawLive(poseVec()); end
    function editChanged(idx)
        sliderH(idx).Value = editH(idx).Value;
        drawLive(poseVec());
    end

%   ----- copy pose to snapshot -----
    function copyPose
        q = poseVec(); drawLive(q);
        cla(axSnap,'reset');
        show(arm,q,"Parent",axSnap,"Visuals","on","Frames","off");
        recolorSnapshot();
        goalT = getTransform(arm,q,"flange","base_link");   %#ok<NASGU> % NEW
    end

%   ----- jog function -----
    function jog(axisID,dir)
        % ----- inside jog(mode,dir) ------------------------------------------
        if axisID<=3
            % translation is still world-frame
            d = zeros(1,3); d(axisID) = dir*stepT();
            goalT = trvec2tform(d) * goalT;
        else
            % --- NEW: rotate about flange origin (post-multiply) -------------
            axLocal = eye(3);
            R = axang2tform([axLocal(axisID-3,:) dir*stepR]);
            goalT = goalT * R;            % <<<  rotate about EE, not world
        end
        q0 = poseVec();
        [qSol,~] = ik("flange",goalT,weights,q0);
        qSol = wrapToPi(qSol);           % −π..π
        for i=1:nJ                     % update widgets
            sliderH(i).Value = max(-limit,min(limit,qSol(i)));
            editH(i).Value   = sliderH(i).Value;
        end
        drawLive(poseVec());
    end

%% ---------- utility --------------------------------------------------
    function recolorSnapshot
        set(findobj(axSnap,"Type","patch"), ...
            "FaceColor",[0.8 0 0],"EdgeColor","none");
    end
end
