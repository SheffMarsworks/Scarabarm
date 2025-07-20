function dual_robot_gui_ik
% Live / Snapshot GUI + IK jog with user-set step size
% – Sliders / edits move the LIVE robot (gray, left)
% – 4×4 flange transform table
% – Jog buttons (XYZαβγ) shift goal pose by stepT (mm) / 5°
% – Apply ➜ Snapshot copies pose; future jogs start from that pose
% – Streams /joint_states if ROS 2 is available

%% ── 1. Load robot & lock fingers ─────────────────────────────────────
arm = importrobot("full_scarab_arm.urdf","DataFormat","row")

showdetails(arm)          % prints a table of each body, joint limits, etc

lockBodies = ["inner_link_x","inner_link_y", ...
              "outer_link_x","finger_x", ...
              "outer_link_y","finger_y"];
for b = lockBodies
    j  = arm.getBody(b).Joint;
    T  = j.JointToParentTransform / j.ChildToJointTransform;
    fj = rigidBodyJoint(j.Name,"fixed");
    setFixedTransform(fj,T);   replaceJoint(arm,b,fj);
end

% collect joint names in tree order
jNames = {};
for k = 1:numel(arm.Bodies)
    if arm.Bodies{k}.Joint.Type ~= "fixed"
        jNames{end+1} = arm.Bodies{k}.Joint.Name; %#ok<SAGROW>
    end
end
nJ = numel(jNames);

%% ---------- 1b.  Manually override joint limits ---------------------
%  List each movable joint once with   [lower  upper]   in **radians**
%  (If the joint was declared 'continuous' the Type is still fine;
%   PositionLimits are simply ignored unless you set them.)

manualLim = { ...
   "joint_1_to_joint_2", [-pi      pi];        % ±180°
   "joint_2_to_link_2",  [-1.5708  1.5708];    % ± 90°
   "link_2_to_joint_3",  [-2.7925  2.7925];    % ±160°
   "joint_4_to_joint_5", [-pi      pi];        % ±180°
   "joint_5_to_joint_6", [-pi      pi];        % ±180°
   "joint_6_to_flange",  [-2*pi    2*pi];      % ±360°
   % add or adjust rows as you wish
};

for i = 1:size(manualLim,1)
    jName = manualLim{i,1};
    lim   = manualLim{i,2};

    % find the body that hosts this joint
    bIdx = find(cellfun(@(b) strcmp(b.Joint.Name,jName), arm.Bodies), ...
                1,"first");
    if isempty(bIdx)
        warning("Joint %s not found in the robot.", jName);
        continue
    end
    body = arm.Bodies{bIdx};
    j    = body.Joint;

    % (optional) turn continuous into revolute so limits take effect
    if j.Type == "continuous"
        j.Type = "revolute";
    end
    j.PositionLimits = lim;

    replaceJoint(arm, body.Name, j);   % write back into the tree
end


%% ── 2. Optional ROS 2 publisher ──────────────────────────────────────
canPub = true;
try
    node = ros2node("/matlab_gui");
    pub  = ros2publisher(node,"/joint_states","sensor_msgs/JointState");
catch
    warning("ROS 2 unavailable – publishing disabled.");
    canPub = false;
end
doPublish = false;                     % start silent

%% ── 3. IK solver & initial goal pose ─────────────────────────────────
ik       = inverseKinematics("RigidBodyTree",arm);
weights  = [0.25 0.25 0.25  1 1 1];        % xyz | rpy
qZero    = zeros(1,nJ);
goalT    = getTransform(arm,qZero,"flange","base_link");

%% ── 4. Figures ───────────────────────────────────────────────────────
fView  = figure("Name","LIVE (gray)  |  SNAPSHOT (red)", ...
                "Position",[330 50 930 550]);
axLive = subplot(1,2,1,parent=fView);  
hold(axLive ,'on');          % prevents show() from resetting the title
title(axLive,"LIVE  (gray)");
axSnap = subplot(1,2,2,parent=fView); 
hold(axSnap,'on');
title(axSnap,"SNAPSHOT  (red)");
view(axLive,125,25);  view(axSnap,125,25);
axis(axLive,"equal"); grid(axLive,"on");
axis(axSnap,"equal"); grid(axSnap,"on");
show(arm,qZero,"Parent",axLive,"Visuals","on","Frames","on");
show(arm,qZero,"Parent",axSnap,"Visuals","on","Frames","on");
recolorSnapshot();
%% ── 5. Control window ────────────────────────────────────────────────
extraGap = 30;
ufH = 60 + 55*nJ + 260 + extraGap;
uf  = uifigure("Name","Joint Controls + IK Jog", ...
               "Position",[60 80 360 ufH]);

sliderH = gobjects(nJ,1);   editH = gobjects(nJ,1);
lowerLim = zeros(1,nJ);     upperLim = zeros(1,nJ);

for k = 1:nJ
    baseY = ufH - 60 - 55*(k-1);

    % real limits from URDF  (continuous ⇒ ±π)
    bodyIdx = find(cellfun(@(b) strcmp(b.Joint.Name,jNames{k}), ...
                           arm.Bodies),1,"first");
    jl = arm.Bodies{bodyIdx}.Joint.PositionLimits;
    if numel(jl)~=2 || jl(1)==jl(2) || any(~isfinite(jl))
        jl = [-pi pi];
    end
    jl = sort(jl);                     % ensure ascending
    lowerLim(k) = jl(1);   upperLim(k) = jl(2);
    val0 = mean(jl);

    % label
    uilabel(uf,"Text",jNames{k}, ...
            "Position",[20 baseY+18 150 15]);

    % edit box
    editH(k) = uieditfield(uf,"numeric", ...
        "Limits",jl,"Value",val0, ...
        "Position",[270 baseY+12 75 22], ...
        "ValueChangedFcn",@(~,~) editChanged(k));

    % slider
    sliderH(k) = uislider(uf, ...
        "Limits",jl,"Value",val0,"MajorTicks",[], ...
        "Position",[20 baseY 325 3], ...
        "ValueChangingFcn",@(~,~) sliderChanging(k), ...
        "ValueChangedFcn", @(~,~) sliderChanged(k));
end

% flange transform table
lblY = ufH - 55*nJ - 10  - extraGap;
tblY = ufH - 55*nJ - 150 - extraGap;
uilabel(uf,"Text","Flange T (base frame)", ...
        "Position",[20 lblY 200 15],"FontWeight","bold");
tbl = uitable(uf,"Position",[20 tblY 325 125], ...
              "Data",goalT,"RowName",[],"ColumnName",[]);

%% ── 6. Jog panel ─────────────────────────────────────────────────────
jogBox = uipanel(uf,"Title","Jog EE pose", ...
                 "Position",[20 80 325 150]);

uilabel(jogBox,"Text","Step (mm):", ...
        "Position",[10 100 65 20],"HorizontalAlignment","left");
stepField = uieditfield(jogBox,"numeric", ...
        "Limits",[0.1 100],"Value",5, ...
        "Position",[80 100 60 22]);
stepR = deg2rad(5);
stepT = @() stepField.Value/1000;

btnCfg = {...
    "X−",1,-1,  5,60;"X+",1,1, 55,60; ...
    "Y−",2,-1,105,60;"Y+",2,1,155,60; ...
    "Z−",3,-1,205,60;"Z+",3,1,255,60; ...
    "α−",4,-1,  5,20;"α+",4,1, 55,20; ...
    "β−",5,-1,105,20;"β+",5,1,155,20; ...
    "γ−",6,-1,205,20;"γ+",6,1,255,20};
for i = 1:size(btnCfg,1)
    uibutton(jogBox,"Text",btnCfg{i,1}, ...
             "Position",[btnCfg{i,4} btnCfg{i,5} 45 25], ...
             "ButtonPushedFcn",@(~,~) jog(btnCfg{i,2},btnCfg{i,3}));
end

% Apply & Re-solve buttons
uibutton(uf,"Text","Apply ➜ Snapshot", ...
         "Position",[110 15 140 32], ...
         "ButtonPushedFcn",@(~,~) copyPose);
uibutton(uf,"Text","Re-solve IK", ...
         "Position",[110 50 140 32], ...
         "ButtonPushedFcn",@(~,~) recalcIK);

%% ── 7. Helpers & callbacks ───────────────────────────────────────────
    function q = poseVec
        q = arrayfun(@(s)s.Value,sliderH)'; 
    end

    function publishJS(q)
        if ~canPub || ~doPublish, return; end
        msg                  = ros2message("sensor_msgs/JointState");
        msg.name             = jNames(:);
        msg.position         = q(:);
        t = rostime("now");
        msg.header.stamp.sec     = int32(t.Sec);
        msg.header.stamp.nanosec = uint32(t.Nsec);
        send(pub,msg);
    end

    function updateFlange(q)
        tbl.Data = round(getTransform(arm,q,"flange","base_link"),4);
    end

    function syncEdits
        for ii = 1:nJ, editH(ii).Value = sliderH(ii).Value; end
    end

    function drawLive(q)
        cla(axLive);                                   % keep axes settings
        hold(axLive,'off');
        show(arm,q,"Parent",axLive,"Visuals","on","Frames","on");
    
        % restore the look every time we redraw
        axis(axLive,'equal');
        view(axLive,125,25);
        grid(axLive,'on');
    
        updateFlange(q);
        drawnow limitrate;
    end


% slider / edit callbacks
    function sliderChanging(~,~), syncEdits(); drawLive(poseVec()); end
    function sliderChanged(~,~),  syncEdits(); drawLive(poseVec()); end
    function editChanged(idx)
        sliderH(idx).Value = editH(idx).Value;
        drawLive(poseVec());
    end

% copy pose to snapshot
    function copyPose
        q = poseVec();
        drawLive(q);
        % -------- redraw the SNAPSHOT pane --------
        cla(axSnap);                % clear patches, keep axis props
        hold(axSnap,'off');         % make sure we don’t stack models
    
        show(arm,q,"Parent",axSnap,"Visuals","on","Frames","off");
    
        axis(axSnap,'equal');
        view(axSnap,125,25);
        grid(axSnap,'on');
        recolorSnapshot();          % turn the model red
    
        goalT = getTransform(arm,q,"flange","base_link");
        doPublish = true;
        publishJS(q);               % send one JointState
    end

% jog EE
    function jog(axisID,dir)
        if axisID <= 3
            d = zeros(1,3); d(axisID) = dir*stepT();
            goalT = trvec2tform(d)*goalT;
        else
            axLocal = eye(3);
            rotAxis = axLocal(axisID-3,:);       % x, y, or z of flange frame
            R       = axang2tform([rotAxis  dir*stepR]);
            goalT   = goalT * R;
        end
        [qSol,~] = ik("flange",goalT,weights,poseVec());
        qSol = wrapToPi(qSol);
        for ii = 1:nJ
            sliderH(ii).Value = max(lowerLim(ii), ...
                               min(upperLim(ii),qSol(ii)));
            editH(ii).Value   = sliderH(ii).Value;
        end
        drawLive(qSol);
    end

% re-solve IK from random seed
    function recalcIK
        qSeed = (rand(1,nJ)-0.5)*2*pi;
        [qAlt,~] = ik("flange",goalT,weights,qSeed);
        qAlt = wrapToPi(qAlt);
        for ii = 1:nJ
            sliderH(ii).Value = max(lowerLim(ii), ...
                               min(upperLim(ii),qAlt(ii)));
            editH(ii).Value   = sliderH(ii).Value;
        end
        drawLive(qAlt);
    end

% recolor snapshot robot (red)
    function recolorSnapshot
        set(findobj(axSnap,"Type","patch"), ...
            "FaceColor",[0.8 0 0], "EdgeColor","none");
    end
end
