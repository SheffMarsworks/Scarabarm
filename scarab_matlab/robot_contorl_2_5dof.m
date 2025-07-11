function dual_robot_gui_traj
% Dual-Robot GUI with jog, snapshot, start/goal trajectory + ROS 2 optional
% Tested R2021b + ROS 2 Toolbox R2023a

%% 1. Load & lock robot
arm = importrobot("full_scarab_arm.urdf","DataFormat","row");
lockBodies = ["inner_link_x","inner_link_y","outer_link_x","finger_x","outer_link_y","finger_y"];
for b = lockBodies
    j  = arm.getBody(b).Joint;
    fj = rigidBodyJoint(j.Name,"fixed");
    setFixedTransform(fj, j.JointToParentTransform / j.ChildToJointTransform);
    replaceJoint(arm,b,fj);
end

jNames = {};
for k = 1:numel(arm.Bodies)
    if arm.Bodies{k}.Joint.Type ~= "fixed"
        jNames{end+1} = arm.Bodies{k}.Joint.Name; %#ok<SAGROW>
    end
end
nJ = numel(jNames);

%% 2. Hard-code continuous joint limits
limTbl = {
    "joint_1_to_joint_2", [-pi      pi];
    "joint_2_to_link_2",  [-2.5708  2.5708];
    "link_2_to_joint_3",  [-2.7925  2.7925];
    % "joint_4_to_joint_5", [-pi      pi];   % REMOVED this line for 5 DOF
    "joint_5_to_joint_6", [-pi      pi];
    "joint_6_to_flange",  [-pi   pi];
};
for i = 1:size(limTbl,1)
    idx = find(cellfun(@(b) strcmp(arm.getBody(b).Joint.Name,limTbl{i,1}),arm.BodyNames),1);
    if isempty(idx), continue; end
    jb = arm.getBody(arm.BodyNames{idx}).Joint;
    if jb.Type=="continuous", jb.Type="revolute"; end
    jb.PositionLimits = limTbl{i,2};
    replaceJoint(arm,arm.BodyNames{idx},jb);
end

%% 3. ROS-2 pubs/subs
canPub = true; doPublish = false;
try
    node      = ros2node("/matlab_gui", 7);
    sub       = ros2subscriber(node,"/joint_states","sensor_msgs/JointState", ...
                  @(msg)fprintf("Heard JS: [%s]\n",join(string(msg.position),", ")) );
    pubState  = ros2publisher(node,"/joint_states","sensor_msgs/JointState");
    pubODrive = ros2publisher(node,"/odrive_can/command_joint_positions","std_msgs/Float64MultiArray");
    pubTraj   = ros2publisher(node,"/command_trajectory","trajectory_msgs/JointTrajectory");
catch
    warning("ROS 2 unavailable – disabling publishers.");
    canPub = false;
end

%% 4. IK + initial goal
ik      = inverseKinematics("RigidBodyTree",arm);
weights = [.25 .25 .25 0.1 0.1 0.1];   % softened orientation weights for 5 DOF
qZero   = zeros(1,nJ);
goalT   = getTransform(arm,qZero,"flange","base_link");

%% 5. Two viewports
fView  = figure("Name","LIVE (gray)  |  SNAPSHOT (red)","Position",[320 40 930 560]);
axLive = subplot(1,2,1,Parent=fView); title(axLive,"LIVE  (gray)"); initAxes(axLive);
axSnap = subplot(1,2,2,Parent=fView); title(axSnap,"SNAPSHOT  (red)"); initAxes(axSnap);
show(arm,qZero,"Parent",axLive,"Visuals","on","Frames","on");
show(arm,qZero,"Parent",axSnap,"Visuals","on","Frames","off"); recolorSnapshot();

%% 6. Control window
ctrlW = 480; 
extraGap = 50; 
ufH = 60 + 55*nJ + 560 + extraGap;  % increase height to fit everything
uf = uifigure("Name","Joint Controls + Trajectory","Position",[40 80 ctrlW ufH]);

sliderH = gobjects(nJ,1);
editH   = gobjects(nJ,1);
lowerLim = zeros(1,nJ);
upperLim = zeros(1,nJ);

for k = 1:nJ
    baseY = ufH - 60 - 55*(k-1);
    bIdx  = find(cellfun(@(b) strcmp(arm.getBody(b).Joint.Name,jNames{k}),arm.BodyNames),1);
    jl    = arm.getBody(arm.BodyNames{bIdx}).Joint.PositionLimits;
    if numel(jl)~=2 || jl(1)==jl(2) || any(~isfinite(jl)), jl = [-pi pi]; end
    jl = sort(jl);
    lowerLim(k) = jl(1);
    upperLim(k) = jl(2);
    val0 = mean(jl);

    uilabel(uf,"Text",jNames{k},...
             "Position",[20 baseY+18 150 15]);
    editH(k) = uieditfield(uf,"numeric",...
                 "Limits",jl,"Value",val0,...
                 "Position",[380 baseY+8 80 22],...
                 "ValueChangedFcn",@(s,e)editChanged(k));
    sliderH(k) = uislider(uf,...
                 "Limits",jl,"Value",val0,"MajorTicks",[],...
                 "Position",[20 baseY 440 3],...
                 "ValueChangingFcn",@(s,e)sliderChanging(k),...
                 "ValueChangedFcn",@(s,e)sliderChanged(k));
end

% flange transform table
lblY = ufH - 55*nJ - 10 - extraGap;
tblY = ufH - 55*nJ - 150 - extraGap;
uilabel(uf,"Text","Flange T (base frame)",...
        "Position",[20 lblY 200 15],"FontWeight","bold");
tbl = uitable(uf,"Position",[20 tblY 440 125],...
              "Data",goalT,"RowName",[],"ColumnName",[],"ColumnWidth","auto");

% Jog panel (lowered to Y=370)
jogBox = uipanel(uf,"Title","Jog EE pose","Position",[20 320 440 170]);
uilabel(jogBox,"Text","Step (mm):","Position",[10 100 65 20]);
stepField = uieditfield(jogBox,"numeric","Limits",[0.1 100],...
                        "Value",5,"Position",[80 100 60 22]);
stepR = deg2rad(5);
stepT = @()stepField.Value/1000;

btnCfg = {
  "X−",1,-1,5,60;  "X+",1,1,55,60;
  "Y−",2,-1,105,60; "Y+",2,1,155,60;
  "Z−",3,-1,205,60; "Z+",3,1,255,60;
  "α−",4,-1,5,20;  "α+",4,1,55,20;
  "β−",5,-1,105,20; "β+",5,1,155,20;
  "γ−",6,-1,205,20; "γ+",6,1,255,20
};
for i=1:size(btnCfg,1)
    uibutton(jogBox,"Text",btnCfg{i,1},...
             "Position",[btnCfg{i,4} btnCfg{i,5} 45 25],...
             "ButtonPushedFcn",@(~,~)jog(btnCfg{i,2},btnCfg{i,3}));
end

% Apply & Re-solve IK & Back buttons
btnRecalc = uibutton(uf,"Text","Re-solve IK",...
         "Position",[175 420 90 32],...
         "ButtonPushedFcn",@(~,~)recalcIK);
btnReset = uibutton(uf,"Text","Back to Actual",...
         "Position",[270 420 90 32],...
         "ButtonPushedFcn",@(~,~)resetToActual);
btnApply = uibutton(uf,"Text","Apply ➜ Snapshot",...
         "Position",[364 420 90 32],...
         "ButtonPushedFcn",@(~,~)copyPose);

% Trajectory panel (Y=160)
trajBox = uipanel(uf,"Title","Trajectory","Position",[20 160 440 150]);

% Row 1 buttons
btnStart = uibutton(trajBox,"Text","Set Start","Position",[10 90 130 28]);
btnGoal  = uibutton(trajBox,"Text","Set Goal","Enable","off","Position",[150 90 130 28]);
btnPTP   = uibutton(trajBox,"Text","PTP","Enable","off","Position",[290 90 130 28]);

% Row 2 buttons
btnLIN   = uibutton(trajBox,"Text","LIN Motion","Enable","off","Position",[10 50 130 28]);
btnPlay  = uibutton(trajBox,"Text","Play","Enable","off","Position",[150 50 130 28]);
btnSend  = uibutton(trajBox,"Text","Send Traj","Enable","off","Position",[290 50 130 28]);

% BEFORE any callbacks:
qStart = [];
qGoal = [];
ppTraj = [];
playTimer = [];

btnStart.ButtonPushedFcn = @(~,~)setStartPose();
btnGoal. ButtonPushedFcn = @(~,~)setGoalPose();
btnPTP.  ButtonPushedFcn = @(~,~)calcTraj();
btnLIN.  ButtonPushedFcn = @(~,~)calcLinTraj();
btnPlay. ButtonPushedFcn = @(~,~)playTraj();
btnSend. ButtonPushedFcn = @(~,~)sendTraj();

% Info label
infoT = uilabel(trajBox,"Text","–","Position",[10 10 410 22]);


% Saved Positions Panel (nice large buttons)
savedPanel = uipanel(uf,...
    "Title","Saved Positions","Position",[20 10 440 140]);

btnLabels = ["Save","Move","Remove"];
btnColors = ["#4CAF50","#2196F3","#F44336"];  % green, blue, red
btnHandles = gobjects(4,3);

for i = 1:4
    for j = 1:3
        xpos = 10 + (j-1)*140;
        ypos = 90 - (i-1)*27;  % more spacing
        btnHandles(i,j) = uibutton(savedPanel,...
            "Text", sprintf("%s %d",btnLabels(j),i),...
            "BackgroundColor", btnColors(j),...
            "FontSize",10,...
            "Position",[xpos ypos 130 23]);
    end
end

% Assign callbacks
for i = 1:4
    btnHandles(i,1).ButtonPushedFcn = @(~,~)savePose(i);
    btnHandles(i,2).ButtonPushedFcn = @(~,~)moveToPose(i);
    btnHandles(i,3).ButtonPushedFcn = @(~,~)removePose(i);
end

% Saved positions storage
savedPoses = cell(1,4);  % 4 slots



%% 7. Nested helper & callback functions
    function q = poseVec
        q = arrayfun(@(s)s.Value,sliderH).';
    end
    function initAxes(ax)
        axis(ax,"equal"); view(ax,125,25); grid(ax,"on");
    end
    function recolorSnapshot
        set(findobj(axSnap,"Type","patch"),"FaceColor",[.8 0 0],"EdgeColor","none");
    end
    function updateFlange(q)
        tbl.Data = round(getTransform(arm,q,"flange","base_link"),4);
    end
    function syncEdits
        for ii=1:nJ, editH(ii).Value = sliderH(ii).Value; end
    end

    function publishJS(q)
      if ~canPub, return; end
      js = ros2message("sensor_msgs/JointState");
      js.name     = jNames(:);
      js.position = q(:);
      t = rostime("now");
      js.header.stamp.sec     = int32(t.Sec);
      js.header.stamp.nanosec = uint32(t.Nsec);
      send(pubState,js);
      od = ros2message("std_msgs/Float64MultiArray");
      od.data = q(:)/(2*pi);
      send(pubODrive,od);
    end

    function drawLive(q,doPublish)
      cla(axLive);
      show(arm,q,"Parent",axLive,"Visuals","on","Frames","on");
      initAxes(axLive);
      updateFlange(q);
      if doPublish
          publishJS(q);
      end
      drawnow limitrate;
    end

    function sliderChanging(~,~)
      syncEdits();
      q = poseVec();
      drawLive(q,false);
      goalT = getTransform(arm,q,"flange","base_link");
    end
    function sliderChanged(~,~)
      syncEdits();
      q = poseVec();
      drawLive(q,false);
      goalT = getTransform(arm,q,"flange","base_link");
    end
    function editChanged(idx)
      sliderH(idx).Value = editH(idx).Value;
      q = poseVec();
      drawLive(q,false);
      goalT = getTransform(arm,q,"flange","base_link");
    end

    function jog(axisID,dir)
      if axisID<=3
        d=zeros(1,3); d(axisID)=dir*stepT();
        goalT = trvec2tform(d)*goalT;
      else
        axL     = eye(3);
        rotAxis = axL(axisID-3,:);
        R       = axang2tform([rotAxis, dir*stepR]);
        goalT   = goalT * R;
      end
      [q,~] = ik("flange",goalT,weights,poseVec());
      q = wrapToPi(q);
      for ii=1:nJ
        sliderH(ii).Value = max(lowerLim(ii),min(upperLim(ii),q(ii)));
        editH(ii).Value   = sliderH(ii).Value;
      end
      drawLive(q,false);
    end

    function copyPose
      q = poseVec(); 
      drawLive(q,true);
      cla(axSnap);
      show(arm,q,"Parent",axSnap,"Visuals","on","Frames","off");
      initAxes(axSnap); 
      recolorSnapshot();
      goalT = getTransform(arm,q,"flange","base_link");
    end

    function recalcIK
      qSeed = (rand(1,nJ)-.5)*2*pi;
      [q,~] = ik("flange",goalT,weights,qSeed);
      q = wrapToPi(q);
      for ii=1:nJ
        sliderH(ii).Value = max(lowerLim(ii),min(upperLim(ii),q(ii)));
        editH(ii).Value   = sliderH(ii).Value;
      end
      drawLive(q,false);
    end

    function resetToActual
        q = qZero;
        for ii=1:nJ
            sliderH(ii).Value = q(ii);
            editH(ii).Value = q(ii);
        end
        drawLive(q,false);  % Show reset pose (no publish)
        goalT = getTransform(arm,q,"flange","base_link");
        infoT.Text = "Reset to actual robot position";
    end

    function savePose(idx)
        savedPoses{idx} = poseVec();
        infoT.Text = sprintf("Saved pose %d", idx);
    end

    function moveToPose(idx)
        q = savedPoses{idx};
        if isempty(q)
            infoT.Text = sprintf("No pose saved in slot %d", idx);
            return;
        end
        for ii=1:nJ
            sliderH(ii).Value = q(ii);
            editH(ii).Value   = q(ii);
        end
        drawLive(q,false);  % simulate only
        goalT = getTransform(arm,q,"flange","base_link");
        infoT.Text = sprintf("Moved to pose %d (simulated)", idx);
    end

    function removePose(idx)
        savedPoses{idx} = [];
        infoT.Text = sprintf("Removed pose %d", idx);
    end

    function qOut = slerpQuat(q1, q2, t)
        % Simple SLERP between two quaternions (unit quaternions)
        dotProd = dot(q1, q2);
        if dotProd < 0.0
            q2 = -q2;
            dotProd = -dotProd;
        end
        DOT_THRESHOLD = 0.9995;
        if dotProd > DOT_THRESHOLD
            % Linear interpolation for very close quaternions
            qOut = (1 - t)*q1 + t*q2;
            qOut = qOut / norm(qOut);
            return
        end
        theta_0 = acos(dotProd);
        sin_theta_0 = sin(theta_0);
        theta = theta_0 * t;
        sin_theta = sin(theta);
        s0 = cos(theta) - dotProd * sin_theta / sin_theta_0;
        s1 = sin_theta / sin_theta_0;
        qOut = s0*q1 + s1*q2;
    end

    function calcLinTraj
        if isempty(qStart) || isempty(qGoal)
            infoT.Text = "Set start and goal first";
            return
        end
    
        Tmove = 4;
        Nsteps = 50;
        ts = linspace(0, Tmove, Nsteps);
    
        % Start and goal transforms
        T_start = getTransform(arm, qStart, "flange", "base_link");
        T_goal  = getTransform(arm, qGoal, "flange", "base_link");
    
        p_start = tform2trvec(T_start);
        p_goal  = tform2trvec(T_goal);
        R_start = tform2rotm(T_start);
        R_goal  = tform2rotm(T_goal);
    
        q_start = rotm2quat(R_start);  % [w x y z]
        q_goal  = rotm2quat(R_goal);
    
        qSolutions = zeros(Nsteps, nJ);
        ikFails = false;
    
        for i = 1:Nsteps
            alpha = (i - 1) / (Nsteps - 1);
    
            % Linear position
            p_i = (1 - alpha)*p_start + alpha*p_goal;
    
            % SLERP quaternion
            q_i = slerpQuat(q_start, q_goal, alpha);
            R_i = quat2rotm(q_i);
    
            T_i = trvec2tform(p_i) * rotm2tform(R_i);
    
            [qStep, info] = ik("flange", T_i, weights, qGoal);
            if info.Status ~= "success"
                ikFails = true;
            end
            qSolutions(i,:) = wrapToPi(qStep);
        end
    
        if ikFails
            infoT.Text = "Warning: Some IK solutions failed";
        else
            infoT.Text = sprintf("Linear Cartesian trajectory ready (%.1f s)", Tmove);
        end
    
        ppTraj = pchip(ts, qSolutions.');
    
        btnPlay.Enable = "on";
        btnSend.Enable = "on";
    end



%% 8. Trajectory helpers
    function setStartPose
      qStart = poseVec();
      btnGoal.Enable = "on";
      infoT.Text     = "Start ✓";
    end
    function setGoalPose
      if isempty(qStart)
        infoT.Text = "Set start first";
        return
      end
      qGoal = poseVec();
      btnPTP.Enable = "on";
      btnLIN.Enable = "on";
      infoT.Text     = "Goal ✓";
    end
    function calcTraj
      if isempty(qStart)||isempty(qGoal), return, end
      Tmove  = 4;
      ppTraj = spline([0 Tmove],[qStart; qGoal].');
      btnPlay.Enable = "on";
      btnSend.Enable = "on";
      infoT.Text     = sprintf("Ready (%.1f s)",Tmove);
    end
    function playTraj
      if isempty(ppTraj), infoT.Text="No trajectory"; return, end
      if ~isempty(playTimer)&&isvalid(playTimer)
        stop(playTimer); delete(playTimer);
      end
      t0 = tic; btnPlay.Enable="off";
      playTimer = timer( ...
        "ExecutionMode","fixedRate", ...
        "Period",1/30, ...
        "TimerFcn",@(~,~)stepPlay, ...
        "StopFcn",@(~,~) set(infoT,"Text","Done"));
      start(playTimer);
      function stepPlay(~,~)
        tt = toc(t0);
        if tt>ppTraj.breaks(end)
          stop(playTimer); btnPlay.Enable="on"; return
        end
        q = ppval(ppTraj,tt).';
        for ii=1:nJ
          sliderH(ii).Value = q(ii);
          editH(ii).Value   = q(ii);
        end
        drawLive(q,false);
      end
    end
    function sendTraj
      if isempty(ppTraj)||~canPub
        infoT.Text="Cannot send trajectory"; return
      end
      Ts = 0.02;
      ts = 0:Ts:ppTraj.breaks(end);
      qS = ppval(ppTraj,ts).';
      N  = numel(ts);

      msg = ros2message("trajectory_msgs/JointTrajectory");
      msg.joint_names = jNames(:);
      msg.points = repmat(ros2message("trajectory_msgs/JointTrajectoryPoint"),1,N+1);

      msg.points(1).positions = qStart;
      msg.points(1).time_from_start.sec     = int32(0);
      msg.points(1).time_from_start.nanosec = uint32(0);

      for ii=1:N
        msg.points(ii+1).positions = qS(ii,:);
        msg.points(ii+1).time_from_start.sec     = int32(floor(ts(ii)));
        msg.points(ii+1).time_from_start.nanosec = uint32(rem(ts(ii),1)*1e9);
      end

      cla(axSnap);
      show(arm,qGoal,"Parent",axSnap,"Visuals","on","Frames","off");
      initAxes(axSnap); 
      recolorSnapshot();

      send(pubTraj,msg);
      infoT.Text="Trajectory sent to ROS.";
    end
end
