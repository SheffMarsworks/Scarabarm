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
    "joint_4_to_joint_5", [-pi      pi];
    "joint_5_to_joint_6", [-pi      pi];
    "joint_6_to_flange",  [-2*pi   2*pi];
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
weights = [.25 .25 .25 1 1 1];
qZero   = zeros(1,nJ);
goalT   = getTransform(arm,qZero,"flange","base_link");

%% 5. Two viewports
fView  = figure("Name","LIVE (gray)  |  SNAPSHOT (red)","Position",[320 40 930 560]);
axLive = subplot(1,2,1,Parent=fView); title(axLive,"LIVE  (gray)"); initAxes(axLive);
axSnap = subplot(1,2,2,Parent=fView); title(axSnap,"SNAPSHOT  (red)"); initAxes(axSnap);
show(arm,qZero,"Parent",axLive,"Visuals","on","Frames","on");
show(arm,qZero,"Parent",axSnap,"Visuals","on","Frames","off"); recolorSnapshot();

%% 6. Control window
ctrlW = 480; extraGap = 50; ufH = 60 + 55*nJ + 460 + extraGap;
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

% jog panel
jogBox = uipanel(uf,"Title","Jog EE pose","Position",[20 210 440 150]);
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

% Apply & Re-solve IK
uibutton(uf,"Text","Apply ➜ Snapshot",...
         "Position",[70 350 140 32],...
         "ButtonPushedFcn",@(~,~)copyPose);
uibutton(uf,"Text","Re-solve IK",...
         "Position",[270 350 140 32],...
         "ButtonPushedFcn",@(~,~)recalcIK);

% Trajectory panel
trajBox = uipanel(uf,"Title","Trajectory","Position",[20 40 440 150]);
btnStart = uibutton(trajBox,"Text","Set Start","Position",[10 90 80 28]);
btnGoal  = uibutton(trajBox,"Text","Set Goal","Enable","off","Position",[100 90 80 28]);
btnCalc  = uibutton(trajBox,"Text","Calc","Enable","off","Position",[190 90 80 28]);
btnPlay  = uibutton(trajBox,"Text","Play","Enable","off","Position",[280 90 80 28]);
btnSend  = uibutton(trajBox,"Text","Send Traj","Enable","off","Position",[370 90 60 28]);
infoT    = uilabel(trajBox,"Text","–","Position",[10 40 410 22]);

% **declare these BEFORE callbacks** so they’re in scope
qStart=[]; qGoal=[]; ppTraj=[]; playTimer=[];

btnStart.ButtonPushedFcn = @(~,~)setStartPose();
btnGoal. ButtonPushedFcn = @(~,~)setGoalPose();
btnCalc. ButtonPushedFcn = @(~,~)calcTraj();
btnPlay. ButtonPushedFcn = @(~,~)playTraj();
btnSend.ButtonPushedFcn = @(~,~)sendTraj();

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
      % joint_states
      js = ros2message("sensor_msgs/JointState");
      js.name     = jNames(:);
      js.position = q(:);
      t = rostime("now");
      js.header.stamp.sec     = int32(t.Sec);
      js.header.stamp.nanosec = uint32(t.Nsec);
      send(pubState,js);
      % ODrive position command (revs)
      od = ros2message("std_msgs/Float64MultiArray");
      od.data = q(:)/(2*pi);
      send(pubODrive,od);
    end

    function drawLive(q)
      cla(axLive);
      show(arm,q,"Parent",axLive,"Visuals","on","Frames","on");
      initAxes(axLive);
      updateFlange(q);
      publishJS(q);
      drawnow limitrate;
    end

    function sliderChanging(~,~)
      syncEdits(); q=poseVec(); drawLive(q);
      goalT = getTransform(arm,q,"flange","base_link");
    end
    function sliderChanged(~,~)
      syncEdits(); q=poseVec(); drawLive(q);
      goalT = getTransform(arm,q,"flange","base_link");
    end
    function editChanged(idx)
      sliderH(idx).Value = editH(idx).Value;
      q=poseVec(); drawLive(q);
      goalT = getTransform(arm,q,"flange","base_link");
    end

    function jog(axisID,dir)
      if axisID<=3
        d=zeros(1,3); d(axisID)=dir*stepT();
        goalT = trvec2tform(d)*goalT;
      else
        % **fixed**: split eye(3) indexing into two lines
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
      drawLive(q);
    end

    function copyPose
      q=poseVec(); 
      drawLive(q);  % already calls publishJS(q)
      % publishJS(q);  % (optional) redundant, can skip
      cla(axSnap);
      show(arm,q,"Parent",axSnap,"Visuals","on","Frames","off");
      initAxes(axSnap); recolorSnapshot();
      goalT=getTransform(arm,q,"flange","base_link");
    end

    function recalcIK
      qSeed = (rand(1,nJ)-.5)*2*pi;
      [q,~] = ik("flange",goalT,weights,qSeed);
      q = wrapToPi(q);
      for ii=1:nJ
        sliderH(ii).Value = max(lowerLim(ii),min(upperLim(ii),q(ii)));
        editH(ii).Value   = sliderH(ii).Value;
      end
      drawLive(q);
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
      btnCalc.Enable = "on";
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
        drawLive(q);
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

      % start point
      msg.points(1).positions = qStart;
      msg.points(1).time_from_start.sec     = int32(0);
      msg.points(1).time_from_start.nanosec = uint32(0);

      % spline samples
      for ii=1:N
        msg.points(ii+1).positions = qS(ii,:);
        msg.points(ii+1).time_from_start.sec     = int32(floor(ts(ii)));
        msg.points(ii+1).time_from_start.nanosec = uint32(rem(ts(ii),1)*1e9);
      end

      % update snapshot pane
      cla(axSnap);
      show(arm,qGoal,"Parent",axSnap,"Visuals","on","Frames","off");
      initAxes(axSnap); recolorSnapshot();

      send(pubTraj,msg);
      infoT.Text="Trajectory sent to ROS.";
    end
end
