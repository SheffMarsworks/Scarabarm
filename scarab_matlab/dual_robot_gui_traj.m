function dual_robot_gui_traj
% Dual-Robot GUI   ·   jog, snapshot, start/goal trajectory   ·   ROS 2 optional
% Tested R2021b + ROS 2 Toolbox R2023a

%% 1. Robot -------------------------------------------------------------
arm = importrobot("full_scarab_arm.urdf","DataFormat","row");

for b = ["inner_link_x","inner_link_y","outer_link_x","finger_x","outer_link_y","finger_y"]
    j = arm.getBody(b).Joint;
    fj = rigidBodyJoint(j.Name,"fixed");
    setFixedTransform(fj, j.JointToParentTransform/j.ChildToJointTransform);
    replaceJoint(arm,b,fj);
end

jNames = {};
for k = 1:numel(arm.Bodies)
    if arm.Bodies{k}.Joint.Type ~= "fixed"
        jNames{end+1} = arm.Bodies{k}.Joint.Name; %#ok<SAGROW>
    end
end
nJ = numel(jNames);

% hard-coded limits for continuous joints
limTbl = {
    "joint_1_to_joint_2", [-pi     pi];
    "joint_2_to_link_2",  [-2.5708 2.5708];
    "link_2_to_joint_3",  [-2.7925 2.7925];
    "joint_4_to_joint_5", [-pi     pi];
    "joint_5_to_joint_6", [-pi     pi];
    "joint_6_to_flange",  [-2*pi   2*pi];
};
for i = 1:size(limTbl,1)
    idx = find(cellfun(@(b) strcmp(arm.getBody(b).Joint.Name,limTbl{i,1}),arm.BodyNames),1);
    if isempty(idx), continue, end
    jb = arm.getBody(arm.BodyNames{idx}).Joint;
    if jb.Type == "continuous", jb.Type = "revolute"; end
    jb.PositionLimits = limTbl{i,2};
    replaceJoint(arm, arm.BodyNames{idx}, jb);
end

%% 2. ROS 2 publishers --------------------------------------------------
canPub = true; doPublish = false;
try
    node     = ros2node("/matlab_gui");
    pubState = ros2publisher(node,"/joint_states","sensor_msgs/JointState");
    pubTraj  = ros2publisher(node,"/command_trajectory","trajectory_msgs/JointTrajectory");
catch
    warning("ROS 2 unavailable – publishing disabled.");
    canPub = false; pubState = []; pubTraj = [];
end

%% 3. IK ----------------------------------------------------------------
ik       = inverseKinematics("RigidBodyTree",arm);
weights  = [.25 .25 .25 1 1 1];
qZero    = zeros(1,nJ);
goalT    = getTransform(arm,qZero,"flange","base_link");

%% 4. Two viewports -----------------------------------------------------
fView = figure("Name","LIVE (gray)  |  SNAPSHOT (red)","Position",[320 40 930 560]);
axLive = subplot(1,2,1,Parent=fView); title(axLive,"LIVE  (gray)");
axSnap = subplot(1,2,2,Parent=fView); title(axSnap,"SNAPSHOT  (red)");
initAxes(axLive); initAxes(axSnap);
show(arm,qZero,"Parent",axLive,"Visuals","on","Frames","on");
show(arm,qZero,"Parent",axSnap,"Visuals","on","Frames","off"); recolorSnapshot();

%% 5. Control window ----------------------------------------------------
ctrlW = 480; extraGap = 50; ufH = 60 + 55*nJ + 460 + extraGap;
uf = uifigure("Name","Joint Controls + Trajectory","Position",[40 80 ctrlW ufH]);

sliderH=gobjects(nJ,1); editH=gobjects(nJ,1); lowerLim=zeros(1,nJ); upperLim=zeros(1,nJ);

for k = 1:nJ
    baseY = ufH - 60 - 55*(k-1);
    bodyIdx = find(cellfun(@(b) strcmp(arm.getBody(b).Joint.Name,jNames{k}),arm.BodyNames),1);
    jl = arm.getBody(arm.BodyNames{bodyIdx}).Joint.PositionLimits;
    if numel(jl)~=2 || jl(1)==jl(2) || any(~isfinite(jl)), jl = [-pi pi]; end
    jl = sort(jl); lowerLim(k)=jl(1); upperLim(k)=jl(2); val0=mean(jl);
    uilabel(uf,"Text",jNames{k},"Position",[20 baseY+18 150 15]);
    editH(k) = uieditfield(uf,"numeric","Limits",jl,"Value",val0,"Position",[380 baseY+8 80 22], ...
                 "ValueChangedFcn",@(~,~)editChanged(k));
    sliderH(k)= uislider(uf,"Limits",jl,"Value",val0,"MajorTicks",[], ...
                 "Position",[20 baseY 440 3], ...
                 "ValueChangingFcn",@(~,~)sliderChanging(k), ...
                 "ValueChangedFcn", @(~,~)sliderChanged(k));
end

% 4×4 flange transform table
lblY = ufH - 55*nJ - 10 - extraGap; tblY = ufH - 55*nJ - 150 - extraGap;
uilabel(uf,"Text","Flange T (base frame)","Position",[20 lblY 200 15],"FontWeight","bold");
tbl = uitable(uf,"Position",[20 tblY 440 125],"Data",goalT,"RowName",[],"ColumnName",[],"ColumnWidth","auto");

%% 5b. Jog panel --------------------------------------------------------
jogBox = uipanel(uf,"Title","Jog EE pose","Position",[20 210 440 150]);
uilabel(jogBox,"Text","Step (mm):","Position",[10 100 65 20]);
stepField = uieditfield(jogBox,"numeric","Limits",[0.1 100],"Value",5,"Position",[80 100 60 22]);
stepR = deg2rad(5);  stepT = @()stepField.Value/1000;
btnCfg = {"X−",1,-1,5,60;"X+",1,1,55,60;"Y−",2,-1,105,60;"Y+",2,1,155,60;"Z−",3,-1,205,60;"Z+",3,1,255,60;"α−",4,-1,5,20;"α+",4,1,55,20;"β−",5,-1,105,20;"β+",5,1,155,20;"γ−",6,-1,205,20;"γ+",6,1,255,20};
for i = 1:size(btnCfg,1)
    uibutton(jogBox,"Text",btnCfg{i,1},"Position",[btnCfg{i,4} btnCfg{i,5} 45 25], ...
             "ButtonPushedFcn",@(~,~)jog(btnCfg{i,2},btnCfg{i,3}));
end

% Apply / IK buttons (20 px gap above Trajectory panel)
uibutton(uf,"Text","Apply ➜ Snapshot","Position",[70 350 140 32],"ButtonPushedFcn",@(~,~)copyPose);
uibutton(uf,"Text","Re-solve IK","Position",[270 350 140 32],"ButtonPushedFcn",@(~,~)recalcIK);

%% 5c. Trajectory panel -------------------------------------------------
trajBox = uipanel(uf,"Title","Trajectory","Position",[20 40 440 150]);

btnStart = uibutton(trajBox,"Text","Set Start","Position",[10 90 80 28]);
btnGoal  = uibutton(trajBox,"Text","Set Goal","Enable","off","Position",[100 90 80 28]);
btnCalc  = uibutton(trajBox,"Text","Calc","Enable","off","Position",[190 90 80 28]);
btnPlay  = uibutton(trajBox,"Text","Play","Enable","off","Position",[280 90 80 28]);
btnSend  = uibutton(trajBox,"Text","Send Traj","Enable","off","Position",[370 90 60 28]);
infoT    = uilabel(trajBox,"Text","–","Position",[10 40 410 22]);

qStart=[]; qGoal=[]; ppTraj=[]; playTimer=[];

btnStart.ButtonPushedFcn=@(~,~)setStartPose();
btnGoal. ButtonPushedFcn=@(~,~)setGoalPose();
btnCalc. ButtonPushedFcn=@(~,~)calcTraj();
btnPlay. ButtonPushedFcn=@(~,~)playTraj();
btnSend.ButtonPushedFcn = @(~,~)sendTraj();

%% 6. Small helpers -----------------------------------------------------
    function q=poseVec, q=arrayfun(@(s)s.Value,sliderH)'; end
    function initAxes(ax), axis(ax,"equal"), view(ax,125,25), grid(ax,"on"), end
    function recolorSnapshot, set(findobj(axSnap,"Type","patch"),"FaceColor",[.8 0 0],"EdgeColor","none"), end
    function updateFlange(q), tbl.Data=round(getTransform(arm,q,"flange","base_link"),4); end
    function syncEdits, for ii=1:nJ, editH(ii).Value=sliderH(ii).Value; end, end

    function publishJS(q)
        if ~canPub || ~doPublish, return, end
        msg=ros2message("sensor_msgs/JointState");
        msg.name=jNames(:); msg.position=q(:);
        t=rostime("now"); msg.header.stamp.sec=int32(t.Sec); msg.header.stamp.nanosec=uint32(t.Nsec);
        send(pubState,msg);
    end

%% 7. GUI draw & callbacks ---------------------------------------------
    function drawLive(q)
        cla(axLive);
        show(arm,q,"Parent",axLive,"Visuals","on","Frames","on");
        initAxes(axLive); updateFlange(q); drawnow limitrate;
    end
    function sliderChanging(~,~), syncEdits(); drawLive(poseVec()); end
    function sliderChanged(~,~),  syncEdits(); drawLive(poseVec()); end
    function editChanged(idx), sliderH(idx).Value=editH(idx).Value; drawLive(poseVec()); end

    function jog(axisID,dir)
        if axisID<=3
            d=zeros(1,3); d(axisID)=dir*stepT(); goalT=trvec2tform(d)*goalT;
        else
            axL=eye(3); goalT=goalT*axang2tform([axL(axisID-3,:) dir*stepR]);
        end
        [q,~]=ik("flange",goalT,weights,poseVec()); q=wrapToPi(q);
        for ii=1:nJ, sliderH(ii).Value=max(lowerLim(ii),min(upperLim(ii),q(ii))); editH(ii).Value=sliderH(ii).Value; end
        drawLive(q);
    end

    function copyPose
        q=poseVec(); drawLive(q);
        cla(axSnap); show(arm,q,"Parent",axSnap,"Visuals","on","Frames","off"); initAxes(axSnap); recolorSnapshot();
        goalT=getTransform(arm,q,"flange","base_link"); doPublish=true; publishJS(q);
    end
    function recalcIK
        qSeed=(rand(1,nJ)-.5)*2*pi; [q,~]=ik("flange",goalT,weights,qSeed); q=wrapToPi(q);
        for ii=1:nJ, sliderH(ii).Value=max(lowerLim(ii),min(upperLim(ii),q(ii))); editH(ii).Value=sliderH(ii).Value; end
        drawLive(q);
    end

%% 8. Trajectory helpers ------------------------------------------------
    function setStartPose
        qStart=poseVec(); btnGoal.Enable="on"; infoT.Text="Start ✓";
    end
    function setGoalPose
        qGoal=poseVec(); if isempty(qStart), infoT.Text="Set start first"; return, end
        btnCalc.Enable="on"; infoT.Text="Goal ✓";
    end
    function calcTraj
        if isempty(qStart)||isempty(qGoal), return, end
        Tmove=4;                                       % seconds
        ppTraj=spline([0 Tmove], [qStart; qGoal].');   % cubic spline
        btnPlay.Enable="on"; infoT.Text=sprintf("Ready (%.1f s)",Tmove); 
        btnSend.Enable = "on"; infoT.Text = sprintf("Ready (%.1f s)",Tmove);
    end
    function playTraj
        if isempty(ppTraj), infoT.Text="No trajectory"; return, end
        if ~isempty(playTimer)&&isvalid(playTimer), stop(playTimer), delete(playTimer), end
        t0=tic; btnPlay.Enable="off";
        playTimer=timer("ExecutionMode","fixedRate","Period",1/30, ...
                        "TimerFcn",@stepPlay, ...
                        "StopFcn",@(~,~)set(infoT,"Text","Done"));
        start(playTimer);
        function stepPlay(~,~)          % <- accept src,event
            tt=toc(t0);
            if tt>ppTraj.breaks(end)
                stop(playTimer); btnPlay.Enable="on"; return
            end
            q=ppval(ppTraj,tt).';
            for ii=1:nJ, sliderH(ii).Value=q(ii); editH(ii).Value=q(ii); end
            drawLive(q); publishJS(q);
        end
    end
    %––– make the red model show an arbitrary joint vector ––––––––––
    function setSnapshot(q)
        cla(axSnap);
        show(arm,q,"Parent",axSnap,"Visuals","on","Frames","off");
        initAxes(axSnap);
        recolorSnapshot();
    end
    function sendTraj
        %––– Guard clauses ––––––––––––––––––––––––––––––––––––––––––––––
        if isempty(ppTraj)
            infoT.Text = "No trajectory to send.";
            return
        end
        if ~(canPub && ~isempty(pubTraj))
            infoT.Text = "ROS publisher unavailable.";
            return
        end

        %––– Build JointTrajectory ––––––––––––––––––––––––––––––––––––––
        Tsamp  = 0.02;                    % sample spacing [s]
        ts     = 0:Tsamp:ppTraj.breaks(end);
        qSamp  = ppval(ppTraj,ts).';      % N×nJ  (spline samples)

        % point 0  = start pose (what you captured with Set Start)
        q0      = qStart;                 % already stored earlier
        N       = numel(ts);              % spline sample count
        msg     = ros2message("trajectory_msgs/JointTrajectory");
        msg.joint_names = jNames(:);
        msg.points      = repmat(ros2message("trajectory_msgs/JointTrajectoryPoint"),1,N+1);

        % fill start point (index 1)
        msg.points(1).positions         = q0;
        msg.points(1).time_from_start.sec     = int32(0);
        msg.points(1).time_from_start.nanosec = uint32(0);

        % fill spline samples (index 2…N+1)
        for ii = 1:N
            msg.points(ii+1).positions         = qSamp(ii,:);
            msg.points(ii+1).time_from_start.sec     = int32(floor(ts(ii)));
            msg.points(ii+1).time_from_start.nanosec = uint32(rem(ts(ii),1)*1e9);
        end

        setSnapshot(qGoal);          % update red robot
        
        %––– Publish once –––––––––––––––––––––––––––––––––––––––––––––––
        send(pubTraj,msg);           % publish to ROS 2
        infoT.Text = "Trajectory sent to ROS.";

        % optional: mirror motion in the GUI
        playTraj();
    end
end
