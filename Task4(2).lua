function fopen()
	file=io.open("G:/log.txt",'a')
	io.output(file)
end

function fprint(str)
	str=str..'\n'
	io.write(str)
end

function fclose()
	io.close()
end

function globalInit()
	--simGetHandleVehicle获得机器人句柄，为根类
	handleTipPlatform = robot.simGetHandleVehicle()

  	--simGetHandleJointTurn获得各轮转向铰接句柄
	handleJointWheelLF0 = robot.simGetHandleJointTurnLeftFront()
	handleJointWheelRB0 = robot.simGetHandleJointTurnRightBack()
	handleJointWheelRF0 = robot.simGetHandleJointTurnRightFront()
	handleJointWheelLB0 = robot.simGetHandleJointTurnLeftBack()
	--simGetHandleJointTurn获得各轮运动铰接句柄
	handleJointWheelLF = robot.simGetHandleJointWheelLeftFront()
	handleJointWheelRF = robot.simGetHandleJointWheelRightFront()
	handleJointWheelLB = robot.simGetHandleJointWheelLeftBack()
	handleJointWheelRB = robot.simGetHandleJointWheelRightBack()
	
	--任务二全局信息
	--当前物体
	ObjNumintask2=1
	--用于检测是否为新物体
	ObjcompareNum=1
	--用于记录已搬运成功物体
	ObjhasbeenMove=0
	
	task2ObjPos={{0,0,0},{0,0,0},{0,0,0},{0,0,0}}
	task2ObjRot={{0,0,0},{0,0,0},{0,0,0},{0,0,0}}
	task2Objtype={0,0,0,0}
	task2ObjtypeCount={0,0,0}
	
	Objtype='chest'
end

function tprint()
	local t=os.date("%Y-%m-%d  %H:%M:%S\n",os.time())
	str=t
	fprint(str)
	simExtPrintInfo(str)
end

function posprint()
	local pos = {0,0,0}
	pos = robot.simGetObjectPosition(handleTipPlatform,pos)
	local rot = {0,0,0}
    rot = robot.simGetObjectOrientation(handleTipPlatform,rot)
	simExtPrintInfo('-----globalPos-----')
	simExtPrintInfo('posX\t'..pos[1]..'\tposY\t'..pos[2]..'\tYaw\t'..rot[3])
	simExtPrintInfo('-------------------')
	fprint('posX\t'..pos[1]..'\t'..'posY\t'..pos[2]..'Yaw \t'..rot[3]..'\n')
end

function log(...)
	local arg={...}
	length=table.getn(arg)
	func,funcname,one,two,three,four,five,six,seven=arg[1],arg[2],arg[3],arg[4],arg[5],arg[6],arg[7],arg[8],arg[9]
	statue,err=pcall(func,one,two,three,four,five,six,seven)
	if(not statue)
	then
		fprint('function  '..funcname..'  error')
		fprint(err)
		fprint(debug.traceback()..'\n')
	end
end

function cleanlog()
	file=io.open("G:/log.txt",'w+')
	io.close(file)
end

function delay(t)
	local initT=os.time()
	local dt=0
	repeat
		dt=os.time()-initT
	until(dt==t or dt>t)
	simExtPrintInfo('delay for '..t..' s')
end

function stopMove()
	robot.simSetJointTargetVelocity(handleJointWheelLF, 0)
    robot.simSetJointTargetVelocity(handleJointWheelRF, 0)
    robot.simSetJointTargetVelocity(handleJointWheelLB, 0)
    robot.simSetJointTargetVelocity(handleJointWheelRB, 0)
end

function getWheelRelativeAngle()
	local LFangle={0,0,0}
	local RFangle={0,0,0}
	local LBangle={0,0,0}
	local RBangle={0,0,0}
	
	robot.simSetJointTargetPosition(handleJointWheelLF0, -47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelRB0, -47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelRF0, 47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelLB0, 47.38/180.0*math.pi)
	
	LFangle=simGetJointPosition(handleJointWheelLF0,LFangle)
	LBangle=simGetJointPosition(handleJointWheelRB0,LBangle)
	RFangle=simGetJointPosition(handleJointWheelRF0,RFangle)
	RBangle=simGetJointPosition(handleJointWheelLB0,RBangle)
	
	return LFangle,LBangle,RFangle,RBangle
end

--以固定速度移动一定距离(moveVel为正则向前，为负向后)
function GoStraight(moveVel,length)
	handleTipPlatform = robot.simGetHandleVehicle()

	handleJointWheelLF0 = robot.simGetHandleJointTurnLeftFront()
	handleJointWheelRB0 = robot.simGetHandleJointTurnRightBack()
	handleJointWheelRF0 = robot.simGetHandleJointTurnRightFront()
	handleJointWheelLB0 = robot.simGetHandleJointTurnLeftBack()

	handleJointWheelLF = robot.simGetHandleJointWheelLeftFront()
	handleJointWheelRF = robot.simGetHandleJointWheelRightFront()
	handleJointWheelLB = robot.simGetHandleJointWheelLeftBack()
	handleJointWheelRB = robot.simGetHandleJointWheelRightBack()

	robot.simSetJointTargetPosition(handleJointWheelLF0, 0)
    robot.simSetJointTargetPosition(handleJointWheelRB0, 0)
    robot.simSetJointTargetPosition(handleJointWheelRF0, 0)
    robot.simSetJointTargetPosition(handleJointWheelLB0, 0)

	local initPos = {0,0,0}
	initPos = robot.simGetObjectPosition(handleTipPlatform, initPos)
	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform,initRot)

	local LFang=0
	local LBang=0
	local RFang=0
	local RBang=0

	local LFvel=moveVel
	local LBvel=moveVel
	local RFvel=moveVel
	local RBvel=moveVel
	local distance=0
	local arrived = false
	local newPos = {0,0,0}
	local newRot = {0,0,0}

	--fprint('***********GoStraight***********')
	--fprint('\tlength'..length)

    repeat

    	--simSetJointTargetVelocity设定某句柄转动速度，即轮转速
		robot.simSetJointTargetVelocity(handleJointWheelLF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelLB, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRB, moveVel)
		--一帧的运动结束后重新测位置并保存

		newPos = robot.simGetObjectPosition(handleTipPlatform, newPos)

		newRot = robot.simGetObjectOrientation(handleTipPlatform,newRot)

		--fprint("newPos="..' '..newPos[1]..' '..newPos[2]..' '..newRot[3]..' '..LFvel..' '..LBvel..' '..RFvel..' '..RBvel..' '..LFang..' '..LBang..' '..RFang..' '..RBang)

   		--检测是否到达
		distance = {newPos[1]-initPos[1], newPos[2]-initPos[2], newPos[3]-initPos[3] }
		distance = math.sqrt(distance[1]*distance[1]+ distance[2]*distance[2]+ distance[3]*distance[3])
		arrived = false

		if(math.abs(math.abs(length)-distance) < 0.02) then
			arrived = true
		end

    until (arrived == true)

	--fprint('******************************\n\n')
end
function GoHorizontal(moveVel,length)
	handleTipPlatform = robot.simGetHandleVehicle()

	handleJointWheelLF0 = robot.simGetHandleJointTurnLeftFront()
	handleJointWheelRB0 = robot.simGetHandleJointTurnRightBack()
	handleJointWheelRF0 = robot.simGetHandleJointTurnRightFront()
	handleJointWheelLB0 = robot.simGetHandleJointTurnLeftBack()

	handleJointWheelLF = robot.simGetHandleJointWheelLeftFront()
	handleJointWheelRF = robot.simGetHandleJointWheelRightFront()
	handleJointWheelLB = robot.simGetHandleJointWheelLeftBack()
	handleJointWheelRB = robot.simGetHandleJointWheelRightBack()

	robot.simSetJointTargetPosition(handleJointWheelLF0, math.pi/2)
    robot.simSetJointTargetPosition(handleJointWheelRB0, math.pi/2)
    robot.simSetJointTargetPosition(handleJointWheelRF0, math.pi/2)
    robot.simSetJointTargetPosition(handleJointWheelLB0, math.pi/2)

	local initPos = {0,0,0}
	initPos = robot.simGetObjectPosition(handleTipPlatform, initPos)
	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform,initRot)

	local LFang=0
	local LBang=0
	local RFang=0
	local RBang=0

	local LFvel=moveVel
	local LBvel=moveVel
	local RFvel=moveVel
	local RBvel=moveVel
	local distance=0
	local arrived = false
	local newPos = {0,0,0}
	local newRot = {0,0,0}

	--fprint('***********GoStraight***********')
	--fprint('\tlength'..length)

    repeat

    	--simSetJointTargetVelocity设定某句柄转动速度，即轮转速
		robot.simSetJointTargetVelocity(handleJointWheelLF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelLB, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRB, moveVel)
		--一帧的运动结束后重新测位置并保存

		newPos = robot.simGetObjectPosition(handleTipPlatform, newPos)

		newRot = robot.simGetObjectOrientation(handleTipPlatform,newRot)

		--fprint("newPos="..' '..newPos[1]..' '..newPos[2]..' '..newRot[3]..' '..LFvel..' '..LBvel..' '..RFvel..' '..RBvel..' '..LFang..' '..LBang..' '..RFang..' '..RBang)

   		--检测是否到达
		distance = {newPos[1]-initPos[1], newPos[2]-initPos[2], newPos[3]-initPos[3] }
		distance = math.sqrt(distance[1]*distance[1]+ distance[2]*distance[2]+ distance[3]*distance[3])
		arrived = false

		if(math.abs(math.abs(length)-distance) < 0.03) then
			arrived = true
		end

    until (arrived == true)

	--fprint('******************************\n\n')
end
--精确运动到给定位置(全局坐标系)，其姿态不变,传入参数为(目标点X,目标点Y)
--定位精度为1cm(可根据需求调整)
--根据距离计算移动速度
function moveToTarget(targetX,targetY,mode,ifinfo)

	local curPos = {0,0,0}
	local curRot = {0,0,0}
	curPos = robot.simGetObjectPosition(handleTipPlatform, curPos)

	local dX=targetX-curPos[1]
	local dY=targetY-curPos[2]

	local tempAngle=0
	local angle=0
	local distance=0
	local moveVel=0

	local P=10
	local I=1
	local D=1
	local moveState=1
	local moveDirection=1
	
	if ifinfo==1 then
		simExtPrintInfo('-----moveToTarget-----')
		simExtPrintInfo('\tinitPos  \tX: '..curPos[1]..'\tY: '..curPos[2])
		simExtPrintInfo('\ttargetPos\tX: '..targetX..'\tY: '..targetY)
	end
	
	repeat
		curPos = robot.simGetObjectPosition(handleTipPlatform, curPos)
		curRot = robot.simGetObjectOrientation(handleTipPlatform,curRot)
		
		dX=targetX-curPos[1]
		dY=targetY-curPos[2]
		tempAngle=math.atan2(-dX,dY)-curRot[3]
		distance=math.sqrt(dX*dX+dY*dY)
		
		if tempAngle>math.pi then
			tempAngle=tempAngle-2*math.pi
		elseif tempAngle<-math.pi then
			tempAngle=tempAngle+2*math.pi
		end
		Lastangle=simGetJointPosition(handleJointWheelLF0,Lastangle)

		
		--simExtPrintInfo('tempAngle1  '..tempAngle..'  Lastangle  '..Lastangle)
		
		local dAngle=Lastangle-tempAngle
		
		if math.abs(dAngle)<math.pi/2 or math.abs(dAngle)>math.pi*3/2 then
			tempAngle=tempAngle
			moveState=1*moveState
			moveDirection=moveDirection
		else
			tempAngle=tempAngle+math.pi	
			if tempAngle>math.pi then
				tempAngle=tempAngle-2*math.pi
			elseif tempAngle<-math.pi then
				tempAngle=mathAngle+2*math.pi
			end
			moveState=-1*moveDirection
		end
		robot.simSetJointTargetPosition(handleJointWheelLF0, tempAngle)
		robot.simSetJointTargetPosition(handleJointWheelRB0, tempAngle)
		robot.simSetJointTargetPosition(handleJointWheelRF0, tempAngle)
		robot.simSetJointTargetPosition(handleJointWheelLB0, tempAngle)

		moveVel=P*distance
		if moveVel<5  then
			moveVel=5
		elseif moveVel>15 then
			moveVel=15
		end

		if mode==2 then
			moveVel=3
		end

		robot.simSetJointTargetVelocity(handleJointWheelLF, moveVel*moveState)
		robot.simSetJointTargetVelocity(handleJointWheelRF, moveVel*moveState)
		robot.simSetJointTargetVelocity(handleJointWheelLB, moveVel*moveState)
		robot.simSetJointTargetVelocity(handleJointWheelRB, moveVel*moveState)

	until(math.abs(curPos[1]-targetX)<0.01 and math.abs(curPos[2]-targetY)<0.01)
	stopMove()
	if ifinfo==1 then
		simExtPrintInfo('-----Arrive-----\n')
	end
end

function rotateToTarget(targetYaw,mode,ifinfo)

	if targetYaw>math.pi then
		targetYaw=targetYaw-2*math.pi
	end
	if targetYaw<-math.pi then
		targetYaw=targetYaw+2*math.pi
	end

    robot.simSetJointTargetPosition(handleJointWheelLF0, -47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelRB0, -47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelRF0, 47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelLB0, 47.38/180.0*math.pi)

	local rotVel=3

	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform,initRot)
	
	if ifinfo==1 then
		simExtPrintInfo('-----rotateToTarget-----')
		simExtPrintInfo('\tinitYaw  :\t'..initRot[3])
		simExtPrintInfo('\ttargetYaw:\t'..targetYaw)
	end
	
	local newRot={0,0,0}
	local Yaw=0
	local P=10
	local direction='r'

	if math.abs(targetYaw-initRot[3])>math.pi then
		if targetYaw<0 then
			direction='l'
		elseif targetYaw>0 then
			direction='r'
		end
	elseif math.abs(targetYaw-initRot[3])<math.pi then
		if targetYaw>initRot[3] then
			direction='l'
		elseif targetYaw<initRot[3] then
			direction='r'
		end
	end
	
	repeat
		newRot=robot.simGetObjectOrientation(handleTipPlatform,newRot)
		Yaw=newRot[3]

		dYaw=math.abs(targetYaw-Yaw)

		if Yaw<0 and targetYaw>0 and direction=='r' then
			dYaw=Yaw+math.pi+math.pi-targetYaw
		elseif Yaw>0 and targetYaw<0 and direction=='l' then
			dYaw=targetYaw+ math.pi +math.pi-Yaw
		end

		rotVel=P*dYaw
		if rotVel>10 then
			rotVel=10
		elseif rotVel<2 then
			rotVel=2
		end

		if mode==2 then
			rotVel=2
		end

		if(direction=='r') then
			robot.simSetJointTargetVelocity(handleJointWheelLF, rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelRF, -rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelLB, rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelRB, -rotVel)
		elseif(direction=='l') then
			robot.simSetJointTargetVelocity(handleJointWheelLF, -rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelRF, rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelLB, -rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelRB, rotVel)
		end
	until(math.abs(Yaw-targetYaw)<(1/180*math.pi))
	
	stopMove()
	if ifinfo==1 then
		simExtPrintInfo('-----rotatearrive------\n\n')
	end
	tprint()
end

--该函数为MoveandTurn子函数，负责计算目标点全局坐标系坐标与姿态
--输入初始小车在全局坐标系下X,Y,Yaw，相对运动位置与姿态，输出目标点在全局坐标系下X，Y，Yaw
function calAbsoluteTarget(initPos,initYaw,dX,dY,dYaw)

	local initX=initPos[1]
	local initY=initPos[2]
	local initYaw=initYaw

	local targetX=initX+dX*math.cos(initYaw)-dY*math.sin(initYaw)
	local targetY=initY+dY*math.cos(initYaw)+dX*math.sin(initYaw)

	local targetYaw=initYaw+dYaw

	if targetYaw>math.pi then
		targetYaw=targetYaw-math.pi*2
	end

	if targetYaw<-math.pi then
		targetYaw=targetYaw+math.pi*2
	end

	fprint('***************calAbsoluteTarget********************\n')
	fprint('absolute Target Pos:\t'..targetX..' '..targetY..' '..targetYaw)
	fprint('\n************************************\n\n')

	return targetX,targetY,targetYaw
end

--该函数给定目标全局坐标系下绝对位置与姿态，计算目标与小车相对位置与姿态
function calRelativeTarget(absoluteTargetX,absoluteTargetY,absoluteTargetYaw)
	handleTipPlatform = robot.simGetHandleVehicle()

	local curPos = {0,0,0}
	curPos = robot.simGetObjectPosition(handleTipPlatform, curPos)
	local curRot = {0,0,0}
    curRot = robot.simGetObjectOrientation(handleTipPlatform, curRot)

	--dYaw>0-->顺时针转动
	local relativeYaw=absoluteTargetYaw-curRot[3]

	local relativeX=(absoluteTargetX-curPos[1])*math.cos(curRot[3])+(absoluteTargetY-curPos[2])*math.sin(curRot[3])
	local relativeY=(absoluteTargetY-curPos[2])*math.cos(curRot[3])-(absoluteTargetX-curPos[1])*math.sin(curRot[3])

	fprint('****************calRelativeTarget********************\n')
	fprint('relative Target Pos:\t'..relativeX..' '..relativeY..' '..relativeYaw)
	fprint('\n************************************\n\n')

	if relativeYaw>math.pi then
		relativeYaw=relativeYaw-2*math.pi
	end

	if relativeYaw<-math.pi then
		relativeYaw=relativeYaw+2*math.pi 
	end

	return relativeX,relativeY,relativeYaw
end


function fourWheelPos()
	
	wheelPos={{},{},{},{}}
	handleTipPlatform=robot.simGetHandleVehicle()
	
	local initPos = {0,0,0}
	initPos = robot.simGetObjectPosition(handleTipPlatform, initPos)--   机器人初始位置
	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform, initRot)--  机器人初始姿态
	
	local lfPos={0,0,0}
	lfPos[1]=initPos[1]+math.cos(initRot[3]+3*math.pi/4)*0.235*math.sqrt(2)
	lfPos[2]=initPos[2]+math.sin(initRot[3]+3*math.pi/4)*0.235*math.sqrt(2)
	
	local lbPos={0,0,0}
	lbPos[1]=initPos[1]+math.cos(initRot[3]+5*math.pi/4)*0.235*math.sqrt(2)
	lbPos[2]=initPos[2]+math.sin(initRot[3]+5*math.pi/4)*0.235*math.sqrt(2)
	
	local rfPos={0,0,0}
	rfPos[1]=initPos[1]+math.cos(initRot[3]+1*math.pi/4)*0.235*math.sqrt(2)
	rfPos[2]=initPos[2]+math.sin(initRot[3]+1*math.pi/4)*0.235*math.sqrt(2)
	
	local rbPos={0,0,0}
	rbPos[1]=initPos[1]+math.cos(initRot[3]+7*math.pi/4)*0.235*math.sqrt(2)
	rbPos[2]=initPos[2]+math.sin(initRot[3]+7*math.pi/4)*0.235*math.sqrt(2)
	
	return lfPos,lbPos,rfPos,rbPos
	
end


function circleAround(objX,objY,r,direction)
	handleTipPlatform = robot.simGetHandleVehicle()
	
	local initPos = {0,0,0}
	initPos = robot.simGetObjectPosition(handleTipPlatform, initPos)
	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform, initRot)
	local tarPos={0,0,0}
	
	--计算在当前点的切线方向，并转到该角
	local theta=0
	local R=math.sqrt((objX-initPos[1])*(objX-initPos[1])+(objY-initPos[2])*(objY-initPos[2]))
	tarPos[1]=objX-r/R*(objX-initPos[1])
	tarPos[2]=objY-r/R*(objY-initPos[2])
	if direction=='l' then
		theta=math.atan2((objY-tarPos[2]),(objX-tarPos[1]))-math.pi
	end
	if direction=='r' then
		theta=math.atan2((objY-tarPos[2]),(objX-tarPos[1]))
	end
	rotateToTarget(theta,2,1)
	stopMove()
	--移动到圆上
	moveToTarget(tarPos[1],tarPos[2],2,1)
	stopMove()
	simExtPrintInfo('circleAround end')
	--获取四轮全局坐标，为后面计算轮控制量服务
	local lfPos={0,0,0}
	local lbPos={0,0,0}
	local rfPos={0,0,0}
	local rbPos={0,0,0}
	lfPos,lbPos,rfPos,rbPos=fourWheelPos()
	
	local lfVel=0
	local lfangle=0
	local lbVel=0
	local lbangle=0
	local rfVel=0
	local rfangle=0
	local rbVel=0
	local rbangle=0
	
	--左右其轮控制计算有区别
	if direction=='r' then 
		local temp=0
		temp=math.atan2((objY-lfPos[2]),objX-lfPos[1])
		lfang=temp-theta
		temp=math.atan2((objY-lbPos[2]),objX-lbPos[1])
		lbang=temp-theta
		temp=math.atan2((objY-rfPos[2]),objX-rfPos[1])
		rfang=temp-theta
		temp=math.atan2((objY-rbPos[2]),objX-rbPos[1])
		rbang=temp-theta
		
		lfVel=math.sqrt((objX-lfPos[1])*(objX-lfPos[1])+(objY-lfPos[2])*(objY-lfPos[2]))+0.5
		lbVel=math.sqrt((objX-lbPos[1])*(objX-lbPos[1])+(objY-lbPos[2])*(objY-lbPos[2]))+0.5
		rfVel=math.sqrt((objX-rfPos[1])*(objX-rfPos[1])+(objY-rfPos[2])*(objY-rfPos[2]))+0.2
		rbVel=math.sqrt((objX-rbPos[1])*(objX-rbPos[1])+(objY-rbPos[2])*(objY-rbPos[2]))+0.2
		--lfVel=0.9461*rfVel/0.475
		--lbVel=lfVel
		--rfVel=1
		--rbVel=rfVel
	end
	
	if direction=='l' then
		local temp=0
		temp=math.atan2((objY-lfPos[2]),(objX-lfPos[1]))-math.pi
		lfang=temp-theta
		temp=math.atan2((objY-lbPos[2]),(objX-lbPos[1]))-math.pi
		lbang=temp-theta
		temp=math.atan2((objY-rfPos[2]),(objX-rfPos[1]))-math.pi
		rfang=temp-theta
		temp=math.atan2((objY-rbPos[2]),(objX-rbPos[1]))-math.pi
		rbang=temp-theta
	
		lfVel=math.sqrt((objX-lfPos[1])*(objX-lfPos[1])+(objY-lfPos[2])*(objY-lfPos[2]))+0.2
		lbVel=math.sqrt((objX-lbPos[1])*(objX-lbPos[1])+(objY-lbPos[2])*(objY-lbPos[2]))+0.2
		rfVel=math.sqrt((objX-rfPos[1])*(objX-rfPos[1])+(objY-rfPos[2])*(objY-rfPos[2]))+0.5
		rbVel=math.sqrt((objX-rbPos[1])*(objX-rbPos[1])+(objY-rbPos[2])*(objY-rbPos[2]))+0.5
		--lfVel=1
		--lbVel=lfVel
		--rfVel=0.9461*rfVel/0.475
		--rbVel=rfVel
	end
	
	return lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel
end
--该函数用于使小车以给定速度，给定相对位置移动到目标点
--运行逻辑为
--1.依据小车目标点的相对位置计算其目标点绝对坐标与姿态
--2.repeat
--		获取小车绝对位置，目标绝对位置，计算相对距离，根据相对距离计算轮角度
--		运行
--	until（绝对位置在精度要求内达到目标点）
function GoAndTurn(vel,x0,y0,yaw,direction)
	--simSetJointTargetPosition设置某句柄铰接的转动位置，即设定轮方向
	--初始状态设置为0
	robot.simSetJointTargetPosition(handleJointWheelLF0, 0)
    robot.simSetJointTargetPosition(handleJointWheelRB0, 0)
    robot.simSetJointTargetPosition(handleJointWheelRF0, 0)
    robot.simSetJointTargetPosition(handleJointWheelLB0, 0)

	--simGetObjectPosition获得物体在世界坐标系下位置，用table进行存储
	local initPos = {0,0,0}
	initPos = robot.simGetObjectPosition(handleTipPlatform, initPos)
	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform,initRot)

	--计算输入中间变量
	local a=0
	local b=0
	local c=0
	local x1=0
	local y1=0
	local r=0
	local lFang=0
	local LBang=0
	local RFang=0
	local RBang=0
	local LFvel=0
	local LBvel=0
	local RFvel=0
	local RBvel=0

	--计算绝对目标位置
	local targetX=0
	local targetY=0
	local targetYaw=0
	targetX,targetY,targetYaw=calAbsoluteTarget(initPos,initRot[3],x0,y0,yaw)

	local curentX=0
	local curentY=0
	local curentYaw=0

	local relativeX=0
	local relativeY=0
	local relativeYaw=0

	local newPos = {0,0,0}
	local newRot = {0,0,0}

	if y0==0 then
		y0=0.001
	end

	simExtPrintInfo('-----GoAndTurn-----')
	repeat
		--simExtPrintInfo('\tdirection:'..direction)
		if direction=='r' then
			--计算相对位置
			relativeX,relativeY,relativeYaw=calRelativeTarget(targetX,targetY,targetYaw)

			a=(relativeX*relativeX+relativeY*relativeY)/(relativeY*relativeY)
			b=(relativeX+(relativeX*relativeX*relativeX)/(relativeY*relativeY))
			c=(relativeX*relativeX)/(4*relativeY*relativeY)*(relativeY*relativeY+relativeX*relativeX)-math.sqrt(relativeX*relativeX+relativeY*relativeY)/(2*math.tan(relativeYaw/2))*math.sqrt(relativeX*relativeX+relativeY*relativeY)/(2*math.tan(relativeYaw/2))

			x1=(b+math.sqrt(b*b-4*a*c))/(2*a)
			y1=-relativeX*x1/relativeY+(relativeX*relativeX+relativeY*relativeY)/2/relativeY

			LFang=math.atan2((y1-0.235),(x1+0.235))
			LBang=math.atan2((y1+0.235),(x1+0.235))
			RFang=math.atan2((y1-0.235),(x1-0.235))
			RBang=math.atan2((y1+0.235),(x1-0.235))

			r=math.sqrt(relativeX*relativeX+relativeY*relativeY)

			LFvel=math.sqrt((x1+0.235)*(x1+0.235)+(y1-0.235)*(y1-0.235))*vel
			LBvel=math.sqrt((x1+0.235)*(x1+0.235)+(y1+0.235)*(y1+0.235))*vel
			RFvel=math.sqrt((x1-0.235)*(x1-0.235)+(y1-0.235)*(y1-0.235))*vel
			RBvel=math.sqrt((x1-0.235)*(x1-0.235)+(y1+0.235)*(y1+0.235))*vel

			--fprint('radius:\t'..r..'\n(X,Y):\t('..x1..','..y1..')'..'\n')
			--simExtPrintInfo('radius:\t'..r..'\n(X,Y):\t('..x1..','..y1..')'..'\n')
		end

		if direction=='l' then
			--计算相对位置
			relativeX,relativeY,relativeYaw=calRelativeTarget(targetX,targetY,targetYaw)

			a=(relativeX*relativeX+relativeY*relativeY)/(relativeY*relativeY)
			b=(relativeX+(relativeX*relativeX*relativeX)/(relativeY*relativeY))
			c=(relativeX*relativeX)/(4*relativeY*relativeY)*(relativeY*relativeY+relativeX*relativeX)-math.sqrt(relativeX*relativeX+relativeY*relativeY)/(2*math.tan(-relativeYaw/2))*math.sqrt(relativeX*relativeX+relativeY*relativeY)/(2*math.tan(-relativeYaw/2))

			x1=(b-math.sqrt(b*b-4*a*c))/(2*a)
			y1=-relativeX*x1/relativeY+(relativeX*relativeX+relativeY*relativeY)/2/relativeY

			LFang=math.atan2((0.235-y1),(-0.235-x1))
			LBang=math.atan2((-0.235-y1),(-0.235-x1))
			RFang=math.atan2((0.235-y1),(0.235-x1))
			RBang=math.atan2((-0.235-y1),(0.235-x1))

			r=math.sqrt(relativeX*relativeX+relativeY*relativeY)

			LFvel=math.sqrt((x1+0.235)*(x1+0.235)+(y1-0.235)*(y1-0.235))*vel
			LBvel=math.sqrt((x1+0.235)*(x1+0.235)+(y1+0.235)*(y1+0.235))*vel
			RFvel=math.sqrt((x1-0.235)*(x1-0.235)+(y1-0.235)*(y1-0.235))*vel
			RBvel=math.sqrt((x1-0.235)*(x1-0.235)+(y1+0.235)*(y1+0.235))*vel

			--fprint('radius:\t'..r..'\n(X,Y):\t('..x1..','..y1..')'..'\n')
			--simExtPrintInfo('radius:\t'..r..'\n(X,Y):\t('..x1..','..y1..')'..'\n')
		end

		robot.simSetJointTargetPosition(handleJointWheelLF0, LFang)
		robot.simSetJointTargetPosition(handleJointWheelRB0, RBang)
		robot.simSetJointTargetPosition(handleJointWheelRF0, RFang)
		robot.simSetJointTargetPosition(handleJointWheelLB0, LBang)
		--simSetJointTargetVelocity设定某句柄转动速度，即轮转速
		robot.simSetJointTargetVelocity(handleJointWheelLF, LFvel)
		robot.simSetJointTargetVelocity(handleJointWheelRF, RFvel)
		robot.simSetJointTargetVelocity(handleJointWheelLB, LBvel)
		robot.simSetJointTargetVelocity(handleJointWheelRB, RBvel)

		--一帧的运动结束后重新测位置并保存
		newPos = robot.simGetObjectPosition(handleTipPlatform, newPos)
		newRot = robot.simGetObjectOrientation(handleTipPlatform,newRot)

		--fprint("newPos:\t"..'('..newPos[1]..','..newPos[2]..')')
		--fprint('Yaw:\t'..newRot[3])
		--fprint('U:\t'..LFvel..' '..LBvel..' '..RFvel..' '..RBvel..' '..LFang..' '..LBang..' '..RFang..' '..RBang)

	until(math.abs(targetX-newPos[1])<0.05 and math.abs(targetY-newPos[2])<0.05)

	fprint('\n******************End*****************\n\n')
	simExtPrintInfo('-----GoandTurnEnd-----\n')
end


function infoTableOfPlatformSensors()
    --获得机身传感器句柄
	handleSensorPlatform = {}
	handleSensorPlatform[1] = robot.simGetHandleProximitySensorPlatformLeft()
	handleSensorPlatform[2] = robot.simGetHandleProximitySensorPlatformRight()
	handleSensorPlatform[3] = robot.simGetHandleProximitySensorPlatformFront()
	handleSensorPlatform[4] = robot.simGetHandleProximitySensorPlatformBack()

	info = {{},{},{},{}}
	detectedPoint={0,0,0}
	detectedSurfaceNormalVector={0,0,0}
	
	result,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector,distance = robot.simGetProximitySensorResult(handleSensorPlatform[1],detectedPoint,detectedSurfaceNormalVector)
    --获得四个传感器数据
	for i=1,4 do
		result,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector,distance = robot.simGetProximitySensorResult(handleSensorPlatform[i],detectedPoint,detectedSurfaceNormalVector)
		info[i].result = result
		info[i].distance = distance
		info[i].point = detectedPoint
		info[i].ObjHandle = detectedObjectHandle
		info[i].SurVector = detectedSurfaceNormalVector
	end
	return info
end 

function infoRidialSensorsHorizontalScan()
	local RidialSensors={}
	local RidialSensorsJointHorizontal={}
	local RidialSensorsJointVertical={}
	
	RidialSensors[3]=robot.simGetHandleSensorPlatform1()--F
	RidialSensors[2]=robot.simGetHandleSensorPlatform2()--R
	RidialSensors[4]=robot.simGetHandleSensorPlatform3()--B
	RidialSensors[1]=robot.simGetHandleSensorPlatform4()--L
	
	--单线传感器水平旋转铰链
	RidialSensorsJointHorizontal[3]=robot.simGetHandleJointSensorPlatform1()
	RidialSensorsJointHorizontal[2]=robot.simGetHandleJointSensorPlatform2()
	RidialSensorsJointHorizontal[4]=robot.simGetHandleJointSensorPlatform3()
	RidialSensorsJointHorizontal[1]=robot.simGetHandleJointSensorPlatform4()
	
	--单线传感器垂直旋转铰链
	RidialSensorsJointVertical[3]=robot.simGetHandleJointSensorPlatformPitching1()
	RidialSensorsJointVertical[2]=robot.simGetHandleJointSensorPlatformPitching2()
	RidialSensorsJointVertical[4]=robot.simGetHandleJointSensorPlatformPitching3()
	RidialSensorsJointVertical[1]=robot.simGetHandleJointSensorPlatformPitching4()
	
	--设置旋转铰链初始位置为0
	robot.simSetJointTargetPosition(RidialSensorsJointHorizontal[1],0)
	robot.simSetJointTargetPosition(RidialSensorsJointHorizontal[2],0)
	robot.simSetJointTargetPosition(RidialSensorsJointHorizontal[3],0)
	robot.simSetJointTargetPosition(RidialSensorsJointHorizontal[4],0)
	
	robot.simSetJointTargetPosition(RidialSensorsJointVertical[1],0)
	robot.simSetJointTargetPosition(RidialSensorsJointVertical[2],0)
	robot.simSetJointTargetPosition(RidialSensorsJointVertical[3],0)
	robot.simSetJointTargetPosition(RidialSensorsJointVertical[4],0)
	
	--水平扫描
	local mindistance={3,3,3,3}
	local info = {{},{},{},{}}
	local point={0,0,0}
	local SurVector={0,0,0}
	local result=-1
	local ObjHandle=-1
    local distance=0
	for j=1,4 do
		for i=-45,45 do
			robot.simSetJointTargetPosition(RidialSensorsJointHorizontal[j],i)
			detectionState, detectedPoint, detectedObjHandle,detectedNormal,distance=robot.simGetProximitySensorResult(RidialSensors[j],detectedPoint,detectedNormal,distance)
			if detectionState==1 then
				temp=distance
				if temp<mindistance[j] then
					mindistance[j]=temp
					aimHandle=detectedObjHandle
					aimdetectionState=detectionState
					aimdetectedPoint=detectedPoint
					aimdetectedNormal=detectedNormal
				end
			end
		end
		info[j].result = detectionState
		info[j].distance = mindistance[j]
		info[j].point = aimdetectedPoint
		info[j].ObjHandle = aimHandle
		info[j].SurVector = aimdetectedNormal
    end
	return info
end

function task1()
    --机械手臂收回
	robot.simMakeLeftArmPosture({90,-90,90,90,90,0})
	robot.simMakeLeftArmPosture({90,-90,-40,-90,70,-90})
	robot.simMakeRightArmPosture({90,-90,90,-90,90,0})
	robot.simMakeRightArmPosture({90,-75,-40,58,90,-90})
	--robot.simTransTwoArmTipInGlobalFrame({-0.15,0,0}, {-0.15,0,0})

	robot.simSetPlatformPositionAndOrientation({-1.2,-6,0},math.pi/2)
	--rotateToTarget(0,0,0)
	robot.simSetCurrentTaskIndex(3)
	fprint('succeed to start\n')
	simExtPrintInfo('succeed to start\n')
	
end
--上下行扫描函数
function Scan(lasttarpos,changedir)

	local info = {{},{},{},{}}
	local leftSensor=0
	local frontSensor=0
	local backSensor=0
	local Objfind=0
	local robotpos={0,0,0}
	local leftpos={0,0,0}
	local frontpos={0,0,0}
	local uplimit=-4.6   -- -4.6
	local downlimit=-6.4
	local targetpos={0,0,0}
	handleTipPlatform = robot.simGetHandleVehicle()
	repeat
		info=infoTableOfPlatformSensors()
		leftSensor=info[1]
		frontSensor=info[3]
		backSensor=info[4]
		robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
		if robotpos[1]>-3.27 then --判断扫描的上下界限
			uplimit=-4.6   -- -4.6
			downlimit=-6.3
		end
		if robotpos[1]>-5 and robotpos[1]<-3.27  then 
			uplimit=-4.6   --  -4.6
			downlimit=-7.8
		end
		if robotpos[1]<-5 then 
			uplimit=-6
			downlimit=-7.8
		end
		simExtPrintInfo('uplimit:'..tostring(uplimit))
		simExtPrintInfo('left:'..tostring(leftSensor.result)..' '..'front:'..tostring(frontSensor.result))		
		simExtPrintInfo('robotpos:'..tostring(robotpos[1])..tostring(robotpos[2]))
		--开始扫描
		--上行
		if changedir==0 then
			if frontSensor.result==0 and leftSensor.result==0 then
				if robotpos[2]>(uplimit-0.2) then 
					moveToTarget(robotpos[1],uplimit,0,0)					
					--simExtPrintInfo('ending')
					for i=1,3 do 
						info=infoTableOfPlatformSensors()
						leftSensor=info[1]
						if  leftSensor.result==1 then
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
							targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
							if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
								targetpos[2]=3
							end
							if targetpos[2]~=3 then
								Objfind=1
								break
							end
						else 
							log(GoHorizontal,'GoHorizontal',10,0.2)
						end
					end
					if Objfind==1 then 
						break
					end
					changedir=1
				else 
					log(GoStraight,'GoStraight',13,0.2)
				end
			elseif frontSensor.result==0 and leftSensor.result==1 then
				robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
				targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
				if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
					targetpos[2]=3
				end
				if targetpos[2]~=3 then
					Objfind=1
					break
				end
			elseif frontSensor.result==1 and leftSensor.result==0 then 
				robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)				
				targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
				if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
					targetpos[2]=3
				end
				--if robotpos[1]<-5 and robotpos[1]>-7.2 and robotpos[2]>-6.2 then
					--targetpos[2]=3
				--end
				if robot.ifHandleIsTask2Obj(frontSensor.ObjHandle)==false and math.abs(targetpos[1]-lasttarpos[1])>0.2 and targetpos[2]~=3 then
					Objfind=1
					break 
				elseif math.abs(targetpos[1]-lasttarpos[1])<0.2 or targetpos[2]==3 then					
					simExtPrintInfo('front=1,left=0,targetpos[2]=3')					
					simExtPrintInfo('robotpos:'..tostring(robotpos[1])..tostring(robotpos[2]))
					if robotpos[2]>(uplimit-0.2) then 
						moveToTarget(robotpos[1],uplimit)
						for i=1,3 do 
							info=infoTableOfPlatformSensors()
							leftSensor=info[1]							
							simExtPrintInfo('front=1,left=0,targetpos[2]=3,horizontal')
							if  leftSensor.result==1 then 
								robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
								targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
								if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
									targetpos[2]=3
								end
								if targetpos[2]~=3 then
									Objfind=1
									break
								end
							else 
								log(GoHorizontal,'GoHorizontal',10,0.2)
							end
						end
						if Objfind==1 then 
							break
						end
						changedir=1
					else 
						log(GoStraight,'GoStraight',13,0.2)
					end
				elseif	robot.ifHandleIsTask2Obj(frontSensor.ObjHandle)==true then
					--log(GoStraight,'GoStraight',5,0.15)
					for i=1,3 do 
						info=infoTableOfPlatformSensors()
						leftSensor=info[1]
						if  leftSensor.result==1 then
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
							targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
							if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
								targetpos[2]=3
							end
							if targetpos[2]~=3 then
								Objfind=1
								break
							end
						else 
							log(GoHorizontal,'GoHorizontal',10,0.2)
						end
					end
					if Objfind==1 then 
						break
					end
					changedir=1
				end
			elseif frontSensor.result==1 and leftSensor.result==1 then 
				robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
				leftpos=robot.simGetObjectPosition(leftSensor.ObjHandle,leftpos)
				frontpos=robot.simGetObjectPosition(frontSensor.ObjHandle,frontpos)
				simExtPrintInfo(frontpos[1]..' '..frontpos[2])
				simExtPrintInfo(leftpos[1]..' '..leftpos[2])
				if robotpos[1]<-3.375 and (frontpos[1]+3.375)<0.3 and (frontpos[1]+3.375)>-0.3 then 
					frontpos[2]=3
				end
				if robotpos[1]<-3.375 and (leftpos[1]+3.375)<0.3 and (leftpos[1]+3.375)>-0.3 then 
					leftpos[2]=3
				end
				if robot.ifHandleIsTask2Obj(frontSensor.ObjHandle)==false and frontpos[2]~=3 then
					targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
					Objfind=1
					break
				end
				if robot.ifHandleIsTask2Obj(frontSensor.ObjHandle)==true and leftpos[2]~=3 then
					targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
					Objfind=1
					break
				end
				if frontpos[2]==3 and leftpos[2]==3 then
					changedir=1
					targetpos=lasttarpos
				end
			end
		end
		--下行		
		info=infoTableOfPlatformSensors()		
		leftSensor=info[1]		
		frontSensor=info[3]		
		backSensor=info[4]		
		robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)		
		simExtPrintInfo('scan up done'..tostring(changedir))		
		simExtPrintInfo('left:'..tostring(leftSensor.result)..' '..'back:'..tostring(backSensor.result))
		if changedir==1 then
			if backSensor.result==0 and leftSensor.result==0 then				
				simExtPrintInfo('enter down1')
				if robotpos[2]<(downlimit+0.2) then 
					moveToTarget(robotpos[1],downlimit,0,0)
					for i=1,3 do 
						info=infoTableOfPlatformSensors()
						leftSensor=info[1]
						if  leftSensor.result==1 then
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
							targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
							if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
								targetpos[2]=3
							end
							if targetpos[2]~=3 then
								Objfind=1
								break
							end
						else 
							log(GoHorizontal,'GoHorizontal',10,0.2)
						end
					end
					if Objfind==1 then 
						break
					end
					changedir=0
				else 
					log(GoStraight,'GoStraight',-13,-0.2)
				end 
			elseif backSensor.result==0 and leftSensor.result==1 then				
				simExtPrintInfo('enter down2')
				robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
				targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
				if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
					targetpos[2]=3
				end
				if targetpos[2]~=3 then
					Objfind=1
					break
				end
			elseif backSensor.result==1 and leftSensor.result==0 then				
				simExtPrintInfo('enter down3')
				targetpos=robot.simGetObjectPosition(backSensor.ObjHandle,targetpos)				
				simExtPrintInfo('targetpos:'..tostring(targetpos[1])..tostring(targetpos[2]))				
				simExtPrintInfo('lasttarpos:'..tostring(lasttarpos[1])..tostring(lasttarpos[2]))
				if math.abs(targetpos[1]+3.8934)<0.02 and math.abs(targetpos[2]+3.4099)<0.02 then
					targetpos[2]=2
				end
				if robot.ifHandleIsTask2Obj(backSensor.ObjHandle)==false and math.abs(targetpos[1]-lasttarpos[1])>0.2 and targetpos[2]~=2 then
					Objfind=1
					break 
				elseif math.abs(targetpos[1]-lasttarpos[1])<0.2 or targetpos[2]==2  then
					robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
					if robotpos[2]<(downlimit+0.2) then 
						moveToTarget(robotpos[1],downlimit,0,0)
						for i=1,3 do 
							info=infoTableOfPlatformSensors()
							leftSensor=info[1]
							if  leftSensor.result==1 then
								robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
								targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
								if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
									targetpos[2]=3
								end
								if targetpos[2]~=3 then	
									Objfind=1
									break
								end
							else 
								log(GoHorizontal,'GoHorizontal',10,0.2)
							end
						end
						if Objfind==1 then 
							break
						end
						changedir=0
					else 
						log(GoStraight,'GoStraight',-13,-0.2)
					end
				elseif	robot.ifHandleIsTask2Obj(backSensor.ObjHandle)==true then
					log(GoStraight,'GoStraight',-13,-0.3)
					for i=1,3 do 
						info=infoTableOfPlatformSensors()
						leftSensor=info[1]
						if  leftSensor.result==1 then 
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
							targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
							if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
								targetpos[2]=3
							end
							if targetpos[2]~=3 then
								--targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
								Objfind=1
								break
							end
						else 
							log(GoHorizontal,'GoHorizontal',13,0.2)
						end
					end
					if Objfind==1 then 
						break
					end
					changedir=0
				end			
			elseif backSensor.result==1 and leftSensor.result==1 then				
				targetpos=robot.simGetObjectPosition(backSensor.ObjHandle,targetpos)				
				if math.abs(targetpos[1]+3.8934)<0.02 and math.abs(targetpos[2]+3.4099)<0.02 then					
					targetpos[2]=2				
				end				
				if targetpos[2]==2 then					
					targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)					
					Objfind=1					
					break				
				else					
					targetpos=robot.simGetObjectPosition(backSensor.ObjHandle,targetpos)					
					Objfind=1					
					break				
				end
			end
		end
	until Objfind==1
	return targetpos
end

function Circlefind(targetpos,count,flag)
	local lfang=0
	local rbang=0
	local rfang=0
	local lbang=0
	local lfVel=0
	local rfVel=0
	local lbVel=0
	local rbVel=0
	local info = {{},{},{},{}}
	local leftSensor=0
	local rightSensor=0
	local backSensor=0
	local frontSensor=0
	local lasttarpos={0,0,0}
	local rot={0,0,0}
	local startScan=0
	local changedir=0--0  left
	local flag1=0-- up and down biaozhiwei,0up,1down  
	local robotpos={0,0,0}
	local dx=0
	local dy=0
	local distance=0
	local flag2=0
	local theta1=0
	local theta2=0
	local rightpos={0,0,0}
	local frontpos={0,0,0}
	local midpos={0,0,0}
	local tranx=0
	local trany=0
	local tantheta=0
	
	lasttarpos=targetpos
	count=count+1
	all_targetpos1[count]=targetpos[1]
	all_targetpos2[count]=targetpos[2]
	robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
	if count<5 then
		if count==1 or flag==1 then
			lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(targetpos[1],targetpos[2],0.475,'l')
			robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
			robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
			robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
			robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)
			while(1) do  
				info=infoRidialSensorsHorizontalScan()
				rightSensor=info[2]
				frontSensor=info[3]
				robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
				rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
				robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)	
				if robotpos[2]<-7.8 then 
					moveToTarget(robotpos[1]-0.5,-7.8)						
					rotateToTarget(0,0,0)
					startScan=1
					flag1=0
					break
				end
				if math.abs(rot[3]+math.pi/2)<0.035 then					
					robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)					
					moveToTarget(robotpos[1]-0.5,robotpos[2])					
					robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)					
					if robotpos[2]>-7.8 then						
						rotateToTarget(0,0,0)					
					else						
						moveToTarget(robotpos[1],-7.8)						
						rotateToTarget(0,0,0)					
					end
					--log(GoHorizontal,'GoHorizontal',-5,-0.2)
					flag1=1
					startScan=1
					break
				end
				if rightSensor.result==1 and robot.ifHandleIsTask2Obj(rightSensor.ObjHandle)==false then
					targetpos=robot.simGetObjectPosition(rightSensor.ObjHandle,targetpos)
					robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
					if math.abs(targetpos[1]+3.8934)<0.02 and math.abs(targetpos[2]+3.4099)<0.02 then
						targetpos[2]=2
					end
					if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
						targetpos[2]=3
					end
					if targetpos[2]~=2 and targetpos[2]~=3 then
						moveToTarget((lasttarpos[1]+targetpos[1])/2,(lasttarpos[2]+targetpos[2])/2,0,0)
						lasttarpos=targetpos
						count=count+1
						all_targetpos1[count]=targetpos[1]
						all_targetpos2[count]=targetpos[2]
						changedir=1
						break					
					else						
						targetpos=lasttarpos
					end
				end
				if frontSensor.result==1 and rightSensor.result==0 then --判断附属物的边界，目前只在该处加，没有办法调试
					robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
					frontpos=robot.simGetObjectPosition(frontSensor.ObjHandle,frontpos)
					if robotpos[1]<-3.375 and (frontpos[1]+3.375)<0.3 and (frontpos[1]+3.375)>-0.3 then 
						frontpos[2]=3
					end
					if frontpos[2]==3 then
						moveToTarget(lasttarpos[1]+0.5,lasttarpos[2])
						moveToTarget(lasttarpos[1]+0.5,lasttarpos[2]-0.5)
						targetpos=lasttarpos
						startScan=1
						flag1=1
						break
					end
				end
			end 	
		end
		stopMove()
		dx=targetpos[1]+5.836
		dy=targetpos[2]+4.9
		distance=math.sqrt(dx*dx+dy*dy)
		if distance<0.8  then
			changedir=1
		end
		if startScan==0 and count>1 and count<5 then
			repeat
				if changedir==1 then
					lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(targetpos[1],targetpos[2],0.475,'r')
					robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
					robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
					robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
					robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)
					if targetpos[2]<-7.45 then 
						lfVel=-lfVel
						rfVel=-rfVel
						lbVel=-lbVel
						rbVel=-rbVel
						flag2=1
					end
					repeat
						info=infoRidialSensorsHorizontalScan()
						leftSensor=info[1]
						frontSensor=info[3]
						--if frontSensor.result==1 and flag2==0 then
							--break
						--end
						robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
						robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
						if robotpos[2]<-7.8 and flag2==1 then
							moveToTarget(robotpos[1]-0.5,-7.8)
							rotateToTarget(0)
							startScan=1
							flag1=0
							break
						end
					until leftSensor.result==0
					--[[if frontSensor.result==1 and flag2==0 then
						rightpos=robot.simGetObjectPosition(rightSensor.ObjHandle,rightpos)
						frontpos=robot.simGetObjectPosition(frontSensor.ObjHandle,frontpos)
						simExtPrintInfo(frontpos[1]..' '..frontpos[2])
						simExtPrintInfo(rightpos[1]..' '..rightpos[2])
						robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
						tantheta=math.abs((rightpos[2]-frontpos[2])/(rightpos[1]-frontpos[1]))
						theta1=(90+math.atan(tantheta)*180/math.pi)*math.pi/180
						simExtPrintInfo(tostring(theta1))
						theta2=theta1-math.pi/2
						midpos={(rightpos[1]+frontpos[1])/2,(rightpos[2]+frontpos[2])/2,0}
						simExtPrintInfo(midpos[1]..' '..midpos[2])
						tranx=(midpos[1]+tantheta*tantheta*robotpos[1]+tantheta*(midpos[2]-robotpos[2]))/(1+tantheta*tantheta)
						simExtPrintInfo(tostring(tranx))
						trany=robotpos[2]-tantheta*(robotpos[1]-tranx)
						rotateToTarget(theta1,0,0)
						moveToTarget(tranx,trany)
						rotateToTarget(theta2)
						log(GoStraight,'GoStraight',5,1)
						break
					end--]]
					while(1) do
						info=infoRidialSensorsHorizontalScan()
						leftSensor=info[1]
						robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
						rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
						robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
						if math.abs(rot[3]+math.pi/2)<0.035 and flag2==0 then
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
							moveToTarget(robotpos[1]-0.5,robotpos[2])					
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)					
							if robotpos[2]<-4.6 then						
								rotateToTarget(0,0,0)					
							else
								rotateToTarget(0,0,0)
								moveToTarget(robotpos[1],-4.6)											
							end
							flag1=1
							startScan=1
							break
						elseif flag2==1 and robotpos[2]<-7.8 then
							moveToTarget(robotpos[1]-0.3,-7.8)
							rotateToTarget(0,0,0)
							flag1=1
							startScan=1
							flag2=0
							break
						end
						if leftSensor.result==1 and robot.ifHandleIsTask2Obj(leftSensor.ObjHandle)==false then
							targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
							if math.abs(targetpos[1]+3.8934)<0.02 and math.abs(targetpos[2]+3.4099)<0.02 then
								targetpos[2]=2
							end
							if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
								targetpos[2]=3
							end
							if targetpos[2]~=2 and targetpos[2]~=3 then
								moveToTarget((lasttarpos[1]+targetpos[1])/2,(lasttarpos[2]+targetpos[2])/2,0,1)
								lasttarpos=targetpos
								count=count+1
								all_targetpos1[count]=targetpos[1]
								all_targetpos2[count]=targetpos[2]
								changedir=0
								break
							else								
								targetpos=lasttarpos							
							end
						end
					end
					if startScan==1 then
						break
					end
					if count==5 then 
						break
					end
				end
				stopMove()
				if changedir==0 then
					lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(targetpos[1],targetpos[2],0.475,'l')
					robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
					robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
					robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
					robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)
					repeat
						info=infoRidialSensorsHorizontalScan()
						rightSensor=info[2]
						robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
						robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
						if robotpos[2]<-7.8 then
							moveToTarget(robotpos[1]-0.5,-7.8)
							rotateToTarget(0)
							startScan=1
							flag1=0
							break
						end
					until rightSensor.result==0
					while(1) do 
						info=infoRidialSensorsHorizontalScan()
						rightSensor=info[2]
						robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
						robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
						rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
						robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
						if robotpos[2]<-7.8 then
							moveToTarget(robotpos[1]-0.5,-7.8)
							rotateToTarget(0)
							startScan=1
							flag1=0
							break
						end
						if math.abs(rot[3]+math.pi/2)<0.035 then							
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)							
							moveToTarget(robotpos[1]-0.5,robotpos[2])							
							if robotpos[2]>-7.8 then								
								rotateToTarget(0,0,0)							
							else								
								moveToTarget(robotpos[1],-7.8)								
								rotateToTarget(0,0,0)							
							end
							flag1=1
							startScan=1
							break						
						end
						if rightSensor.result==1 and robot.ifHandleIsTask2Obj(rightSensor.ObjHandle)==false then
							targetpos=robot.simGetObjectPosition(rightSensor.ObjHandle,targetpos)
							robotpos=robot.simGetObjectPosition(handleTipPlatform,robotpos)
							if math.abs(targetpos[1]+3.8934)<0.02 and math.abs(targetpos[2]+3.4099)<0.02 then
								targetpos[2]=2
							end
							if robotpos[1]<-3.375 and (targetpos[1]+3.375)<0.3 and (targetpos[1]+3.375)>-0.3 then 
								targetpos[2]=3
							end
							if targetpos[2]~=2 and targetpos[2]~=3 then
								moveToTarget((lasttarpos[1]+targetpos[1])/2,(lasttarpos[2]+targetpos[2])/2,0,0)
								lasttarpos=targetpos
								count=count+1
								all_targetpos1[count]=targetpos[1]
								all_targetpos2[count]=targetpos[2]
								changedir=1
								break							
							else								
								targetpos=lasttarpos
							end
						end
					end
					if startScan==1 then
						break
					end
				end
				stopMove()
			until count==5
		end
	end
	return targetpos,startScan,count,flag1
end
function task3()
	local targetpos={0,0,0}
	local robotpos={0,0,0}
	local rot={0,0,0}
	local lasttarpos={0,0,0}
	local startScan=0
	local count=0
	local flag1=0
	local info = {{},{},{},{}}
	local backSensor=0
	local frontSensor=0
	local leftSensor=0
	local rightSensor=0
	all_targetpos1={}  
	all_targetpos2={}

	targetpos=Scan(lasttarpos,0)
	simExtPrintInfo('Scan done')
	simExtPrintInfo(tostring(targetpos[1])..' '..tostring(targetpos[2]))
	targetpos,startScan,count,flag1=Circlefind(targetpos,0,0)
	simExtPrintInfo('1circle end')
	repeat 
		if startScan==1 then 			
			simExtPrintInfo('targetpos:'..tostring(targetpos[1])..' '..tostring(targetpos[2]))
			targetpos=Scan(targetpos,flag1)
			stopMove()
			info=infoTableOfPlatformSensors()
			backSensor=info[4]
			frontSensor=info[3]
			leftSensor=info[1]
			simExtPrintInfo(targetpos[1]..' '..targetpos[2])
			simExtPrintInfo(tostring(frontSensor.result)..' '..tostring(backSensor.result)..' '..tostring(leftSensor.result))
			if backSensor.result==1 and leftSensor.result==0 and frontSensor.result==0 then
				moveToTarget(targetpos[1],targetpos[2]+0.7,0,0)
				targetpos,startScan,count,flag1=Circlefind(targetpos,count,1)
			elseif backSensor.result==1 and leftSensor.result==1 and frontSensor.result==0 then 
				targetpos,startScan,count,flag1=Circlefind(targetpos,count,1)
			elseif frontSensor.result==1 then
				simExtPrintInfo('enter f=1')
				moveToTarget(targetpos[1],targetpos[2]-0.7,0,0)
				targetpos,startScan,count,flag1=Circlefind(targetpos,count,1)
			elseif leftSensor.result==1 and backSensor.result==0 and frontSensor.result==0 then
				targetpos,startScan,count,flag1=Circlefind(targetpos,count,1)
			end
		end
	until count==5
	local leftpos={0,0,0}
	local rightpos={0,0,0}	
	simExtPrintInfo('result5:'..tostring(targetpos[1])..' '..tostring(targetpos[2]))
	--dX=targetpos[1]+7.8--7.885
	--dY=targetpos[2]+8--8.394
	--distance=math.sqrt(dX*dX+dY*dY)
	info=infoTableOfPlatformSensors()
	leftSensor=info[1]
	rightSensor=info[2]
	simExtPrintInfo(tostring(leftSensor.result)..' '..tostring(rightSensor.result))
	if leftSensor.result==1 and rightSensor.result==1 then
		simExtPrintInfo('enter L=1 R=1')
		leftpos=robot.simGetObjectPosition(leftSensor.ObjHandle, leftpos)
		rightpos=robot.simGetObjectPosition(rightSensor.ObjHandle, rightpos) 
		if  leftpos[1]<rightpos[1] then			
			lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(targetpos[1],targetpos[2],0.48,'l')			
			robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)			
			robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)			
			robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)			
			robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)			
			if targetpos[2]<-7.4 then
				while(1) do
					robotpos=robot.simGetObjectPosition(handleTipPlatform, robotpos)
					rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
					robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
					if math.abs(rot[3]+math.pi)<0.02 and robotpos[1]<targetpos[1] then
						break
					end
				end 
				rotateToTarget(math.pi,0,0)			
			else				
				while(1) do					
					robotpos=robot.simGetObjectPosition(handleTipPlatform, robotpos)					
					rot=robot.simGetObjectOrientation(handleTipPlatform,rot)					
					robot.simSetJointTargetVelocity(handleJointWheelLF, -lfVel*5)					
					robot.simSetJointTargetVelocity(handleJointWheelRF, -rfVel*5)					
					robot.simSetJointTargetVelocity(handleJointWheelLB, -lbVel*5)					
					robot.simSetJointTargetVelocity(handleJointWheelRB, -rbVel*5)					
					if math.abs(rot[3]+math.pi/2)<0.02 then	
						simExtPrintInfo('L=1 R=1 -math.pi/2')
						break					
					end				
				end 				
				rotateToTarget(-math.pi/2,0,0)			
			end
		else
			lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(targetpos[1],targetpos[2],0.48,'r')
			robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
			robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
			robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
			robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)			
			if targetpos[2]>-7.4 then
				while(1) do
					robotpos=robot.simGetObjectPosition(handleTipPlatform, robotpos)
					rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
					robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
					if math.abs(rot[3]-math.pi/2)<0.02 then
						break
					end
				end 
				rotateToTarget(math.pi/2,0,0)			
			else				
				while(1) do					
					robotpos=robot.simGetObjectPosition(handleTipPlatform, robotpos)					
					rot=robot.simGetObjectOrientation(handleTipPlatform,rot)					
					robot.simSetJointTargetVelocity(handleJointWheelLF, -lfVel*5)					
					robot.simSetJointTargetVelocity(handleJointWheelRF, -rfVel*5)					
					robot.simSetJointTargetVelocity(handleJointWheelLB, -lbVel*5)					
					robot.simSetJointTargetVelocity(handleJointWheelRB, -rbVel*5)					
					if math.abs(rot[3])<0.02 and robotpos[1]<targetpos[1] then						
						break					
					end				
				end 				
				rotateToTarget(0,0,0)			
			end
		end
	elseif leftSensor.result==1 and rightSensor.result==0 then
		simExtPrintInfo('enter L=1 R=0')
		lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(targetpos[1],targetpos[2],0.48,'l')
		robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
		robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
		robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
		robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)
		if targetpos[2]<-7.4 then
			while(1) do
				robotpos=robot.simGetObjectPosition(handleTipPlatform, robotpos)
				rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
				robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
				if math.abs(rot[3]-math.pi)<0.02 then
					break
				end
			end 
			rotateToTarget(math.pi,0,0)
		else
			while(1) do
				robotpos=robot.simGetObjectPosition(handleTipPlatform, robotpos)
				rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
				robot.simSetJointTargetVelocity(handleJointWheelLF, -lfVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelRF, -rfVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelLB, -lbVel*5)
				robot.simSetJointTargetVelocity(handleJointWheelRB, -rbVel*5)
				if math.abs(rot[3]+math.pi/2)<0.02 then
					break
				end
			end 
			rotateToTarget(-math.pi/2,0,0)
		end
	end
	moveToTarget(robotpos[1],-7.85,0,0)
	moveToTarget(-8,-7.85,0,0)
	simExtPrintInfo('succeed to end\n')
	simExtPrintInfo(all_targetpos1[1]..' '..all_targetpos1[2]..' '..all_targetpos1[3]..' '..all_targetpos1[4]..' '..all_targetpos1[5])
	simExtPrintInfo(all_targetpos2[1]..' '..all_targetpos2[2]..' '..all_targetpos2[3]..' '..all_targetpos2[4]..' '..all_targetpos2[5])
	fopen()
	fprint(all_targetpos1[1]..' '..all_targetpos1[2]..' '..all_targetpos1[3]..' '..all_targetpos1[4]..' '..all_targetpos1[5])
	fprint(all_targetpos2[1]..' '..all_targetpos2[2]..' '..all_targetpos2[3]..' '..all_targetpos2[4]..' '..all_targetpos2[5])
	fclose()
end
function test()
	local mmpos={0,0,0}
	local leftSensor=0
	local info={{},{},{},{}}
	local start
	local last
	local t=0
	simExtPrintInfo(tostring(os.time()))
	info=infoRidialSensorsHorizontalScan()
	simExtPrintInfo(tostring(os.time()))
	
	
end

cleanlog()
tprint()
globalInit()
log(task1,'task1')
delay(10)
--log(test,'test')
log(task3,'task3')
stopMove()


