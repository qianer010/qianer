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
	
	task2ScanStartY=-5.0
	
	task2DectedNum=0
	
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

function GoStraight(moveVel,length)
	robot.simSetJointTargetPosition(handleJointWheelLF0, 0)
    robot.simSetJointTargetPosition(handleJointWheelRB0, 0)
    robot.simSetJointTargetPosition(handleJointWheelRF0, 0)
    robot.simSetJointTargetPosition(handleJointWheelLB0, 0)

	local initPos = {0,0,0}
	initPos = robot.simGetObjectPosition(handleTipPlatform, initPos)
	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform,initRot)
	
	local targetX=0
	local targetY=0
	local targeYaw=0
	targetX,targetY,targeYaw=calAbsoluteTarget(initPos,initRot[3],0,length,0)
	
	fprint('-----GoStraight-----')
	moveToTarget(targetX,targetY)
	
	--simExtPrintInfo('-----GoStraightArrive-----\n')
	fprint('-----Arrive-----\n')
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

    	--simSetJointTargetVelocityéè?¨?3??±ú×a?ˉ?ù?è￡??′??×a?ù
		robot.simSetJointTargetVelocity(handleJointWheelLF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelLB, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRB, moveVel)
		--ò???μ????ˉ?áê?oó??D?2a????2￠±￡′?

		newPos = robot.simGetObjectPosition(handleTipPlatform, newPos)

		newRot = robot.simGetObjectOrientation(handleTipPlatform,newRot)

		--fprint("newPos="..' '..newPos[1]..' '..newPos[2]..' '..newRot[3]..' '..LFvel..' '..LBvel..' '..RFvel..' '..RBvel..' '..LFang..' '..LBang..' '..RFang..' '..RBang)

   		--?ì2aê?·?μ?′?
		distance = {newPos[1]-initPos[1], newPos[2]-initPos[2], newPos[3]-initPos[3] }
		distance = math.sqrt(distance[1]*distance[1]+ distance[2]*distance[2]+ distance[3]*distance[3])
		arrived = false

		if(math.abs(math.abs(length)-distance) < 0.03) then
			arrived = true
		end

    until (arrived == true)

	--fprint('******************************\n\n')
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
			moveVel=4
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
		if rotVel>8 then
			rotVel=8
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
	elseif targetYaw<-math.pi then
		targetYaw=targetYaw+math.pi*2
	end
	
	--simExtPrintInfo('\tabsolute Target Pos:\t'..targetX..' '..targetY..' '..targetYaw)
	
	return targetX,targetY,targetYaw
end

function calRelativeTarget(absoluteTargetX,absoluteTargetY,absoluteTargetYaw)

	local curPos = {0,0,0}
	curPos = robot.simGetObjectPosition(handleTipPlatform, curPos)
	local curRot = {0,0,0}
    curRot = robot.simGetObjectOrientation(handleTipPlatform, curRot)
	--dYaw>0-->顺时针转动
	local relativeYaw=absoluteTargetYaw-curRot[3]
	local relativeX=(absoluteTargetX-curPos[1])*math.cos(curRot[3])+(absoluteTargetY-curPos[2])*math.sin(curRot[3])
	local relativeY=(absoluteTargetY-curPos[2])*math.cos(curRot[3])-(absoluteTargetX-curPos[1])*math.sin(curRot[3])

	--simExtPrintInfo('\trelative Target Pos:\t'..relativeX..' '..relativeY..' '..relativeYaw)
	
	if relativeYaw>math.pi then
		relativeYaw=relativeYaw-2*math.pi
	elseif relativeYaw<-math.pi then
		relativeYaw=relativeYaw+2*math.pi
	end

	return relativeX,relativeY,relativeYaw
end

--该函数用于使小车以给定速度，给定相对位置移动到目标点
--运行逻辑为
--1.依据小车目标点的相对位置计算其目标点绝对坐标与姿态
--2.repeat
--		获取小车绝对位置，目标绝对位置，计算相对距离，根据相对距离计算轮角度
--		运行
--	until（绝对位置在精度要求内达到目标点?
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

--返回四个轮全局坐标准确位置
function getFourWheelPos()
	local lfPos={0,0,0}
	local lbPos={0,0,0}
	local rfPos={0,0,0}
	local rbPos={0,0,0}
	lfPos=robot.simGetObjectPosition(handleJointWheelLF0,lfPos)	
	lbPos=robot.simGetObjectPosition(handleJointWheelLB0,lbPos)
	rfPos=robot.simGetObjectPosition(handleJointWheelRF0,rfPos)	
	rbPos=robot.simGetObjectPosition(handleJointWheelRB0,rbPos)
	return lfPos,lbPos,rfPos,rbPos
end

function getLeftArmPos()
	local handles={0,0}
	local Pos={0,0,0}
	handles=robot.simGetHandleJointLeftWrist(handles)	
	Pos=robot.simGetObjectPosition(handles[1],Pos)
	
	return Pos
end

function circleAround(objX,objY,r,direction)
	handleTipPlatform = robot.simGetHandleVehicle()
	
	local initPos = {0,0,0}
	initPos = robot.simGetObjectPosition(handleTipPlatform, initPos)
	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform, initRot)
	local tarPos={0,0,0}
	
	--?????úμ±?°μ?μ??D??·??ò￡?2￠×aμ?????
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
	--ò??ˉμ??2é?
	moveToTarget(tarPos[1],tarPos[2],2,1)
	stopMove()
	simExtPrintInfo('circleAround end')
	--??è?????è???×?±ê￡??aoó????????????á?·t??
	local lfPos={0,0,0}
	local lbPos={0,0,0}
	local rfPos={0,0,0}
	local rbPos={0,0,0}
	lfPos,lbPos,rfPos,rbPos=getFourWheelPos()
	
	local lfVel=0
	local lfangle=0
	local lbVel=0
	local lbangle=0
	local rfVel=0
	local rfangle=0
	local rbVel=0
	local rbangle=0
	
	--×óóò????????????óD??±e
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
	end
	
	return lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel
end

function infoTableOfPlatformSensors()
    --获得机身传感器句柄
	local handleSensorPlatform = {}
	handleSensorPlatform[1] = robot.simGetHandleProximitySensorPlatformLeft()
	handleSensorPlatform[2] = robot.simGetHandleProximitySensorPlatformRight()
	handleSensorPlatform[3] = robot.simGetHandleProximitySensorPlatformFront()
	handleSensorPlatform[4] = robot.simGetHandleProximitySensorPlatformBack()

	local info = {{},{},{},{}}
	local detectedPoint={0,0,0}
	local detectedSurfaceNormalVector={0,0,0}

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

function leftArmSensors()
	local info = {{},{},{},{},{}}
	local handleSensorArmLeft = {}
	handleSensorArmLeft[1] = robot.simGetHandleProximitySensorArmFront()
	handleSensorArmLeft[2] = robot.simGetHandleProximitySensorArmDown()
	handleSensorArmLeft[3] = robot.simGetHandleProximitySensorArmUp()
	handleSensorArmLeft[4] = robot.simGetHandleProximitySensorArmLeft()
	handleSensorArmLeft[5] = robot.simGetHandleProximitySensorArmRight()

	local detectedPoint={0,0,0}
	local detectedSurfaceNormalVector={0,0,0}
	result,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector,distance = robot.simGetProximitySensorResult(handleSensorArmLeft[1],detectedPoint,detectedSurfaceNormalVector)
	for i=1,5 do
		result,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector,distance = robot.simGetProximitySensorResult(handleSensorArmLeft[i],detectedPoint,detectedSurfaceNormalVector)
		info[i].result = result
		info[i].distance = distance
		info[i].detectedPoint = detectedPoint
		info[i].detectedObjectHandle = detectedObjectHandle
		info[i].detectedSurfaceNormalVector = detectedSurfaceNormalVector
	end
	return info
end

function judgeIfinTable(newX,newY)
	local answer=false
	
	for i=1,4 do
		if math.abs(task2ObjPos[i][1]-newX)<0.01 and math.abs(task2ObjPos[i][2]-newY)<0.01 then
			answer=true
		end
	end
	return answer
end

--starX,starY,startYaw为初始扫描起始位置
function Task2scan(startX,startY,startYaw)
	--移动到扫描初始位置
	simExtPrintInfo('Task2 scan')
	moveToTarget(startX,startY)
	rotateToTarget(startYaw)
	
	local moveState='f'
	local ifDetected=false
	
	repeat
		local scanCount=1
		local ifObstacle=false
		
		repeat
			local platformInfo=infoTableOfPlatformSensors()
			local rightSensor=platformInfo[2]
			local frontSensor=platformInfo[3]
			--移动
			if moveState=='f' then
				GoStraight(10,0.1)
			elseif moveState=='b' then
				GoStraight(10,-0.1)
			end
			
			if rightSensor.result==1 then  --右边传感器扫描到物体
				if(robot.ifHandleIsTask2Obj(rightSensor.ObjHandle)) then
					tempPosintask2={0,0,0}
					tempRotintask2={0,0,0}
					
					--simExtPrintInfo('rightSensor detected task2Obj '..ObjNumintask2)
					tempPosintask2=robot.simGetObjectPosition(rightSensor.ObjHandle,tempPosintask2)--记录位置和方向
					tempRotintask2=robot.simGetObjectOrientation(rightSensor.ObjHandle,tempRotintask2)
					--simExtPrintInfo('tempPos: '..tempPosintask2[1]..'  '..tempPosintask2[2])
					local ifinTable=judgeIfinTable(tempPosintask2[1],tempPosintask2[2])--判断是否在表中
					
					if ifinTable==false then
						ifDetected=true
						if ObjNumintask2==1 then
							ObjcompareNum=1
						elseif ObjNumintask2<5 then
							ObjcompareNum=ObjcompareNum+1
						end
						
						task2ObjPos[ObjNumintask2][1]=tempPosintask2[1]--记录入表
						task2ObjPos[ObjNumintask2][2]=tempPosintask2[2]
						task2ObjRot[ObjNumintask2][3]=tempRotintask2[3]
						simExtPrintInfo('Obj '..ObjNumintask2..'\tX'..tostring(task2ObjPos[ObjNumintask2][1])..'\tY'..tostring(task2ObjPos[ObjNumintask2][2])..'\tYaw'..tostring(task2ObjRot[ObjNumintask2][3]))
						ObjNumintask2=ObjNumintask2+1
						
						task2DectedNum=task2DectedNum+1
					end
				end
			elseif rightSensor.result==0 then
				simExtPrintInfo('\tNot detected')
			end

			if frontSensor.result==1 then --前边遇到障碍物
				simExtPrintInfo('obs')
				if frontSensor.distance<0.6 then--前边障碍物在0.6以内会触发停止扫描
					simExtPrintInfo('\tObstacle')
					ifObstacle=true
				end
			end
			scanCount=scanCount+1
		until(scanCount==30 or ifObstacle==true)--遇到障碍物会停止
		stopMove()
		
		local curPos={0,0,0}
		local newPos={0,0,0}
		if ifDetected~=true then
			if moveState=='f' then
				moveState='b'
			elseif moveState=='b' then
				moveState='f'
			end
			simExtPrintInfo('\tmoveState:'..moveState)
			
			curPos = robot.simGetObjectPosition(handleTipPlatform,curPos)
			newPos = {curPos[1],curPos[2]+0.2,curPos[3]}
			moveToTarget(newPos[1],newPos[2])
			rotateToTarget(math.pi/2)
			stopMove()
		end
	until(ifDetected==true)

	local minYobj=1
	local minY=0
	for i=1,4 do
		if task2ObjPos[i][2]<minY then
			minY=task2ObjPos[i][2]
			minYobj=i
		end
	end
	simExtPrintInfo('minYobj  '..minYobj..'  minY  '..minY)
	moveToTarget(startX,minY-0.75)
	rotateToTarget(startYaw)
	
	local scanCount2=1
	if task2DectedNum<4 then
		repeat
			local platformInfo=infoTableOfPlatformSensors()
			local rightSensor=platformInfo[2]
			local frontSensor=platformInfo[3]
			--移动
			GoStraight(10,0.1)
			
			if rightSensor.result==1 then
				if(robot.ifHandleIsTask2Obj(rightSensor.ObjHandle)) then
					tempPosintask2={0,0,0}
					tempRotintask2={0,0,0}
					
					--simExtPrintInfo('rightSensor detected task2Obj '..ObjNumintask2)
					tempPosintask2=robot.simGetObjectPosition(rightSensor.ObjHandle,tempPosintask2)
					tempRotintask2=robot.simGetObjectOrientation(rightSensor.ObjHandle,tempRotintask2)
					--simExtPrintInfo('tempPos: '..tempPosintask2[1]..'  '..tempPosintask2[2])
					local ifinTable=judgeIfinTable(tempPosintask2[1],tempPosintask2[2])	
					if ifinTable==false then
						ifDetected=true
						if ObjNumintask2==1 then
							ObjcompareNum=1
						elseif ObjNumintask2<5 then
							ObjcompareNum=ObjcompareNum+1
						end
						
						task2ObjPos[ObjNumintask2][1]=tempPosintask2[1]
						task2ObjPos[ObjNumintask2][2]=tempPosintask2[2]
						task2ObjRot[ObjNumintask2][3]=tempRotintask2[3]
						simExtPrintInfo('Obj '..ObjNumintask2..'\tX'..tostring(task2ObjPos[ObjNumintask2][1])..'\tY'..tostring(task2ObjPos[ObjNumintask2][2])..'\tYaw'..tostring(task2ObjRot[ObjNumintask2][3]))
						ObjNumintask2=ObjNumintask2+1
						
						task2DectedNum=task2DectedNum+1
					end
				end
			elseif rightSensor.result==0 then
				simExtPrintInfo('\tNot detected')
			end

			if frontSensor.result==1 then
				simExtPrintInfo('obs')
				if frontSensor.distance<0.5 then
					simExtPrintInfo('\tObstacle')
					ifObstacle=true
				end
			end
			scanCount2=scanCount2+1
		until(scanCount2==35 or ifObstacle==true)
	end
	
	for i=1,4 do
		simExtPrintInfo(tostring(task2ObjPos[i][1])..'\t'..(task2ObjPos[i][2]))
	end
	simExtPrintInfo('-----scan end-----')
end

function Task2selectObj()--选择X坐标最大的物体，确定二次扫描的起始点，返回表中的对应物体
	local waitMoveObj=0
	local maxX=-50
	
	for i=1,4 do
		if task2ObjPos[i][1]~=0 and task2ObjPos[i][1]>maxX then
			maxX=task2ObjPos[i][1]
			waitMoveObj=i
		end
	end
	simExtPrintInfo('waitmoveObj: '..waitMoveObj)
	if waitMoveObj~=0 then
		task2ScanStartY=task2ObjPos[waitMoveObj][2]-0.2
	end
	return waitMoveObj
end

function Task2detectType(ObjNum)
	--移动到可探测位置
	moveToTarget(task2ObjPos[ObjNum][1],task2ObjPos[ObjNum][2]-1.3,1,1)
	rotateToTarget(0,2,1)
	moveToTarget(task2ObjPos[ObjNum][1],task2ObjPos[ObjNum][2]-1.3,1,1)
	robot.simMakeLeftArmPosture({90,-10,90,0,0,10})
	robot.simMakeRightArmPosture({90,-10,90,0,0,10})
	robot.simTransTwoArmTipInGlobalFrame({0.05,0,0}, {-0.05,0,0})
	moveToTarget(task2ObjPos[ObjNum][1],task2ObjPos[ObjNum][2]-0.85,2)
	
	local type='none'
	--柜子0.198 桌子左0.244 桌子右0.374
	repeat
		leftArminfo=leftArmSensors()
		if leftArminfo[3].result==1 then
			local leftArmPos={0,0,0}
			local height=0
			leftArmPos=getLeftArmPos()
			height=leftArmPos[3]+leftArminfo[3].distance
			simExtPrintInfo('height: '..height)
			if height>0.53 and height<0.57 then
				type='table'
				simExtPrintInfo('this is table')
			elseif height>0.31 and height<0.36 then
				type='chair'
				simExtPrintInfo('this is chair')
			elseif height>0.48 and height<0.52 then
				type='chest'
				simExtPrintInfo('this is chest')
			end
		else
			simExtPrintInfo('result: '..leftArminfo[3].result)
			
			local lefthandle={0,0,0,0,0,0}
			lefthandle=robot.simGetHandlesJointLeftArm(lefthandle)
			local curleftHandAngle1=0
			local curleftHandAngle2=0
			
			local righthandle={0,0,0,0,0,0}
			righthandle=robot.simGetHandlesJointRightArm(righthandle)
			local currightHandAngle1=0
			local currightHandAngle2=0

			curleftHandAngle1=simGetJointPosition(lefthandle[2],curleftHandAngle1)
			curleftHandAngle2=simGetJointPosition(lefthandle[6],curleftHandAngle2)
			
			currightHandAngle1=simGetJointPosition(righthandle[2],currightHandAngle1)
			currightHandAngle2=simGetJointPosition(righthandle[6],currightHandAngle2)
			
			robot.simMakeRightArmPosture({90,1+currightHandAngle1*180/math.pi,90,0,0,currightHandAngle2*180/math.pi-1})
			robot.simMakeLeftArmPosture({90,1+curleftHandAngle1*180/math.pi,90,0,0,curleftHandAngle2*180/math.pi-1})
		end
	until(type~='none')
	return type
end

function Task2transport(ObjNum,type)

	local lefthandle={0,0,0,0,0,0}
	lefthandle=robot.simGetHandlesJointLeftArm(lefthandle)
	local curleftHandAngle1=0
	local curleftHandAngle2=0
	local curleftHandAngle3=0
	local curleftHandAngle4=0
	local curleftHandAngle5=0
	local curleftHandAngle6=0
	
	local righthandle={0,0,0,0,0,0}
	righthandle=robot.simGetHandlesJointRightArm(righthandle)
	local currightHandAngle1=0
	local currightHandAngle2=0
	local currightHandAngle3=0
	local currightHandAngle4=0
	local currightHandAngle5=0
	local currightHandAngle6=0
	
	local pos={0,0,0}
	if type=='table' then
		robot.simMakeRightArmPosture({90,34,90,0,0,-34})
		robot.simMakeLeftArmPosture({90,24,90,0,0,-24})
		repeat
			curleftHandAngle2=simGetJointPosition(lefthandle[2],curleftHandAngle2)
			curleftHandAngle6=simGetJointPosition(lefthandle[6],curleftHandAngle6)
			
			currightHandAngle2=simGetJointPosition(righthandle[2],currightHandAngle2)
			currightHandAngle6=simGetJointPosition(righthandle[6],currightHandAngle6)
			
			robot.simMakeRightArmPosture({90,2+currightHandAngle2*180/math.pi,90,0,0,currightHandAngle6*180/math.pi-2})
			robot.simMakeLeftArmPosture({90,2+curleftHandAngle2*180/math.pi,90,0,0,curleftHandAngle6*180/math.pi-2})
		
			pos=getLeftArmPos()
		until(pos[3]>0.63)
		simExtPrintInfo('leave up table')
	elseif type=='chair' then
		robot.simMakeRightArmPosture({90,-10,90,0,0,10})
		robot.simMakeLeftArmPosture({90,-10,90,0,0,10})
		repeat		
			curleftHandAngle2=simGetJointPosition(lefthandle[2],curleftHandAngle2)
			curleftHandAngle6=simGetJointPosition(lefthandle[6],curleftHandAngle6)
			
			currightHandAngle2=simGetJointPosition(righthandle[2],currightHandAngle2)
			currightHandAngle6=simGetJointPosition(righthandle[6],currightHandAngle6)
			
			robot.simMakeRightArmPosture({90,1+currightHandAngle2*180/math.pi,90,0,0,currightHandAngle6*180/math.pi-1})
			robot.simMakeLeftArmPosture({90,1.25+curleftHandAngle2*180/math.pi,90,0,0,curleftHandAngle6*180/math.pi-1.25})
			
			pos=getLeftArmPos()
		until(pos[3]>0.35)
		simExtPrintInfo('leave up chair')
	elseif type=='chest' then
		robot.simMakeRightArmPosture({90,10,90,0,0,-10})
		robot.simMakeLeftArmPosture({90,10,90,0,0,-10})
		repeat		
			curleftHandAngle1=simGetJointPosition(lefthandle[1],curleftHandAngle1)
			curleftHandAngle2=simGetJointPosition(lefthandle[2],curleftHandAngle2)
			curleftHandAngle3=simGetJointPosition(lefthandle[3],curleftHandAngle3)
			curleftHandAngle4=simGetJointPosition(lefthandle[4],curleftHandAngle4)
			curleftHandAngle5=simGetJointPosition(lefthandle[5],curleftHandAngle5)
			curleftHandAngle6=simGetJointPosition(lefthandle[6],curleftHandAngle6)
			
			currightHandAngle1=simGetJointPosition(righthandle[1],currightHandAngle1)
			currightHandAngle2=simGetJointPosition(righthandle[2],currightHandAngle2)
			currightHandAngle3=simGetJointPosition(righthandle[3],currightHandAngle3)
			currightHandAngle4=simGetJointPosition(righthandle[4],currightHandAngle4)
			currightHandAngle5=simGetJointPosition(righthandle[5],currightHandAngle5)
			currightHandAngle6=simGetJointPosition(righthandle[6],currightHandAngle6)
			
			robot.simMakeRightArmPosture({90,1+currightHandAngle2*180/math.pi,90,0,0,currightHandAngle6*180/math.pi-1})
			robot.simMakeLeftArmPosture({90,1.25+curleftHandAngle2*180/math.pi,90,0,0,curleftHandAngle6*180/math.pi-1.25})
			pos=getLeftArmPos()
		until(pos[3]>0.55)
		simExtPrintInfo(''..curleftHandAngle1..' '..curleftHandAngle2..' '..curleftHandAngle3..' '..curleftHandAngle4..' '..curleftHandAngle5..' '..curleftHandAngle6)
		simExtPrintInfo(''..currightHandAngle1..' '..currightHandAngle2..' '..currightHandAngle3..' '..currightHandAngle4..' '..currightHandAngle5..' '..currightHandAngle6)
	end
	posprint()
	if ObjhasbeenMove==0 then
		moveToTarget(-0.8,-5,2)
		rotateToTarget(-math.pi/2,2)
		rotateToTarget(-math.pi+0.001,2)
		moveToTarget(-0.4,-5.2,2)
		GoAndTurn(4,0.8,2.25,-math.pi/2,'r')
		posprint()
		moveToTarget(-2.05,-7.45,2)
		rotateToTarget(math.pi/2,2)
		
	elseif 	ObjhasbeenMove==1 then
		moveToTarget(-0.8,-5,2)
		rotateToTarget(-math.pi/2,2)
		rotateToTarget(-math.pi+0.001,2)
		moveToTarget(-0.4,-5.2,2)
		GoAndTurn(4,0.4,2.25,-math.pi/2,'r')
		posprint()
		moveToTarget(-1.15,-7.45,2)
		rotateToTarget(math.pi/2,2)
		
	elseif ObjhasbeenMove==2 then
		moveToTarget(-0.8,-5,2)
		rotateToTarget(-math.pi/2,2)
		rotateToTarget(-math.pi+0.001,2)
		moveToTarget(-0.8,-7,2)
		posprint()
		
	elseif ObjhasbeenMove==3 then
		moveToTarget(-0.8,-5,2)
		rotateToTarget(-math.pi/2,2)
		rotateToTarget(-math.pi+0.001,2)
		moveToTarget(-0.8,-6,2)
		posprint()
	end
	
	
	if type=='table' then
		repeat
			curleftHandAngle1=simGetJointPosition(lefthandle[2],curleftHandAngle1)
			curleftHandAngle2=simGetJointPosition(lefthandle[6],curleftHandAngle2)
			
			currightHandAngle1=simGetJointPosition(righthandle[2],currightHandAngle1)
			currightHandAngle2=simGetJointPosition(righthandle[6],currightHandAngle2)
			
			robot.simMakeRightArmPosture({90,-2+currightHandAngle1*180/math.pi,90,0,0,currightHandAngle2*180/math.pi+2})
			robot.simMakeLeftArmPosture({90,-2+curleftHandAngle1*180/math.pi,90,0,0,curleftHandAngle2*180/math.pi+2})
		
			pos=getLeftArmPos()
		until(pos[3]<0.4)
		robot.simMakeRightArmPosture({90,0,90,0,0,0})
		robot.simMakeLeftArmPosture({90,0,90,0,0,0})
		
	elseif type=='chair' then
		repeat
			curleftHandAngle1=simGetJointPosition(lefthandle[2],curleftHandAngle1)
			curleftHandAngle2=simGetJointPosition(lefthandle[6],curleftHandAngle2)
			
			currightHandAngle1=simGetJointPosition(righthandle[2],currightHandAngle1)
			currightHandAngle2=simGetJointPosition(righthandle[6],currightHandAngle2)
			
			robot.simMakeRightArmPosture({90,-2+currightHandAngle1*180/math.pi,90,0,0,currightHandAngle2*180/math.pi+2})
			robot.simMakeLeftArmPosture({90,-2+curleftHandAngle1*180/math.pi,90,0,0,curleftHandAngle2*180/math.pi+2})
		
			pos=getLeftArmPos()
		until(pos[3]<0.25)
		robot.simMakeRightArmPosture({90,-10,90,0,0,10})
		robot.simMakeLeftArmPosture({90,-10,90,0,0,10})
		
	elseif type=='chest' then
		repeat
			curleftHandAngle1=simGetJointPosition(lefthandle[2],curleftHandAngle1)
			curleftHandAngle2=simGetJointPosition(lefthandle[6],curleftHandAngle2)
			
			currightHandAngle1=simGetJointPosition(righthandle[2],currightHandAngle1)
			currightHandAngle2=simGetJointPosition(righthandle[6],currightHandAngle2)
			
			robot.simMakeRightArmPosture({90,-2+currightHandAngle1*180/math.pi,90,0,0,currightHandAngle2*180/math.pi+2})
			robot.simMakeLeftArmPosture({90,-2+curleftHandAngle1*180/math.pi,90,0,0,curleftHandAngle2*180/math.pi+2})
		
			pos=getLeftArmPos()
		until(pos[3]<0.3)
	end
	

	if ObjhasbeenMove==0 then
		moveToTarget(-0.2,-7.45)
		robot.simMakeRightArmPosture({90,0,90,0,0,90})
		robot.simMakeLeftArmPosture({90,0,90,0,0,90})
		robot.simTransTwoArmTipInGlobalFrame({0.15,0,0}, {0.15,0,0})
		rotateToTarget(0.001)
	elseif ObjhasbeenMove==1 then
		moveToTarget(-0.2,-7.45)
		robot.simMakeRightArmPosture({90,0,90,0,0,90})
		robot.simMakeLeftArmPosture({90,0,90,0,0,90})
		robot.simTransTwoArmTipInGlobalFrame({0.15,0,0}, {0.15,0,0})
		rotateToTarget(0.001)
	elseif ObjhasbeenMove==2 then
		robot.simMakeRightArmPosture({90,0,90,0,0,0})
		robot.simMakeLeftArmPosture({90,0,90,0,0,0})
		moveToTarget(-0.8,-5.5)
		robot.simMakeRightArmPosture({90,0,90,0,0,90})
		robot.simMakeLeftArmPosture({90,0,90,0,0,90})
		robot.simTransTwoArmTipInGlobalFrame({0,0.15,0}, {0,0.15,0})
		rotateToTarget(0.001)
	elseif ObjhasbeenMove==3 then
		robot.simMakeRightArmPosture({90,0,90,0,0,0})
		robot.simMakeLeftArmPosture({90,0,90,0,0,0})
		moveToTarget(-0.8,-4.5)
		robot.simMakeRightArmPosture({90,0,90,0,0,90})
		robot.simMakeLeftArmPosture({90,0,90,0,0,90})
		robot.simTransTwoArmTipInGlobalFrame({0,0.15,0}, {0,0.15,0})
		rotateToTarget(0.001)
	end
	
	moveToTarget(-0.2,-5.2,1,1)

	task2ObjPos[ObjNum][1]=0
	task2ObjPos[ObjNum][2]=0
end
leftArmFingerClip=function()
	jointFingerHandlers={-1,-1}
	jointFingerHandlers = robot.simGetHandlesJointLeftFinger(jointFingerHandlers)

	forceSensorHandlers={-1,-1}
	forceSensorHandlers=robot.simGetHandlesForceSensorLeftFinger(forceSensorHandlers)
	repeat
		robot.simSetJointTargetVelocity(jointFingerHandlers[1],0.005)
		robot.simSetJointTargetVelocity(jointFingerHandlers[2],0.005)

	    forceVector={0,0,0}
		torqueVector={0,0,0}
		forceVector,torqueVector=robot.simGetForceSensorResult(forceSensorHandlers[1],forceVector,torqueVector)
		force1 = math.sqrt(forceVector[1]*forceVector[1] + forceVector[2]*forceVector[2] + forceVector[3]*forceVector[3])

	    forceVector={0,0,0}
		torqueVector={0,0,0}
		forceVector,torqueVector=robot.simGetForceSensorResult(forceSensorHandlers[2],forceVector,torqueVector)
		force2 = math.sqrt(forceVector[1]*forceVector[1] + forceVector[2]*forceVector[2] + forceVector[3]*forceVector[3])

	until (force1 > 20 and force2 > 20)--force1 > 20 and force2 > 20

end
rightArmFingerClip=function()
	jointFingerHandlers={-1,-1}
	jointFingerHandlers = robot.simGetHandlesJointRightFinger(jointFingerHandlers)

	forceSensorHandlers={-1,-1}
	forceSensorHandlers=robot.simGetHandlesForceSensorRightFinger(forceSensorHandlers)
	repeat
		robot.simSetJointTargetVelocity(jointFingerHandlers[1],0.005)
		robot.simSetJointTargetVelocity(jointFingerHandlers[2],0.005)

	    forceVector={0,0,0}
		torqueVector={0,0,0}
		forceVector,torqueVector=robot.simGetForceSensorResult(forceSensorHandlers[1],forceVector,torqueVector)
		force1 = math.sqrt(forceVector[1]*forceVector[1] + forceVector[2]*forceVector[2] + forceVector[3]*forceVector[3])

	    forceVector={0,0,0}
		torqueVector={0,0,0}
		forceVector,torqueVector=robot.simGetForceSensorResult(forceSensorHandlers[2],forceVector,torqueVector)
		force2 = math.sqrt(forceVector[1]*forceVector[1] + forceVector[2]*forceVector[2] + forceVector[3]*forceVector[3])

	until (force1 > 20 and force2 > 20)--force1 > 20 and force2 > 20

end

leftArmFingerOpen=function()
	jointFingerHandlers={-1,-1}
	jointFingerHandlers = robot.simGetHandlesJointLeftFinger(jointFingerHandlers)

	robot.simSetJointTargetVelocity(jointFingerHandlers[1],-0.05)
	robot.simSetJointTargetVelocity(jointFingerHandlers[2],-0.05)
end

rightArmFingerOpen=function()
	jointFingerHandlers={-1,-1}
	jointFingerHandlers = robot.simGetHandlesJointRightFinger(jointFingerHandlers)

	robot.simSetJointTargetVelocity(jointFingerHandlers[1],-0.05)
	robot.simSetJointTargetVelocity(jointFingerHandlers[2],-0.05)
end


function task1()
	simExtPrintInfo('-----Task1-----')
	--收手
	robot.simTransTwoArmTipInGlobalFrame({-0.15,0,0}, {-0.15,0,0})
	--穿过第一通道
	GoStraight(10,4)
	--穿过第一弯道
	GoAndTurn(10,1.3,1.3,-math.pi/2,'r')
	stopMove()
	--调整姿态
	rotateToTarget(-math.pi)
	--穿过第二弯道
	GoStraight(10,1.6)
	stopMove()
	--调整姿态
	moveToTarget(2.0,-1.4)
	rotateToTarget(3*math.pi/4+math.pi/24)
	posprint()
	--穿过第二弯道
	GoStraight(10,2.9)
	--调整到任务二起始标准位置
	moveToTarget(-0.1,-4.9)
	rotateToTarget(math.pi/2)
	
	simExtPrintInfo('-----Task1End-----')
	posprint()
end

function task2()
	local whichToMove=0
	local objtype=' '
	for i=1,4 do
		whichToMove=Task2selectObj()
		if whichToMove==0 then
			Task2scan(0.1,task2ScanStartY,math.pi/2)
			whichToMove=Task2selectObj()
		end
		objtype=Task2detectType(whichToMove)
		Task2transport(whichToMove,objtype)
		stopMove()
		simExtPrintInfo('-----round '..i..' end-----')
		ObjhasbeenMove=ObjhasbeenMove+1
	end
end

function task21()
    --机械手臂收回
	robot.simMakeLeftArmPosture({90,-90,90,90,90,0})
	robot.simMakeLeftArmPosture({90,-90,-40,-90,70,-90})
	robot.simMakeRightArmPosture({90,-90,90,-90,90,0})
	robot.simMakeRightArmPosture({90,-75,-40,58,90,-90})
	--robot.simSetPlatformPositionAndOrientation({-1,-5.8,0},math.pi)

	moveToTarget(-1,-5.8)
	rotateToTarget(0)
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
				info=infoTableOfPlatformSensors()
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
				if rightSensor.result==1 then
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
						info=infoTableOfPlatformSensors()
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
						info=infoTableOfPlatformSensors()
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
						if leftSensor.result==1 then
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
						info=infoTableOfPlatformSensors()
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
						info=infoTableOfPlatformSensors()
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
						if rightSensor.result==1 then
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
	--simExtPrintInfo('distance:'..tostring(distance))
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
						simExtPrintInfo('enter LF')
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
						simExtPrintInfo('enter RF')
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
						simExtPrintInfo('enter RB')
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
					simExtPrintInfo('enter LF R=0')
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
					simExtPrintInfo('enter LB R=0')
					break
				end
			end 
			rotateToTarget(-math.pi/2,0,0)
		end
	end
	robotpos=robot.simGetObjectPosition(handleTipPlatform, robotpos)
	simExtPrintInfo('succeed to complate')
	moveToTarget(robotpos[1],-7.85,0,0)
	moveToTarget(-8,-7.85,0,0)
	simExtPrintInfo('succeed to end\n')
	simExtPrintInfo(all_targetpos1[1]..' '..all_targetpos1[2]..' '..all_targetpos1[3]..' '..all_targetpos1[4]..' '..all_targetpos1[5])
	simExtPrintInfo(all_targetpos2[1]..' '..all_targetpos2[2]..' '..all_targetpos2[3]..' '..all_targetpos2[4]..' '..all_targetpos2[5])
	fprint(all_targetpos1[1]..' '..all_targetpos1[2]..' '..all_targetpos1[3]..' '..all_targetpos1[4]..' '..all_targetpos1[5])
	fprint(all_targetpos2[1]..' '..all_targetpos2[2]..' '..all_targetpos2[3]..' '..all_targetpos2[4]..' '..all_targetpos2[5])
end

function task4_1()

    --robot.simSetPlatformPositionAndOrientation({-11,-8.17,0.1430},math.pi)
    moveToTarget(-11,-8.17)
	rotateToTarget(math.pi/2)
	robot.simMakeRightArmPosture({90,-90,90,-90,90,0})
	robot.simMakeLeftArmPosture({90,-90,90,90,90,0})
	robot.simMakeRightArmPosture({90,0,90,0,0,90})
	robot.simMakeLeftArmPosture({90,0,90,0,0,90})
	robot.simSetCurrentTaskIndex(4)
	local handle={0,0}

    handle=robot.simGetHandleJointLeftWrist(handle)

	robot.simSetJointTargetPosition(handle[1],math.rad(0))
	robot.simSetJointTargetPosition(handle[2],math.rad(0))
	
	

	
	--search gastank by detection
    local detectedPoint={0,0,0}
	local detectedNormal={0,0,0}
	local detectionState=-1
	local detectedObjHandle=-1
    local distance_left=0
	local isgasmark=false
	
	local gastankpos={0,0,0}
	local gastankmark1={0,0,0}
    local gastankmark2={0,0,0}
	
	local LeftArmHandle={0,0,0,0,0,0}
	local RightArmHandle={0,0,0,0,0,0}
	
    local LeftFingerHandle={0,0}
	local RightFingerHandle={0,0}
	
	local leftFingerpos1={0,0,0}
	local leftFingerpos2={0,0,0}
	local rightFingerpos1={0,0,0}
	local rightFingerpos2={0,0,0}
	
	local leftJointpos1={0,0,0}
	local leftJointpos2={0,0,0}
	local leftJointpos3={0,0,0} 
	local leftJointpos4={0,0,0} 
	local leftJointpos5={0,0,0} 
	local leftJointpos6={0,0,0} 
	
	local rightJointpos1={0,0,0} 
	local rightJointpos2={0,0,0} 
	local rightJointpos3={0,0,0} 
	local rightJointpos4={0,0,0} 
	local rightJointpos5={0,0,0} 
	local rightJointpos6={0,0,0} 
	
	LeftFingerHandle=robot.simGetHandlesForceSensorLeftFinger(LeftFingerHandle)
	RightFingerHandle=robot.simGetHandlesForceSensorLeftFinger(RightFingerHandle)

	local handleSensorPlatform={}
	handleSensorPlatform[3]=robot.simGetHandleProximitySensorPlatformLeft()
	
	while(isgasmark==false)
	do
	  
	  detectionState,detectedPoint,detectedObjHandle,detectedNormal,distance_left=robot.simGetProximitySensorResult(handleSensorPlatform[3], detectedPoint, detectedNormal,distance)
      if detectionState==1 then
	  
	     isgasmark=robot.ifHandleIsGasTank(detectedObjectHandle)
		 if isgasmark==false then 
		   detectionState=-1
		 end
		
	  end
	   log(GoStraight,'Gostraight',3,0.5)
	end
	gastankpos=robot.simGetObjectPosition(detectedObjHandle,gastankpos)	
	gastankmark1=robot.simGetHandleGasTankMark(detectedObjHandle)
	gastankmark2[1]=2*gastankpos[1]-gastankmark1[1]
	gastankmark2[2],gastankmark2[3]=gastankmark1[2],gastankmark1[3]

	moveToTarget(gastankpos[1]+0.01,gastankpos[2]+1)
	
	log(rotateToTarget,'rotateTotarget',math.pi,'r')
	
	stopMove()
    
	local jointangle1=math.asin((0.66412-gastankmark1[3])/0.362)
	
	robot.simMakeLeftArmPosture({jointangle1*180/math.pi-5,10,82,-(jointangle1*180/math.pi-5),0,80})
    robot.simMakeRightArmPosture({jointangle1*180/math.pi-5,10,82,jointangle1*180/math.pi-5,0,80})
	
	robot.simRotateLeftArmTipInLocalFrame({0,0,1},math.rad(5))
	robot.simRotateRightArmTipInLocalFrame({0,0,1},-math.rad(5))
	
	moveToTarget(gastankpos[1],gastankpos[2]+0.268+0.36)
	
	simExtPrintInfo(tostring(gastankmark1[1]))
	
	stopMove()
	
	LeftArmHandle=robot.simGetHandlesJointLeftArm(LeftArmHandle)
	RightArmHandle=robot.simGetHandlesJointRightArm(RightArmHandle)
	simExtPrintInfo(tostring(LeftArmHandle[1]))
	
  repeat 
      leftJointpos1=simGetJointPosition(LeftArmHandle[1],leftJointpos1)/math.pi*180
	  leftJointpos2=simGetJointPosition(LeftArmHandle[2],leftJointpos2)/math.pi*180
	  leftJointpos3=simGetJointPosition(LeftArmHandle[3],leftJointpos3)/math.pi*180
	  leftJointpos4=simGetJointPosition(LeftArmHandle[4],leftJointpos4)/math.pi*180
	  leftJointpos5=simGetJointPosition(LeftArmHandle[5],leftJointpos5)/math.pi*180
	  leftJointpos6=simGetJointPosition(LeftArmHandle[6],leftJointpos6)/math.pi*180
	  

	
	  rightJointpos1=simGetJointPosition(RightArmHandle[1],rightJointpos1)/math.pi*180
	  rightJointpos2=simGetJointPosition(RightArmHandle[2],rightJointpos2)/math.pi*180
	  rightJointpos3=simGetJointPosition(RightArmHandle[3],rightJointpos3)/math.pi*180
	  rightJointpos4=simGetJointPosition(RightArmHandle[4],rightJointpos4)/math.pi*180
	  rightJointpos5=simGetJointPosition(RightArmHandle[5],rightJointpos5)/math.pi*180
	  rightJointpos6=simGetJointPosition(RightArmHandle[6],rightJointpos6)/math.pi*180
	
	
	  robot.simMakeLeftArmPosture({leftJointpos1,leftJointpos2,leftJointpos3+0.5,leftJointpos4,leftJointpos5,leftJointpos6-0.5})
	  robot.simMakeRightArmPosture({rightJointpos1,rightJointpos2,rightJointpos3+0.5,rightJointpos4,rightJointpos5,rightJointpos6-0.5})
	  
	  simExtPrintInfo(tostring(simGetJointPosition(LeftArmHandle[6],rightJointpos6)/math.pi*180))
	  
	  leftFingerpos1=robot.simGetObjectPosition(LeftFingerHandle[1],leftFingerpos1)
	  leftFingerpos2=robot.simGetObjectPosition(LeftFingerHandle[2],leftFingerpos2)

	  simExtPrintInfo(tostring(leftFingerpos1[1]))
	  simExtPrintInfo(tostring(gastankmark1[1]))
	  
	until ((leftFingerpos1[1])<gastankmark1[1])
    
    
	
    leftArmFingerClip()
	rightArmFingerClip()

	for i=1,5 do
	    
        robot.simMakeLeftArmPosture({leftJointpos1,leftJointpos2,leftJointpos3+0.5,leftJointpos4+1.2*i,leftJointpos5,leftJointpos6-0.5})
        robot.simMakeRightArmPosture({rightJointpos1,rightJointpos2,rightJointpos3+0.5,rightJointpos4-1.2*i,rightJointpos5,rightJointpos6-0.5})
        leftArmFingerClip()
	    rightArmFingerClip()
	end
    log(rotateToTarget,'rotateTotarget',0)
	stopMove()
	

	log(moveToTarget,'moveToTarget',-12.13,-7.2)
	
	stopMove()
	
	--rotateToTarget(math.rad(45))
	for i=1,5 do
     	robot.simMakeLeftArmPosture({leftJointpos1,leftJointpos2,leftJointpos3+0.5,leftJointpos4-1.2*i,leftJointpos5,leftJointpos6-0.5})
        robot.simMakeRightArmPosture({rightJointpos1,rightJointpos2,rightJointpos3+0.5,rightJointpos4+1.2*i,rightJointpos5,rightJointpos6-0.5})
	end
	leftArmFingerOpen()
	rightArmFingerOpen()
	
	
	
	robot.simMakeLeftArmPosture({60,-90,90,0,0,90})
    robot.simMakeRightArmPosture({60,-90,90,0,0,90})
	
	log(moveToTarget,'moveToTarget',-12,-8.5)
	
end

function task4_2()

    robot.simMakeLeftArmPosture({90,0,90,0,0,90})--初始化任务四位置

    robot.simMakeRightArmPosture({90,0,90,0,0,90})

	robot.simTransTwoArmTipInGlobalFrame({0,-0.15,0}, {0,-0.15,0})

    rotateToTarget(math.pi)

    moveToTarget(-9.9,-7.9)--初始化开门的位置

	

	jointPos = {90,90,25,0,0,-25}

    robot.simMakeLeftArmPosture(jointPos)--移动机械臂到阀门

	

	local handle={0,0}

    handle=robot.simGetHandleJointLeftWrist(handle)

	robot.simSetJointTargetPosition(handle[1],math.rad(0))
	robot.simSetJointTargetPosition(handle[2],math.rad(0))

	

	

    handle=robot.simGetHandleJointLeftWrist(handle)

	robot.simSetJointTargetPosition(handle[1],math.rad(70))

	

	moveToTarget(-9.9,-8)

	stopMove()



    leftArmFingerClip()--加紧把手

	

	jointPos = {90,90,25,0,0,-40}

	robot.simMakeLeftArmPosture(jointPos)

	

	handle=robot.simGetHandleJointLeftWrist(handle)

	robot.simSetJointTargetPosition(handle[1],math.rad(40))

	

	jointPos = {90,90,25,0,-20,-40}

	robot.simMakeLeftArmPosture(jointPos)

	

	leftArmFingerOpen()--松开手指

	

	moveToTarget(-9.9,-7.5)

	stopMove()

		

	robot.simMakeLeftArmPosture({90,0,90,0,0,90})--初始化任务四位置

    robot.simMakeRightArmPosture({90,0,90,0,0,90})

	robot.simTransTwoArmTipInGlobalFrame({0,0.15,0}, {0,0.15,0})

	

	handle=robot.simGetHandleJointLeftWrist(handle)

	robot.simSetJointTargetPosition(handle[1],math.rad(0))
	
	local handle={0,0}

    handle=robot.simGetHandleJointLeftWrist(handle)

	robot.simSetJointTargetPosition(handle[1],math.rad(0))
	robot.simSetJointTargetPosition(handle[2],math.rad(0))

end

function task5()

    
	local detectedPoint={0,0,0}

	local detectedNormal={0,0,0}

	local detectionState=-1

	local detectedObjHandle=-1

    local distance_left=0

	local isgasmark=false

	

	local gastankpos={0,0,0}

	local gastankmark1={0,0,0}

    local gastankmark2={0,0,0}

	

	local LeftArmHandle={0,0,0,0,0,0}

	local RightArmHandle={0,0,0,0,0,0}

	

    local LeftFingerHandle={0,0}

	local RightFingerHandle={0,0}

	

	local leftFingerpos1={0,0,0}

	local leftFingerpos2={0,0,0}

	local rightFingerpos1={0,0,0}

	local rightFingerpos2={0,0,0}

	

	local leftJointpos1={0,0,0}

	local leftJointpos2={0,0,0}

	local leftJointpos3={0,0,0} 

	local leftJointpos4={0,0,0} 

	local leftJointpos5={0,0,0} 

	local leftJointpos6={0,0,0} 

	

	local rightJointpos1={0,0,0} 

	local rightJointpos2={0,0,0} 

	local rightJointpos3={0,0,0} 

	local rightJointpos4={0,0,0} 

	local rightJointpos5={0,0,0} 

	local rightJointpos6={0,0,0} 

	

	LeftFingerHandle=robot.simGetHandlesForceSensorLeftFinger(LeftFingerHandle)

	RightFingerHandle=robot.simGetHandlesForceSensorLeftFinger(RightFingerHandle)

    rotateToTarget(0)

	moveToTarget(-12,-7.6)

	--get left sensor's handle

	local handleSensorPlatform={}

	isgasmark=false

	handleSensorPlatform[1]=robot.simGetHandleProximitySensorPlatformFront()

	

	--get left sensor's result when go straight

	

	while(isgasmark==false)

	do

	  

	  detectionState,detectedPoint,detectedObjHandle,detectedNormal,distance_front=robot.simGetProximitySensorResult(handleSensorPlatform[1], detectedPoint, detectedNormal,distance)

      if detectionState==1 then

	  

	     isgasmark=robot.ifHandleIsGasTank(detectedObjectHandle)

		 if isgasmark==false then 

	       --fprint('isgasmark=false')

		   --fprint('distance_left\t'..distance_left)

		   detectionState=-1

		 end

		

	  end

	   log(GoStraight,'Gostraight',3,0.3)

	end

	gastankpos=robot.simGetObjectPosition(detectedObjHandle,gastankpos)	

	--fprint('gastankpos:'..gastankpos[1]..'  '..gastankpos[2]..'  '..gastankpos[3])

	--fprint('gastankpos:'..type(gastankpos))

	gastankmark1=robot.simGetHandleGasTankMark(detectedObjHandle)

	--fprint('gastankmark:'..gastankmark1[1]..gastankmark1[2]..gastankmark1[3])

	gastankmark2[1]=2*gastankpos[1]-gastankmark1[1]

	gastankmark2[2],gastankmark2[3]=gastankmark1[2],gastankmark1[3]



	moveToTarget(gastankpos[1],gastankpos[2]-1)

	

	log(rotateToTarget,'rotateTotarget',0)

	

	stopMove()

    

	local jointangle1=math.asin((0.66412-gastankmark1[3])/0.362)

	

	robot.simMakeLeftArmPosture({jointangle1*180/math.pi-5,10,82,-(jointangle1*180/math.pi-5),0,80})

    robot.simMakeRightArmPosture({jointangle1*180/math.pi-5,10,82,jointangle1*180/math.pi-5,0,80})

	

	robot.simRotateLeftArmTipInLocalFrame({0,0,1},math.rad(5))

	robot.simRotateRightArmTipInLocalFrame({0,0,1},-math.rad(5))

	

	moveToTarget(gastankpos[1],gastankpos[2]-0.268-0.36)

	

	simExtPrintInfo(tostring(gastankmark1[1]))

	

	stopMove()

	

	LeftArmHandle=robot.simGetHandlesJointLeftArm(LeftArmHandle)

	RightArmHandle=robot.simGetHandlesJointRightArm(RightArmHandle)

	simExtPrintInfo(tostring(LeftArmHandle[1]))

	

  repeat 

      leftJointpos1=simGetJointPosition(LeftArmHandle[1],leftJointpos1)/math.pi*180

	  leftJointpos2=simGetJointPosition(LeftArmHandle[2],leftJointpos2)/math.pi*180

	  leftJointpos3=simGetJointPosition(LeftArmHandle[3],leftJointpos3)/math.pi*180

	  leftJointpos4=simGetJointPosition(LeftArmHandle[4],leftJointpos4)/math.pi*180

	  leftJointpos5=simGetJointPosition(LeftArmHandle[5],leftJointpos5)/math.pi*180

	  leftJointpos6=simGetJointPosition(LeftArmHandle[6],leftJointpos6)/math.pi*180

	  



	

	  rightJointpos1=simGetJointPosition(RightArmHandle[1],rightJointpos1)/math.pi*180

	  rightJointpos2=simGetJointPosition(RightArmHandle[2],rightJointpos2)/math.pi*180

	  rightJointpos3=simGetJointPosition(RightArmHandle[3],rightJointpos3)/math.pi*180

	  rightJointpos4=simGetJointPosition(RightArmHandle[4],rightJointpos4)/math.pi*180

	  rightJointpos5=simGetJointPosition(RightArmHandle[5],rightJointpos5)/math.pi*180

	  rightJointpos6=simGetJointPosition(RightArmHandle[6],rightJointpos6)/math.pi*180

	

	

	  robot.simMakeLeftArmPosture({leftJointpos1,leftJointpos2,leftJointpos3+0.5,leftJointpos4,leftJointpos5,leftJointpos6-0.5})

	  --simExtPrintInfo('succeed2')

	  --simExtPrintInfo(tostring(simGetJointPosition(LeftArmHandle[6],leftJointpos6)/math.pi*180))

	  robot.simMakeRightArmPosture({rightJointpos1,rightJointpos2,rightJointpos3+0.5,rightJointpos4,rightJointpos5,rightJointpos6-0.5})

	  

	  simExtPrintInfo(tostring(simGetJointPosition(LeftArmHandle[6],rightJointpos6)/math.pi*180))

	  

	  leftFingerpos1=robot.simGetObjectPosition(LeftFingerHandle[1],leftFingerpos1)

	  leftFingerpos2=robot.simGetObjectPosition(LeftFingerHandle[2],leftFingerpos2)

	  



	  --rightFingerpos1=robot.simGetObjectPosition(RightFingerHandle,rightFingerpos1)

	  



	  simExtPrintInfo(tostring(leftFingerpos1[1]))

	  simExtPrintInfo(tostring(gastankmark1[1]))

	  

	until ((leftFingerpos1[1])>gastankmark1[1])

    

    

	

    leftArmFingerClip()

	rightArmFingerClip()



	for i=1,5 do

	    

        robot.simMakeLeftArmPosture({leftJointpos1,leftJointpos2,leftJointpos3+0.5,leftJointpos4+1.2*i,leftJointpos5,leftJointpos6-0.5})

        robot.simMakeRightArmPosture({rightJointpos1,rightJointpos2,rightJointpos3+0.5,rightJointpos4-1.2*i,rightJointpos5,rightJointpos6-0.5})

        leftArmFingerClip()

	    rightArmFingerClip()

	end

    log(rotateToTarget,'rotateTotarget',-math.pi/2,'r')

	--log(rotateToTarget,'rotateTotarget',-math.pi,'r')

	stopMove()

	

	moveToTarget(-12.2,-13)
	
    log(rotateToTarget,'rotateTotarget',-math.pi,'r')
	
	moveToTarget(-6,-13)

	

	for i=1,5 do

     	robot.simMakeLeftArmPosture({leftJointpos1,leftJointpos2,leftJointpos3+0.5,leftJointpos4-1.2*i,leftJointpos5,leftJointpos6-0.5})

        robot.simMakeRightArmPosture({rightJointpos1,rightJointpos2,rightJointpos3+0.5,rightJointpos4+1.2*i,rightJointpos5,rightJointpos6-0.5})

	end

	leftArmFingerOpen()

	rightArmFingerOpen()

	

	robot.simMakeLeftArmPosture({60,-90,90,0,0,90})

    robot.simMakeRightArmPosture({60,-90,90,0,0,90})

end


cleanlog()
fopen()
globalInit()
task1()
task2()
task21()
task3()
task4_1()
task4_2()
task5()
stopMove()
fclose()