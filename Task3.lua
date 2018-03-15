--打印当前系统时间
function tprint()
	file=io.open("G:/log.txt",'a')
	local t=os.date("%Y-%m-%d  %H:%M:%S\n\n",os.time())
	str=t
	io.output(file)
	io.write(str)
	io.close(file)
end

--打印字符串
function fprint(str)
	file=io.open("G:/log.txt",'a')
	str=str..'\n'
	io.output(file)
	io.write(str)
	io.close(file)
end

function log(...)--检查语法错误与逻辑错误
	local arg={...}
	length=table.getn(arg)

	func,funcname,one,two,three,four,five,six,seven=arg[1],arg[2],arg[3],arg[4],arg[5],arg[6],arg[7],arg[8],arg[9],arg[10]

	statue,err=pcall(func,one,two,three,four,five,six,seven)

	if(not statue)
	then
		fprint('function  '..funcname..'  error')
		fprint(err)
		fprint(debug.traceback()..'\n')
	end
end

--清除Log
function cleanlog()
	file=io.open("G:/log.txt",'w+')
	io.close(file)
end

function stopMove()
  	--将转动速度降低为0
    robot.simSetJointTargetVelocity(handleJointWheelLF, 0)
    robot.simSetJointTargetVelocity(handleJointWheelRF, 0)
    robot.simSetJointTargetVelocity(handleJointWheelLB, 0)
    robot.simSetJointTargetVelocity(handleJointWheelRB, 0)
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

--精确运动到给定位置(全局坐标系)，其姿态不变,传入参数为(目标点X,目标点Y)
--定位精度为1cm(可根据需求调整)
--根据距离计算移动速度
function moveToTarget(targetX,targetY)
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

	local curPos = {0,0,0}
	local curRot = {0,0,0}
	curPos = robot.simGetObjectPosition(handleTipPlatform, curPos)
	
	local dX=targetX-curPos[1]
	local dY=targetY-curPos[2]
	
	local angle1=0
	local angle=0
	local distance=0
	local moveVel=0
	
	local P=10
	local I=1
	local D=1
	
	--fprint('initPos  \tX: '..curPos[1]..'\tY: '..curPos[2])
	--fprint('targetPos\tX: '..targetX..'\tY: '..targetY)
	repeat
		curPos = robot.simGetObjectPosition(handleTipPlatform, curPos)
		curRot = robot.simGetObjectOrientation(handleTipPlatform,curRot)
		
		angle1=math.atan2(dY,dX)
		angle=math.pi/2+curRot[3]-angle1
		dX=targetX-curPos[1]
		dY=targetY-curPos[2]
		distance=math.sqrt(dX*dX+dY*dY)
		
		--fprint('dX\t'..dX..'\t'..dY)
		--fprint('theta\t'..angle1*180/math.pi..'\tYaw'..curRot[3]*180/math.pi..'\tAngle\t'..angle*180/math.pi)
		
		robot.simSetJointTargetPosition(handleJointWheelLF0, -angle)
		robot.simSetJointTargetPosition(handleJointWheelRB0, -angle)
		robot.simSetJointTargetPosition(handleJointWheelRF0, -angle)
		robot.simSetJointTargetPosition(handleJointWheelLB0, -angle)
		
		moveVel=P*distance
		
		if moveVel<5  then
			moveVel=5
		else if moveVel>20 then
			moveVel=20
			end
		end

		robot.simSetJointTargetVelocity(handleJointWheelLF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRF, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelLB, moveVel)
		robot.simSetJointTargetVelocity(handleJointWheelRB, moveVel)
	
	until(math.abs(curPos[1]-targetX)<0.01 and math.abs(curPos[2]-targetY)<0.01)

	--fprint('\tArrive')
	--tprint()
end

function rotateToTarget(targetYaw)--转到目标位置

	if targetYaw>math.pi then
		targetYaw=targetYaw-2*math.pi
	end
	if targetYaw<-math.pi then
		targetYaw=targetYaw+2*math.pi
	end
	
	local rotVel=3
	handleTipPlatform = robot.simGetHandleVehicle()
	handleJointWheelLF0 = robot.simGetHandleJointTurnLeftFront()
	handleJointWheelRB0 = robot.simGetHandleJointTurnRightBack()
	handleJointWheelRF0 = robot.simGetHandleJointTurnRightFront()
	handleJointWheelLB0 = robot.simGetHandleJointTurnLeftBack()

	handleJointWheelLF = robot.simGetHandleJointWheelLeftFront()
	handleJointWheelRF = robot.simGetHandleJointWheelRightFront()
	handleJointWheelLB = robot.simGetHandleJointWheelLeftBack()
	handleJointWheelRB = robot.simGetHandleJointWheelRightBack()

	robot.simSetJointTargetPosition(handleJointWheelLF0, -47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelRB0, -47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelRF0, 47.38/180.0*math.pi)
    robot.simSetJointTargetPosition(handleJointWheelLB0, 47.38/180.0*math.pi)

	local initRot = {0,0,0}
    initRot = robot.simGetObjectOrientation(handleTipPlatform,initRot)
	--fprint('initYaw  : '..initRot[3])
	--fprint('targetYaw: '..targetYaw)
	local newRot={0,0,0}
	local Yaw=0
	local P=10
	local direction='r'


	
	if math.abs(targetYaw-initRot[3])>math.pi then
		if targetYaw<0 then
			direction='l'
			--fprint('1')
		end
		if targetYaw>0 then
			direction='r'
			fprint('2')
		end
	end
	if math.abs(targetYaw-initRot[3])<math.pi then
		if targetYaw>initRot[3] then
			direction='l'
		end
		if targetYaw<initRot[3] then
			direction='r'
		end
	end

	repeat
		newRot=robot.simGetObjectOrientation(handleTipPlatform,newRot)
		Yaw=newRot[3]
		
		dYaw=math.abs(targetYaw-Yaw)
		
		if Yaw<0 and targetYaw>0 and direction=='r' then
			dYaw=Yaw+math.pi+math.pi-targetYaw
		end
		if Yaw>0 and targetYaw<0 and direction=='l' then
			dYaw=targetYaw+ math.pi +math.pi-Yaw
		end
		
		rotVel=P*dYaw
		if rotVel>10 then
			rotVel=10
		end
		if rotVel<2 then
			rotVel=2
		end
		
		--fprint('direction\t'..direction)
		
		if(direction=='r') then
			robot.simSetJointTargetVelocity(handleJointWheelLF, rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelRF, -rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelLB, rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelRB, -rotVel)
		end
		
		if(direction=='l') then
			robot.simSetJointTargetVelocity(handleJointWheelLF, -rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelRF, rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelLB, -rotVel)
			robot.simSetJointTargetVelocity(handleJointWheelRB, rotVel)
		end
	until(math.abs(Yaw-targetYaw)<(1/180*math.pi))
	
	robot.simSetJointTargetVelocity(handleJointWheelLF, 0)
    robot.simSetJointTargetVelocity(handleJointWheelRF, 0)
	robot.simSetJointTargetVelocity(handleJointWheelLB, 0)
	robot.simSetJointTargetVelocity(handleJointWheelRB, 0)
	--fprint('\tarrive')
	--tprint()
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
	rotateToTarget(theta)
	
	--移动到圆上
	moveToTarget(tarPos[1],tarPos[2])
	stopMove()
	
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
		
		lfVel=math.sqrt((objX-lfPos[1])*(objX-lfPos[1])+(objY-lfPos[2])*(objY-lfPos[2]))
		lbVel=math.sqrt((objX-lbPos[1])*(objX-lbPos[1])+(objY-lbPos[2])*(objY-lbPos[2]))
		rfVel=math.sqrt((objX-rfPos[1])*(objX-rfPos[1])+(objY-rfPos[2])*(objY-rfPos[2]))
		rbVel=math.sqrt((objX-rbPos[1])*(objX-rbPos[1])+(objY-rbPos[2])*(objY-rbPos[2]))
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
	
		lfVel=math.sqrt((objX-lfPos[1])*(objX-lfPos[1])+(objY-lfPos[2])*(objY-lfPos[2]))
		lbVel=math.sqrt((objX-lbPos[1])*(objX-lbPos[1])+(objY-lbPos[2])*(objY-lbPos[2]))
		rfVel=math.sqrt((objX-rfPos[1])*(objX-rfPos[1])+(objY-rfPos[2])*(objY-rfPos[2]))
		rbVel=math.sqrt((objX-rbPos[1])*(objX-rbPos[1])+(objY-rbPos[2])*(objY-rbPos[2]))
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

	--计算目标全局位置
	local targetX=0
	local targetY=0
	local targetYaw=0

	targetX,targetY,targetYaw=calAbsoluteTarget(initPos,initRot[3],x0,y0,yaw)
	--小车当前全局位置
	local curentX=0
	local curentY=0
	local curentYaw=0
	--小车与目标相对位置
	local relativeX=0
	local relativeY=0
	local relativeYaw=0

	local newPos = {0,0,0}
	local newRot = {0,0,0}

	if y0==0 then
		y0=0.001
	end

	---fprint('****************GoAndTurn*****************\n')

	repeat
		fprint(direction)
		if direction=='r' then
			--计算相对位置
			fprint('in')
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

			fprint('radius:\t'..r..'\n(X,Y):\t('..x1..','..y1..')'..'\n')
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

			fprint('radius:\t'..r..'\n(X,Y):\t('..x1..','..y1..')'..'\n')

		end

		robot.simSetJointTargetPosition(handleJointWheelLF0, LFang)
		robot.simSetJointTargetPosition(handleJointWheelRB0, RBang)
		robot.simSetJointTargetPosition(handleJointWheelRF0, RFang)
		robot.simSetJointTargetPosition(handleJointWheelLB0, LBang)

		robot.simSetJointTargetVelocity(handleJointWheelLF, LFvel)
		robot.simSetJointTargetVelocity(handleJointWheelRF, RFvel)
		robot.simSetJointTargetVelocity(handleJointWheelLB, LBvel)
		robot.simSetJointTargetVelocity(handleJointWheelRB, RBvel)

		--一帧的运动结束后重新测位置并保存
		newPos = robot.simGetObjectPosition(handleTipPlatform, newPos)
		newRot = robot.simGetObjectOrientation(handleTipPlatform,newRot)

		--fprint('newPos:\t'..'('..newPos[1]..','..newPos[2]..')')
		--fprint('Yaw:\t'..newRot[3])
		--fprint('U:\t'..LFvel..' '..LBvel..' '..RFvel..' '..RBvel..' '..LFang..' '..LBang..' '..RFang..' '..RBang)

	until(math.abs(targetX-newPos[1])<0.05 and math.abs(targetY-newPos[2])<0.05)

	--fprint('\n******************End*****************\n\n')

end


function infoTableOfPlatformSensors()
    --获得机身传感器句柄
	handleSensorPlatform = {}
	handleSensorPlatform[1] = robot.simGetHandleProximitySensorPlatformLeft()
	handleSensorPlatform[2] = robot.simGetHandleProximitySensorPlatformRight()
	handleSensorPlatform[3] = robot.simGetHandleProximitySensorPlatformFront()
	handleSensorPlatform[4] = robot.simGetHandleProximitySensorPlatformBack()
	
	
	--[[--获取机器人底座单线传感器左右旋转控制铰接，顺时针为正
    handleSensorPlatform[5] = robot.simGetHandleJointSensorPlatform1()--frontjoint0
	handleSensorPlatform[6] = robot.simGetHandleJointSensorPlatform2()--rightjoint0
	handleSensorPlatform[7]  = robot.simGetHandleJointSensorPlatform3()--backjoint0
	handleSensorPlatform[4]  = robot.simGetHandleJointSensorPlatform4()--leftjoint0
	
	--获取机器人底座单线传感器俯仰旋转控制铰接，向上为正
	handleSensorPlatform[8] = robot.simGetHandleJointSensorPlatformPitching1()--frontjoint
	handleSensorPlatform[9] = robot.simGetHandleJointSensorPlatformPitching2()--rightjoint
	handleSensorPlatform[10]  = robot.simGetHandleJointSensorPlatformPitching3()--backjoint
	handleSensorPlatform[11]  = robot.simGetHandleJointSensorPlatformPitching4()--leftjoint--]]

	info = {{},{},{},{}}
	detectedPoint={0,0,0}
	detectedSurfaceNormalVector={0,0,0}
	--distance=0
	--result=0
	--detectedObjectHandle=-1
	

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

--function test()
	--movetoTarget(-2.3,6.71)
	--rotateToTarget(math.pi/2,'l')
--end


function task1()
    --机械手臂收回
	leftarm = {90,-20,90,-90,90,90}
    robot.simMakeLeftArmPosture(leftarm)
	rightarm = {90,-20,90,90,90,90}
    robot.simMakeRightArmPosture(rightarm)
	
	GoStraight(10,4)
	GoAndTurn(10,1.3,1.3,-math.pi/2,'r')
	stopMove()
	rotateToTarget(-math.pi)
	GoStraight(10,1.6)
	rotateToTarget(3*math.pi/4+math.pi/20)
	GoStraight(10,4)
	
	rotateToTarget(0)
	moveToTarget(-1,-6.3)
	rotateToTarget(0)
	--GoStraight(10,2)
	--rotateToTarget(0)
	simExtPrintInfo(tostring('succeed to start'))
	
end
--上下行扫描的函数（利用左右传感器）
function Scan(ifchangedir,downy,targetposx,count)
    local ifObjfind=0   --是否找到障碍物1
    local info = {{},{},{},{}}
    local rightSensor=0
	local leftSensor=0
	local frontSensor=0
	--local ifchangedir=0  --是否换向标志
	local pos = {0,0,0}
	local rot={0,0,0}
	local targetpos={0,0,0}
	--local rvalue={}
	---local count =0--计数
	handleTipPlatform = robot.simGetHandleVehicle()
    repeat
		if ifchangedir==1 and ifObjfind==0 then
			--simExtPrintInfo('achieve turn')
			handleTipPlatform=robot.simGetHandleVehicle()
			pos=robot.simGetObjectPosition(handleTipPlatform,pos)
			info=infoTableOfPlatformSensors()
			frontSensor=info[3]
			leftSensor=info[1]
			
			if frontSensor.result==0 and leftSensor.result==0 then
				if pos[2]>-4.5 then
					log(GoStraight,'GoStraight',5,pos[2]+4.7)
					rotateToTarget(math.pi/2)
					for i=1,3 do
						info=infoTableOfPlatformSensors()
						frontSensor=info[3]
						if frontSensor.result==0 then
							log(GoStraight,'GoStraight',5,0.2)
						else
							ifObjfind=1
							targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
							break
						end
					end
					if ifObjfind==0 then
						rotateToTarget(math.pi)
						ifchangedir=0
						break
					else
						break
					end
				else
					log(GoStraight,'GoStraight',5,0.2)
				end
				
			end

			if frontSensor.result==1 and leftSensor.result==0 then
				if robot.ifHandleIsTask2Obj(frontSensor.ObjHandle)==false then
					targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
					if math.abs(targetpos[1]-targetposx)>0.25 then
						--log(GoStraight,'GoStraight',5,0.45)
						ifObjfind=1
						targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
						moveToTarget(targetpos[1],targetpos[2]-0.7)
						break
					else
						log(GoStraight,'GoStraight',5,0.2)
					end
				end
				if robot.ifHandleIsTask2Obj(frontSensor.ObjHandle)==true then
					log(GoStraight,'GoStraight',5,0.3)
					rotateToTarget(math.pi/2)
					for i=1,3 do
						info=infoTableOfPlatformSensors()
						frontSensor=info[3]
						if frontSensor.result==0 then
							log(GoStraight,'GoStraight',5,0.2)
						else
							ifObjfind=1
							targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
							break
						end
					end
					rotateToTarget(math.pi)
					ifchangedir=0
				end
			end
			
			if leftSensor.result==1 and frontSensor.result==0 then 
				ifObjfind=1
				targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
				break
			end
		end
		if ifchangedir==0 and ifObjfind==0 then
			handleTipPlatform=robot.simGetHandleVehicle()
			pos =robot.simGetObjectPosition(handleTipPlatform,pos)
			info=infoTableOfPlatformSensors()
			frontSensor=info[3]
			rightSensor=info[2]
			simExtPrintInfo('f'..tostring(frontSensor.result))
			simExtPrintInfo('r'..tostring(rightSensor.result))
			--分3种情况判断
			if frontSensor.result==0 and rightSensor.result==0 then --未扫到障碍物
				if pos[2]<-(-downy-0.2) then --6                                --该处对应位置的数字需要细调
					log(GoStraight,'GoStraight',5,pos[2]-downy)--6.2
					simExtPrintInfo('test2')
					rotateToTarget(math.pi/2)
					for i=1,3 do
						info=infoTableOfPlatformSensors()
						frontSensor=info[3]
						if frontSensor.result==0 then
							log(GoStraight,'GoStraight',5,0.2)
						else
							ifObjfind=1
							targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
							break
						end
					end
					if ifObjfind==0 then
						rotateToTarget(0)
						ifchangedir=1
						break
					else
						break
					end	
				else
					log(GoStraight,'GoStraight',5,0.2)
				end
			end
			
			if frontSensor.result==1 and rightSensor.result==0 and tostring(frontSensor.ObjHandle)~='97' then
				if robot.ifHandleIsTask2Obj(frontSensor.ObjHandle)==false then
					targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
					if math.abs(targetpos[1]-targetposx)>0.25 then
						simExtPrintInfo('succeed')
						--log(GoStraight,'GoStraight',5,0.45)
						ifObjfind=1
						targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
						moveToTarget(targetpos[1],targetpos[2]+0.7)
						break
					else
						log(GoStraight,'GoStraight',5,0.2)
					end
				end
				if robot.ifHandleIsTask2Obj(frontSensor.ObjHandle)==true then
					log(GoStraight,'GoStraight',5,0.3)
					rotateToTarget(math.pi/2)
					for i=1,3 do
						info=infoTableOfPlatformSensors()
						frontSensor=info[3]
						if frontSensor.result==0 then
							log(GoStraight,'GoStraight',5,0.2)
						else
							ifObjfind=1
							targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
							break
						end
					end
					rotateToTarget(0)
					ifchangedir=1
				end
			end
			
			if rightSensor.result==1 and frontSensor.result==0 then
				ifObjfind=1
				targetpos=robot.simGetObjectPosition(rightSensor.ObjHandle,targetpos)
				--moveToTarget()
				break
			end			
		end
	until ifObjfind==1
	return ifObjfind,targetpos,count
end
function circlefind(targetpos,changedir,flag,count)--1:控制直线扫描的方向（1为左边传感器，0为右边传感器），2：控制圆圈扫描的方向（0为右转，1为左转）
	local ifObjfind=0
	local initpos={0,0,0}
	local pos={0,0,0}
	local rot={0,0,0}
	local info = {{},{},{},{}}
	local targetpos1={0,0,0}
	--local count=0--计数障碍物的个数
	local lfang
	local rbang
	local rfang
	local lbang
	local lfVel
	local rfVel
	local lbVel
	local rbVel
	local rightSensor=info[2]
	local leftSensor=info[1]
	local frontSensor=info[3]
	local count1=0
	local zhangaipos={0,0,0}
	local rot={0,0,0}
	local startscan=0
	local targetposlimit={0,0,0}
	--local changedir=0 --默认转圈方向为右转
	handleTipPlatform=robot.simGetHandleVehicle()
	info=infoTableOfPlatformSensors()
	targetpos1=targetpos     --targetpos1用来记录上一个目标的位置，targetpos用来记录当前目标的位置
	
	
	local dx=0
	local dy=0
	local distance=0
	dx=targetpos[1]+5
	dy=targetpos[2]+6
	distance=math.sqrt(dx*dx+dy*dy)
	
	--判断目标位置是否接近边界
	if (targetpos1[2]+8.2)<0.65 or distance>0.8 then--防止出现提前-math.pi/2的情况
		changedir=1
	end
	if targetpos1[2]<-4.5 and targetpos1[2]>-6.2 and (targetpos1[1]+5.2)<0.65 then
		changedir=1
	end
	if targetpos1[1]<-5.2 and targetpos1[1]>-6.8 and (targetpos1[2]+6.2)<0.65 then
		changedir=0
	end
	
	
	
	repeat
		if changedir==1 and count<4 then
		    count=count+1
			lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(targetpos[1],targetpos[2],0.55,'l')
			robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
			robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
			robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
			robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)
			if count==1 or flag==1 then
				repeat
					info=infoTableOfPlatformSensors()
					rightSensor=info[2]
					robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
				until rightSensor.result==1
				stopMove()
				targetpos=robot.simGetObjectPosition(rightSensor.ObjHandle,targetpos)
				moveToTarget((targetpos[1]+targetpos1[1])/2,(targetpos[2]+targetpos1[2])/2)
				targetpos1=targetpos
				changedir=0
				if flag==1 then
					flag=0
				end
			else
				repeat
					info=infoTableOfPlatformSensors()
					rightSensor=info[2]
					robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
				until rightSensor.result==0 
				while(1) do
					info=infoTableOfPlatformSensors()
					rightSensor=info[2]
					frontSensor=info[3]
					robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
					if rightSensor.result==1 and robot.ifHandleIsTask2Obj(rightSensor.ObjHandle)==false and tostring(rightSensor.ObjHandle)~='97' then
						zhangaipos=robot.simGetObjectPosition(rightSensor.ObjHandle,zhangaipos)
						if zhangaipos[1]<-5 and zhangaipos[2]<-6 then--确定扫到的是障碍物而不是边界
							break
						end
						if zhangaipos[1]>-5 and zhangaipos[2]<-4.6 then
							break
						end
						--[[zhangaipos=robot.simGetObjectPosition(handleTipPlatform,zhangaipos)--机器人的位置
						if frontSensor.result==1 then --扫到边界之后进行上下行扫描处理
							if zhangaipos[1]<-4.4 or zhangaipos[2]>-5 then
								startscan=1
								break
							end
						end--]]
					end
					rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
					if math.abs(rot[3]+math.pi/2)<0.02 then
						startscan=1
						break
					end
				end 
				if startscan==1 then
					break
				end
				stopMove()
				targetpos=robot.simGetObjectPosition(rightSensor.ObjHandle,targetpos)
				moveToTarget((targetpos[1]+targetpos1[1])/2,(targetpos[2]+targetpos1[2])/2)
				targetpos1=targetpos
				changedir=0		
			end
		end
		if changedir==0 and count<4 then
			count=count+1
			lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(targetpos[1],targetpos[2],0.55,'r')
			robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
			robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
			robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
			robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)
			--转向
			--排除开始传感器的影响
			if count==1 or flag==1 then
				while(1) do
					info=infoTableOfPlatformSensors()
					leftSensor=info[1]
					robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
					if leftSensor.result==1 then
						break
					end
				end 
				stopMove()
				targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
				moveToTarget((targetpos[1]+targetpos1[1])/2,(targetpos[2]+targetpos1[2])/2)
				targetpos1=targetpos
				changedir=1
				if flag==1 then
					flag=0
				end
			else	
				repeat
					info=infoTableOfPlatformSensors()
					leftSensor=info[1]
					frontSensor=info[3]
					robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
				until leftSensor.result==0
				while(1) do
					info=infoTableOfPlatformSensors()
					leftSensor=info[1]
					robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
					robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
					rot=robot.simGetObjectOrientation(handleTipPlatform,rot)
					--[[zhangaipos=robot.simGetObjectPosition(handleTipPlatform,hangaipos)
					if frontSensor.result==1 then 
						if zhangaipos[1]<-4.4 or zhangaipos[2]>-5 then
							startscan=1
							break
						end			
					end--]]
					if math.abs(rot[3]+math.pi/2)<0.02 then
						startscan=1
						break
					end
					simExtPrintInfo('l:'..tostring(leftSensor.result))
					if leftSensor.result==1 and robot.ifHandleIsTask2Obj(leftSensor.ObjHandle)==false and tostring(leftSensor.ObjHandle)~='97' then
						targetposlimit=robot.simGetObjectPosition(leftSensor.ObjHandle,targetposlimit)
						simExtPrintInfo(tostring(targetposlimit[1])..tostring(targetposlimit[2]))
						if targetposlimit[2]<-3.5 then --边界的值不对？？？？？
							break
						end
					end
				end 
				if startscan==1 then
					break
				end
				stopMove()
				targetpos=robot.simGetObjectPosition(leftSensor.ObjHandle,targetpos)
				moveToTarget((targetpos[1]+targetpos1[1])/2,(targetpos[2]+targetpos1[2])/2)
				targetpos1=targetpos
				changedir=1
			end
		end	
	until count==4
	return count,startscan,targetpos
end
function task3()
	local startscan=0
	local ifObjfind=0
	local targetpos={0,0,0}
	local initpos={0,0,0}
	local count
	local info = {{},{},{},{}}
    local rightSensor=0
	local leftSensor=0
	local frontSensor=0
	local rot={0,0,0}
	local lasttargetpos={0,0,0}       --获得最后一个障碍物的位置坐标
	ifObjind,targetpos,count=Scan(1,-6.2,0,0)--固定从下向上扫描障碍物，获得第一个障碍物的位置

	--判断第一个障碍物的绕向
	info=infoTableOfPlatformSensors()
	frontSensor=info[3]
	rightSensor=info[2]
	leftSensor=info[1]
	rot = robot.simGetObjectOrientation(handleTipPlatform,rot)
	if rightSensor.result==1 or rot[3]==math.pi then
		count,startscan,lasttargetpos=circlefind(targetpos,1,0,count)
	end
	if leftSensor.result==1 or rot[3]==0 then 
		count,startscan,lasttargetpos=circlefind(targetpos,0,0,count)
	end
	repeat
		if startscan==1 and count<4 then
			info=infoTableOfPlatformSensors()
			rightSensor=info[2]
			leftSensor=info[1]
			if rightSensor.result==1 and startscan==1 then
				log(GoStraight,'GoStraight',-5,-0.45)
				rotateToTarget(0)
				initpos=robot.simGetObjectPosition(handleTipPlatform,initpos)
				moveToTarget(initpos[1],-4.6)
				rotateToTarget(-math.pi)
				ifObjind,targetpos,count=Scan(0,-7.7,lasttargetpos[1],count)
				startscan=0
				count,startscan,lasttargetpos=circlefind(targetpos,0,1,count)
			end
			if leftSensor.result==1 and startscan==1 then
				log(GoStraight,'GoStraight',-5,-0.45)
				rotateToTarget(-math.pi)
				initpos=robot.simGetObjectPosition(handleTipPlatform,initpos)
				moveToTarget(initpos[1],-7.8)
				rotateToTarget(0)
				ifObjind,targetpos,count=Scan(1,-7.7,lasttargetpos[1],count)
				startscan=0
				count,startscan,lasttargetpos=circlefind(targetpos,0,1,count)
			end
		end
	until count==4
	--**************************--	
	local dX=0
	local dY=0
	local distance=0 
	dX=lasttargetpos[1]+6.7
	dY=lasttargetpos[2]+6.5
	distance=math.sqrt(dX*dX+dY*dY)
	if distance>0.8 then
		lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(lasttargetpos[1],lasttargetpos[2],0.55,'l')
		robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
		robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
		robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
		robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)
		repeat
			initpos=robot.simGetObjectPosition(handleTipPlatform, initpos) 
			robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
			robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
			robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
			robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
		until math.abs(initpos[2]+7.8)<0.02 and initpos[1]<lasttargetpos[1]
		rotateToTarget(math.pi)
		moveToTarget(-7.5,-7.8)
	end
	if distance<0.8 then
		lfang,rbang,rfang,lbang,lfVel,rfVel,lbVel,rbVel=circleAround(lasttargetpos[1],lasttargetpos[2],0.55,'r')
		robot.simSetJointTargetPosition(handleJointWheelLF0, lfang)
		robot.simSetJointTargetPosition(handleJointWheelRB0, rbang)
		robot.simSetJointTargetPosition(handleJointWheelRF0, rfang)
		robot.simSetJointTargetPosition(handleJointWheelLB0, lbang)
		repeat
			initpos=robot.simGetObjectPosition(handleTipPlatform, initpos) 
			robot.simSetJointTargetVelocity(handleJointWheelLF, lfVel*5)
			robot.simSetJointTargetVelocity(handleJointWheelRF, rfVel*5)
			robot.simSetJointTargetVelocity(handleJointWheelLB, lbVel*5)
			robot.simSetJointTargetVelocity(handleJointWheelRB, rbVel*5)
		until math.abs(initpos[2]+7.8)<0.02 and initpos[1]<lasttargetpos[1]
		rotateToTarget(0)
		moveToTarget(-7.3,-7.5)
	end
    
	simExtPrintInfo(tostring('succeed to end'))
end

function GoHorizon(moveVel,length)
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

		if(math.abs(math.abs(length)-distance) < 0.02) then
			arrived = true
		end

    until (arrived == true)

	--fprint('******************************\n\n')
end
function test()
	local targetpos={0,0,0}
	rotateToTarget(-math.pi)
	repeat 
		info=infoTableOfPlatformSensors()
		frontSensor=info[3]
		log(GoStraight,'GoStraight',5,0.2)
	until frontSensor.result==1	
	targetpos=robot.simGetObjectPosition(frontSensor.ObjHandle,targetpos)
	simExtPrintInfo(tostring(targetpos[1])..tostring(targetpos[2]))
end

cleanlog()
tprint()
log(task1,'task1')
--log(test,'test')
log(task3,'task3')
stopMove()


