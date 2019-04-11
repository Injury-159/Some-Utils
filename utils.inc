#if defined _inc_utils_
	#endinput
#endif 

#define _inc_utils_


#define GetRotationFor2Point2D(%0,%1,%2,%3,%4)					(CompRotationFloat((atan2((%3)-(%1),(%2)-(%0))-90.0),(%4)))
#define GetDistanceBetweenPoints1D(%1,%2)						VectorSize((%1)-(%2),0.0,0.0)
#define GetDistanceBetweenPoints2D(%1,%2,%3,%4)					VectorSize((%1)-(%3),(%2)-(%4),0.0)
#define GetDistanceBetweenPoints3D(%1,%2,%3,%4,%5,%6)			VectorSize((%1)-(%4),(%2)-(%5),(%3)-(%6))


enum EULER_MODE 
{
	// Proper / Classic Euler angles
	euler_xzx,
	euler_xyx,
	euler_yxy,
	euler_yzy,
	euler_zyz,
	euler_zxz,

	euler_xzy,
	euler_xyz,
	euler_yxz,
	euler_yzx,
	euler_zyx, // pitch roll yaw
	euler_zxy // sa-mp
}


stock RandomString(string[], length = 10)
{
    while(length--) string[length] = random(2) ? (random(26) + (random(2) ? 'a' : 'A')) : (random(10) + '0');
}


stock Float:CompRotationFloat(Float:rotation, &Float:crotation = 0.0)
{
	crotation = rotation;
	while(crotation < 0.0) crotation += 360.0;
	while(crotation >= 360.0) crotation -= 360.0;
	return crotation;
}


stock QuatToEuler(&Float:rx, &Float:ry, &Float:rz, Float:qw, Float:qx, Float:qy, Float:qz)
{
	CompRotationFloat(asin(2 * qy * qz - 2 * qx * qw), rx);
	CompRotationFloat(-atan2(qx * qz + qy * qw, 0.5 - qx * qx - qy * qy), ry);
	CompRotationFloat(-atan2(qx * qy + qz * qw, 0.5 - qx * qx - qz * qz), rz);
}


stock GetQuatFromEuler(Float:rx, Float:ry, Float:rz, &Float:qw, &Float:qx, &Float:qy, &Float:qz, EULER_MODE:mode = euler_zxy)
{
	rx /= 2.0; // could be done with multiplication but would be slower
	ry /= 2.0;
	rz /= 2.0;

	new	Float:cosX = floatcos(rx,degrees),
		Float:cosY = floatcos(ry,degrees),
		Float:cosZ = floatcos(rz,degrees),
		Float:sinX = floatsin(rx,degrees),
		Float:sinY = floatsin(ry,degrees),
		Float:sinZ = floatsin(rz,degrees);

	switch(mode) 
	{
		case euler_xzx: 
		{
			qw = sinX * cosY * sinZ - cosX * cosY * cosZ;
			qx = cosX * cosY * sinZ + sinX * cosY * cosZ;
			qy = cosX * sinY * sinZ - sinX * sinY * cosZ;
			qz = cosX * sinY * cosZ + sinX * sinY * sinZ;
		}
		case euler_xyx: 
		{
			qw = sinX * cosY * sinZ - cosX * cosY * cosZ;
			qx = cosX * cosY * sinZ + sinX * cosY * cosZ;
			qy = cosX * sinY * cosZ + sinX * sinY * sinZ;
			qz = sinX * sinY * cosZ - cosX * sinY * sinZ;
		}
		case euler_yxy: 
		{
			qw = sinX * cosY * sinZ - cosX * cosY * cosZ;
			qx = cosX * sinY * cosZ + sinX * sinY * sinZ;
			qy = cosX * cosY * sinZ + sinX * cosY * cosZ;
			qz = cosX * sinY * sinZ - sinX * sinY * cosZ;
		}
		case euler_yzy: 
		{
			qw = sinX * cosY * sinZ - cosX * cosY * cosZ;
			qx = sinX * sinY * cosZ - cosX * sinY * sinZ;
			qy = cosX * cosY * sinZ + sinX * cosY * cosZ;
			qz = cosX * sinY * cosZ + sinX * sinY * sinZ;
		}
		case euler_zyz:
		{
			qw = sinX * cosY * sinZ - cosX * cosY * cosZ;
			qx = cosX * sinY * sinZ - sinX * sinY * cosZ;
			qy = cosX * sinY * cosZ + sinX * sinY * sinZ;
			qz = cosX * cosY * sinZ + sinX * cosY * cosZ;
		}
		case euler_zxz: 
		{
			qw = sinX * cosY * sinZ - cosX * cosY * cosZ;
			qx = cosX * sinY * cosZ + sinX * sinY * sinZ;
			qy = sinX * sinY * cosZ - cosX * sinY * sinZ;
			qz = cosX * cosY * sinZ + sinX * cosY * cosZ;
		}
		case euler_xzy: 
		{
			qw = cosX * cosY * cosZ + sinX * sinY * sinZ;
			qx = cosX * sinY * sinZ - sinX * cosY * cosZ;
			qy = sinX * cosY * sinZ - cosX * sinY * cosZ;
			qz = -(cosX * cosY * sinZ + sinX * sinY * cosZ);
		}
		case euler_xyz: 
		{
			qw = sinX * sinY * sinZ - cosX * cosY * cosZ;
			qx = sinX * cosY * cosZ + cosX * sinY * sinZ;
			qy = cosX * sinY * cosZ - sinX * cosY * sinZ;
			qz = cosX * cosY * sinZ + sinX * sinY * cosZ;
		}
		case euler_yxz: 
		{
			qw = cosX * cosY * cosZ + sinX * sinY * sinZ;
			qx = -(sinX * cosY * cosZ + cosX * sinY * sinZ);
			qy = sinX * cosY * sinZ - cosX * sinY * cosZ;
			qz = sinX * sinY * cosZ - cosX * cosY * sinZ;
		}
		case euler_yzx: 
		{
			qw = sinX * sinY * sinZ - cosX * cosY * cosZ;
			qx = sinX * cosY * cosZ + cosX * sinY * sinZ;
			qy = cosX * sinY * cosZ + sinX * cosY * sinZ;
			qz = cosX * cosY * sinZ - sinX * sinY * cosZ;
		}
		case euler_zyx: 
		{
			qw = cosX * cosY * cosZ + sinX * sinY * sinZ;
			qx = cosX * sinY * sinZ - sinX * cosY * cosZ;
			qy = -(cosX * sinY * cosZ + sinX * cosY * sinZ);
			qz = sinX * sinY * cosZ - cosX * cosY * sinZ;
		}
		case euler_zxy: 
		{
			qw = sinX * sinY * sinZ - cosX * cosY * cosZ;
			qx = sinX * cosY * cosZ - cosX * sinY * sinZ;
			qy = cosX * sinY * cosZ + sinX * cosY * sinZ;
			qz = cosX * cosY * sinZ + sinX * sinY * cosZ;
		}
	}
}


stock QuatRotate(Float:qw, Float:qx, Float:qy, Float:qz, Float:oX, Float:oY, Float:oZ, &Float:tx, &Float:ty, &Float:tz)
{
	new Float:dot = qx * oX + qy * oY + qz * oZ, Float:abs = qw * qw - 0.5;

	tx += 2.0 * (dot * qx + abs * oX + qw * (qz * oY - qy * oZ));
	ty += 2.0 * (dot * qy + abs * oY + qw * (qx * oZ - qz * oX));
	tz += 2.0 * (dot * qz + abs * oZ + qw * (qy * oX - qx * oY));
}


stock EulerRotate(Float:rx, Float:ry, Float:rz, Float:oX, Float:oY, Float:oZ, &Float:tx, &Float:ty, &Float:tz, EULER_MODE:mode = euler_zxy)
{
	GetQuatFromEuler(rx, ry, rz, Float:mode, rx, ry, rz, mode);
	QuatRotate(Float:mode, rx, ry, rz, oX, oY, oZ, tx, ty, tz);
}	


stock GetPlayerCameraRotation(playerid,&Float:rx,&Float:rz)
{
	new Float:mx, Float:my, Float:mz;
	GetPlayerCameraFrontVector(playerid, mx ,my ,mz);
	CompRotationFloat(-(acos(mz) - 90.0), rx);
	CompRotationFloat((atan2(my, mx) - 90.0), rz);
}


stock Float:GetPlayerCameraZAngle(playerid, &Float:rz = 0.0)
{
	new Float:mx, Float:my, Float:mz;
	GetPlayerCameraFrontVector(playerid, mx, my, mz);
	CompRotationFloat((atan2(my, mx) - 90.0), rz);
	return rz;
}


stock SetPlayerCameraZAngle(playerid,Float:rz)
{
	new Float:x, Float:y, Float:z, Float:tx, Float:ty, Float:tz, Float:rx, Float:tmp;
	GetPlayerCameraPos(playerid, x, y, z);
	GetPlayerCameraRotation(playerid, rx, tmp);
	GetPointInFront3D(x, y, z, rx, rz, 1.0, tx, ty, tz);
	SetPlayerCameraLookAt(playerid, tx, ty, tz);
}


stock GetRotationFor2Point3D(Float:x, Float:y, Float:z, Float:tx, Float:ty, Float:tz, &Float:rx, &Float:rz)
{
	new Float:radius = GetDistanceBetweenPoints3D(x, y, z, tx, ty, tz);
	if(radius <= 0.0) return 0;
	CompRotationFloat(-(acos((tz - z) / radius) - 90.0), rx);
	CompRotationFloat((atan2(ty - y, tx - x) - 90.0), rz);
	return 1;
}


stock GetPointInFront2D(Float:x, Float:y, Float:rz, Float:radius, &Float:tx, &Float:ty)
{
	tx = x - (radius * floatsin(rz, degrees));
	ty = y + (radius * floatcos(rz, degrees));
}


stock GetPointInFront3D(Float:x, Float:y, Float:z, Float:rx, Float:rz, Float:radius, &Float:tx, &Float:ty, &Float:tz)
{
	tx = x - (radius * floatcos(rx, degrees) * floatsin(rz, degrees));
	ty = y + (radius * floatcos(rx, degrees) * floatcos(rz, degrees));
	tz = z + (radius * floatsin(rx, degrees));
}


stock GetPointInFront3DEx(Float:x, Float:y, Float:z, Float:rx, Float:ry, Float:rz, Float:radius, &Float:tx, &Float:ty, &Float:tz)
{
	new Float:vx, Float:vy, Float:vz, Float:trx, Float:trz;
	EulerRotate(rx, ry, rz, 0.0, 1.0, 0.0, vx, vy, vz, euler_zyx);
	GetRotationFor2Point3D(0.0, 0.0, 0.0, vx, vy, vz, trx, trz);
	GetPointInFront3D(x, y, z, trx, trz, radius, tx, ty, tz); 
}


stock GetVehicleRotation(vehicleid, &Float:rx, &Float:ry, &Float:rz)
{
	new Float:qw ,Float:qx, Float:qy, Float:qz;
	GetVehicleRotationQuat(vehicleid, qw, qx, qy, qz);
	QuatToEuler(rx, ry, rz, qw, qx, qy, qz);
}


stock Float:GetVehicleSpeed(vehicleid)
{
	new Float:x, Float:y, Float:z;
	GetVehicleVelocity(vehicleid, x, y, z);
	return floatmul(VectorSize(x, y, z), 170.0);
}


stock SetVehicleSpeed(vehicleid,Float:speed)
{
	new Float:rx, Float:ry, Float:rz, Float:vx, Float:vy, Float:vz;
	GetVehicleRotation(vehicleid, rx, ry, rz);
	GetPointInFront3DEx(0.0, 0.0, 0.0,rx, ry, rz, (speed / 170.0), vx, vy, vz);
	SetVehicleVelocity(vehicleid, vx, vy, vz);
}


stock Float:GetPlayerSpeed(playerid)
{
	new Float:x, Float:y, Float:z;
	GetPlayerVelocity(playerid,x, y, z);
	return floatmul(VectorSize(x, y, z), 1.0);
}


stock Float:SetPlayerTargetAngle(playerid, Float:x, Float:y, &Float:rz = 0.0)
{
	new Float:mx, Float:my;
	GetPlayerPos(playerid, mx, my, rz);
	GetRotationFor2Point2D(mx, my, x, y, rz);
	SetPlayerFacingAngle(playerid, rz);
	return rz;
}


timer _Kick[200](playerid)
{
	return Kick(playerid);
}