#include "Script.h"

const char* Script::strOpenCLScript = TOSTRING
(

typedef struct 
{
	float x;
	float y;
	float z;
}
Vector3;

Vector3 XYZToVector3(float x, float y, float z) 
{
	Vector3 ret;
	ret.x = x;
	ret.y = y;
	ret.z = z;
	return ret;
}

Vector3 Float3ToVector3(float3 v) 
{
	Vector3 ret;
	ret.x = v.x;
	ret.y = v.y;
	ret.z = v.z;
	return ret;
}

float3 XYZToFloat3(float x, float y, float z) 
{
	float3 ret;
	ret.x = x;
	ret.y = y;
	ret.z = z;
	return ret;
}

float3 Vector3ToFloat3(Vector3 v)
{
	float3 ret;
	ret.x = v.x;
	ret.y = v.y;
	ret.z = v.z;
	return ret;
}

Vector3 Vector3_Add(Vector3 a, Vector3 b)
{
	Vector3 ret;
	ret.x = a.x + b.x;
	ret.y = a.y + b.y;
	ret.z = a.z + b.z;
	return ret;
}

typedef struct
{
	Vector3 v3Min;
	Vector3 v3Max;
}
BBox;

typedef struct 
{
    BBox bbox;
    int nLeft;
    int nRight;
}
BVHObject;

typedef struct
{
	int nTriMeshId;

	BBox bbox;

	float fMass;
	float fRestitution;
	float fFriction;
	float fLinearDamping;
	float fAngularDamping;

	Vector3 v3Force;
	Vector3 v3LinearAcceleration;
	Vector3 v3LinearVelocity;
	Vector3 v3Position;

	Vector3 v3Torque;
	Vector3 v3AngularAcceleration;
	Vector3 v3AngularVelocity;
	Vector3 v3Rotate;
}
RigidBody;

BBox CreateBBox(BBox a, BBox b)
{
	float fMinX = +1000000000.0f;
	float fMinY = +1000000000.0f;
	float fMinZ = +1000000000.0f;

	fMinX = min(fMinX, a.v3Min.x);
	fMinX = min(fMinX, a.v3Max.x);
	fMinX = min(fMinX, b.v3Min.x);
	fMinX = min(fMinX, b.v3Max.x);

	fMinY = min(fMinY, a.v3Min.y);
	fMinY = min(fMinY, a.v3Max.y);
	fMinY = min(fMinY, b.v3Min.y);
	fMinY = min(fMinY, b.v3Max.y);

	fMinZ = min(fMinZ, a.v3Min.z);
	fMinZ = min(fMinZ, a.v3Max.z);
	fMinZ = min(fMinZ, b.v3Min.z);
	fMinZ = min(fMinZ, b.v3Max.z);

	float fMaxX = -1000000000.0f;
	float fMaxY = -1000000000.0f;
	float fMaxZ = -1000000000.0f;

	fMaxX = max(fMaxX, a.v3Min.x);
	fMaxX = max(fMaxX, a.v3Max.x);
	fMaxX = max(fMaxX, b.v3Min.x);
	fMaxX = max(fMaxX, b.v3Max.x);
	
	fMaxY = max(fMaxY, a.v3Min.y);
	fMaxY = max(fMaxY, a.v3Max.y);
	fMaxY = max(fMaxY, b.v3Min.y);
	fMaxY = max(fMaxY, b.v3Max.y);
	
	fMaxZ = max(fMaxZ, a.v3Min.z);
	fMaxZ = max(fMaxZ, a.v3Max.z);
	fMaxZ = max(fMaxZ, b.v3Min.z);
	fMaxZ = max(fMaxZ, b.v3Max.z);

	BBox bbox;
	bbox.v3Min = XYZToVector3(fMinX, fMinY, fMinZ);
	bbox.v3Max = XYZToVector3(fMaxX, fMaxY, fMaxZ);

	return bbox;
}

__kernel void UpdateBVHObjects(__global BVHObject *inoutBVHObjects, __global RigidBody* inoutRigidBodies, int nOffset, int nCount)
{
    int id = get_global_id(0);

	BVHObject bvhObject = inoutBVHObjects[nOffset + id];
	
	if (bvhObject.nLeft == -1 && bvhObject.nRight == -1) // is Leaf?
	{
		RigidBody rigidBody = inoutRigidBodies[id];
		BBox bbox = rigidBody.bbox;

		bbox.v3Min = Vector3_Add(bbox.v3Min, rigidBody.v3Position);
		bbox.v3Max = Vector3_Add(bbox.v3Max, rigidBody.v3Position);

		bvhObject.bbox = bbox;
	}
	else if (bvhObject.nLeft > -1 && bvhObject.nRight == -1) // is bbox?
	{
		BVHObject bvhLeftObject = inoutBVHObjects[bvhObject.nLeft];
		bvhObject.bbox = bvhLeftObject.bbox;
	}
	else if (bvhObject.nLeft == -1 && bvhObject.nRight > -1) // is bbox?
	{
		BVHObject bvhRightObject = inoutBVHObjects[bvhObject.nRight];
		bvhObject.bbox = bvhRightObject.bbox;
	}
	else if (bvhObject.nLeft > -1 && bvhObject.nRight > -1) // is bbox?
	{
		BVHObject bvhLeftObject = inoutBVHObjects[bvhObject.nLeft];
		BVHObject bvhRightObject = inoutBVHObjects[bvhObject.nRight];

		BBox bbox = CreateBBox(bvhLeftObject.bbox, bvhRightObject.bbox);

		bvhObject.bbox = bbox;
	}

	// cheat
	bvhObject.bbox.v3Min = Vector3_Add(bvhObject.bbox.v3Min, XYZToVector3(-0.0001f, -0.0001f, -0.0001f));
	bvhObject.bbox.v3Max = Vector3_Add(bvhObject.bbox.v3Max, XYZToVector3(+0.0001f, +0.0001f, +0.0001f));

	// save
	inoutBVHObjects[nOffset + id] = bvhObject;
}

__kernel void Integrate(__global RigidBody* inoutRigidBodies, int nCount, float dt, Vector3 v3Gravity)
{
	int id = get_global_id(0);

	RigidBody rigidBody = inoutRigidBodies[id];

	// init
	float fMass = rigidBody.fMass;

	// if static mesh ?
	if (fMass <= 0.0f)
	{
		return;
	}

	// get
	float3 f3Gravity = Vector3ToFloat3(v3Gravity);

	float3 f3Force          = Vector3ToFloat3(rigidBody.v3Force);
	float3 f3LinearVelocity = Vector3ToFloat3(rigidBody.v3LinearVelocity);
	float3 f3Position       = Vector3ToFloat3(rigidBody.v3Position);
	
	float3 f3Torque          = Vector3ToFloat3(rigidBody.v3Torque);
	float3 f3AngularVelocity = Vector3ToFloat3(rigidBody.v3AngularVelocity);
	float3 f3Rotate          = Vector3ToFloat3(rigidBody.v3Rotate);

	// calc - Euler method (deprecated)
	float3 f3LinearAcceleration = f3Gravity + (f3Force / fMass);
	f3LinearVelocity           += f3LinearAcceleration * dt;
	f3Position                 += f3LinearVelocity * dt;
	
	float3 f3AngularAcceleration = (f3Torque / fMass);
	f3AngularVelocity           += f3AngularAcceleration * dt;
	f3Rotate                    += f3AngularVelocity * dt;

	// set	
	inoutRigidBodies[id].v3LinearAcceleration = Float3ToVector3(f3LinearAcceleration);
	inoutRigidBodies[id].v3LinearVelocity     = Float3ToVector3(f3LinearVelocity);
	inoutRigidBodies[id].v3Position           = Float3ToVector3(f3Position);

	inoutRigidBodies[id].v3AngularAcceleration = Float3ToVector3(f3AngularAcceleration);
	inoutRigidBodies[id].v3AngularVelocity     = Float3ToVector3(f3AngularVelocity);
	inoutRigidBodies[id].v3Rotate              = Float3ToVector3(f3Rotate);
}

);

const char* Script::GetText() 
{
    return strOpenCLScript;
}
