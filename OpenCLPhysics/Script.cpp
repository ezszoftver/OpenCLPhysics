#include "Script.h"

const char* Script::strOpenCLScript = TOSTRING
(

typedef struct
{
    float fMinX;
    float fMinY;
    float fMinZ;
    
    float fMaxX;
    float fMaxY;
    float fMaxZ;
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

	float v3ForceX;
	float v3ForceY;
	float v3ForceZ;

	float v3LinearAccelerationX;
	float v3LinearAccelerationY;
	float v3LinearAccelerationZ;

	float v3LinearVelocityX;
	float v3LinearVelocityY;
	float v3LinearVelocityZ;

	float v3PositionX;
	float v3PositionY;
	float v3PositionZ;

	float v3TorqueX;
	float v3TorqueY;
	float v3TorqueZ;

	float v3AngularAccelerationX;
	float v3AngularAccelerationY;
	float v3AngularAccelerationZ;

	float v3AngularVelocityX;
	float v3AngularVelocityY;
	float v3AngularVelocityZ;

	float v3RotateX;
	float v3RotateY;
	float v3RotateZ;
}
RigidBody;

BBox CreateBBox(BBox a, BBox b)
{
	float fMinX = +1000000000.0f;
	float fMinY = +1000000000.0f;
	float fMinZ = +1000000000.0f;

	fMinX = min(fMinX, a.fMinX);
	fMinX = min(fMinX, a.fMaxX);
	fMinX = min(fMinX, b.fMinX);
	fMinX = min(fMinX, b.fMaxX);

	fMinY = min(fMinY, a.fMinY);
	fMinY = min(fMinY, a.fMaxY);
	fMinY = min(fMinY, b.fMinY);
	fMinY = min(fMinY, b.fMaxY);

	fMinZ = min(fMinZ, a.fMinZ);
	fMinZ = min(fMinZ, a.fMaxZ);
	fMinZ = min(fMinZ, b.fMinZ);
	fMinZ = min(fMinZ, b.fMaxZ);

	float fMaxX = -1000000000.0f;
	float fMaxY = -1000000000.0f;
	float fMaxZ = -1000000000.0f;

	fMaxX = max(fMaxX, a.fMinX);
	fMaxX = max(fMaxX, a.fMaxX);
	fMaxX = max(fMaxX, b.fMinX);
	fMaxX = max(fMaxX, b.fMaxX);
	
	fMaxY = max(fMaxY, a.fMinY);
	fMaxY = max(fMaxY, a.fMaxY);
	fMaxY = max(fMaxY, b.fMinY);
	fMaxY = max(fMaxY, b.fMaxY);
	
	fMaxZ = max(fMaxZ, a.fMinZ);
	fMaxZ = max(fMaxZ, a.fMaxZ);
	fMaxZ = max(fMaxZ, b.fMinZ);
	fMaxZ = max(fMaxZ, b.fMaxZ);

	BBox bbox;
	bbox.fMinX = fMinX;
	bbox.fMinY = fMinY;
	bbox.fMinZ = fMinZ;
	bbox.fMaxX = fMaxX;
	bbox.fMaxY = fMaxY;
	bbox.fMaxZ = fMaxZ;

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

		bbox.fMinX += rigidBody.v3PositionX;
		bbox.fMinY += rigidBody.v3PositionY;
		bbox.fMinZ += rigidBody.v3PositionZ;
		bbox.fMaxX += rigidBody.v3PositionX;
		bbox.fMaxY += rigidBody.v3PositionY;
		bbox.fMaxZ += rigidBody.v3PositionZ;

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
	bvhObject.bbox.fMinX -= 0.0001f;
	bvhObject.bbox.fMinY -= 0.0001f;
	bvhObject.bbox.fMinZ -= 0.0001f;
	bvhObject.bbox.fMaxX += 0.0001f;
	bvhObject.bbox.fMaxY += 0.0001f;
	bvhObject.bbox.fMaxZ += 0.0001f;

	// save
	inoutBVHObjects[nOffset + id] = bvhObject;
}

__kernel void Integrate(__global RigidBody* inoutRigidBodies, int nCount, float dt) 
{
}

);

const char* Script::GetText() 
{
    return strOpenCLScript;
}
