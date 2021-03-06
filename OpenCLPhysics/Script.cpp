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

typedef struct
{
	float m11;
	float m12;
	float m13;
	float m14;

	float m21;
	float m22;
	float m23;
	float m24;

	float m31;
	float m32;
	float m33;
	float m34;

	float m41;
	float m42;
	float m43;
	float m44;
}
Matrix4;

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

float4 XYZWToFloat4(float x, float y, float z, float w)
{
	float4 ret;
	ret.x = x;
	ret.y = y;
	ret.z = z;
	ret.w = w;
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

Matrix4 Mult_Mat4Mat4(Matrix4 T2, Matrix4 T1)
{
	Matrix4 ret;

	float4 T1row1 = XYZWToFloat4(T1.m11, T1.m12, T1.m13, T1.m14);
	float4 T1row2 = XYZWToFloat4(T1.m21, T1.m22, T1.m23, T1.m24);
	float4 T1row3 = XYZWToFloat4(T1.m31, T1.m32, T1.m33, T1.m34);
	float4 T1row4 = XYZWToFloat4(T1.m41, T1.m42, T1.m43, T1.m44);

	float4 T2column1 = XYZWToFloat4(T2.m11, T2.m21, T2.m31, T2.m41);
	float4 T2column2 = XYZWToFloat4(T2.m12, T2.m22, T2.m32, T2.m42);
	float4 T2column3 = XYZWToFloat4(T2.m13, T2.m23, T2.m33, T2.m43);
	float4 T2column4 = XYZWToFloat4(T2.m14, T2.m24, T2.m34, T2.m44);

	float m11 = dot(T1row1, T2column1);
	float m12 = dot(T1row1, T2column2);
	float m13 = dot(T1row1, T2column3);
	float m14 = dot(T1row1, T2column4);

	float m21 = dot(T1row2, T2column1);
	float m22 = dot(T1row2, T2column2);
	float m23 = dot(T1row2, T2column3);
	float m24 = dot(T1row2, T2column4);

	float m31 = dot(T1row3, T2column1);
	float m32 = dot(T1row3, T2column2);
	float m33 = dot(T1row3, T2column3);
	float m34 = dot(T1row3, T2column4);

	float m41 = dot(T1row4, T2column1);
	float m42 = dot(T1row4, T2column2);
	float m43 = dot(T1row4, T2column3);
	float m44 = dot(T1row4, T2column4);

	ret.m11 = m11;
	ret.m12 = m12;
	ret.m13 = m13;
	ret.m14 = m14;

	ret.m21 = m21;
	ret.m22 = m22;
	ret.m23 = m23;
	ret.m24 = m24;

	ret.m31 = m31;
	ret.m32 = m32;
	ret.m33 = m33;
	ret.m34 = m34;

	ret.m41 = m41;
	ret.m42 = m42;
	ret.m43 = m43;
	ret.m44 = m44;

	return ret;
}

Matrix4 Mat4_CreateTranslate(Vector3 v3Translate) 
{
	Matrix4 ret;

	ret.m11 = 1.0f;
	ret.m12 = 0.0f;
	ret.m13 = 0.0f;
	ret.m14 = v3Translate.x;

	ret.m21 = 0.0f;
	ret.m22 = 1.0f;
	ret.m23 = 0.0f;
	ret.m24 = v3Translate.y;

	ret.m31 = 0.0f;
	ret.m32 = 0.0f;
	ret.m33 = 1.0f;
	ret.m34 = v3Translate.z;

	ret.m41 = 0.0f;
	ret.m42 = 0.0f;
	ret.m43 = 0.0f;
	ret.m44 = 1.0f;

	return ret;
}

Matrix4 Mat4_CreateRotateX(float fRad)
{
	Matrix4 ret;

	ret.m11 = 1.0f;
	ret.m12 = 0.0f;
	ret.m13 = 0.0f;
	ret.m14 = 0.0f;

	ret.m21 = 0.0f;
	ret.m22 = cos(fRad);
	ret.m23 = sin(fRad);
	ret.m24 = 0.0f;

	ret.m31 = 0.0f;
	ret.m32 = -sin(fRad);
	ret.m33 = cos(fRad);
	ret.m34 = 0.0f;

	ret.m41 = 0.0f;
	ret.m42 = 0.0f;
	ret.m43 = 0.0f;
	ret.m44 = 1.0f;

	return ret;
}

Matrix4 Mat4_CreateRotateY(float fRad)
{
	Matrix4 ret;

	ret.m11 = cos(fRad);
	ret.m12 = 0.0f;
	ret.m13 = sin(fRad);
	ret.m14 = 0.0f;

	ret.m21 = 0.0f;
	ret.m22 = 1.0f;
	ret.m23 = 0.0f;
	ret.m24 = 0.0f;

	ret.m31 = -sin(fRad);
	ret.m32 = 0.0f;
	ret.m33 = cos(fRad);
	ret.m34 = 0.0f;

	ret.m41 = 0.0f;
	ret.m42 = 0.0f;
	ret.m43 = 0.0f;
	ret.m44 = 1.0f;

	return ret;
}

Matrix4 Mat4_CreateRotateZ(float fRad)
{
	Matrix4 ret;

	ret.m11 = cos(fRad);
	ret.m12 = sin(fRad);
	ret.m13 = 0.0f;
	ret.m14 = 0.0f;

	ret.m21 = -sin(fRad);
	ret.m22 = cos(fRad);
	ret.m23 = 0.0f;
	ret.m24 = 0.0f;

	ret.m31 = 0.0f;
	ret.m32 = 0.0f;
	ret.m33 = 1.0f;
	ret.m34 = 0.0f;

	ret.m41 = 0.0f;
	ret.m42 = 0.0f;
	ret.m43 = 0.0f;
	ret.m44 = 1.0f;

	return ret;
}

Matrix4 Mat4_CreateEulerRotate(Vector3 v3Rotate)
{
	Matrix4 T1 = Mat4_CreateRotateX(v3Rotate.x);
	Matrix4 T2 = Mat4_CreateRotateY(v3Rotate.y);
	Matrix4 T3 = Mat4_CreateRotateZ(v3Rotate.z);

	Matrix4 ret = Mult_Mat4Mat4(T3, Mult_Mat4Mat4(T2, T1));
	return ret;
}

Vector3 Mult_Mat4Vector3(Matrix4 T, Vector3 v3In, float w) 
{
	float4 v = XYZWToFloat4(v3In.x, v3In.y, v3In.z, w);
	float4 ret;

	ret.x = dot(XYZWToFloat4(T.m11, T.m12, T.m13, T.m14), v);
	ret.y = dot(XYZWToFloat4(T.m21, T.m22, T.m23, T.m24), v);
	ret.z = dot(XYZWToFloat4(T.m31, T.m32, T.m33, T.m34), v);
	ret.w = dot(XYZWToFloat4(T.m41, T.m42, T.m43, T.m44), v);

	return XYZToVector3(ret.x, ret.y, ret.z);
}

typedef struct
{
	Vector3 v3Min;
	Vector3 v3Max;
}
BBox;

typedef struct
{
	Vector3 v3A;
	Vector3 v3B;
	Vector3 v3C;
	Vector3 v3N;
}
Triangle;

typedef struct
{
	int nLeft;
	int nRight;

	Triangle triangle;
	BBox bbox;
}
BVHNodeTriangle;

typedef struct
{
	int nOffset;
	int nCount;
}
BVHNodeTriangleOffset;

typedef struct 
{
    BBox bbox;
	int nRigidBodyId;
    int nLeft;
    int nRight;
}
BVHObject;

typedef struct
{
	int nId;
	int nTriMeshId;
	int nIsEnabled;

	BBox inBBox;
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

	Vector3 v3ElapsedPosition;
	Vector3 v3ElapsedRotate;
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
		BBox bbox = rigidBody.inBBox;

		//Matrix4 mat4Translate = Mat4_CreateTranslate(rigidBody.v3Position);
		//Matrix4 mat4EulerRotate = Mat4_CreateEulerRotate(rigidBody.v3Rotate);
		//Matrix4 mat4Transform = Mult_Mat4Mat4(mat4Translate, mat4EulerRotate);
		Matrix4 mat4Transform = Mat4_CreateEulerRotate(rigidBody.v3Rotate);
		
		Vector3 v3Min = bbox.v3Min;
		Vector3 v3Max = bbox.v3Max;

		Vector3 v3In1 = XYZToVector3(v3Min.x, v3Min.y, v3Min.z);
		Vector3 v3In2 = XYZToVector3(v3Max.x, v3Min.y, v3Min.z);
		Vector3 v3In3 = XYZToVector3(v3Max.x, v3Min.y, v3Max.z);
		Vector3 v3In4 = XYZToVector3(v3Min.x, v3Min.y, v3Max.z);
		Vector3 v3In5 = XYZToVector3(v3Min.x, v3Max.y, v3Min.z);
		Vector3 v3In6 = XYZToVector3(v3Max.x, v3Max.y, v3Min.z);
		Vector3 v3In7 = XYZToVector3(v3Max.x, v3Max.y, v3Max.z);
		Vector3 v3In8 = XYZToVector3(v3Min.x, v3Max.y, v3Max.z);

		Vector3 v3Out1 = Mult_Mat4Vector3(mat4Transform, v3In1, 1.0f);
		Vector3 v3Out2 = Mult_Mat4Vector3(mat4Transform, v3In2, 1.0f);
		Vector3 v3Out3 = Mult_Mat4Vector3(mat4Transform, v3In3, 1.0f);
		Vector3 v3Out4 = Mult_Mat4Vector3(mat4Transform, v3In4, 1.0f);
		Vector3 v3Out5 = Mult_Mat4Vector3(mat4Transform, v3In5, 1.0f);
		Vector3 v3Out6 = Mult_Mat4Vector3(mat4Transform, v3In6, 1.0f);
		Vector3 v3Out7 = Mult_Mat4Vector3(mat4Transform, v3In7, 1.0f);
		Vector3 v3Out8 = Mult_Mat4Vector3(mat4Transform, v3In8, 1.0f);

		// 1
		v3Min = v3Out1;
		v3Max = v3Out1;

		// 2
		v3Min.x = min(v3Min.x, v3Out2.x);
		v3Min.y = min(v3Min.y, v3Out2.y);
		v3Min.z = min(v3Min.z, v3Out2.z);
		v3Max.x = max(v3Max.x, v3Out2.x);
		v3Max.y = max(v3Max.y, v3Out2.y);
		v3Max.z = max(v3Max.z, v3Out2.z);

		// 3
		v3Min.x = min(v3Min.x, v3Out3.x);
		v3Min.y = min(v3Min.y, v3Out3.y);
		v3Min.z = min(v3Min.z, v3Out3.z);
		v3Max.x = max(v3Max.x, v3Out3.x);
		v3Max.y = max(v3Max.y, v3Out3.y);
		v3Max.z = max(v3Max.z, v3Out3.z);

		// 4
		v3Min.x = min(v3Min.x, v3Out4.x);
		v3Min.y = min(v3Min.y, v3Out4.y);
		v3Min.z = min(v3Min.z, v3Out4.z);
		v3Max.x = max(v3Max.x, v3Out4.x);
		v3Max.y = max(v3Max.y, v3Out4.y);
		v3Max.z = max(v3Max.z, v3Out4.z);

		// 5
		v3Min.x = min(v3Min.x, v3Out5.x);
		v3Min.y = min(v3Min.y, v3Out5.y);
		v3Min.z = min(v3Min.z, v3Out5.z);
		v3Max.x = max(v3Max.x, v3Out5.x);
		v3Max.y = max(v3Max.y, v3Out5.y);
		v3Max.z = max(v3Max.z, v3Out5.z);

		// 6
		v3Min.x = min(v3Min.x, v3Out6.x);
		v3Min.y = min(v3Min.y, v3Out6.y);
		v3Min.z = min(v3Min.z, v3Out6.z);
		v3Max.x = max(v3Max.x, v3Out6.x);
		v3Max.y = max(v3Max.y, v3Out6.y);
		v3Max.z = max(v3Max.z, v3Out6.z);

		// 7
		v3Min.x = min(v3Min.x, v3Out7.x);
		v3Min.y = min(v3Min.y, v3Out7.y);
		v3Min.z = min(v3Min.z, v3Out7.z);
		v3Max.x = max(v3Max.x, v3Out7.x);
		v3Max.y = max(v3Max.y, v3Out7.y);
		v3Max.z = max(v3Max.z, v3Out7.z);

		// 8
		v3Min.x = min(v3Min.x, v3Out8.x);
		v3Min.y = min(v3Min.y, v3Out8.y);
		v3Min.z = min(v3Min.z, v3Out8.z);
		v3Max.x = max(v3Max.x, v3Out8.x);
		v3Max.y = max(v3Max.y, v3Out8.y);
		v3Max.z = max(v3Max.z, v3Out8.z);

		inoutRigidBodies[id].bbox.v3Min = v3Min;
		inoutRigidBodies[id].bbox.v3Max = v3Max;
		
		rigidBody = inoutRigidBodies[id];
		bbox = rigidBody.bbox;
		
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

	if (0 == rigidBody.nIsEnabled)
	{
		return;
	}

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

	// calc
	inoutRigidBodies[id].v3ElapsedPosition = Float3ToVector3(f3Position);
	inoutRigidBodies[id].v3ElapsedRotate   = Float3ToVector3(f3Rotate);
	float3 f3LinearAcceleration = f3Gravity + (f3Force / fMass);
	float3 f3AngularAcceleration = (f3Torque / fMass);

	int nSteps = 10;
	float fStepDt = dt / (float)nSteps;
	for (int i = 0; i < nSteps; i++)
	{
		// calc -> Euler method
		f3LinearVelocity += f3LinearAcceleration * fStepDt;
		f3Position += f3LinearVelocity * fStepDt;

		f3AngularVelocity += f3AngularAcceleration * fStepDt;
		f3Rotate += f3AngularVelocity * fStepDt;
	}

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
