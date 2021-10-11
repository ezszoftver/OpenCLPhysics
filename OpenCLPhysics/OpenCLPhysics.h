#pragma once

#include <vector>
#include <string>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "CL/cl.h"

#include "Script.h"

namespace OpenCLPhysics
{
	typedef struct _structVector3
	{
		float x = 0;
		float y = 0;
		float z = 0;
	}
	structVector3;

	typedef struct _structBBox
	{
		structVector3 v3Min;
		structVector3 v3Max;
	}
	structBBox;

	typedef struct _structBVHObject 
	{
		structBBox m_BBox;
		int32_t m_nRigidBodyId = -1;

		int32_t m_nLeft = -1;
		int32_t m_nRight = -1;
	}
	structBVHObject;

	typedef struct _structRigidBody
	{
		int32_t m_nRigidBodyId = -1;
		int32_t m_nTriMeshId = -1;
		int32_t m_nIsEnabled = 1;
		int32_t m_nIsIntegrateEnabled = 1;

		structBBox m_inBBox;
		structBBox m_BBox;

		float m_fMass = 0;
		float m_fRestitution = 0;
		float m_fFriction = 0;
		float m_fLinearDamping = 0;
		float m_fAngularDamping = 0;

		structVector3 m_v3Force;
		structVector3 m_v3LinearAcceleration;
		structVector3 m_v3LinearVelocity;
		structVector3 m_v3Position;

		structVector3 m_v3Torque;
		structVector3 m_v3AngularAcceleration;
		structVector3 m_v3AngularVelocity;
		structVector3 m_v3Rotate;

		structVector3 m_v3ElapsedPosition;
		structVector3 m_v3ElapsedRotate;
	}
	structRigidBody;

	typedef struct _structTriangle 
	{
		structVector3 m_v3PosA;
		structVector3 m_v3PosB;
		structVector3 m_v3PosC;
		structVector3 m_v3Normal;
	}
	structTriangle;

	class Triangle
	{
	public:
		Triangle();
		Triangle(glm::vec3 v3PosA, glm::vec3 v3PosB, glm::vec3 v3PosC, glm::vec3 v3Normal);
		~Triangle();

		structTriangle GetStructTriangle();

		glm::vec3 m_v3PosA;
		glm::vec3 m_v3PosB;
		glm::vec3 m_v3PosC;
		glm::vec3 m_v3Normal;
	};

	class BBox 
	{
	public:
		BBox();
		BBox(glm::vec3 v3Min, glm::vec3 v3Max);

		static BBox* Create(Triangle *pTriangle);
		static BBox* Create(Triangle *pTriangle1, Triangle* pTriangle2);
		static BBox* Create(BBox* pBBox);
		static BBox* Create(BBox *pBBox1, BBox *pBBox2);

		void Add(structBBox bbox);
		void Add(glm::vec3 v);
		structBBox GetStructBBox();

		glm::vec3 m_v3Min;
		glm::vec3 m_v3Max;
	};

	typedef struct _structHit 
	{
		int32_t m_nRigidBodyAId = -1;
		int32_t m_nRigidBodyBId = -1;

		structVector3 m_v3HitPointInWorld;
		structVector3 m_v3Normal;
	}
	structHit;

	typedef struct _structHits 
	{
		int32_t m_nNumHits = 0;
		structHit m_hits[MAX_HITS];
	}
	structHits;

	typedef struct _structBVHNodeTriangle 
	{
		int32_t m_nLeft = -1;
		int32_t m_nRight = -1;

		structTriangle m_Triangle;
		structBBox m_BBox;
	}
	structBVHNodeTriangle;

	typedef struct _structBVHNodeTriangleOffset
	{
		int32_t m_nOffset;
		int32_t m_nCount;
	}
	structBVHNodeTriangleOffset;

	class BVHNodeTriangle
	{
	public:
		BVHNodeTriangle(int32_t nId);
		bool IsLeaf();

		structBVHNodeTriangle GetStructBVHNodeTriangle();

		int32_t m_nId;
		BVHNodeTriangle* m_pLeft;
		BVHNodeTriangle* m_pRight;

		Triangle* m_pTriangle;
		BBox *m_pBBox;
	};

	class TriMesh
	{
	public:
		TriMesh();
		~TriMesh();

		std::vector< BVHNodeTriangle* > *m_pListBVHNodeTriangles;
		int32_t m_nOffset;
		int32_t m_nCount;
	};

	class Physics
	{
		int32_t TRIMESH_COUNT = 0;

	public:
		Physics();
		~Physics();

		std::vector<std::string> GetDevices();
		bool CreateDevice(std::string strDeviceName, int32_t nTriMeshsCount = 1000000);
		void CloseDevice();

		int32_t CreateTriMesh(std::vector<glm::vec3>* pListVertices, bool bIsCommit = true);
		int32_t CreateFromId(int32_t nFromId, bool bIsCommit = true);
		bool DeleteTriMesh(int32_t nId);

		bool Commit();

		void SetEnabled(int32_t nId, bool bValue);
		bool IsEnabled(int32_t nId);

		void SetGravity(glm::vec3 v3Gravity);
		glm::vec3 GetGravity();

		glm::mat4 GetTransform(int32_t nId);

		void SetPosition(int32_t nId, glm::vec3 v3Position);
		glm::vec3 GetPosition(int32_t nId);
		void SetEulerRotate(int32_t nId, glm::vec3 v3EulerRotate);
		glm::vec3 GetEulerRotate(int32_t nId);
		void SetLinearVelocity(int32_t nId, glm::vec3 v3LinearVelocity);
		glm::vec3 GetLinearVelocity(int32_t nId);
		void SetAngularVelocity(int32_t nId, glm::vec3 v3AngularVelocity);
		glm::vec3 GetAngularVelocity(int32_t nId);
		void SetLinearAcceleration(int32_t nId, glm::vec3 v3LinearAcceleration);
		glm::vec3 GetLinearAcceleration(int32_t nId);
		void SetAngularAcceleration(int32_t nId, glm::vec3 v3AngularAcceleration);
		glm::vec3 GetAngularAcceleration(int32_t nId);
		void SetForce(int32_t nId, glm::vec3 v3Force);
		glm::vec3 GetForce(int32_t nId);
		void SetTorque(int32_t nId, glm::vec3 v3Torque);
		glm::vec3 GetTorque(int32_t nId);

		void SetMass(int32_t nId, float fMass);
		float GetMass(int32_t nId);
		void SetRestitution(int32_t nId, float fRestitution);
		float GetRestitution(int32_t nId);
		void SetFriction(int32_t nId, float fFriction);
		float GetFriction(int32_t nId);
		void SetLinearDamping(int32_t nId, float fLinearDamping);
		float GetLinearDamping(int32_t nId);
		void SetAngularDamping(int32_t nId, float fAngularDamping);
		float GetAngularDamping(int32_t nId);
		
		glm::vec3 GetBBoxMin(int32_t nId);
		glm::vec3 GetBBoxMax(int32_t nId);

		bool Update(float dt, uint16_t nNumSteps = 1);

		uint32_t MaxRigidBodies();
		uint32_t NumRigidBodies();

		std::vector < structHits >* GetHits();

	private:
		void SetTriMesh(int32_t nId, std::vector<glm::vec3>* pListVertices);
		bool StepUpdate(float dt);
		void CreateBVHObjects();
		bool UpdateBVHObjects();
		void ReleaseBVHObjects();
		bool Integrate(float dt);
		bool CollisionDetection();
		void CollisionResponse(float dt);

		cl_device_id m_device;
		cl_context m_context;
		cl_command_queue m_command_queue;
		cl_program m_program;

		cl_kernel m_kernelUpdateBVHObjects;
		cl_kernel m_kernelIntegrate;
		cl_kernel m_kernelCollisionDetection;

		int32_t m_nUpdateBVHObjects_Local;
		int32_t m_nIntegrate_Local;
		int32_t m_nCollisionDetection_Local;

		cl_mem m_clmem_inoutRigidBodies = 0;
		cl_mem m_clmem_inoutBVHObjects = 0;
		cl_mem m_clmem_inBVHNodeTriangles = 0;
		cl_mem m_clmem_inBVHNodeTrianglesOffsets = 0;
		cl_mem m_clmem_inoutHits = 0;
		cl_mem m_clmem_inoutIsCollisionResponse = 0;

		structVector3 m_v3Gravity;
		std::vector< int32_t > m_listFreeIds;
		std::vector< structRigidBody > m_listRigidBodies;
		std::vector< TriMesh* > m_listTriMeshs;

		// objects
		std::vector< std::vector< structBVHObject >* > m_BVHObjectsLevels; // ebbõl kiszámítható a "count", és az "offset"
		std::vector< structBVHObject > m_listBVHObjects;

		// triangles
		std::vector< structBVHNodeTriangle > m_listBVHNodeTriangles;
		std::vector< structBVHNodeTriangleOffset > m_listBVHNodeTrianglesOffsets;

		// hits
		std::vector < structHits > m_listHits;
		std::vector < int32_t > m_listIsCollisionResponse;
	};

}
