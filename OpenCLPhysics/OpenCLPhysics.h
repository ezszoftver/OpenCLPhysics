#pragma once

#include <vector>
#include <string>
#include "glm/glm.hpp"
#include "CL/cl.h"

#define MAX_HITS_COUNT_PER_OBJECTS 32

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
		// min
		float v3MinX = 0;
		float v3MinY = 0;
		float v3MinZ = 0;
		// max
		float v3MaxX = 0;
		float v3MaxY = 0;
		float v3MaxZ = 0;
	}
	structBBox;

	typedef struct _structBVHObject 
	{
		structBBox m_BBox;
		int32_t m_nLeft = -1;
		int32_t m_nRight = -1;
	}
	structBVHObject;

	typedef struct _structRigidBody
	{
		int32_t m_nRigidBodyId = -1;
		int32_t m_nTriMeshId = -1;
		int32_t m_nIsEnabled = 1;

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

	class Triangle
	{
	public:
		Triangle();
		Triangle(glm::vec3 v3PosA, glm::vec3 v3PosB, glm::vec3 v3PosC, glm::vec3 v3Normal);
		~Triangle();

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

	class Hit 
	{
	public:

		Hit();
		Hit(int32_t nBodyAId, int32_t nBodyBId, glm::vec3 v3WorldPosition, glm::vec3 v3Normal);
		~Hit();

		int32_t m_nBodyAId;
		int32_t m_nBodyBId;
	};

	class BVHNodeTriangle
	{
	public:
		BVHNodeTriangle();
		bool IsLeaf();

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

		int32_t m_nRigidBodyId;

		std::vector< BVHNodeTriangle* > *m_pListBVHNodeTriangles;

		uint8_t m_nTop;
		Hit m_arrHits[MAX_HITS_COUNT_PER_OBJECTS];
	};

	class Physics
	{
		int32_t TRIMESH_START = 0;
		int32_t TRIMESH_COUNT = 1;

	public:
		Physics();
		~Physics();

		std::vector<std::string> GetDevices();
		bool CreateDevice(std::string strDeviceName, int32_t nTriMeshsCount = 1000000);
		void CloseDevice();

		int32_t CreateTriMesh(std::vector<glm::vec3>* pListVertices);
		int32_t CreateFromId(int32_t nFromId);
		bool DeleteTriMesh(int32_t nId);

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
		
		bool Update(float dt, uint16_t nNumSteps = 1);

		uint32_t MaxRigidBodies();
		uint32_t NumRigidBodies();

	private:
		bool Commit();
		void SetTriMesh(int32_t nId, std::vector<glm::vec3>* pListVertices);
		bool StepUpdate(float dt);
		void CreateBVHObjects();
		bool UpdateBVHObjects();
		void ReleaseBVHObjects();
		bool Integrate(float dt);

		cl_context m_context;
		cl_command_queue m_command_queue;
		cl_program m_program;
		cl_kernel m_kernelUpdateBVHObjects;
		cl_kernel m_kernelIntegrate;
		cl_mem m_clmem_inoutRigidBodies = 0;
		cl_mem m_clmem_inoutBVHObjects = 0;

		structVector3 m_v3Gravity;
		std::vector< int32_t > m_listFreeIds;
		std::vector< structRigidBody > m_listRigidBodies;
		std::vector< TriMesh* > m_listTriMeshs;

		std::vector< std::vector< structBVHObject >* > m_BVHObjectsLevels; // ebbõl kiszámítható a "count", és az "offset"
		std::vector< structBVHObject > m_listBVHObjects;
	};

}
