#pragma once

#include <vector>
#include <string>
#include "glm/glm.hpp"
#include "CL/cl.h"

#define MAX_HITS_COUNT_PER_OBJECTS 16

namespace OpenCLPhysics
{

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

		std::vector< BVHNodeTriangle* > m_listBVHNodeTriangles;

		uint8_t m_nTop;
		Hit m_arrHits[MAX_HITS_COUNT_PER_OBJECTS];
	};

	typedef struct structRigidBody
	{
		int32_t m_nTriMeshId = -1;
		float m_fRadius = 0;

		float m_fMass = 0;
		float m_fRestitution = 0;
		float m_fFriction = 0;
		float m_fLinearDamping = 0;
		float m_fAngularDamping = 0;

		float m_v3ForceX = 0;
		float m_v3ForceY = 0;
		float m_v3ForceZ = 0;

		float m_v3LinearAccelerationX = 0;
		float m_v3LinearAccelerationY = 0;
		float m_v3LinearAccelerationZ = 0;

		float m_v3LinearVelocityX = 0;
		float m_v3LinearVelocityY = 0;
		float m_v3LinearVelocityZ = 0;

		float m_v3PositionX = 0;
		float m_v3PositionY = 0;
		float m_v3PositionZ = 0;

		float m_v3TorqueX = 0;
		float m_v3TorqueY = 0;
		float m_v3TorqueZ = 0;

		float m_v3AngularAccelerationX = 0;
		float m_v3AngularAccelerationY = 0;
		float m_v3AngularAccelerationZ = 0;

		float m_v3AngularVelocityX = 0;
		float m_v3AngularVelocityY = 0;
		float m_v3AngularVelocityZ = 0;

		float m_v3RotateX = 0;
		float m_v3RotateY = 0;
		float m_v3RotateZ = 0;
	}
	RigidBody;

	class Physics
	{
		static const int32_t TRIMESH_START = 0;
		static const int32_t TRIMESH_COUNT = 100000000; // 100 million

	public:
		Physics();
		~Physics();

		std::vector<std::string> GetDevices();
		bool CreateDevice(std::string strDeviceName);
		void CloseDevice();

		int32_t GenTriMesh();
		void SetTriMesh(int32_t nId, std::vector<glm::vec3> *listVertices);

		void SetGravity(glm::vec3 vec3Gravity);
		glm::vec3 GetGravity();

		glm::mat4 GetTransform(int32_t nId);

		void SetPosition(int32_t nId, glm::vec3 vec3Position);
		glm::vec3 GetPosition(int32_t nId);
		void SetEulerRotate(int32_t nId, glm::vec3 vec3EulerRotate);
		glm::vec3 GetEulerRotate(int32_t nId);
		void SetLinearVelocity(int32_t nId, glm::vec3 vec3LinearVelocity);
		glm::vec3 GetLinearVelocity(int32_t nId);
		void SetAngularVelocity(int32_t nId, glm::vec3 vec3AngularVelocity);
		glm::vec3 GetAngularVelocity(int32_t nId);
		void SetLinearAcceleration(int32_t nId, glm::vec3 vec3LinearVelocity);
		glm::vec3 GetLinearAcceleration(int32_t nId);
		void SetAngularAcceleration(int32_t nId, glm::vec3 vec3AngularVelocity);
		glm::vec3 GetAngularAcceleration(int32_t nId);
		void SetForce(int32_t nId, glm::vec3 vec3LinearVelocity);
		glm::vec3 GetForce(int32_t nId);
		void SetTorque(int32_t nId, glm::vec3 vec3AngularVelocity);
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

		bool Commit();
		void Update(float dt);

	private:
		void OpenCL_RefitTree();

		cl_context m_context;
		cl_command_queue m_command_queue;
		cl_program m_program;
		cl_kernel m_kernelRefitTree;
		cl_mem m_clmem_RigidBodies;

		std::vector< RigidBody > m_listRigidBodies;
		std::vector< TriMesh* > m_listTriMeshs;
	};

}
