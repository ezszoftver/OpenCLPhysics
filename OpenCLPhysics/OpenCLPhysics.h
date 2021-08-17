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

	class RigidBody
	{
	public:
		RigidBody();

		int32_t m_nTriMeshId;
		float m_fRadius;

		float m_fMass;
		float m_fRestitution;
		float m_fFriction;
		float m_fLinearDamping;
		float m_fAngularDamping;

		glm::vec3 m_v3Force;
		glm::vec3 m_v3LinearAcceleration;
		glm::vec3 m_v3LinearVelocity;
		glm::vec3 m_v3Position;
		
		glm::vec3 m_v3Torque;
		glm::vec3 m_v3AngularAcceleration;
		glm::vec3 m_v3AngularVelocity;
		glm::vec3 m_v3Rotate;
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

		void Commit();
		void Update(float dt);

	private:
		void OpenCL_RefitTree();

		cl_context m_context;
		cl_command_queue m_command_queue;
		cl_program m_program;
		cl_kernel kernelRefitTree;

		std::vector< RigidBody* > m_listRigidBodies;
		std::vector< TriMesh* > m_listTriMeshs;
	};

}