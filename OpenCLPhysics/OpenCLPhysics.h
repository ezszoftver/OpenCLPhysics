#pragma once

#include <vector>
#include <string>
#include "glm/glm.hpp"
#include "CL/cl.h"

namespace OpenCLPhysics
{

	class Triangle
	{
	public:
		Triangle();
		Triangle(glm::vec3 v3PosA, glm::vec3 v3PosB, glm::vec3 v3PosC, glm::vec3 v3NormalA, glm::vec3 v3NormalB, glm::vec3 v3NormalC);
		~Triangle();

		glm::vec3 m_v3PosA;
		glm::vec3 m_v3PosB;
		glm::vec3 m_v3PosC;
		glm::vec3 m_v3NormalA;
		glm::vec3 m_v3NormalB;
		glm::vec3 m_v3NormalC;
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
		static const uint8_t HITTYPE_STATICMESH_DYNAMICMESH = 1;
		static const uint8_t HITTYPE_STATICMESH_SPHERE = 2;

		static const uint8_t HITTYPE_DYNAMICMESH_DYNAMICMESH = 3;
		static const uint8_t HITTYPE_DYNAMICMESH_SPHERE = 4;

		static const uint8_t HITTYPE_SPHERE_SPHERE = 5;
		static const uint8_t HITTYPE_SPHERE_DYNAMICMESH = 6;

		Hit();
		Hit(uint8_t nType, int32_t nBodyAId, int32_t nBodyBId, glm::vec3 v3WorldPosition, glm::vec3 v3Normal);
		~Hit();

		uint8_t m_nType;
		int32_t m_nBodyAId;
		int32_t m_nBodyBId;
	};

	class Sphere
	{
	public:
		Sphere();
		Sphere(glm::vec3 v3Center, float fRadius);
		~Sphere();

		glm::vec3 m_v3Position;
		float m_fRadius;

		Hit m_hits[32];
		uint8_t m_nNumHits;
	};

	class BVHNodeSphere
	{
	public:
		BVHNodeSphere();
		bool IsLeaf();

		BVHNodeSphere* m_pLeft;
		BVHNodeSphere* m_pRight;

		Sphere* m_pSphere;
		BBox* m_pBBox;
	};

	class DynamicMesh
	{
	public:
		DynamicMesh();
		~DynamicMesh();

		std::vector< BVHNodeSphere* > m_listBVHNodeSpheres;

		Hit m_hits[32];
		uint8_t m_nNumHits;
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

	class StaticMesh
	{
	public:
		StaticMesh();
		~StaticMesh();

		std::vector< BVHNodeTriangle* > m_listBVHNodeTriangles;
	};

	class Physics
	{
	public:
		Physics();
		~Physics();

		std::vector<std::string> GetDevices();
		bool CreateDevice(std::string strDeviceName);
		void CloseDevice();

		uint32_t GenStaticMesh();
		uint32_t GenDynamicMesh();
		uint32_t GenSphere();

		void SetStaticMesh(uint32_t nId, std::vector<glm::vec3> *listVertices, std::vector<glm::vec3>* listNormals, glm::mat4 matTransform = glm::mat4(1.0f));
		void SetDynamicMesh(uint32_t nId, std::vector<glm::vec3>* listVertices, std::vector<glm::vec3>* listNormals, float fDistance, float fRadius);
		void SetSphere(uint32_t nId, glm::vec3 v3Position, float fRadius);

		void SetGravity(glm::vec3 vec3Gravity);
		glm::vec3 GetGravity();

		glm::mat4 GetTransform(uint32_t nId);

		void SetPosition(uint32_t nId, glm::vec3 vec3Position);
		glm::vec3 GetPosition(uint32_t nId);
		void SetEulerRotate(uint32_t nId, glm::vec3 vec3EulerRotate);
		glm::vec3 GetEulerRotate(uint32_t nId);
		void SetLinearVelocity(uint32_t nId, glm::vec3 vec3LinearVelocity);
		glm::vec3 GetLinearVelocity(uint32_t nId);
		void SetAngularVelocity(uint32_t nId, glm::vec3 vec3AngularVelocity);
		glm::vec3 GetAngularVelocity(uint32_t nId);
		void SetLinearAcceleration(uint32_t nId, glm::vec3 vec3LinearVelocity);
		glm::vec3 GetLinearAcceleration(uint32_t nId);
		void SetAngularAcceleration(uint32_t nId, glm::vec3 vec3AngularVelocity);
		glm::vec3 GetAngularAcceleration(uint32_t nId);
		void SetForce(uint32_t nId, glm::vec3 vec3LinearVelocity);
		glm::vec3 GetForce(uint32_t nId);
		void SetTorque(uint32_t nId, glm::vec3 vec3AngularVelocity);
		glm::vec3 GetTorque(uint32_t nId);

		void SetMass(uint32_t nId, float fMass);
		float GetMass(uint32_t nId);
		void SetRestitution(uint32_t nId, float fRestitution);
		float GetRestitution(uint32_t nId);
		void SetFriction(uint32_t nId, float fFriction);
		float GetFriction(uint32_t nId);

		void SetLinearDamping(uint32_t nId, float fLinearDamping);
		float GetLinearDamping(uint32_t nId);

		void SetAngularDamping(uint32_t nId, float fAngularDamping);
		float GetAngularDamping(uint32_t nId);

		void Commit();
		void Update(float dt);

	private:
		cl_context m_context;
		cl_command_queue m_command_queue;

		std::vector< StaticMesh* > m_listStaticMeshs;
		std::vector< DynamicMesh* > m_listDynamicMeshs;
		std::vector< Sphere* > m_listSpheres;
	};

}