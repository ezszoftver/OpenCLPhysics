#include "OpenCLPhysics.h"

// sort algorithm example
#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector

namespace OpenCLPhysics 
{

	Triangle::Triangle()
	{
		m_v3PosA = glm::vec3(0,0,0);
		m_v3PosB = glm::vec3(0,0,0);
		m_v3PosC = glm::vec3(0,0,0);

		m_v3NormalA = glm::vec3(0, 0, 0);
		m_v3NormalB = glm::vec3(0, 0, 0);
		m_v3NormalC = glm::vec3(0, 0, 0);
	}

	Triangle::Triangle(glm::vec3 v3PosA, glm::vec3 v3PosB, glm::vec3 v3PosC, glm::vec3 v3NormalA, glm::vec3 v3NormalB, glm::vec3 v3NormalC)
	{
		m_v3PosA = v3PosA;
		m_v3PosB = v3PosB;
		m_v3PosC = v3PosC;

		m_v3NormalA = v3NormalA;
		m_v3NormalB = v3NormalB;
		m_v3NormalC = v3NormalC;
	}

	Triangle::~Triangle()
	{
	}

	BBox::BBox() 
	{
		m_v3Min = glm::vec3(0, 0, 0);
		m_v3Max = glm::vec3(0, 0, 0);
	}

	BBox::BBox(glm::vec3 v3Min, glm::vec3 v3Max) 
	{
		m_v3Min = v3Min;
		m_v3Max = v3Max;
	}

	BBox* BBox::Create(Triangle* pTriangle) 
	{
		glm::vec3 v3Min(+FLT_MAX, +FLT_MAX, +FLT_MAX);
		glm::vec3 v3Max(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		std::vector<glm::vec3> vertices;
		vertices.push_back(pTriangle->m_v3PosA);
		vertices.push_back(pTriangle->m_v3PosB);
		vertices.push_back(pTriangle->m_v3PosC);

		for (uint64_t i = 0; i < vertices.size(); i++)
		{
			glm::vec3 v = vertices[i];

			v3Min.x = std::min(v3Min.x, v.x);
			v3Min.y = std::min(v3Min.y, v.y);
			v3Min.z = std::min(v3Min.z, v.z);

			v3Max.x = std::max(v3Max.x, v.x);
			v3Max.y = std::max(v3Max.y, v.y);
			v3Max.z = std::max(v3Max.z, v.z);
		}
		
		return new BBox(v3Min, v3Max);
	}

	BBox* BBox::Create(Triangle* pTriangle1, Triangle* pTriangle2) 
	{
		glm::vec3 v3Min(+FLT_MAX, +FLT_MAX, +FLT_MAX);
		glm::vec3 v3Max(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		std::vector<glm::vec3> vertices;
		vertices.push_back(pTriangle1->m_v3PosA);
		vertices.push_back(pTriangle1->m_v3PosB);
		vertices.push_back(pTriangle1->m_v3PosC);
		vertices.push_back(pTriangle2->m_v3PosA);
		vertices.push_back(pTriangle2->m_v3PosB);
		vertices.push_back(pTriangle2->m_v3PosC);

		for (uint64_t i = 0; i < vertices.size(); i++)
		{
			glm::vec3 v = vertices[i];

			v3Min.x = std::min(v3Min.x, v.x);
			v3Min.y = std::min(v3Min.y, v.y);
			v3Min.z = std::min(v3Min.z, v.z);

			v3Max.x = std::max(v3Max.x, v.x);
			v3Max.y = std::max(v3Max.y, v.y);
			v3Max.z = std::max(v3Max.z, v.z);
		}

		return new BBox(v3Min, v3Max);
	}

	BBox* BBox::Create(BBox* pBBox) 
	{
		return new BBox(pBBox->m_v3Min, pBBox->m_v3Max);
	}

	BBox* BBox::Create(BBox* pBBox1, BBox* pBBox2)
	{
		glm::vec3 v3Min(+FLT_MAX, +FLT_MAX, +FLT_MAX);
		glm::vec3 v3Max(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		std::vector<glm::vec3> vertices;
		vertices.push_back(pBBox1->m_v3Min);
		vertices.push_back(pBBox1->m_v3Max);
		vertices.push_back(pBBox2->m_v3Min);
		vertices.push_back(pBBox2->m_v3Max);

		for (uint64_t i = 0; i < vertices.size(); i++)
		{
			glm::vec3 v = vertices[i];

			v3Min.x = std::min(v3Min.x, v.x);
			v3Min.y = std::min(v3Min.y, v.y);
			v3Min.z = std::min(v3Min.z, v.z);

			v3Max.x = std::max(v3Max.x, v.x);
			v3Max.y = std::max(v3Max.y, v.y);
			v3Max.z = std::max(v3Max.z, v.z);
		}

		return new BBox(v3Min, v3Max);
	}

	Sphere::Sphere() 
	{
		m_v3Position = glm::vec3(0, 0, 0);
		m_fRadius = 0.0f;

		m_nNumHits = 0;
	}

	Sphere::Sphere(glm::vec3 v3Center, float fRadius) 
	{
		m_v3Position = v3Center;
		m_fRadius = fRadius;
	}

	Sphere::~Sphere() 
	{
	}

	Hit::Hit() 
	{
		m_nType = 0;
		m_nBodyAId = -1;
		m_nBodyBId = -1;
	}

	Hit::Hit(uint8_t nType, int32_t nBodyAId, int32_t nBodyBId, glm::vec3 v3WorldPosition, glm::vec3 v3Normal)
	{
		m_nType = nType;
		m_nBodyAId = nBodyAId;
		m_nBodyBId = nBodyBId;
	}

	Hit::~Hit() 
	{
	}

	BVHNodeSphere::BVHNodeSphere()
	{
		m_pLeft = nullptr;
		m_pRight = nullptr;

		m_pSphere = nullptr;
		m_pBBox = nullptr;
	}

	bool BVHNodeSphere::IsLeaf()
	{
		return (m_pLeft == nullptr && m_pRight == nullptr);
	}

	DynamicMesh::DynamicMesh()
	{
		m_nNumHits = 0;
	}

	DynamicMesh::~DynamicMesh()
	{
		for (uint64_t i = 0; i < m_listBVHNodeSpheres.size(); i++)
		{
			delete m_listBVHNodeSpheres[i];
		}
		m_listBVHNodeSpheres.clear();
	}

	BVHNodeTriangle::BVHNodeTriangle()
	{
		m_pLeft = nullptr;
		m_pRight = nullptr;
		
		m_pTriangle = nullptr;
		m_pBBox = nullptr;
	}

	bool BVHNodeTriangle::IsLeaf()
	{
		return (m_pLeft == nullptr && m_pRight == nullptr);
	}

	StaticMesh::StaticMesh()
	{
	}
		
	StaticMesh::~StaticMesh() 
	{
		for (uint64_t i = 0; i < m_listBVHNodeTriangles.size(); i++) 
		{
			delete m_listBVHNodeTriangles[i];
		}
		m_listBVHNodeTriangles.clear();
	}

	Physics::Physics()
	{
		m_context = nullptr;
		m_command_queue = nullptr;
	}

	Physics::~Physics()
	{
	}

	std::vector<std::string> Physics::GetDevices()
	{
		std::vector<std::string> listRet;

		cl_int status = CL_SUCCESS;
		cl_uint numPlatforms;
		status = clGetPlatformIDs(0, nullptr, &numPlatforms);
		if (status != CL_SUCCESS) { return listRet; }

		if (0 == numPlatforms)
		{
			return listRet;
		}

		std::vector<cl_platform_id> platformIds(numPlatforms);
		std::vector<cl_platform_id> validIds;

		status = clGetPlatformIDs(numPlatforms, &platformIds[0], nullptr);
		if (status != CL_SUCCESS) { return listRet; }

		cl_device_type type = CL_DEVICE_TYPE_GPU;

		for (cl_platform_id& platformId : platformIds)
		{
			size_t size = 0;
			status = clGetPlatformInfo(platformId, CL_PLATFORM_VERSION, 0, nullptr, &size);
			if (status != CL_SUCCESS) { return listRet; }

			std::vector<char> version(size);

			status = clGetPlatformInfo(platformId, CL_PLATFORM_VERSION, size, &version[0], nullptr);
			if (status != CL_SUCCESS) { return listRet; }

			std::string versionstr(version.begin(), version.end());

			// Only use CL1.2+ platforms
			if (versionstr.find("OpenCL 1.0") != std::string::npos ||
				versionstr.find("OpenCL 1.1") != std::string::npos)
			{
				continue;
			}

			cl_uint numDevices = 0;
			status = clGetDeviceIDs(platformId, type, 0, NULL, &numDevices);
			if (status != CL_SUCCESS) { return listRet; }

			if (0 == numDevices)
			{
				continue;
			}

			std::vector<cl_device_id> devices(numDevices);
			clGetDeviceIDs(platformId, type, numDevices, &devices[0], NULL);

			for (cl_device_id& deviceId : devices)
			{
				size_t size = 0;
				clGetDeviceInfo(deviceId, CL_DEVICE_NAME, 0, NULL, &size);

				std::vector<char> name(size);

				clGetDeviceInfo(deviceId, CL_DEVICE_NAME, size, &name[0], NULL);

				std::string namestr(name.begin(), name.end());

				listRet.push_back(namestr);
			}
		}

		return listRet;
	}

	bool Physics::CreateDevice(std::string strDeviceName)
	{
		cl_int status = CL_SUCCESS;
		cl_uint numPlatforms;
		status = clGetPlatformIDs(0, nullptr, &numPlatforms);
		if (status != CL_SUCCESS) { return false; }

		if (0 == numPlatforms)
		{
			return false;
		}

		std::vector<cl_platform_id> platformIds(numPlatforms);
		std::vector<cl_platform_id> validIds;

		status = clGetPlatformIDs(numPlatforms, &platformIds[0], nullptr);
		if (status != CL_SUCCESS) { return false; }

		cl_device_type type = CL_DEVICE_TYPE_GPU;

		for (cl_platform_id& platformId : platformIds)
		{
			size_t size = 0;
			status = clGetPlatformInfo(platformId, CL_PLATFORM_VERSION, 0, nullptr, &size);
			if (status != CL_SUCCESS) { return false; }

			std::vector<char> version(size);

			status = clGetPlatformInfo(platformId, CL_PLATFORM_VERSION, size, &version[0], nullptr);
			if (status != CL_SUCCESS) { return false; }

			std::string versionstr(version.begin(), version.end());

			// Only use CL1.2+ platforms
			if (versionstr.find("OpenCL 1.0") != std::string::npos ||
				versionstr.find("OpenCL 1.1") != std::string::npos)
			{
				continue;
			}

			cl_uint numDevices = 0;
			status = clGetDeviceIDs(platformId, type, 0, NULL, &numDevices);
			if (status != CL_SUCCESS) { return false; }

			if (0 == numDevices)
			{
				continue;
			}

			std::vector<cl_device_id> devices(numDevices);
			clGetDeviceIDs(platformId, type, numDevices, &devices[0], NULL);

			for (cl_device_id& deviceId : devices)
			{
				size_t size = 0;
				clGetDeviceInfo(deviceId, CL_DEVICE_NAME, 0, NULL, &size);

				std::vector<char> name(size);

				clGetDeviceInfo(deviceId, CL_DEVICE_NAME, size, &name[0], NULL);

				std::string namestr(name.begin(), name.end());

				//listRet.push_back(namestr);
				if (namestr == strDeviceName)
				{
					/* Create OpenCL context */
					m_context = clCreateContext(NULL, 1, &deviceId, NULL, NULL, &status);
					if (status != CL_SUCCESS) { return false; }

					/* Create Command Queue */
					m_command_queue = clCreateCommandQueue(m_context, deviceId, 0, &status);
					if (status != CL_SUCCESS) { return false; }

					return true;
				}
			}
		}

		return false;
	}

	void Physics::CloseDevice()
	{
	}

	uint32_t Physics::GenStaticMesh()
	{
		uint32_t nId = (uint32_t)m_listStaticMeshs.size();
		m_listStaticMeshs.push_back( new StaticMesh() );
		return nId;
	}

	uint32_t Physics::GenDynamicMesh()
	{
		uint32_t nId = (uint32_t)m_listDynamicMeshs.size();
		m_listDynamicMeshs.push_back(new DynamicMesh());
		return nId;
	}

	uint32_t Physics::GenSphere() 
	{
		uint32_t nId = (uint32_t)m_listSpheres.size();
		m_listSpheres.push_back(new Sphere());
		return nId;
	}

	bool SortTrianglesFunc(Triangle *a, Triangle *b) 
	{ 
		if (fabs(a->m_v3PosA.x - b->m_v3PosA.x) > 0.0001f) { return (a->m_v3PosA.x < b->m_v3PosA.x); }
		if (fabs(a->m_v3PosA.y - b->m_v3PosA.y) > 0.0001f) { return (a->m_v3PosA.y < b->m_v3PosA.y); }
		return (a->m_v3PosA.z < b->m_v3PosA.z);
	}

	int SearchNeightboorBBox(BBox *pBBoxA, std::vector< BVHNodeTriangle* > *pListBBoxs, int nRange)
	{
		glm::vec3 v3CenterA = pBBoxA->m_v3Min + ((pBBoxA->m_v3Max - pBBoxA->m_v3Min) * 0.5f);

		int min = 0;
		int max = std::min((int)pListBBoxs->size(), nRange);

		float fMinDistance = FLT_MAX;
		int min_id = min;

		for (int i = min; i < max; i++)
		{
			BBox* pBBoxB = pListBBoxs->at(i)->m_pBBox;

			glm::vec3 v3CenterB = pBBoxB->m_v3Min + ((pBBoxB->m_v3Max - pBBoxB->m_v3Min) * 0.5f);

			float fDistance = glm::distance(v3CenterA, v3CenterB);

			if (fDistance < fMinDistance)
			{
				fMinDistance = fDistance;
				min_id = i;
			}

			if (fMinDistance < 0.001f) // the best neightboor bbox
			{
				return i;
			}
		}

		return min_id;
	}

	void Physics::SetStaticMesh(uint32_t nId, std::vector<glm::vec3>* listVertices, std::vector<glm::vec3>* listNormals, glm::mat4 matTransform)
	{
		StaticMesh *pTheStaticMesh = m_listStaticMeshs.at(nId);

		// vertices to triangles
		std::vector< Triangle* > listTriangles;
		for (uint64_t i = 0; i < listVertices->size(); i += 3)
		{
			glm::vec3 vA = listVertices->at(i + 0);
			glm::vec3 vB = listVertices->at(i + 1);
			glm::vec3 vC = listVertices->at(i + 2);

			glm::vec3 nA = listNormals->at(i + 0);
			glm::vec3 nB = listNormals->at(i + 1);
			glm::vec3 nC = listNormals->at(i + 2);

			// transform
			vA = glm::vec3(glm::vec4(vA, 1) * matTransform);
			vB = glm::vec3(glm::vec4(vB, 1) * matTransform);
			vC = glm::vec3(glm::vec4(vC, 1) * matTransform);

			nA = glm::normalize( glm::vec3(glm::vec4(nA, 0) * matTransform) );
			nB = glm::normalize( glm::vec3(glm::vec4(nB, 0) * matTransform) );
			nC = glm::normalize( glm::vec3(glm::vec4(nC, 0) * matTransform) );

			listTriangles.push_back( new Triangle(vA, vB, vC, nA, nB, nC) );
		}

		// sort
		std::sort(listTriangles.begin(), listTriangles.end(), SortTrianglesFunc);

		int range = 1000; //(int)listTriangles.size();
		std::vector< BVHNodeTriangle* > *pIN = nullptr;
		std::vector< BVHNodeTriangle* >* pOUT = new std::vector< BVHNodeTriangle* >();

		// triangles to bboxs
		while (listTriangles.size() > 0)
		{
			// node
			BVHNodeTriangle* pNode = new BVHNodeTriangle();
			pNode->m_pTriangle = listTriangles.at(0);
			pNode->m_pBBox = BBox::Create(pNode->m_pTriangle);
			listTriangles.erase(listTriangles.begin() + 0);

			// add
			pOUT->push_back(pNode);	
		}

		// copy
		pTheStaticMesh->m_listBVHNodeTriangles.insert(pTheStaticMesh->m_listBVHNodeTriangles.end(), pOUT->begin(), pOUT->end());
		// reset
		pIN = pOUT;
		pOUT = new std::vector< BVHNodeTriangle* >();

		// BBoxs to BBoxs
		do
		{
			while (pIN->size() > 0)
			{
				if (pIN->size() >= 2) // > 2
				{
					// node
					BVHNodeTriangle* pNode = new BVHNodeTriangle();
					pNode->m_pLeft = pIN->at(0);
					pIN->erase(pIN->begin() + 0);
					int i = SearchNeightboorBBox(pNode->m_pLeft->m_pBBox, pIN, range);
					pNode->m_pRight = pIN->at(i);
					pIN->erase(pIN->begin() + i);
					pNode->m_pBBox = BBox::Create(pNode->m_pLeft->m_pBBox, pNode->m_pRight->m_pBBox);

					// add
					pOUT->push_back(pNode);
				}
				else // 1
				{
					// node
					BVHNodeTriangle* pNode = new BVHNodeTriangle();
					pNode->m_pLeft = pIN->at(0);
					pNode->m_pBBox = BBox::Create(pNode->m_pLeft->m_pBBox);
					pIN->erase(pIN->begin() + 0);

					// add
					pOUT->push_back(pNode);
				}
			}

			// copy
			pTheStaticMesh->m_listBVHNodeTriangles.insert(pTheStaticMesh->m_listBVHNodeTriangles.end(), pOUT->begin(), pOUT->end());
			// reset
			pIN = pOUT;
			pOUT = new std::vector< BVHNodeTriangle* >();
		}
		while (pIN->size() > 1);

		// root
		BVHNodeTriangle* pRoot = new BVHNodeTriangle();
		pRoot->m_pLeft = pIN->at(0);
		pRoot->m_pBBox = BBox::Create(pRoot->m_pLeft->m_pBBox);
		pIN->erase(pIN->begin() + 0);

		// add
		pTheStaticMesh->m_listBVHNodeTriangles.insert(pTheStaticMesh->m_listBVHNodeTriangles.begin(), pRoot);
	}

	void Physics::SetDynamicMesh(uint32_t nId, std::vector<glm::vec3>* listVertices, std::vector<glm::vec3>* listNormals, float fDistance, float fRadius)
	{
		//DynamicMesh* pTheDynamicMesh = m_listDynamicMeshs.at(nId);
		//
		//// vertices to triangles
		//std::vector< Triangle* > listTriangles;
		//for (uint64_t i = 0; i < listVertices->size(); i += 3)
		//{
		//	glm::vec3 vA = listVertices->at(i + 0);
		//	glm::vec3 vB = listVertices->at(i + 1);
		//	glm::vec3 vC = listVertices->at(i + 2);
		//
		//	glm::vec3 N = glm::normalize(glm::cross(vB - vA, vC - vA));		
		//
		//	listTriangles.push_back(new Triangle(vA, vB, vC, N, N, N));
		//}

		;
	}

	void Physics::SetSphere(uint32_t nId, glm::vec3 v3Position, float fRadius)
	{
		Sphere* pTheSphere = m_listSpheres.at(nId);

		pTheSphere->m_v3Position = v3Position;
		pTheSphere->m_fRadius = fRadius;
	}

	void Physics::SetGravity(glm::vec3 vec3Gravity)
	{
	}

	glm::vec3 Physics::GetGravity()
	{
		return glm::vec3(0, 0, 0);
	}

	glm::mat4 Physics::GetTransform(uint32_t nId)
	{
		return glm::mat4(1);
	}

	void Physics::SetPosition(uint32_t nId, glm::vec3 vec3Position)
	{
	}

	glm::vec3 Physics::GetPosition(uint32_t nId)
	{
		return glm::vec3(0, 0, 0);
	}

	void Physics::SetEulerRotate(uint32_t nId, glm::vec3 vec3EulerRotate)
	{
	}

	glm::vec3 Physics::GetEulerRotate(uint32_t nId)
	{
		return glm::vec3(0, 0, 0);
	}

	void Physics::SetLinearVelocity(uint32_t nId, glm::vec3 vec3LinearVelocity)
	{
	}

	glm::vec3 Physics::GetLinearVelocity(uint32_t nId)
	{
		return glm::vec3(0, 0, 0);
	}

	void Physics::SetAngularVelocity(uint32_t nId, glm::vec3 vec3AngularVelocity)
	{
	}

	glm::vec3 Physics::GetAngularVelocity(uint32_t nId)
	{
		return glm::vec3(0, 0, 0);
	}

	void Physics::SetLinearAcceleration(uint32_t nId, glm::vec3 vec3LinearVelocity)
	{
	}

	glm::vec3 Physics::GetLinearAcceleration(uint32_t nId)
	{
		return glm::vec3(0, 0, 0);
	}

	void Physics::SetAngularAcceleration(uint32_t nId, glm::vec3 vec3AngularVelocity)
	{
	}

	glm::vec3 Physics::GetAngularAcceleration(uint32_t nId)
	{
		return glm::vec3(0, 0, 0);
	}

	void Physics::SetForce(uint32_t nId, glm::vec3 vec3LinearVelocity)
	{
	}

	glm::vec3 Physics::GetForce(uint32_t nId)
	{
		return glm::vec3(0, 0, 0);
	}

	void Physics::SetTorque(uint32_t nId, glm::vec3 vec3AngularVelocity)
	{
	}

	glm::vec3 Physics::GetTorque(uint32_t nId)
	{
		return glm::vec3(0, 0, 0);
	}

	void Physics::SetMass(uint32_t nId, float fMass)
	{
	}

	float Physics::GetMass(uint32_t nId)
	{
		return 0.0f;
	}

	void Physics::SetRestitution(uint32_t nId, float fRestitution)
	{
	}

	float Physics::GetRestitution(uint32_t nId)
	{
		return 0.0f;
	}

	void Physics::SetFriction(uint32_t nId, float fFriction)
	{
	}

	float Physics::GetFriction(uint32_t nId)
	{
		return 0.0f;
	}

	void Physics::SetLinearDamping(uint32_t nId, float fLinearDamping)
	{
	}

	float Physics::GetLinearDamping(uint32_t nId)
	{
		return 0.0f;
	}

	void Physics::SetAngularDamping(uint32_t nId, float fAngularDamping)
	{
	}

	float Physics::GetAngularDamping(uint32_t nId)
	{
		return 0.0f;
	}

	void Physics::Commit()
	{
	}

	void Physics::Update(float dt)
	{
	}

}