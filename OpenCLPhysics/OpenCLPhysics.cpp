#include "OpenCLPhysics.h"
#include "Script.h"

// sort algorithm example
#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"

namespace OpenCLPhysics 
{
	Triangle::Triangle()
	{
		m_v3PosA = glm::vec3(0,0,0);
		m_v3PosB = glm::vec3(0,0,0);
		m_v3PosC = glm::vec3(0,0,0);

		m_v3Normal = glm::vec3(1, 0, 0);
	}

	Triangle::Triangle(glm::vec3 v3PosA, glm::vec3 v3PosB, glm::vec3 v3PosC, glm::vec3 v3Normal)
	{
		m_v3PosA = v3PosA;
		m_v3PosB = v3PosB;
		m_v3PosC = v3PosC;

		m_v3Normal = v3Normal;
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

	Hit::Hit() 
	{
		m_nBodyAId = -1;
		m_nBodyBId = -1;
	}

	Hit::Hit(int32_t nBodyAId, int32_t nBodyBId, glm::vec3 v3WorldPosition, glm::vec3 v3Normal)
	{
		m_nBodyAId = nBodyAId;
		m_nBodyBId = nBodyBId;
	}

	Hit::~Hit() 
	{
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

	TriMesh::TriMesh()
	{
		m_nRigidBodyId = -1;
		m_nTop = 0;
	}

	TriMesh::~TriMesh()
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

					// create program
					const char* strText = Script::GetText();
					m_program = clCreateProgramWithSource(m_context, 1, (const char**)&strText, NULL, &status);
					if (status != CL_SUCCESS) { return false; }

					// build program
					status = clBuildProgram(m_program, 0, NULL, NULL, NULL, NULL);
					if (status != CL_SUCCESS)
					{
						size_t len;
						char buffer[2048];

						clGetProgramBuildInfo(m_program, deviceId, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
						printf("%s\n", buffer);
						return false;
					}

					// create kernels
					m_kernelRefitTree = clCreateKernel(m_program, "RefitTree", &status);
					if (!m_kernelRefitTree || status != CL_SUCCESS) { return false; }

					return true;
				}
			}
		}

		return false;
	}

	void Physics::CloseDevice()
	{
	}

	int32_t Physics::GenTriMesh()
	{		
		int32_t nTriMeshId = (int32_t)m_listTriMeshs.size();
		if (nTriMeshId >= TRIMESH_COUNT)
		{
			return -1;
		}

		// new trimesh
		m_listTriMeshs.push_back( new TriMesh() );

		// new rigidbody
		int32_t nRigidBodyId = (int32_t)m_listRigidBodies.size();
		RigidBody newRigidBody;
		m_listRigidBodies.push_back(newRigidBody);

		// update
		m_listTriMeshs.at(nTriMeshId)->m_nRigidBodyId = nRigidBodyId;
		m_listRigidBodies.at(nRigidBodyId).m_nTriMeshId = nTriMeshId;

		return (TRIMESH_START + nTriMeshId);
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

	void Physics::SetTriMesh(int32_t nId, std::vector<glm::vec3>* listVertices)
	{
		TriMesh *pTheTriMesh = m_listTriMeshs.at(nId);
		RigidBody theRigidBody = m_listRigidBodies.at( pTheTriMesh->m_nRigidBodyId );

		// calc radius
		theRigidBody.m_fRadius = 0.0f;
		for (uint64_t i = 0; i < listVertices->size(); i++) 
		{
			float fCurrRadius = glm::length(listVertices->at(i));
			if (theRigidBody.m_fRadius < fCurrRadius) 
			{
				theRigidBody.m_fRadius = fCurrRadius;
			}
		}

		m_listRigidBodies[pTheTriMesh->m_nRigidBodyId] = theRigidBody;

		// vertices to triangles
		std::vector< Triangle* > listTriangles;
		for (uint64_t i = 0; i < listVertices->size(); i += 3)
		{
			glm::vec3 vA = listVertices->at(i + 0);
			glm::vec3 vB = listVertices->at(i + 1);
			glm::vec3 vC = listVertices->at(i + 2);

			glm::vec3 vN = glm::normalize(glm::cross(vB - vA, vC - vA));

			listTriangles.push_back( new Triangle(vA, vB, vC, vN) );
		}

		// sort
		std::sort(listTriangles.begin(), listTriangles.end(), SortTrianglesFunc);

		int range = (int)listTriangles.size();
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
		pTheTriMesh->m_listBVHNodeTriangles.insert(pTheTriMesh->m_listBVHNodeTriangles.end(), pOUT->begin(), pOUT->end());
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
			pTheTriMesh->m_listBVHNodeTriangles.insert(pTheTriMesh->m_listBVHNodeTriangles.end(), pOUT->begin(), pOUT->end());
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
		pTheTriMesh->m_listBVHNodeTriangles.insert(pTheTriMesh->m_listBVHNodeTriangles.begin(), pRoot);
	}

	void Physics::SetGravity(glm::vec3 vec3Gravity)
	{
	}

	glm::vec3 Physics::GetGravity()
	{
		return glm::vec3(0, 0, 0);
	}

	glm::mat4 Physics::GetTransform(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;

			glm::vec3 v3Rotate = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3RotateX, m_listRigidBodies[nRigidBodyId].m_v3RotateY, m_listRigidBodies[nRigidBodyId].m_v3RotateZ);
			glm::vec3 v3Position = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3PositionX, m_listRigidBodies[nRigidBodyId].m_v3PositionY, m_listRigidBodies[nRigidBodyId].m_v3PositionZ);

			return (glm::translate(glm::mat4(1.0f), v3Position) * glm::eulerAngleXYZ(v3Rotate.x, v3Rotate.y, v3Rotate.z));			
		}

		return glm::mat4(1.0f);
	}

	void Physics::SetPosition(int32_t nId, glm::vec3 v3Position)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3PositionX = v3Position.x;
			m_listRigidBodies[nRigidBodyId].m_v3PositionY = v3Position.y;
			m_listRigidBodies[nRigidBodyId].m_v3PositionZ = v3Position.z;
		}
	}

	glm::vec3 Physics::GetPosition(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3PositionX, m_listRigidBodies[nRigidBodyId].m_v3PositionY, m_listRigidBodies[nRigidBodyId].m_v3PositionZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetEulerRotate(int32_t nId, glm::vec3 v3EulerRotate)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3RotateX = v3EulerRotate.x;
			m_listRigidBodies[nRigidBodyId].m_v3RotateY = v3EulerRotate.y;
			m_listRigidBodies[nRigidBodyId].m_v3RotateZ = v3EulerRotate.z;
		}
	}

	glm::vec3 Physics::GetEulerRotate(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3RotateX, m_listRigidBodies[nRigidBodyId].m_v3RotateY, m_listRigidBodies[nRigidBodyId].m_v3RotateZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetLinearVelocity(int32_t nId, glm::vec3 v3LinearVelocity)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityX = v3LinearVelocity.x;
			m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityY = v3LinearVelocity.y;
			m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityZ = v3LinearVelocity.z;
		}
	}

	glm::vec3 Physics::GetLinearVelocity(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityX, m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityY, m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetAngularVelocity(int32_t nId, glm::vec3 v3AngularVelocity)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityX = v3AngularVelocity.x;
			m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityY = v3AngularVelocity.y;
			m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityZ = v3AngularVelocity.z;
		}
	}

	glm::vec3 Physics::GetAngularVelocity(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityX, m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityY, m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetLinearAcceleration(int32_t nId, glm::vec3 v3LinearAcceleration)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationX = v3LinearAcceleration.x;
			m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationY = v3LinearAcceleration.y;
			m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationZ = v3LinearAcceleration.z;
		}
	}

	glm::vec3 Physics::GetLinearAcceleration(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationX, m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationY, m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetAngularAcceleration(int32_t nId, glm::vec3 v3AngularAcceleration)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationX = v3AngularAcceleration.x;
			m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationY = v3AngularAcceleration.y;
			m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationZ = v3AngularAcceleration.z;
		}
	}

	glm::vec3 Physics::GetAngularAcceleration(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationX, m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationY, m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetForce(int32_t nId, glm::vec3 v3Force)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3ForceX = v3Force.x;
			m_listRigidBodies[nRigidBodyId].m_v3ForceY = v3Force.y;
			m_listRigidBodies[nRigidBodyId].m_v3ForceZ = v3Force.z;
		}
	}

	glm::vec3 Physics::GetForce(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3ForceX, m_listRigidBodies[nRigidBodyId].m_v3ForceY, m_listRigidBodies[nRigidBodyId].m_v3ForceZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetTorque(int32_t nId, glm::vec3 v3Torque)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3TorqueX = v3Torque.x;
			m_listRigidBodies[nRigidBodyId].m_v3TorqueY = v3Torque.y;
			m_listRigidBodies[nRigidBodyId].m_v3TorqueZ = v3Torque.z;
		}
	}

	glm::vec3 Physics::GetTorque(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3TorqueX, m_listRigidBodies[nRigidBodyId].m_v3TorqueY, m_listRigidBodies[nRigidBodyId].m_v3TorqueZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetMass(int32_t nId, float fMass)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_fMass = fMass;
		}
	}

	float Physics::GetMass(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			return m_listRigidBodies[nRigidBodyId].m_fMass;
		}

		return 0.0f;
	}

	void Physics::SetRestitution(int32_t nId, float fRestitution)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_fRestitution = fRestitution;
		}
	}

	float Physics::GetRestitution(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			return m_listRigidBodies[nRigidBodyId].m_fRestitution;
		}

		return 0.0f;
	}

	void Physics::SetFriction(int32_t nId, float fFriction)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_fFriction = fFriction;
		}
	}

	float Physics::GetFriction(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			return m_listRigidBodies[nRigidBodyId].m_fFriction;
		}

		return 0.0f;
	}

	void Physics::SetLinearDamping(int32_t nId, float fLinearDamping)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_fLinearDamping = fLinearDamping;
		}
	}

	float Physics::GetLinearDamping(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			return m_listRigidBodies[nRigidBodyId].m_fLinearDamping;
		}

		return 0.0f;
	}

	void Physics::SetAngularDamping(int32_t nId, float fAngularDamping)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_fAngularDamping = fAngularDamping;
		}
	}

	float Physics::GetAngularDamping(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			return m_listRigidBodies[nRigidBodyId].m_fAngularDamping;
		}

		return 0.0f;
	}

	bool Physics::Commit()
	{
		cl_int err;

		// create rigidBodies buffer
		m_clmem_RigidBodies = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(RigidBody) * m_listRigidBodies.size(), NULL, NULL);
		if (!m_clmem_RigidBodies) { return false; }
		// -> copy
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_RigidBodies, CL_TRUE, 0, sizeof(RigidBody) * m_listRigidBodies.size(), m_listRigidBodies.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		;

		return true;
	}

	bool SortRigidBodiesFunc(RigidBody &a, RigidBody &b)
	{
		if (fabs(a.m_v3PositionX - b.m_v3PositionX) > 0.0001f) { return (a.m_v3PositionX < b.m_v3PositionX); }
		if (fabs(a.m_v3PositionY - b.m_v3PositionY) > 0.0001f) { return (a.m_v3PositionY < b.m_v3PositionY); }
		return (a.m_v3PositionZ < b.m_v3PositionZ);
	}

	void Physics::Update(float dt)
	{
		// sort
		std::sort(m_listRigidBodies.begin(), m_listRigidBodies.end(), SortRigidBodiesFunc);
		OpenCL_RefitTree();

		// CollisionDetection
		;
		// CollisionResponse
		;
	}

	void Physics::OpenCL_RefitTree()
	{
		//m_listRigidBodies;
	}
}