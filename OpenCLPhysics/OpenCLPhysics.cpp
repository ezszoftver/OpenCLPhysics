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
	structVector3 ToVector3(glm::vec3 v) 
	{
		structVector3 ret;
		ret.x = v.x;
		ret.y = v.y;
		ret.z = v.z;
		return ret;
	}

	glm::vec3 ToVector3(structVector3 v)
	{
		glm::vec3 ret(v.x, v.y, v.z);
		return ret;
	}

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
		m_v3Min = glm::vec3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
		m_v3Max = glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	}

	BBox::BBox(glm::vec3 v3Min, glm::vec3 v3Max) 
	{
		m_v3Min = v3Min;
		m_v3Max = v3Max;
	}

	BBox* BBox::Create(Triangle* pTriangle) 
	{
		BBox* pBBox = new BBox();

		std::vector<glm::vec3> vertices;
		vertices.push_back(pTriangle->m_v3PosA);
		vertices.push_back(pTriangle->m_v3PosB);
		vertices.push_back(pTriangle->m_v3PosC);

		for (uint64_t i = 0; i < vertices.size(); i++)
		{
			glm::vec3 v = vertices[i];
			pBBox->Add(v);
		}
		
		return pBBox;
	}

	BBox* BBox::Create(Triangle* pTriangle1, Triangle* pTriangle2) 
	{
		BBox* pBBox = new BBox();

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
			pBBox->Add(v);
		}

		return pBBox;
	}

	BBox* BBox::Create(BBox* pBBox) 
	{
		return new BBox(pBBox->m_v3Min, pBBox->m_v3Max);
	}

	BBox* BBox::Create(BBox* pBBox1, BBox* pBBox2)
	{
		BBox* pBBox = new BBox();

		std::vector<glm::vec3> vertices;
		vertices.push_back(pBBox1->m_v3Min);
		vertices.push_back(pBBox1->m_v3Max);
		vertices.push_back(pBBox2->m_v3Min);
		vertices.push_back(pBBox2->m_v3Max);

		for (uint64_t i = 0; i < vertices.size(); i++)
		{
			glm::vec3 v = vertices[i];
			pBBox->Add(v);
		}

		return pBBox;
	}

	void BBox::Add(structBBox bbox) 
	{
		m_v3Min.x = std::fmin(m_v3Min.x, bbox.v3MinX);
		m_v3Min.y = std::fmin(m_v3Min.y, bbox.v3MinY);
		m_v3Min.z = std::fmin(m_v3Min.z, bbox.v3MinZ);

		m_v3Max.x = std::fmax(m_v3Max.x, bbox.v3MaxX);
		m_v3Max.y = std::fmax(m_v3Max.y, bbox.v3MaxY);
		m_v3Max.z = std::fmax(m_v3Max.z, bbox.v3MaxZ);
	}

	void BBox::Add(glm::vec3 v) 
	{
		m_v3Min.x = std::fmin(m_v3Min.x, v.x);
		m_v3Min.y = std::fmin(m_v3Min.y, v.y);
		m_v3Min.z = std::fmin(m_v3Min.z, v.z);
		
		m_v3Max.x = std::fmax(m_v3Max.x, v.x);
		m_v3Max.y = std::fmax(m_v3Max.y, v.y);
		m_v3Max.z = std::fmax(m_v3Max.z, v.z);
	}

	structBBox BBox::GetStructBBox() 
	{
		structBBox ret;

		ret.v3MinX = m_v3Min.x;
		ret.v3MinY = m_v3Min.y;
		ret.v3MinZ = m_v3Min.z;

		ret.v3MaxX = m_v3Max.x;
		ret.v3MaxY = m_v3Max.y;
		ret.v3MaxZ = m_v3Max.z;

		return ret;
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
		m_pListBVHNodeTriangles = nullptr;
	}

	TriMesh::~TriMesh()
	{
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

	bool Physics::CreateDevice(std::string strDeviceName, int32_t nTriMeshsCount)
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
					m_kernelUpdateBVHObjects = clCreateKernel(m_program, "UpdateBVHObjects", &status);
					if (!m_kernelUpdateBVHObjects || status != CL_SUCCESS) { return false; }
					m_kernelIntegrate = clCreateKernel(m_program, "Integrate", &status);
					if (!m_kernelIntegrate || status != CL_SUCCESS) { return false; }

					// max trimeshs
					TRIMESH_COUNT = nTriMeshsCount;
					m_listFreeIds.clear();
					m_listTriMeshs.clear();
					m_listRigidBodies.clear();
					for (int i = 0; i < TRIMESH_COUNT; i++)
					{
						m_listFreeIds.push_back(i);

						m_listTriMeshs.push_back(new TriMesh());

						structRigidBody newRigidBody;
						newRigidBody.m_nRigidBodyId = i;
						m_listRigidBodies.push_back(newRigidBody);
					}

					return true;
				}
			}
		}

		return false;
	}

	void Physics::CloseDevice()
	{
	}

	int32_t Physics::CreateTriMesh(std::vector<glm::vec3>* pListVertices)
	{
		if (0 == m_listFreeIds.size()) 
		{
			return -1;
		}

		// get free id
		int32_t nId = m_listFreeIds.at(0);
		m_listFreeIds.erase(m_listFreeIds.begin() + 0);

		// new trimesh
		int32_t nTriMeshId = nId;

		// new rigidbody
		int32_t nRigidBodyId = nId;

		// update
		structRigidBody newRigidBody;
		newRigidBody.m_nRigidBodyId = nRigidBodyId;
		m_listRigidBodies.at(nRigidBodyId) = newRigidBody;

		delete m_listTriMeshs.at(nTriMeshId);
		m_listTriMeshs.at(nTriMeshId) = new TriMesh();

		m_listTriMeshs.at(nTriMeshId)->m_nRigidBodyId = nRigidBodyId;
		m_listRigidBodies.at(nRigidBodyId).m_nTriMeshId = nTriMeshId;

		int32_t nRet = TRIMESH_START + nTriMeshId;
		SetTriMesh(nRet, pListVertices);
		SetEnabled(nId, true);

		if (false == Commit())
		{
			return -1;
		}

		return nRet;
	}

	int32_t Physics::CreateFromId(int32_t nFromId) 
	{
		if (0 == m_listFreeIds.size())
		{
			return -1;
		}

		// get free id
		int32_t nId = m_listFreeIds.at(0);
		m_listFreeIds.erase(m_listFreeIds.begin() + 0);

		// new trimesh
		int32_t nTriMeshId = nId;

		// new rigidbody
		int32_t nRigidBodyId = nId;

		// update
		structRigidBody newRigidBody;
		newRigidBody.m_nRigidBodyId = nRigidBodyId;
		m_listRigidBodies.at(nRigidBodyId) = newRigidBody;

		delete m_listTriMeshs.at(nTriMeshId);
		m_listTriMeshs.at(nTriMeshId) = new TriMesh();

		m_listTriMeshs.at(nTriMeshId)->m_nRigidBodyId = nRigidBodyId;
		m_listRigidBodies.at(nRigidBodyId).m_nTriMeshId = nTriMeshId;
		SetEnabled(nId, true);

		int32_t nNewId = TRIMESH_START + nTriMeshId;

		// copy fromId to newId
		if (nFromId >= TRIMESH_START && nFromId < (TRIMESH_START + TRIMESH_COUNT))
		{
			m_listTriMeshs.at(nTriMeshId)->m_pListBVHNodeTriangles = m_listTriMeshs.at(nFromId - TRIMESH_START)->m_pListBVHNodeTriangles;
		}
		else 
		{
			return -1;
		}
		
		if (false == Commit()) 
		{
			return -1;
		}

		return nNewId;
	}

	bool Physics::DeleteTriMesh(int32_t nId) 
	{
		if (nId == -1) 
		{
			return false;
		}

		SetEnabled(nId, false);
		m_listFreeIds.insert(m_listFreeIds.begin() + 0, nId);

		return true;
	}

	void Physics::SetEnabled(int32_t nId, bool bValue) 
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_nIsEnabled = (true == bValue) ? 1 : 0;
		}
	}

	bool Physics::IsEnabled(int32_t nId) 
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			bool ret = (m_listRigidBodies[nRigidBodyId].m_nIsEnabled == 0) ? false : true;
			return ret;
		}

		return 0;
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

	void Physics::SetTriMesh(int32_t nId, std::vector<glm::vec3>* pListVertices)
	{
		TriMesh *pTheTriMesh = m_listTriMeshs.at(nId);
		pTheTriMesh->m_pListBVHNodeTriangles = new std::vector< BVHNodeTriangle* >();
		structRigidBody theRigidBody = m_listRigidBodies.at( pTheTriMesh->m_nRigidBodyId );

		// calc bbox (local min/max)
		BBox* pBBox = new BBox();
		for (uint64_t i = 0; i < pListVertices->size(); i++) 
		{
			glm::vec3 v = pListVertices->at(i);
			pBBox->Add(v);
		}
		theRigidBody.m_BBox = pBBox->GetStructBBox();
		delete pBBox;

		m_listRigidBodies[pTheTriMesh->m_nRigidBodyId] = theRigidBody;

		// vertices to triangles
		std::vector< Triangle* > listTriangles;
		for (uint64_t i = 0; i < pListVertices->size(); i += 3)
		{
			glm::vec3 vA = pListVertices->at(i + 0);
			glm::vec3 vB = pListVertices->at(i + 1);
			glm::vec3 vC = pListVertices->at(i + 2);

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
		pTheTriMesh->m_pListBVHNodeTriangles->insert(pTheTriMesh->m_pListBVHNodeTriangles->end(), pOUT->begin(), pOUT->end());
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
			pTheTriMesh->m_pListBVHNodeTriangles->insert(pTheTriMesh->m_pListBVHNodeTriangles->end(), pOUT->begin(), pOUT->end());
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
		pTheTriMesh->m_pListBVHNodeTriangles->insert(pTheTriMesh->m_pListBVHNodeTriangles->begin(), pRoot);
	}

	void Physics::SetGravity(glm::vec3 vec3Gravity)
	{
		m_v3Gravity = ToVector3(vec3Gravity);
	}

	glm::vec3 Physics::GetGravity()
	{
		return ToVector3(m_v3Gravity);
	}

	glm::mat4 Physics::GetTransform(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;

			glm::vec3 v3Rotate = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3Rotate);
			glm::vec3 v3Position = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3Position);

			return (glm::translate(glm::mat4(1.0f), v3Position) * glm::eulerAngleXYZ(v3Rotate.x, v3Rotate.y, v3Rotate.z));
		}

		return glm::mat4(1.0f);
	}

	void Physics::SetPosition(int32_t nId, glm::vec3 v3Position)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3Position = ToVector3(v3Position);
		}
	}

	glm::vec3 Physics::GetPosition(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3Position); //glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3PositionX, m_listRigidBodies[nRigidBodyId].m_v3PositionY, m_listRigidBodies[nRigidBodyId].m_v3PositionZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetEulerRotate(int32_t nId, glm::vec3 v3EulerRotate)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3Rotate = ToVector3(v3EulerRotate);
		}
	}

	glm::vec3 Physics::GetEulerRotate(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3Rotate); //glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3RotateX, m_listRigidBodies[nRigidBodyId].m_v3RotateY, m_listRigidBodies[nRigidBodyId].m_v3RotateZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetLinearVelocity(int32_t nId, glm::vec3 v3LinearVelocity)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3LinearVelocity = ToVector3(v3LinearVelocity);
		}
	}

	glm::vec3 Physics::GetLinearVelocity(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3LinearVelocity); //glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityX, m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityY, m_listRigidBodies[nRigidBodyId].m_v3LinearVelocityZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetAngularVelocity(int32_t nId, glm::vec3 v3AngularVelocity)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3AngularVelocity = ToVector3(v3AngularVelocity);
		}
	}

	glm::vec3 Physics::GetAngularVelocity(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3AngularVelocity); //glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityX, m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityY, m_listRigidBodies[nRigidBodyId].m_v3AngularVelocityZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetLinearAcceleration(int32_t nId, glm::vec3 v3LinearAcceleration)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3LinearAcceleration = ToVector3(v3LinearAcceleration);
		}
	}

	glm::vec3 Physics::GetLinearAcceleration(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3LinearAcceleration); //glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationX, m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationY, m_listRigidBodies[nRigidBodyId].m_v3LinearAccelerationZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetAngularAcceleration(int32_t nId, glm::vec3 v3AngularAcceleration)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3AngularAcceleration = ToVector3(v3AngularAcceleration);
		}
	}

	glm::vec3 Physics::GetAngularAcceleration(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3AngularAcceleration); //glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationX, m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationY, m_listRigidBodies[nRigidBodyId].m_v3AngularAccelerationZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetForce(int32_t nId, glm::vec3 v3Force)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3Force = ToVector3(v3Force);
		}
	}

	glm::vec3 Physics::GetForce(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3Force); //glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3ForceX, m_listRigidBodies[nRigidBodyId].m_v3ForceY, m_listRigidBodies[nRigidBodyId].m_v3ForceZ);
			return ret;
		}

		return glm::vec3(0, 0, 0);
	}

	void Physics::SetTorque(int32_t nId, glm::vec3 v3Torque)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			m_listRigidBodies[nRigidBodyId].m_v3Torque = ToVector3(v3Torque);
		}
	}

	glm::vec3 Physics::GetTorque(int32_t nId)
	{
		if (nId >= TRIMESH_START && nId < (TRIMESH_START + TRIMESH_COUNT))
		{
			int32_t nRigidBodyId = m_listTriMeshs.at(nId - TRIMESH_START)->m_nRigidBodyId;
			glm::vec3 ret = ToVector3(m_listRigidBodies[nRigidBodyId].m_v3Torque); //glm::vec3(m_listRigidBodies[nRigidBodyId].m_v3TorqueX, m_listRigidBodies[nRigidBodyId].m_v3TorqueY, m_listRigidBodies[nRigidBodyId].m_v3TorqueZ);
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
		if (m_listRigidBodies.size() < 1)
		{
			return false;
		}

		cl_int err;

		// Release if exists
		if (0 != m_clmem_inoutRigidBodies)
		{
			clReleaseMemObject(m_clmem_inoutRigidBodies);
			m_clmem_inoutRigidBodies = 0;
		}
		if (0 != m_clmem_inoutBVHObjects)
		{
			clReleaseMemObject(m_clmem_inoutBVHObjects);
			m_clmem_inoutBVHObjects = 0;
		}
		ReleaseBVHObjects();

		// create rigidBodies buffer
		// -> ealpsed
		m_clmem_inoutRigidBodies = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(structRigidBody) * m_listRigidBodies.size(), NULL, NULL);
		if (!m_clmem_inoutRigidBodies) { return false; }
		// -> copy
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), m_listRigidBodies.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// create BVHObjects buffer
		CreateBVHObjects();
		m_clmem_inoutBVHObjects = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(structBVHObject) * m_listBVHObjects.size(), NULL, NULL);
		if (!m_clmem_inoutBVHObjects) { return false; }
		// -> copy
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutBVHObjects, CL_TRUE, 0, sizeof(structBVHObject) * m_listBVHObjects.size(), m_listBVHObjects.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		return true;
	}

	bool SortRigidBodiesFunc_BVH(structRigidBody &a, structRigidBody &b)
	{
		if (fabs(a.m_v3Position.x - b.m_v3Position.x) > 0.0001f) { return (a.m_v3Position.x < b.m_v3Position.x); }
		if (fabs(a.m_v3Position.y - b.m_v3Position.y) > 0.0001f) { return (a.m_v3Position.y < b.m_v3Position.y); }
		return (a.m_v3Position.z < b.m_v3Position.z);
	}

	bool SortRigidBodiesFunc_Inc(structRigidBody& a, structRigidBody& b)
	{
		return (a.m_nRigidBodyId < b.m_nRigidBodyId);
	}

	bool Physics::Update(float dt, uint16_t nSteps) 
	{
		if (nSteps < 1)
		{
			nSteps = 1;
		}

		float fStepDt = dt / (float)nSteps;

		for (uint16_t i = 0; i < nSteps; i++)
		{
			if (false == StepUpdate(fStepDt)) 
			{
				return false;
			}
		}

		return true;
	}

	bool Physics::StepUpdate(float dt)
	{
		if (m_listRigidBodies.size() < 1)
		{
			return false;
		}

		cl_int err = 0;

		// write to GPU the current dta
		err |= clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(m_listRigidBodies[0]), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }
		// 1. INTEGRATE with GPU
		if (false == Integrate(dt)) { return false; }
		// read to RAM for CPU
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(m_listRigidBodies[0]), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// 2. SORT to BVH
		std::sort(m_listRigidBodies.begin(), m_listRigidBodies.end(), SortRigidBodiesFunc_BVH);
		// write to GPU
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), m_listRigidBodies.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// 3. UPDATE BVHObjects with GPU
		if (false == UpdateBVHObjects()) { return false; }

		// CollisionDetection
		;
		// CollisionResponse
		;

		// read to RAM for CPU
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(m_listRigidBodies[0]), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// SORT to Original
		std::sort(m_listRigidBodies.begin(), m_listRigidBodies.end(), SortRigidBodiesFunc_Inc);

		return true;
	}

	uint32_t Physics::NumRigidBodies() 
	{
		int32_t nCount = TRIMESH_COUNT - (int32_t)m_listFreeIds.size();
		return nCount;
	}

	uint32_t Physics::MaxRigidBodies() 
	{
		return TRIMESH_COUNT;
	}

	void Physics::CreateBVHObjects()
	{
		// 1/2 - CREATE LEVELS
		float epsilon = 0.0001f;

		m_BVHObjectsLevels.clear();
		uint32_t numRigidBodies = (uint32_t)m_listRigidBodies.size();

		if (1 == numRigidBodies)
		{
			// create current level
			std::vector< structBVHObject >* pCurrentLevel = new std::vector< structBVHObject >;

			structBVHObject element;

			structRigidBody theRigidBody = m_listRigidBodies.at(0);
			element.m_BBox = theRigidBody.m_BBox;

			element.m_nLeft = -1;
			element.m_nRight = -1;
			pCurrentLevel->push_back(element);

			// add to tree, the current level
			m_BVHObjectsLevels.push_back(pCurrentLevel);
		}
		
		for (uint32_t nLevel = 0; numRigidBodies > 1; nLevel++)
		{
			// create current level
			std::vector< structBVHObject >* pCurrentLevel = new std::vector< structBVHObject >;

			// fill current level
			uint32_t j = 0;
			for (uint32_t i = 0; i < numRigidBodies; i++)
			{
				if (nLevel == 0) // is Leaf? fill RigidBody
				{
					structBVHObject element;

					structRigidBody theRigidBody = m_listRigidBodies.at(i);
					element.m_BBox = theRigidBody.m_BBox;

					element.m_nLeft = -1;
					element.m_nRight = -1;
					pCurrentLevel->push_back(element);
				}
				else // else, n. level
				{
					std::vector< structBVHObject > *pElapsedLevel = m_BVHObjectsLevels.at(m_BVHObjectsLevels.size() - 1);

					structBVHObject element;

					BBox bbox;
					// left
					if (j < pElapsedLevel->size())
					{
						element.m_nLeft = j;
						bbox.Add( pElapsedLevel->at(j).m_BBox );
					}
					else 
					{
						element.m_nLeft = -1;
					}
					j++;
					// right
					if (j < pElapsedLevel->size())
					{
						element.m_nRight = j;
						bbox.Add(pElapsedLevel->at(j).m_BBox);
					}
					else 
					{
						element.m_nRight = -1;
					}
					j++;
					element.m_BBox = bbox.GetStructBBox();

					pCurrentLevel->push_back(element);
				}
			}

			// add to tree, the current level
			m_BVHObjectsLevels.push_back(pCurrentLevel);

			// div 2
			numRigidBodies = ((numRigidBodies % 2) == 0) ? (numRigidBodies / 2) : ((numRigidBodies / 2) + 1);
		}

		// last level
		std::vector< structBVHObject > *pLastLevel = new std::vector< structBVHObject >;
		std::vector< structBVHObject >* pElapsedLevel = m_BVHObjectsLevels.at(m_BVHObjectsLevels.size() - 1);
		structBVHObject element;
		BBox bbox;
		// left
		if (0 < pElapsedLevel->size())
		{
			element.m_nLeft = 0;
			bbox.Add(pElapsedLevel->at(0).m_BBox);
		}
		else
		{
			element.m_nLeft = -1;
		}
		// right
		if (1 < pElapsedLevel->size())
		{
			element.m_nRight = 1;
			bbox.Add(pElapsedLevel->at(1).m_BBox);
		}
		else
		{
			element.m_nRight = -1;
		}
		element.m_BBox = bbox.GetStructBBox();

		pLastLevel->push_back(element);

		// add to tree, the last level
		m_BVHObjectsLevels.push_back(pLastLevel);

		// 2/2 - LEVELS TO ONE LIST
		m_listBVHObjects.clear();
		
		// ROOT element
		structBVHObject root;
		m_listBVHObjects.push_back(root);
		uint32_t nElapsedOffset;
		uint32_t nOffset = 1;

		for (uint32_t i = 0; i < m_BVHObjectsLevels.size(); i++)
		{
			nElapsedOffset = (uint32_t)m_listBVHObjects.size();

			std::vector< structBVHObject >* pLevel = m_BVHObjectsLevels.at(i);

			if (i == (m_BVHObjectsLevels.size() - 1)) // root
			{
				structBVHObject element = pLevel->at(0);
				if (element.m_nLeft > -1) { element.m_nLeft += nOffset; }
				if (element.m_nRight > -1) { element.m_nRight += nOffset; }
				m_listBVHObjects[0] = element;
			}
			else
			{
				for (uint32_t j = 0; j < pLevel->size(); j++) // n. level
				{
					structBVHObject element = pLevel->at(j);
					if (element.m_nLeft > -1) { element.m_nLeft += nOffset; }
					if (element.m_nRight > -1) { element.m_nRight += nOffset; }
					m_listBVHObjects.push_back(element);
				}

				nOffset = nElapsedOffset;
			}
		}
	}

	void Physics::ReleaseBVHObjects() 
	{
		for (uint32_t i = 0; i < m_BVHObjectsLevels.size(); i++) 
		{
			delete m_BVHObjectsLevels.at(i);
		}
		m_BVHObjectsLevels.clear();

		m_listBVHObjects.clear();
	}

	bool Physics::UpdateBVHObjects()
	{
		// update leafs
		int32_t nOffset = 1;
		int32_t nCount = 0;
		size_t nLocal = 1;
		for (uint32_t i = 0; i < m_BVHObjectsLevels.size(); i++)
		{
			std::vector< structBVHObject > *pCurrentLevel = m_BVHObjectsLevels.at(i);

			if (i == (m_BVHObjectsLevels.size() - 1))
			{
				nOffset = 0;
			}

			nCount = (int32_t)pCurrentLevel->size();
			size_t nGlobal = nCount;

			cl_int err = 0;

			err |= clSetKernelArg(m_kernelUpdateBVHObjects, 0, sizeof(cl_mem), &m_clmem_inoutBVHObjects);
			err |= clSetKernelArg(m_kernelUpdateBVHObjects, 1, sizeof(cl_mem), &m_clmem_inoutRigidBodies);
			err |= clSetKernelArg(m_kernelUpdateBVHObjects, 2, sizeof(int32_t), &nOffset);
			err |= clSetKernelArg(m_kernelUpdateBVHObjects, 3, sizeof(int32_t), &nCount);
			err |= clEnqueueNDRangeKernel(m_command_queue, m_kernelUpdateBVHObjects, 1, NULL, &nGlobal, &nLocal, 0, NULL, NULL);
			clFinish(m_command_queue);

			// debug check
			/*std::vector<structBVHObject> results;
			results.resize(m_listBVHObjects.size());
			err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutBVHObjects, CL_TRUE, 0, sizeof(structBVHObject) * m_listBVHObjects.size(), &(results[0]), 0, NULL, NULL);*/
			if (err != CL_SUCCESS) 
			{ 
				return false;
			}

			nOffset += nCount;
		}
		
		return true;
	}

	bool Physics::Integrate(float dt)
	{
		cl_int err = 0;
		
		int32_t nCount = 0;
		size_t nLocal = 1;
		nCount = (int32_t)m_listRigidBodies.size();
		size_t nGlobal = nCount;
		
		// calc
		err |= clSetKernelArg(m_kernelIntegrate, 0, sizeof(cl_mem), &m_clmem_inoutRigidBodies);
		err |= clSetKernelArg(m_kernelIntegrate, 1, sizeof(int32_t), &nCount);
		err |= clSetKernelArg(m_kernelIntegrate, 2, sizeof(float), &dt);
		err |= clSetKernelArg(m_kernelIntegrate, 3, sizeof(structVector3), &m_v3Gravity);
		err |= clEnqueueNDRangeKernel(m_command_queue, m_kernelIntegrate, 1, NULL, &nGlobal, &nLocal, 0, NULL, NULL);
		clFinish(m_command_queue);
		
		/*std::vector<structRigidBody> results;
		results.resize(m_listRigidBodies.size());
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(results[0]), 0, NULL, NULL);*/
		if (err != CL_SUCCESS)
		{
			return false;
		}

		return true;
	}
}