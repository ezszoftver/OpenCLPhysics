#include "OpenCLPhysics.h" 

#include <execution>

// sort algorithm example
#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#define GJK_MAX_NUM_ITERATIONS 64
#define EPA_TOLERANCE 0.0001f
#define EPA_MAX_NUM_FACES 64
#define EPA_MAX_NUM_LOOSE_EDGES 32
#define EPA_MAX_NUM_ITERATIONS 64

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

	structTriangle Triangle::GetStructTriangle()
	{
		structTriangle ret;
		ret.m_v3PosA = ToVector3(m_v3PosA);
		ret.m_v3PosB = ToVector3(m_v3PosB);
		ret.m_v3PosC = ToVector3(m_v3PosC);
		ret.m_v3Normal = ToVector3(m_v3Normal);
		return ret;
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
		
		pBBox->m_v3Min -= glm::vec3(0.001f, 0.001f, 0.001f);
		pBBox->m_v3Max += glm::vec3(0.001f, 0.001f, 0.001f);

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

		pBBox->m_v3Min -= glm::vec3(0.001f, 0.001f, 0.001f);
		pBBox->m_v3Max += glm::vec3(0.001f, 0.001f, 0.001f);

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

		pBBox->m_v3Min -= glm::vec3(0.001f, 0.001f, 0.001f);
		pBBox->m_v3Max += glm::vec3(0.001f, 0.001f, 0.001f);

		return pBBox;
	}

	void BBox::Add(structBBox bbox) 
	{
		m_v3Min.x = std::fmin(m_v3Min.x, bbox.v3Min.x);
		m_v3Min.y = std::fmin(m_v3Min.y, bbox.v3Min.y);
		m_v3Min.z = std::fmin(m_v3Min.z, bbox.v3Min.z);

		m_v3Max.x = std::fmax(m_v3Max.x, bbox.v3Max.x);
		m_v3Max.y = std::fmax(m_v3Max.y, bbox.v3Max.y);
		m_v3Max.z = std::fmax(m_v3Max.z, bbox.v3Max.z);
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

		ret.v3Min.x = m_v3Min.x;
		ret.v3Min.y = m_v3Min.y;
		ret.v3Min.z = m_v3Min.z;

		ret.v3Max.x = m_v3Max.x;
		ret.v3Max.y = m_v3Max.y;
		ret.v3Max.z = m_v3Max.z;

		return ret;
	}

	BVHNodeTriangle::BVHNodeTriangle(int32_t nId)
	{
		m_nId = nId;

		m_pLeft = nullptr;
		m_pRight = nullptr;
		
		m_pTriangle = nullptr;
		m_pBBox = nullptr;
	}

	bool BVHNodeTriangle::IsLeaf()
	{
		return (m_pLeft == nullptr && m_pRight == nullptr);
	}

	structBVHNodeTriangle BVHNodeTriangle::GetStructBVHNodeTriangle() 
	{
		structBVHNodeTriangle ret;
		ret.m_nLeft = -1;
		ret.m_nRight = -1;
		if (nullptr != this->m_pLeft) 
		{
			ret.m_nLeft = this->m_pLeft->m_nId;
		}
		if (nullptr != this->m_pRight)
		{
			ret.m_nRight = this->m_pRight->m_nId;
		}
		if (nullptr != this->m_pTriangle) 
		{
			ret.m_Triangle = this->m_pTriangle->GetStructTriangle();
		}
		if (nullptr != this->m_pBBox)
		{
			ret.m_BBox = this->m_pBBox->GetStructBBox();
		}
		return ret;
	}

	TriMesh::TriMesh()
	{
		m_pListBVHNodeTriangles = nullptr;
		m_nOffset = -1;
		m_nCount = 0;
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
					m_device = deviceId;

					/* Create OpenCL context */
					m_context = clCreateContext(NULL, 1, &deviceId, NULL, NULL, &status);
					if (status != CL_SUCCESS) { return false; }

					/* Create Command Queue */
					m_command_queue = clCreateCommandQueue(m_context, deviceId, 0, &status);
					if (status != CL_SUCCESS) { return false; }

					// create program
					const char* strText = Script::GetText();
					if (nullptr == strText) { return false; }
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
					m_kernelCollisionDetection = clCreateKernel(m_program, "CollisionDetection", &status);
					if (!m_kernelCollisionDetection || status != CL_SUCCESS) { return false; }

					// locals
					status = clGetKernelWorkGroupInfo(m_kernelUpdateBVHObjects, m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &m_nUpdateBVHObjects_Local, NULL);
					if (status != CL_SUCCESS) { return false; }
					status = clGetKernelWorkGroupInfo(m_kernelIntegrate, m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &m_nIntegrate_Local, NULL);
					if (status != CL_SUCCESS) { return false; }
					status = clGetKernelWorkGroupInfo(m_kernelCollisionDetection, m_device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &m_nCollisionDetection_Local, NULL);
					if (status != CL_SUCCESS) { return false; }

					m_nUpdateBVHObjects_Local = (m_nUpdateBVHObjects_Local >= 8) ? 8 : m_nUpdateBVHObjects_Local;
					m_nIntegrate_Local = (m_nIntegrate_Local >= 8) ? 8 : m_nIntegrate_Local;
					m_nCollisionDetection_Local = 1;

					// max trimeshs
					TRIMESH_COUNT = nTriMeshsCount;
					m_listFreeIds.clear();
					m_listTriMeshs.clear();
					m_listRigidBodies.clear();
					for (int i = 0; i < TRIMESH_COUNT; i++)
					{
						m_listFreeIds.push_back(i);
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

	int32_t Physics::CreateTriMesh(std::vector<glm::vec3>* pListVertices, TriMeshType type, bool bIsCommit)
	{
		if (0 == m_listFreeIds.size()) 
		{
			return -1;
		}

		cl_int err = 0;
		if (0 != m_clmem_inoutHits)
		{
			err = clEnqueueReadBuffer(m_command_queue, m_clmem_inoutHits, CL_TRUE, 0, sizeof(structHits) * m_listHits.size(), &(m_listHits[0]), 0, NULL, NULL);
			if (err != CL_SUCCESS) { return -1; }
		}
		if (0 != m_clmem_inoutIsCollisionResponse)
		{
			err = clEnqueueReadBuffer(m_command_queue, m_clmem_inoutIsCollisionResponse, CL_TRUE, 0, sizeof(int32_t) * m_listIsCollisionResponse.size(), &(m_listIsCollisionResponse[0]), 0, NULL, NULL);
			if (err != CL_SUCCESS) { return -1; }
		}

		// get free id
		int32_t nId = m_listFreeIds.at(0);
		m_listFreeIds.erase(m_listFreeIds.begin() + 0);

		// resize
		while(nId >= m_listTriMeshs.size())
		{
			m_listTriMeshs.push_back(new TriMesh());

			structRigidBody newRigidBody;
			newRigidBody.m_nRigidBodyId = (int32_t)m_listRigidBodies.size();
			m_listRigidBodies.push_back(newRigidBody);

			structHits hits;
			m_listHits.push_back(hits);
			m_listIsCollisionResponse.push_back(0);
		}

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

		//m_listTriMeshs.at(nTriMeshId)->m_nRigidBodyId = nRigidBodyId;
		m_listRigidBodies.at(nRigidBodyId).m_nTriMeshId = nTriMeshId;
		m_listRigidBodies.at(nRigidBodyId).m_nIsConvex = (int32_t)type;

		int32_t nRet = nTriMeshId;
		SetTriMesh(nRet, pListVertices);
		SetEnabled(nId, true);
		if (type == TriMeshType::Dynamic) 
		{
			SetMass(nId, 1.0f);
		}

		if (true == bIsCommit) 
		{
			if (false == Commit())
			{
				return -1;
			}
		}

		return nRet;
	}

	int32_t Physics::CreateFromId(int32_t nFromId, bool bIsCommit) 
	{
		if (0 == m_listFreeIds.size())
		{
			return -1;
		}

		cl_int err = 0;
		if (0 != m_clmem_inoutHits)
		{
			err = clEnqueueReadBuffer(m_command_queue, m_clmem_inoutHits, CL_TRUE, 0, sizeof(structHits) * m_listHits.size(), &(m_listHits[0]), 0, NULL, NULL);
			if (err != CL_SUCCESS) { return -1; }
		}
		if (0 != m_clmem_inoutIsCollisionResponse)
		{
			err = clEnqueueReadBuffer(m_command_queue, m_clmem_inoutIsCollisionResponse, CL_TRUE, 0, sizeof(int32_t) * m_listIsCollisionResponse.size(), &(m_listIsCollisionResponse[0]), 0, NULL, NULL);
			if (err != CL_SUCCESS) { return -1; }
		}

		// get free id
		int32_t nId = m_listFreeIds.at(0);
		m_listFreeIds.erase(m_listFreeIds.begin() + 0);

		// resize
		while (nId >= m_listTriMeshs.size())
		{
			m_listTriMeshs.push_back(new TriMesh());

			structRigidBody newRigidBody;
			newRigidBody.m_nRigidBodyId = (int32_t)m_listRigidBodies.size();
			m_listRigidBodies.push_back(newRigidBody);

			structHits hits;
			m_listHits.push_back(hits);
			m_listIsCollisionResponse.push_back(0);
		}
		
		// new trimesh
		int32_t nTriMeshId = nId;

		// new rigidbody
		int32_t nRigidBodyId = nId;

		// update
		structRigidBody newRigidBody;
		newRigidBody.m_nRigidBodyId = nRigidBodyId;
		newRigidBody.m_inBBox = m_listRigidBodies.at(nFromId).m_inBBox;
		m_listRigidBodies.at(nRigidBodyId) = newRigidBody;

		delete m_listTriMeshs.at(nTriMeshId);
		m_listTriMeshs.at(nTriMeshId) = new TriMesh();

		//m_listTriMeshs.at(nTriMeshId)->m_nRigidBodyId = nRigidBodyId;
		m_listRigidBodies.at(nRigidBodyId).m_nTriMeshId = nTriMeshId;
		int32_t nIsConvex = m_listRigidBodies.at(nFromId).m_nIsConvex;
		m_listRigidBodies.at(nRigidBodyId).m_nIsConvex = nIsConvex;
		SetEnabled(nId, true);

		int32_t nNewId = nTriMeshId;

		// copy fromId to newId
		m_listTriMeshs.at(nTriMeshId)->m_pListBVHNodeTriangles = m_listTriMeshs.at(nFromId)->m_pListBVHNodeTriangles;
		m_listRigidBodies.at(nRigidBodyId).m_nTriMeshId = nFromId;
		
		if (true == bIsCommit) 
		{
			if (false == Commit())
			{
				return -1;
			}
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
		m_listRigidBodies[nId].m_nIsEnabled = (true == bValue) ? 1 : 0;
	}

	bool Physics::IsEnabled(int32_t nId) 
	{
		bool ret = (m_listRigidBodies[nId].m_nIsEnabled == 0) ? false : true;
		return ret;
	}

	bool SortTrianglesFunc(Triangle *a, Triangle *b) 
	{ 
		glm::vec3 v3CenterA = (a->m_v3PosA + a->m_v3PosB + a->m_v3PosC) / 3.0f;
		glm::vec3 v3CenterB = (b->m_v3PosA + b->m_v3PosB + b->m_v3PosC) / 3.0f;

		if (fabs(v3CenterA.x - v3CenterB.x) > 0.00001f) { return (v3CenterA.x < v3CenterB.x); }
		if (fabs(v3CenterA.y - v3CenterB.y) > 0.00001f) { return (v3CenterA.y < v3CenterB.y); }
		return (v3CenterA.z < v3CenterB.z);
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
		structRigidBody theRigidBody = m_listRigidBodies.at( nId/*pTheTriMesh->m_nRigidBodyId*/);

		// calc bbox (local min/max)
		BBox* pBBox = new BBox();
		for (uint64_t i = 0; i < pListVertices->size(); i++) 
		{
			glm::vec3 v = pListVertices->at(i);
			pBBox->Add(v);
		}
		theRigidBody.m_inBBox = pBBox->GetStructBBox();
		delete pBBox;

		m_listRigidBodies[nId/*pTheTriMesh->m_nRigidBodyId*/] = theRigidBody;

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

		// root
		BVHNodeTriangle* pRoot = new BVHNodeTriangle(0);
		pTheTriMesh->m_pListBVHNodeTriangles->push_back(pRoot);

		const int nMaxLevel = 20;
		int nLevel = 1;
		// triangles to bboxs
		while (listTriangles.size() > 0)
		{
			int32_t nId = (int32_t)pTheTriMesh->m_pListBVHNodeTriangles->size() + (int32_t)pOUT->size();

			// node
			BVHNodeTriangle* pNode = new BVHNodeTriangle(nId);
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
					int32_t nId = (int32_t)pTheTriMesh->m_pListBVHNodeTriangles->size() + (int32_t)pOUT->size();

					// node
					BVHNodeTriangle* pNode = new BVHNodeTriangle(nId);
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
					int32_t nId = (int32_t)pTheTriMesh->m_pListBVHNodeTriangles->size() + (int32_t)pOUT->size();

					// node
					BVHNodeTriangle* pNode = new BVHNodeTriangle(nId);
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

			nLevel++;
		}
		while (pIN->size() > 1);

		// resize Tree to MAX_LEVEL
		bool bFirst = true;
		for (int32_t i = nLevel; i < nMaxLevel; i++)
		{
			BVHNodeTriangle* pNode;
			int32_t nId = (int32_t)pTheTriMesh->m_pListBVHNodeTriangles->size();
			pNode = new BVHNodeTriangle(nId);

			if (true == bFirst) 
			{
				pNode->m_pLeft = pIN->at(0);
				pNode->m_pBBox = BBox::Create(pNode->m_pLeft->m_pBBox);
				pIN->erase(pIN->begin() + 0);

				bFirst = false;
			}
			else 
			{
				pNode->m_pLeft = pTheTriMesh->m_pListBVHNodeTriangles->back();
				pNode->m_pBBox = BBox::Create(pNode->m_pLeft->m_pBBox);
			}

			pTheTriMesh->m_pListBVHNodeTriangles->push_back(pNode);
		}

		// add root
		pRoot->m_pLeft = pTheTriMesh->m_pListBVHNodeTriangles->back();
		pRoot->m_pBBox = BBox::Create(pRoot->m_pLeft->m_pBBox);
		pTheTriMesh->m_pListBVHNodeTriangles->at(0) = pRoot;

		// create structBVHTriangle array
		int32_t nOffset = (int32_t)m_listBVHNodeTriangles.size();
		int32_t nCount = (int32_t)pTheTriMesh->m_pListBVHNodeTriangles->size();
		for (int32_t i = 0; i < pTheTriMesh->m_pListBVHNodeTriangles->size(); i++)
		{
			// get
			BVHNodeTriangle *pBVHNodeTriangle = pTheTriMesh->m_pListBVHNodeTriangles->at(i);
			structBVHNodeTriangle node = pBVHNodeTriangle->GetStructBVHNodeTriangle();

			// update
			if (-1 != node.m_nLeft) 
			{
				node.m_nLeft += nOffset;
			}
			if (-1 != node.m_nRight)
			{
				node.m_nRight += nOffset;
			}

			// set
			m_listBVHNodeTriangles.push_back(node);
		}

		pTheTriMesh->m_nOffset = nOffset;
		pTheTriMesh->m_nCount = nCount;
		/*structBVHNodeTriangleOffset offset;
		offset.m_nOffset = nOffset;
		offset.m_nCount = nCount;
		m_listBVHNodeTrianglesOffsets.push_back(offset);*/
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
		glm::vec3 v3Rotate = ToVector3(m_listRigidBodies[nId].m_v3Rotate);
		glm::vec3 v3Position = ToVector3(m_listRigidBodies[nId].m_v3Position);

		return (glm::translate(glm::mat4(1.0f), v3Position) * glm::eulerAngleXYZ(v3Rotate.x, v3Rotate.y, v3Rotate.z));
	}

	void Physics::SetPosition(int32_t nId, glm::vec3 v3Position)
	{
		m_listRigidBodies[nId].m_v3Position = ToVector3(v3Position);
	}

	glm::vec3 Physics::GetPosition(int32_t nId)
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_v3Position);
		return ret;
	}

	void Physics::SetEulerRotate(int32_t nId, glm::vec3 v3EulerRotate)
	{
		m_listRigidBodies[nId].m_v3Rotate = ToVector3(v3EulerRotate);
	}

	glm::vec3 Physics::GetEulerRotate(int32_t nId)
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_v3Rotate);
		return ret;
	}

	void Physics::SetLinearVelocity(int32_t nId, glm::vec3 v3LinearVelocity)
	{
		m_listRigidBodies[nId].m_v3LinearVelocity = ToVector3(v3LinearVelocity);
	}

	glm::vec3 Physics::GetLinearVelocity(int32_t nId)
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_v3LinearVelocity);
		return ret;
	}

	void Physics::SetAngularVelocity(int32_t nId, glm::vec3 v3AngularVelocity)
	{
		m_listRigidBodies[nId].m_v3AngularVelocity = ToVector3(v3AngularVelocity);
	}

	glm::vec3 Physics::GetAngularVelocity(int32_t nId)
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_v3AngularVelocity);
		return ret;
	}

	void Physics::SetLinearAcceleration(int32_t nId, glm::vec3 v3LinearAcceleration)
	{
		m_listRigidBodies[nId].m_v3LinearAcceleration = ToVector3(v3LinearAcceleration);
	}

	glm::vec3 Physics::GetLinearAcceleration(int32_t nId)
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_v3LinearAcceleration);
		return ret;
	}

	void Physics::SetAngularAcceleration(int32_t nId, glm::vec3 v3AngularAcceleration)
	{
		m_listRigidBodies[nId].m_v3AngularAcceleration = ToVector3(v3AngularAcceleration);
	}

	glm::vec3 Physics::GetAngularAcceleration(int32_t nId)
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_v3AngularAcceleration);
		return ret;
	}

	void Physics::SetForce(int32_t nId, glm::vec3 v3Force)
	{
		m_listRigidBodies[nId].m_v3Force = ToVector3(v3Force);
	}

	glm::vec3 Physics::GetForce(int32_t nId)
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_v3Force);
		return ret;
	}

	void Physics::SetTorque(int32_t nId, glm::vec3 v3Torque)
	{
		m_listRigidBodies[nId].m_v3Torque = ToVector3(v3Torque);
	}

	glm::vec3 Physics::GetTorque(int32_t nId)
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_v3Torque);
		return ret;
	}

	void Physics::SetMass(int32_t nId, float fMass)
	{
		m_listRigidBodies[nId].m_fMass = fMass;
	}

	float Physics::GetMass(int32_t nId)
	{
		return m_listRigidBodies[nId].m_fMass;		
	}

	void Physics::SetRestitution(int32_t nId, float fRestitution)
	{
		m_listRigidBodies[nId].m_fRestitution = fRestitution;
	}

	float Physics::GetRestitution(int32_t nId)
	{
		return m_listRigidBodies[nId].m_fRestitution;
	}

	void Physics::SetFriction(int32_t nId, float fFriction)
	{
		m_listRigidBodies[nId].m_fFriction = fFriction;
	}

	float Physics::GetFriction(int32_t nId)
	{
		return m_listRigidBodies[nId].m_fFriction;
	}

	void Physics::SetLinearDamping(int32_t nId, float fLinearDamping)
	{
		m_listRigidBodies[nId].m_fLinearDamping = fLinearDamping;
	}

	float Physics::GetLinearDamping(int32_t nId)
	{
		return m_listRigidBodies[nId].m_fLinearDamping;
	}

	void Physics::SetAngularDamping(int32_t nId, float fAngularDamping)
	{
		m_listRigidBodies[nId].m_fAngularDamping = fAngularDamping;
	}

	float Physics::GetAngularDamping(int32_t nId)
	{
		return m_listRigidBodies[nId].m_fAngularDamping;
	}

	glm::vec3 Physics::GetBBoxMin(int32_t nId) 
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_BBox.v3Min);
		return ret;
	}
	glm::vec3 Physics::GetBBoxMax(int32_t nId) 
	{
		glm::vec3 ret = ToVector3(m_listRigidBodies[nId].m_BBox.v3Max);
		return ret;
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

		if (0 != m_clmem_inBVHNodeTriangles)
		{
			clReleaseMemObject(m_clmem_inBVHNodeTriangles);
			m_clmem_inBVHNodeTriangles = 0;
		}
		if (0 != m_clmem_inBVHNodeTrianglesOffsets)
		{
			clReleaseMemObject(m_clmem_inBVHNodeTrianglesOffsets);
			m_clmem_inBVHNodeTrianglesOffsets = 0;
		}

		if (0 != m_clmem_inoutHits)
		{
			clReleaseMemObject(m_clmem_inoutHits);
			m_clmem_inoutHits = 0;
		}
		if (0 != m_clmem_inoutIsCollisionResponse)
		{
			clReleaseMemObject(m_clmem_inoutIsCollisionResponse);
			m_clmem_inoutIsCollisionResponse = 0;
		}

		// create rigidBodies buffer
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

		// create BVHNodeTriangles buffer
		m_clmem_inBVHNodeTriangles = clCreateBuffer(m_context, CL_MEM_READ_ONLY, sizeof(structBVHNodeTriangle) * m_listBVHNodeTriangles.size(), NULL, NULL);
		if (!m_clmem_inBVHNodeTriangles) { return false; }
		// -> copy
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inBVHNodeTriangles, CL_TRUE, 0, sizeof(structBVHNodeTriangle) * m_listBVHNodeTriangles.size(), m_listBVHNodeTriangles.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// create BVHNodeTrianglesOffsets buffer
		m_listBVHNodeTrianglesOffsets.clear();
		for (int32_t i = 0; i < m_listRigidBodies.size(); i++) 
		{
			int32_t nTriMeshId = m_listRigidBodies.at(i).m_nTriMeshId;
			TriMesh* pMesh = m_listTriMeshs.at(nTriMeshId);

			structBVHNodeTriangleOffset offset;
			offset.m_nOffset = pMesh->m_nOffset;
			offset.m_nCount = pMesh->m_nCount;
			m_listBVHNodeTrianglesOffsets.push_back(offset);
		}

		m_clmem_inBVHNodeTrianglesOffsets = clCreateBuffer(m_context, CL_MEM_READ_ONLY, sizeof(structBVHNodeTriangleOffset) * m_listBVHNodeTrianglesOffsets.size(), NULL, NULL);
		if (!m_clmem_inBVHNodeTrianglesOffsets) { return false; }
		// -> copy
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inBVHNodeTrianglesOffsets, CL_TRUE, 0, sizeof(structBVHNodeTriangleOffset) * m_listBVHNodeTrianglesOffsets.size(), m_listBVHNodeTrianglesOffsets.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// create HitsBuffer
		m_clmem_inoutHits = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(structHits) * m_listHits.size(), NULL, NULL);
		if (!m_clmem_inoutHits) { return false; }
		// -> copy
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutHits, CL_TRUE, 0, sizeof(structHits) * m_listHits.size(), m_listHits.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		m_clmem_inoutIsCollisionResponse = clCreateBuffer(m_context, CL_MEM_READ_WRITE, sizeof(int32_t) * m_listIsCollisionResponse.size(), NULL, NULL);
		if (!m_clmem_inoutIsCollisionResponse) { return false; }
		// -> copy
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutIsCollisionResponse, CL_TRUE, 0, sizeof(int32_t) * m_listIsCollisionResponse.size(), m_listIsCollisionResponse.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		return true;
	}

	bool SortRigidBodiesFunc_BVH(structRigidBody &a, structRigidBody &b)
	{
		if (fabs(a.m_v3Position.x - b.m_v3Position.x) > 0.001f) { return (a.m_v3Position.x < b.m_v3Position.x); }
		if (fabs(a.m_v3Position.y - b.m_v3Position.y) > 0.001f) { return (a.m_v3Position.y < b.m_v3Position.y); }
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

		// write to GPU the current data
		err |= clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(m_listRigidBodies[0]), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }
		// 1. INTEGRATE with GPU
		if (false == Integrate(dt)) { return false; }
		// read to RAM for CPU
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(m_listRigidBodies[0]), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// 2. SORT to BVH
		std::sort(std::execution::par_unseq, m_listRigidBodies.begin(), m_listRigidBodies.end(), SortRigidBodiesFunc_BVH);
		// write to GPU
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), m_listRigidBodies.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// 3. UPDATE BVHObjects with GPU
		if (false == UpdateBVHObjects()) { return false; }

		// read BBOXs
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(m_listRigidBodies[0]), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// 4. SORT to Original
		std::sort(std::execution::par_unseq, m_listRigidBodies.begin(), m_listRigidBodies.end(), SortRigidBodiesFunc_Inc);
		// write to GPU
		err = clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), m_listRigidBodies.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

		// 5. CollisionDetection
		if (false == CollisionDetectionAndResponse(dt)) { return false; }

		// 6. CollisionResponse
		//CollisionResponse(dt);

		// 7. read to RAM for CPU
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(m_listRigidBodies[0]), 0, NULL, NULL);
		if (err != CL_SUCCESS) { return false; }

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
			element.m_BBox = theRigidBody.m_inBBox;
			element.m_nRigidBodyId = 0;

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
					element.m_BBox = theRigidBody.m_inBBox;
					element.m_nRigidBodyId = i;

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
					element.m_nRigidBodyId = -1;

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
		element.m_nRigidBodyId = -1;

		pLastLevel->push_back(element);

		// add to tree, the last level
		m_BVHObjectsLevels.push_back(pLastLevel);

		// 2/2 - LEVELS TO ONE LIST
		m_listBVHObjects.clear();
		
		// ROOT element
		structBVHObject root;
		root.m_nRigidBodyId = -1;
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
		// DEBUG 
		/*{
			cl_int err = 0;
			std::vector<structBVHObject> results;
			results.resize(m_listBVHObjects.size());
			err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutBVHObjects, CL_TRUE, 0, sizeof(structBVHObject) * m_listBVHObjects.size(), &(results[0]), 0, NULL, NULL);
			if (err != CL_SUCCESS)
			{
				return false;
			}
		}*/

		// update leafs
		int32_t nOffset = 1;
		int32_t nCount = 0;
		for (uint32_t i = 0; i < m_BVHObjectsLevels.size(); i++)
		{
			std::vector< structBVHObject > *pCurrentLevel = m_BVHObjectsLevels.at(i);
	
			if (i == (m_BVHObjectsLevels.size() - 1))
			{
				nOffset = 0;
			}
	
			nCount = (int32_t)pCurrentLevel->size();
			size_t nLocal = m_nUpdateBVHObjects_Local;
			size_t nGlobal = ((nCount + nLocal - 1) / nLocal) * nLocal;
	
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
		
		// DEBUG 
		/*{
			cl_int err = 0;
			std::vector<structBVHObject> results;
			results.resize(m_listBVHObjects.size());
			err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutBVHObjects, CL_TRUE, 0, sizeof(structBVHObject) * m_listBVHObjects.size(), &(results[0]), 0, NULL, NULL);
			if (err != CL_SUCCESS)
			{
				return false;
			}
		}*/

		return true;
	}

	bool Physics::Integrate(float dt)
	{
		cl_int err = 0;
		
		int32_t nCount = (int32_t)m_listRigidBodies.size();
		size_t nLocal = m_nIntegrate_Local;
		size_t nGlobal = ((nCount + nLocal - 1) / nLocal) * nLocal;

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

	//// DEBUG - EZ MAJD NEM KELL
	bool IsCollide(structBBox bbox1, structBBox bbox2)
	{
		if (bbox1.v3Max.x < bbox2.v3Min.x || bbox1.v3Min.x > bbox2.v3Max.x) return false;
		if (bbox1.v3Max.y < bbox2.v3Min.y || bbox1.v3Min.y > bbox2.v3Max.y) return false;
		if (bbox1.v3Max.z < bbox2.v3Min.z || bbox1.v3Min.z > bbox2.v3Max.z) return false;
		
		return true;
	}
	
	bool IsLeaf(structBVHNodeTriangle node)
	{
		if (-1 == node.m_nLeft && -1 == node.m_nRight) 
		{
			return true;
		}
		return false;
	}
	
	structBBox TransformBBox(glm::mat4 T, structBBox bbox)
	{
		structBBox ret;
	
		glm::vec3 v3Min = ToVector3(bbox.v3Min);
		glm::vec3 v3Max = ToVector3(bbox.v3Max);
	
		glm::vec3 v3In1 = glm::vec3(v3Min.x, v3Min.y, v3Min.z);
		glm::vec3 v3In2 = glm::vec3(v3Max.x, v3Min.y, v3Min.z);
		glm::vec3 v3In3 = glm::vec3(v3Max.x, v3Min.y, v3Max.z);
		glm::vec3 v3In4 = glm::vec3(v3Min.x, v3Min.y, v3Max.z);
		glm::vec3 v3In5 = glm::vec3(v3Min.x, v3Max.y, v3Min.z);
		glm::vec3 v3In6 = glm::vec3(v3Max.x, v3Max.y, v3Min.z);
		glm::vec3 v3In7 = glm::vec3(v3Max.x, v3Max.y, v3Max.z);
		glm::vec3 v3In8 = glm::vec3(v3Min.x, v3Max.y, v3Max.z);
	
		glm::vec3 v3Out1 = glm::vec3(T * glm::vec4(v3In1, 1.0f));
		glm::vec3 v3Out2 = glm::vec3(T * glm::vec4(v3In2, 1.0f));
		glm::vec3 v3Out3 = glm::vec3(T * glm::vec4(v3In3, 1.0f));
		glm::vec3 v3Out4 = glm::vec3(T * glm::vec4(v3In4, 1.0f));
		glm::vec3 v3Out5 = glm::vec3(T * glm::vec4(v3In5, 1.0f));
		glm::vec3 v3Out6 = glm::vec3(T * glm::vec4(v3In6, 1.0f));
		glm::vec3 v3Out7 = glm::vec3(T * glm::vec4(v3In7, 1.0f));
		glm::vec3 v3Out8 = glm::vec3(T * glm::vec4(v3In8, 1.0f));
	
		// 1
		v3Min = v3Out1;
		v3Max = v3Out1;
	
		// 2
		v3Min.x = std::fmin(v3Min.x, v3Out2.x);
		v3Min.y = std::fmin(v3Min.y, v3Out2.y);
		v3Min.z = std::fmin(v3Min.z, v3Out2.z);
		v3Max.x = std::fmax(v3Max.x, v3Out2.x);
		v3Max.y = std::fmax(v3Max.y, v3Out2.y);
		v3Max.z = std::fmax(v3Max.z, v3Out2.z);
	
		// 3
		v3Min.x = std::fmin(v3Min.x, v3Out3.x);
		v3Min.y = std::fmin(v3Min.y, v3Out3.y);
		v3Min.z = std::fmin(v3Min.z, v3Out3.z);
		v3Max.x = std::fmax(v3Max.x, v3Out3.x);
		v3Max.y = std::fmax(v3Max.y, v3Out3.y);
		v3Max.z = std::fmax(v3Max.z, v3Out3.z);
	
		// 4
		v3Min.x = std::fmin(v3Min.x, v3Out4.x);
		v3Min.y = std::fmin(v3Min.y, v3Out4.y);
		v3Min.z = std::fmin(v3Min.z, v3Out4.z);
		v3Max.x = std::fmax(v3Max.x, v3Out4.x);
		v3Max.y = std::fmax(v3Max.y, v3Out4.y);
		v3Max.z = std::fmax(v3Max.z, v3Out4.z);
	
		// 5
		v3Min.x = std::fmin(v3Min.x, v3Out5.x);
		v3Min.y = std::fmin(v3Min.y, v3Out5.y);
		v3Min.z = std::fmin(v3Min.z, v3Out5.z);
		v3Max.x = std::fmax(v3Max.x, v3Out5.x);
		v3Max.y = std::fmax(v3Max.y, v3Out5.y);
		v3Max.z = std::fmax(v3Max.z, v3Out5.z);
	
		// 6
		v3Min.x = std::fmin(v3Min.x, v3Out6.x);
		v3Min.y = std::fmin(v3Min.y, v3Out6.y);
		v3Min.z = std::fmin(v3Min.z, v3Out6.z);
		v3Max.x = std::fmax(v3Max.x, v3Out6.x);
		v3Max.y = std::fmax(v3Max.y, v3Out6.y);
		v3Max.z = std::fmax(v3Max.z, v3Out6.z);
	
		// 7
		v3Min.x = std::fmin(v3Min.x, v3Out7.x);
		v3Min.y = std::fmin(v3Min.y, v3Out7.y);
		v3Min.z = std::fmin(v3Min.z, v3Out7.z);
		v3Max.x = std::fmax(v3Max.x, v3Out7.x);
		v3Max.y = std::fmax(v3Max.y, v3Out7.y);
		v3Max.z = std::fmax(v3Max.z, v3Out7.z);
	
		// 8
		v3Min.x = std::fmin(v3Min.x, v3Out8.x);
		v3Min.y = std::fmin(v3Min.y, v3Out8.y);
		v3Min.z = std::fmin(v3Min.z, v3Out8.z);
		v3Max.x = std::fmax(v3Max.x, v3Out8.x);
		v3Max.y = std::fmax(v3Max.y, v3Out8.y);
		v3Max.z = std::fmax(v3Max.z, v3Out8.z);
	
		ret.v3Min = ToVector3(v3Min);
		ret.v3Max = ToVector3(v3Max);
	
		return ret;
	}
	//
	//
	//float angle(structVector3 v3A, structVector3 v3B)
	//{
	//	float fAngle = std::acosf(glm::dot(ToVector3(v3A), ToVector3(v3B)));
	//
	//	return fAngle;
	//}
	//
	//bool IsEqual(structHit hit1, structHit hit2)
	//{
	//	if (glm::distance(ToVector3(hit1.m_v3HitPointInWorld), ToVector3(hit2.m_v3HitPointInWorld)) < 0.05f)
	//	{
	//		if (angle(hit1.m_v3Normal, hit2.m_v3Normal) < (3.141592f / 180.0f)) // 1 degree 
	//		{
	//			return true;
	//		}
	//	}
	//
	//	return false;
	//}
	//
	//bool IsContains(structHits* pHits, structHit hit)
	//{
	//	for (int i = 0; i < pHits->m_nNumHits; i++)
	//	{
	//		if (true == IsEqual(pHits->m_hits[i], hit))
	//		{
	//			return true;
	//		}
	//	}
	//
	//	return false;
	//}
	//
	//void Intersect_LineTriangle(structHits* pHits, bool isInvertNormal, glm::vec3 v3LineA, glm::vec3 v3LineB, glm::vec3 v3TriA, glm::vec3 v3TriB, glm::vec3 v3TriC, glm::vec3 v3TriN)
	//{
	//	if (pHits->m_nNumHits >= MAX_HITS_OBJECT_OBJECT)
	//	{
	//		return;
	//	}
	//
	//	glm::vec3 v3Dir = glm::normalize(v3LineB - v3LineA);
	//
	//	float cost = glm::dot(v3Dir, v3TriN);
	//	if (std::fabs(cost) < 0.001f)
	//	{
	//		return;
	//	}
	//		
	//	float t = glm::dot(v3TriA - v3LineA, v3TriN) / cost;
	//	float fMaxLenght = glm::distance(v3LineA, v3LineB);
	//	if (t < 0.0f || t > fMaxLenght)
	//	{
	//		return;
	//	}
	//		
	//	glm::vec3 v3P = v3LineA + (v3Dir * t);
	//
	//	glm::vec3 v3N2;
	//	v3N2 = glm::cross(v3TriB - v3TriA, v3P - v3TriA);
	//	float c1 = glm::dot(v3TriN, v3N2);
	//
	//	v3N2 = glm::cross(v3TriC - v3TriB, v3P - v3TriB);
	//	float c2 = glm::dot(v3TriN, v3N2);
	//
	//	v3N2 = glm::cross(v3TriA - v3TriC, v3P - v3TriC);
	//	float c3 = glm::dot(v3TriN, v3N2);
	//
	//	if ((c1 > 0.0f && c2 > 0.0f && c3 > 0.0f) || (c1 < 0.0f && c2 < 0.0f && c3 < 0.0f))
	//	{
	//		structHit hit;
	//
	//		//hit.m_v3HitPointInWorld = ToVector3(v3P);
	//		if (glm::dot(v3LineA - v3TriA, v3TriN) < 0.0f)
	//		{
	//			hit.m_v3HitPointInWorld = ToVector3(v3LineA);
	//		}
	//		else
	//		{
	//			hit.m_v3HitPointInWorld = ToVector3(v3LineB);
	//		}
	//
	//		if (false == isInvertNormal) { hit.m_v3Normal = ToVector3(v3TriN); }
	//		else { hit.m_v3Normal = ToVector3(-1.0f * v3TriN); }
	//
	//		if (false == IsContains(pHits, hit))
	//		{
	//			pHits->m_hits[pHits->m_nNumHits] = hit;
	//			pHits->m_nNumHits++;
	//		}
	//	}
	//}
	//
	//void Intersect_TriangleTriangle(structHits* pHits, glm::vec3 tri1_a, glm::vec3 tri1_b, glm::vec3 tri1_c, glm::vec3 tri1_n, glm::vec3 tri2_a, glm::vec3 tri2_b, glm::vec3 tri2_c, glm::vec3 tri2_n)
	//{
	//	Intersect_LineTriangle(pHits, false, tri1_a, tri1_b, tri2_a, tri2_b, tri2_c, tri2_n);
	//	Intersect_LineTriangle(pHits, false, tri1_b, tri1_c, tri2_a, tri2_b, tri2_c, tri2_n);
	//	Intersect_LineTriangle(pHits, false, tri1_c, tri1_a, tri2_a, tri2_b, tri2_c, tri2_n);
	//
	//	Intersect_LineTriangle(pHits, true, tri2_a, tri2_b, tri1_a, tri1_b, tri1_c, tri1_n);
	//	Intersect_LineTriangle(pHits, true, tri2_b, tri2_c, tri1_a, tri1_b, tri1_c, tri1_n);
	//	Intersect_LineTriangle(pHits, true, tri2_c, tri2_a, tri1_a, tri1_b, tri1_c, tri1_n);
	//}									  
	//
	//// hordó - pálya egy háromszöge
	//void SearchHits2(structHits* pHits, structRigidBody structRigidBody1/*only dynamic*/, structRigidBody structRigidBody2/*static or dynamic*/, int32_t nId2, glm::mat4 T1, glm::mat4 T2, structBVHNodeTriangleOffset offset1, structBVHNodeTriangleOffset offset2, structBVHNodeTriangle* pListBVHNodeTriangles)
	//{
	//	structBVHNodeTriangle structNodeOrTriangle2 = pListBVHNodeTriangles[nId2];
	//	structTriangle triangle2 = structNodeOrTriangle2.m_Triangle;
	//	glm::vec3 tri2_a = glm::vec3(T2 * glm::vec4(ToVector3(triangle2.m_v3PosA), 1.0f));
	//	glm::vec3 tri2_b = glm::vec3(T2 * glm::vec4(ToVector3(triangle2.m_v3PosB), 1.0f));
	//	glm::vec3 tri2_c = glm::vec3(T2 * glm::vec4(ToVector3(triangle2.m_v3PosC), 1.0f));
	//	glm::vec3 tri2_n = glm::vec3(T2 * glm::vec4(ToVector3(triangle2.m_v3Normal), 0.0f));
	//
	//	structNodeOrTriangle2.m_BBox = TransformBBox(T2, structNodeOrTriangle2.m_BBox);
	//
	//	int nTop = -1;
	//	int arrStack[64];
	//
	//	nTop++;
	//	arrStack[nTop] = offset1.m_nOffset;
	//
	//	int nId1;
	//	structBVHNodeTriangle structNodeOrTriangle;
	//
	//	while (nTop > -1)
	//	{
	//		nId1 = arrStack[nTop];
	//		nTop--;
	//
	//		structNodeOrTriangle = pListBVHNodeTriangles[nId1];
	//		structNodeOrTriangle.m_BBox = TransformBBox(T1, structNodeOrTriangle.m_BBox);
	//
	//		if (true == IsLeaf(structNodeOrTriangle)) 
	//		{
	//			if (true == IsCollide(structNodeOrTriangle2.m_BBox, structNodeOrTriangle.m_BBox))
	//			{
	//				// TRANSFORM TRIANGLE 1
	//				structTriangle triangle1 = structNodeOrTriangle.m_Triangle;
	//				glm::vec3 tri1_a = glm::vec3(T1 * glm::vec4(ToVector3(triangle1.m_v3PosA), 1.0f));
	//				glm::vec3 tri1_b = glm::vec3(T1 * glm::vec4(ToVector3(triangle1.m_v3PosB), 1.0f));
	//				glm::vec3 tri1_c = glm::vec3(T1 * glm::vec4(ToVector3(triangle1.m_v3PosC), 1.0f));
	//				glm::vec3 tri1_n = glm::vec3(T1 * glm::vec4(ToVector3(triangle1.m_v3Normal), 0.0f));
	//
	//				// CollisionDetection tri1, tri2
	//				Intersect_TriangleTriangle(pHits, tri1_a, tri1_b, tri1_c, tri1_n, tri2_a, tri2_b, tri2_c, tri2_n);
	//
	//				if (pHits->m_nNumHits >= MAX_HITS_OBJECT_OBJECT)
	//				{
	//					return;
	//				}
	//
	//			}
	//		}
	//		else 
	//		{
	//			if (true == IsCollide(structNodeOrTriangle2.m_BBox, structNodeOrTriangle.m_BBox))
	//			{
	//				if (structNodeOrTriangle.m_nLeft != -1)
	//				{
	//					nTop++;
	//					arrStack[nTop] = structNodeOrTriangle.m_nLeft;
	//				}
	//
	//				if (structNodeOrTriangle.m_nRight != -1)
	//				{
	//					nTop++;
	//					arrStack[nTop] = structNodeOrTriangle.m_nRight;
	//				}
	//			}
	//		}
	//	}
	//
	//}

	glm::vec3 Support(glm::vec3 v3Dir, glm::mat4 T, structRigidBody structRigidBody, structBVHNodeTriangleOffset offset, structBVHNodeTriangle* pListBVHNodeTriangles)
	{
		v3Dir = glm::inverse(T) * glm::vec4(v3Dir, 0.0f);

		glm::vec3 furthest_point(0, 0, 0);
		float max_dot = -FLT_MAX;

		structBVHNodeTriangle node;
		
		for (int i = offset.m_nOffset; i < (offset.m_nOffset + offset.m_nCount); i++)
		{
			node = pListBVHNodeTriangles[i];
		
			if (true == IsLeaf(node))
			{
				structTriangle triangle = node.m_Triangle;
		
				// A
				{
					glm::vec3 v = ToVector3(triangle.m_v3PosB);
					float d = glm::dot(v, v3Dir);
					if (d > max_dot)
					{
						max_dot = d;
						furthest_point = v;
					}
				}
		
				// B
				{
					glm::vec3 v = ToVector3(triangle.m_v3PosB);
					float d = glm::dot(v, v3Dir);
					if (d > max_dot)
					{
						max_dot = d;
						furthest_point = v;
					}
				}
		
				// C
				{
					glm::vec3 v = ToVector3(triangle.m_v3PosC);
					float d = glm::dot(v, v3Dir);
					if (d > max_dot)
					{
						max_dot = d;
						furthest_point = v;
					}
				}
			}
		}

		glm::vec3 v3Ret = T * glm::vec4(furthest_point, 1); //convert support to world space
		return v3Ret;
	}
	
	glm::vec3 Support(glm::vec3 v3Dir, structTriangle triangle) 
	{
		glm::vec3 v3A = ToVector3(triangle.m_v3PosA);
		glm::vec3 v3B = ToVector3(triangle.m_v3PosB);
		glm::vec3 v3C = ToVector3(triangle.m_v3PosC);
		glm::vec3 v3N = ToVector3(triangle.m_v3Normal);

		//Find which triangle vertex is furthest along dir
		float dot0 = glm::dot(v3A, v3Dir);
		float dot1 = glm::dot(v3B, v3Dir);
		float dot2 = glm::dot(v3C, v3Dir);
		glm::vec3 furthest_point = v3A;
		if (dot1 > dot0) {
			furthest_point = v3B;
			if (dot2 > dot1)
				furthest_point = v3C;
		}
		else if (dot2 > dot0) {
			furthest_point = v3C;
		}

		//fake some depth behind triangle so we have volume
		if (glm::dot(v3Dir, v3N) < 0) furthest_point -= v3N;

		return furthest_point;

	}

	struct Point 
	{
		glm::vec3 p; //Conserve Minkowski Difference
		glm::vec3 a; //Result coordinate of object A's support function 
		glm::vec3 b; //Result coordinate of object B's support function 
	};

	struct Plane 
	{
		//Unit-length plane normal
		glm::vec3 normal;
		//Distance from origin
		float	distance;
	};

	Plane PlaneFromTri(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2)
	{
		glm::vec3 v1v0 = v1 - v0;
		glm::vec3 v2v0 = v2 - v0;

		glm::vec3 normal = glm::normalize(glm::cross(v1v0, v2v0));

		float d = -glm::dot(v0, normal);

		Plane plane;
		plane.normal = normal;
		plane.distance = d;

		return plane;
	}

	float DistanceFromPlane(Plane plane, glm::vec3 in)
	{
		return glm::dot(in, plane.normal) + plane.distance;
	}

	glm::vec3 ProjectPointOntoPlane(Plane plane, glm::vec3 point)
	{
		float distance = DistanceFromPlane(plane, point);

		return point - (plane.normal * distance);
	}

	void Barycentric(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 p, float* u, float* v, float* w)
	{
		glm::vec3 v0 = b - a, v1 = c - a, v2 = p - a;
		float d00 = glm::dot(v0, v0);
		float d01 = glm::dot(v0, v1);
		float d11 = glm::dot(v1, v1);
		float d20 = glm::dot(v2, v0);
		float d21 = glm::dot(v2, v1);
		float denom = d00 * d11 - d01 * d01;
		*v = (d11 * d20 - d01 * d21) / denom;
		*w = (d00 * d21 - d01 * d20) / denom;
		*u = 1.0f - *v - *w;

		if (isnan(*u) || isnan(*v) || isnan(*w))
		{
			*u = 1.0f / 3.0f;
			*v = 1.0f / 3.0f;
			*w = 1.0f / 3.0f;
		}
		if (isinf(*u) || isinf(*v) || isinf(*w))
		{
			*u = 1.0f / 3.0f;
			*v = 1.0f / 3.0f;
			*w = 1.0f / 3.0f;
		}
	}

	void CalculateSearchPoint(Point* pPoint, glm::vec3 search_dir, structRigidBody structRigidBody1/*only dynamic*/, structRigidBody structRigidBody2/*static or dynamic*/, glm::mat4 T1, glm::mat4 T2, structBVHNodeTriangleOffset offset1, structBVHNodeTriangleOffset offset2, structBVHNodeTriangle* pListBVHNodeTriangles1, structBVHNodeTriangle* pListBVHNodeTriangles2)
	{
		pPoint->b = Support(search_dir, T2, structRigidBody2, offset2, pListBVHNodeTriangles2);
		pPoint->a = Support(-search_dir, T1, structRigidBody1, offset1, pListBVHNodeTriangles1);
		pPoint->p = pPoint->b - pPoint->a;
	}

	void CalculateSearchPoint(Point* pPoint, glm::vec3 search_dir, structRigidBody structRigidBody1/*only dynamic*/, structTriangle triangle/*static or dynamic*/, glm::mat4 T1, structBVHNodeTriangleOffset offset1, structBVHNodeTriangle* pListBVHNodeTriangles)
	{
		pPoint->b = Support(search_dir, triangle);
		pPoint->a = Support(-search_dir, T1, structRigidBody1, offset1, pListBVHNodeTriangles);
		pPoint->p = pPoint->b - pPoint->a;
	}

	void EPA(structHits* pHits, Point a, Point b, Point c, Point d, structRigidBody structRigidBody1/*only dynamic*/, structRigidBody structRigidBody2/*static or dynamic*/, glm::mat4 T1, glm::mat4 T2, structBVHNodeTriangleOffset offset1, structBVHNodeTriangleOffset offset2, structBVHNodeTriangle* pListBVHNodeTriangles1, structBVHNodeTriangle* pListBVHNodeTriangles2)
	{
		Point faces[EPA_MAX_NUM_FACES][4]; //Array of faces, each with 3 verts and a normal

		//glm::vec3 VertexA[3];
		//glm::vec3 VertexB[3];

		//Init with final simplex from GJK
		faces[0][0] = a;
		faces[0][1] = b;
		faces[0][2] = c;
		faces[0][3].p = glm::normalize(glm::cross(b.p - a.p, c.p - a.p)); //ABC
		faces[1][0] = a;
		faces[1][1] = c;
		faces[1][2] = d;
		faces[1][3].p = glm::normalize(glm::cross(c.p - a.p, d.p - a.p)); //ACD
		faces[2][0] = a;
		faces[2][1] = d;
		faces[2][2] = b;
		faces[2][3].p = glm::normalize(glm::cross(d.p - a.p, b.p - a.p)); //ADB
		faces[3][0] = b;
		faces[3][1] = d;
		faces[3][2] = c;
		faces[3][3].p = glm::normalize(glm::cross(d.p - b.p, c.p - b.p)); //BDC

		int num_faces = 4;
		int closest_face;

		for (int iterations = 0; iterations < EPA_MAX_NUM_ITERATIONS; iterations++) {
			//Find face that's closest to origin
			float min_dist = glm::dot(faces[0][0].p, faces[0][3].p);
			closest_face = 0;
			for (int i = 1; i < num_faces; i++) {
				float dist = glm::dot(faces[i][0].p, faces[i][3].p);
				if (dist < min_dist) {
					min_dist = dist;
					closest_face = i;
				}
			}

			//search normal to face that's closest to origin
			glm::vec3 search_dir = faces[closest_face][3].p;

			Point p;
			CalculateSearchPoint(&p, search_dir, structRigidBody1, structRigidBody2, T1, T2, offset1, offset2, pListBVHNodeTriangles1, pListBVHNodeTriangles2);

			if (glm::dot(p.p, search_dir) - min_dist < EPA_TOLERANCE) {

				/*Core of calculating collision information*/
				Plane closestPlane = PlaneFromTri(faces[closest_face][0].p, faces[closest_face][1].p, faces[closest_face][2].p); //plane of closest triangle face
				glm::vec3 projectionPoint = ProjectPointOntoPlane(closestPlane, glm::vec3(0, 0, 0)); //projecting the origin onto the triangle(both are in Minkowski space)
				float u, v, w;
				Barycentric(faces[closest_face][0].p, faces[closest_face][1].p, faces[closest_face][2].p,
					projectionPoint, &u, &v, &w); //finding the barycentric coordinate of this projection point to the triangle

				//The contact points just have the same barycentric coordinate in their own triangles which  are composed by result coordinates of support function 
				glm::vec3 localA = faces[closest_face][0].a * u + faces[closest_face][1].a * v + faces[closest_face][2].a * w;
				glm::vec3 localB = faces[closest_face][0].b * u + faces[closest_face][1].b * v + faces[closest_face][2].b * w;
				float penetration = glm::length(localA - localB);
				glm::vec3 normal = glm::normalize(localA - localB);

				if (isnan(normal.x) || isnan(normal.y) || isnan(normal.z))
				{
					normal = glm::vec3(1, 0, 0);
				}

				//Convergence (new point is not significantly further from origin)
				//localA -= coll1->GetTransform().GetPosition();
				//localB -= coll2->GetTransform().GetPosition();

				//collisionInfo.AddContactPoint(localA, localB, normal, penetration);
				if (pHits->m_nNumHits < MAX_HITS_OBJECT_OBJECT)
				{
					structHit hit;
					hit.m_v3HitPointInWorldA = ToVector3(localA);
					hit.m_v3HitPointInWorldB = ToVector3(localB);
					hit.m_v3Normal = ToVector3(-normal);

					//hit.m_fPenetrationDepth = penetration;
					if (1 == structRigidBody1.m_nIsConvex && 1 == structRigidBody2.m_nIsConvex) 
					{
						hit.m_v3SeparateVelocity = ToVector3((std::fabs(penetration) + 0.001f) * glm::normalize(ToVector3(structRigidBody1.m_v3Position) - ToVector3(structRigidBody2.m_v3Position)));

						if (std::isnan(hit.m_v3SeparateVelocity.x) || std::isinf(hit.m_v3SeparateVelocity.x)) { hit.m_v3SeparateVelocity.x = 0.0f; }
						if (std::isnan(hit.m_v3SeparateVelocity.y) || std::isinf(hit.m_v3SeparateVelocity.y)) { hit.m_v3SeparateVelocity.y = 0.0f; }
						if (std::isnan(hit.m_v3SeparateVelocity.z) || std::isinf(hit.m_v3SeparateVelocity.z)) { hit.m_v3SeparateVelocity.z = 0.0f; }
					}
				
					pHits->m_hits[pHits->m_nNumHits] = hit;
					pHits->m_nNumHits++;
				}
				/*Core of calculating collision information*/

				return;
			}

			Point loose_edges[EPA_MAX_NUM_LOOSE_EDGES][2]; //keep track of edges we need to fix after removing faces
			int num_loose_edges = 0;

			//Find all triangles that are facing p
			for (int i = 0; i < num_faces; i++)
			{
				if (glm::dot(faces[i][3].p, p.p - faces[i][0].p) > 0) //triangle i faces p, remove it
				{
					//Add removed triangle's edges to loose edge list.
					//If it's already there, remove it (both triangles it belonged to are gone)
					for (int j = 0; j < 3; j++) //Three edges per face
					{
						Point current_edge[2] = { faces[i][j], faces[i][(j + 1) % 3] };
						bool found_edge = false;
						for (int k = 0; k < num_loose_edges; k++) //Check if current edge is already in list
						{
							if (loose_edges[k][1].p == current_edge[0].p && loose_edges[k][0].p == current_edge[1].p) {
								loose_edges[k][0] = loose_edges[num_loose_edges - 1][0]; //Overwrite current edge
								loose_edges[k][1] = loose_edges[num_loose_edges - 1][1]; //with last edge in list
								num_loose_edges--;
								found_edge = true;
								k = num_loose_edges; //exit loop because edge can only be shared once
							}
						}//endfor loose_edges

						if (!found_edge) { //add current edge to list
							// assert(num_loose_edges<EPA_MAX_NUM_LOOSE_EDGES);
							if (num_loose_edges >= EPA_MAX_NUM_LOOSE_EDGES) break;
							loose_edges[num_loose_edges][0] = current_edge[0];
							loose_edges[num_loose_edges][1] = current_edge[1];
							num_loose_edges++;
						}
					}

					//Remove triangle i from list
					faces[i][0] = faces[num_faces - 1][0];
					faces[i][1] = faces[num_faces - 1][1];
					faces[i][2] = faces[num_faces - 1][2];
					faces[i][3] = faces[num_faces - 1][3];
					num_faces--;
					i--;
				}//endif p can see triangle i
			}//endfor num_faces

			//Reconstruct polytope with p added
			for (int i = 0; i < num_loose_edges; i++)
			{
				// assert(num_faces<EPA_MAX_NUM_FACES);
				if (num_faces >= EPA_MAX_NUM_FACES) break;

				glm::vec3 v3P = glm::normalize(glm::cross(loose_edges[i][0].p - loose_edges[i][1].p, loose_edges[i][0].p - p.p));
				if (isnan(v3P.x) || isnan(v3P.y) || isnan(v3P.z)) 
				{ 
					continue;
				}

				faces[num_faces][0] = loose_edges[i][0];
				faces[num_faces][1] = loose_edges[i][1];
				faces[num_faces][2] = p;
				faces[num_faces][3].p = v3P;

				//Check for wrong normal to maintain CCW winding
				float bias = 0.000001f; //in case dot result is only slightly < 0 (because origin is on face)
				if (glm::dot(faces[num_faces][0].p, faces[num_faces][3].p) + bias < 0) {
					Point temp = faces[num_faces][0];
					faces[num_faces][0] = faces[num_faces][1];
					faces[num_faces][1] = temp;
					faces[num_faces][3].p = -faces[num_faces][3].p;
				}
				num_faces++;
			}
		} //End for iterations

		//return;

		//printf("EPA did not converge\n");
		//Return most recent closest point
		glm::vec3 search_dir = faces[closest_face][3].p;

		Point p;
		CalculateSearchPoint(&p, search_dir, structRigidBody1, structRigidBody2, T1, T2, offset1, offset2, pListBVHNodeTriangles1, pListBVHNodeTriangles2);

		Plane closestPlane = PlaneFromTri(faces[closest_face][0].p, faces[closest_face][1].p, faces[closest_face][2].p);
		glm::vec3 projectionPoint = ProjectPointOntoPlane(closestPlane, glm::vec3(0, 0, 0));
		float u, v, w;
		Barycentric(faces[closest_face][0].p, faces[closest_face][1].p, faces[closest_face][2].p,
			projectionPoint, &u, &v, &w);
		glm::vec3 localA = faces[closest_face][0].a * u + faces[closest_face][1].a * v + faces[closest_face][2].a * w;
		glm::vec3 localB = faces[closest_face][0].b * u + faces[closest_face][1].b * v + faces[closest_face][2].b * w;
		float penetration = glm::length(localA - localB);
		glm::vec3 normal = glm::normalize(localA - localB);

		if (isnan(normal.x) || isnan(normal.y) || isnan(normal.z))
		{
			normal = glm::vec3(1, 0, 0);
		}

		//collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		localA += ToVector3(structRigidBody1.m_v3Position);
		localB += ToVector3(structRigidBody2.m_v3Position);

		if (pHits->m_nNumHits < MAX_HITS_OBJECT_OBJECT)
		{
			structHit hit;
			hit.m_v3HitPointInWorldA = ToVector3(localA);
			hit.m_v3HitPointInWorldB = ToVector3(localB);
			hit.m_v3Normal = ToVector3(-normal);
			
			//hit.m_fPenetrationDepth = penetration;
			if (1 == structRigidBody1.m_nIsConvex && 1 == structRigidBody2.m_nIsConvex)
			{
				hit.m_v3SeparateVelocity = ToVector3((std::fabs(penetration) + 0.001f) * glm::normalize(ToVector3(structRigidBody1.m_v3Position) - ToVector3(structRigidBody2.m_v3Position)));

				if (std::isnan(hit.m_v3SeparateVelocity.x) || std::isinf(hit.m_v3SeparateVelocity.x)) { hit.m_v3SeparateVelocity.x = 0.0f; }
				if (std::isnan(hit.m_v3SeparateVelocity.y) || std::isinf(hit.m_v3SeparateVelocity.y)) { hit.m_v3SeparateVelocity.y = 0.0f; }
				if (std::isnan(hit.m_v3SeparateVelocity.z) || std::isinf(hit.m_v3SeparateVelocity.z)) { hit.m_v3SeparateVelocity.z = 0.0f; }
			}

			pHits->m_hits[pHits->m_nNumHits] = hit;
			pHits->m_nNumHits++;
		}

		return;
	}

	void EPA(structHits* pHits, Point a, Point b, Point c, Point d, structRigidBody structRigidBody1/*only dynamic*/, structTriangle triangle, glm::mat4 T1, structBVHNodeTriangleOffset offset1, structBVHNodeTriangle* pListBVHNodeTriangles)
	{
		Point faces[EPA_MAX_NUM_FACES][4]; //Array of faces, each with 3 verts and a normal

		//glm::vec3 VertexA[3];
		//glm::vec3 VertexB[3];

		//Init with final simplex from GJK
		faces[0][0] = a;
		faces[0][1] = b;
		faces[0][2] = c;
		faces[0][3].p = glm::normalize(glm::cross(b.p - a.p, c.p - a.p)); //ABC
		faces[1][0] = a;
		faces[1][1] = c;
		faces[1][2] = d;
		faces[1][3].p = glm::normalize(glm::cross(c.p - a.p, d.p - a.p)); //ACD
		faces[2][0] = a;
		faces[2][1] = d;
		faces[2][2] = b;
		faces[2][3].p = glm::normalize(glm::cross(d.p - a.p, b.p - a.p)); //ADB
		faces[3][0] = b;
		faces[3][1] = d;
		faces[3][2] = c;
		faces[3][3].p = glm::normalize(glm::cross(d.p - b.p, c.p - b.p)); //BDC

		int num_faces = 4;
		int closest_face;

		for (int iterations = 0; iterations < EPA_MAX_NUM_ITERATIONS; iterations++) {
			//Find face that's closest to origin
			float min_dist = glm::dot(faces[0][0].p, faces[0][3].p);
			closest_face = 0;
			for (int i = 1; i < num_faces; i++) {
				float dist = glm::dot(faces[i][0].p, faces[i][3].p);
				if (dist < min_dist) {
					min_dist = dist;
					closest_face = i;
				}
			}

			//search normal to face that's closest to origin
			glm::vec3 search_dir = faces[closest_face][3].p;

			Point p;
			CalculateSearchPoint(&p, search_dir, structRigidBody1, triangle, T1, offset1, pListBVHNodeTriangles);

			if (glm::dot(p.p, search_dir) - min_dist < EPA_TOLERANCE) {

				/*Core of calculating collision information*/
				Plane closestPlane = PlaneFromTri(faces[closest_face][0].p, faces[closest_face][1].p, faces[closest_face][2].p); //plane of closest triangle face
				glm::vec3 projectionPoint = ProjectPointOntoPlane(closestPlane, glm::vec3(0, 0, 0)); //projecting the origin onto the triangle(both are in Minkowski space)
				float u, v, w;
				Barycentric(faces[closest_face][0].p, faces[closest_face][1].p, faces[closest_face][2].p,
					projectionPoint, &u, &v, &w); //finding the barycentric coordinate of this projection point to the triangle

				//The contact points just have the same barycentric coordinate in their own triangles which  are composed by result coordinates of support function 
				glm::vec3 localA = faces[closest_face][0].a * u + faces[closest_face][1].a * v + faces[closest_face][2].a * w;
				glm::vec3 localB = faces[closest_face][0].b * u + faces[closest_face][1].b * v + faces[closest_face][2].b * w;
				float penetration = glm::length(localA - localB);
				glm::vec3 normal = glm::normalize(localA - localB);

				if (isnan(normal.x) || isnan(normal.y) || isnan(normal.z))
				{
					normal = glm::vec3(1, 0, 0);
				}

				//Convergence (new point is not significantly further from origin)
				//localA -= coll1->GetTransform().GetPosition();
				//localB -= coll2->GetTransform().GetPosition();

				//collisionInfo.AddContactPoint(localA, localB, normal, penetration);
				if (pHits->m_nNumHits < MAX_HITS_OBJECT_OBJECT)
				{
					structHit hit;
					hit.m_v3HitPointInWorldA = ToVector3(localA);
					hit.m_v3HitPointInWorldB = ToVector3(localA);
					hit.m_v3Normal = ToVector3(-normal);
					
					//hit.m_fPenetrationDepth = penetration;
					if (1 == structRigidBody1.m_nIsConvex)
					{
						hit.m_v3SeparateVelocity = ToVector3((std::fabs(penetration) + 0.001f) * ToVector3(triangle.m_v3Normal));

						if (std::isnan(hit.m_v3SeparateVelocity.x) || std::isinf(hit.m_v3SeparateVelocity.x)) { hit.m_v3SeparateVelocity.x = 0.0f; }
						if (std::isnan(hit.m_v3SeparateVelocity.y) || std::isinf(hit.m_v3SeparateVelocity.y)) { hit.m_v3SeparateVelocity.y = 0.0f; }
						if (std::isnan(hit.m_v3SeparateVelocity.z) || std::isinf(hit.m_v3SeparateVelocity.z)) { hit.m_v3SeparateVelocity.z = 0.0f; }
					}

					pHits->m_hits[pHits->m_nNumHits] = hit;
					pHits->m_nNumHits++;
				}
				/*Core of calculating collision information*/

				return;
			}

			Point loose_edges[EPA_MAX_NUM_LOOSE_EDGES][2]; //keep track of edges we need to fix after removing faces
			int num_loose_edges = 0;

			//Find all triangles that are facing p
			for (int i = 0; i < num_faces; i++)
			{
				if (glm::dot(faces[i][3].p, p.p - faces[i][0].p) > 0) //triangle i faces p, remove it
				{
					//Add removed triangle's edges to loose edge list.
					//If it's already there, remove it (both triangles it belonged to are gone)
					for (int j = 0; j < 3; j++) //Three edges per face
					{
						Point current_edge[2] = { faces[i][j], faces[i][(j + 1) % 3] };
						bool found_edge = false;
						for (int k = 0; k < num_loose_edges; k++) //Check if current edge is already in list
						{
							if (loose_edges[k][1].p == current_edge[0].p && loose_edges[k][0].p == current_edge[1].p) {
								loose_edges[k][0] = loose_edges[num_loose_edges - 1][0]; //Overwrite current edge
								loose_edges[k][1] = loose_edges[num_loose_edges - 1][1]; //with last edge in list
								num_loose_edges--;
								found_edge = true;
								k = num_loose_edges; //exit loop because edge can only be shared once
							}
						}//endfor loose_edges

						if (!found_edge) { //add current edge to list
							// assert(num_loose_edges<EPA_MAX_NUM_LOOSE_EDGES);
							if (num_loose_edges >= EPA_MAX_NUM_LOOSE_EDGES) break;
							loose_edges[num_loose_edges][0] = current_edge[0];
							loose_edges[num_loose_edges][1] = current_edge[1];
							num_loose_edges++;
						}
					}

					//Remove triangle i from list
					faces[i][0] = faces[num_faces - 1][0];
					faces[i][1] = faces[num_faces - 1][1];
					faces[i][2] = faces[num_faces - 1][2];
					faces[i][3] = faces[num_faces - 1][3];
					num_faces--;
					i--;
				}//endif p can see triangle i
			}//endfor num_faces

			//Reconstruct polytope with p added
			for (int i = 0; i < num_loose_edges; i++)
			{
				// assert(num_faces<EPA_MAX_NUM_FACES);
				if (num_faces >= EPA_MAX_NUM_FACES) break;

				glm::vec3 v3P = glm::normalize(glm::cross(loose_edges[i][0].p - loose_edges[i][1].p, loose_edges[i][0].p - p.p));
				if (isnan(v3P.x) || isnan(v3P.y) || isnan(v3P.z)) 
				{ 
					continue; 
				}

				faces[num_faces][0] = loose_edges[i][0];
				faces[num_faces][1] = loose_edges[i][1];
				faces[num_faces][2] = p;
				faces[num_faces][3].p = v3P;

				//Check for wrong normal to maintain CCW winding
				float bias = 0.000001f; //in case dot result is only slightly < 0 (because origin is on face)
				if (glm::dot(faces[num_faces][0].p, faces[num_faces][3].p) + bias < 0) {
					Point temp = faces[num_faces][0];
					faces[num_faces][0] = faces[num_faces][1];
					faces[num_faces][1] = temp;
					faces[num_faces][3].p = -faces[num_faces][3].p;
				}
				num_faces++;
			}
		} //End for iterations

		//return;

		//printf("EPA did not converge\n");
		//Return most recent closest point
		glm::vec3 search_dir = faces[closest_face][3].p;

		Point p;
		CalculateSearchPoint(&p, search_dir, structRigidBody1, triangle, T1, offset1, pListBVHNodeTriangles);

		Plane closestPlane = PlaneFromTri(faces[closest_face][0].p, faces[closest_face][1].p, faces[closest_face][2].p);
		glm::vec3 projectionPoint = ProjectPointOntoPlane(closestPlane, glm::vec3(0, 0, 0));
		float u, v, w;
		Barycentric(faces[closest_face][0].p, faces[closest_face][1].p, faces[closest_face][2].p,
			projectionPoint, &u, &v, &w);
		glm::vec3 localA = faces[closest_face][0].a * u + faces[closest_face][1].a * v + faces[closest_face][2].a * w;
		glm::vec3 localB = faces[closest_face][0].b * u + faces[closest_face][1].b * v + faces[closest_face][2].b * w;
		float penetration = glm::length(localA - localB);
		glm::vec3 normal = glm::normalize(localA - localB);

		if (isnan(normal.x) || isnan(normal.y) || isnan(normal.z))
		{
			normal = glm::vec3(1, 0, 0);
		}

		//collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		localA += ToVector3(structRigidBody1.m_v3Position);
		localB += glm::vec3(0,0,0);

		if (pHits->m_nNumHits < MAX_HITS_OBJECT_OBJECT)
		{
			structHit hit;
			hit.m_v3HitPointInWorldA = ToVector3(localA);
			hit.m_v3HitPointInWorldB = ToVector3(localA);
			hit.m_v3Normal = ToVector3(-normal);

			//hit.m_fPenetrationDepth = penetration;
			if (1 == structRigidBody1.m_nIsConvex)
			{
				hit.m_v3SeparateVelocity = ToVector3((std::fabs(penetration) + 0.001f) * ToVector3(triangle.m_v3Normal));

				if (std::isnan(hit.m_v3SeparateVelocity.x) || std::isinf(hit.m_v3SeparateVelocity.x)) { hit.m_v3SeparateVelocity.x = 0.0f; }
				if (std::isnan(hit.m_v3SeparateVelocity.y) || std::isinf(hit.m_v3SeparateVelocity.y)) { hit.m_v3SeparateVelocity.y = 0.0f; }
				if (std::isnan(hit.m_v3SeparateVelocity.z) || std::isinf(hit.m_v3SeparateVelocity.z)) { hit.m_v3SeparateVelocity.z = 0.0f; }
			}

			pHits->m_hits[pHits->m_nNumHits] = hit;
			pHits->m_nNumHits++;
		}

		return;
	}
	
	void update_simplex3(Point* a, Point* b, Point* c, Point* d, int* simp_dim, glm::vec3 *pSearch_dir)
	{
		/* Required winding order:
		   //  b
		   //  | \
		   //  |   \
		   //  |    a
		   //  |   /
		   //  | /
		   //  c
		   */
		glm::vec3 n = glm::cross(b->p - a->p, c->p - a->p); //triangle's normal
		glm::vec3 AO = -a->p; //direction to origin

		//Determine which feature is closest to origin, make that the new simplex

		*simp_dim = 2;
		if (glm::dot(glm::cross(b->p - a->p, n), AO) > 0) { //Closest to edge AB
			*c = *a;
			//simp_dim = 2;
			*pSearch_dir = glm::cross(glm::cross(b->p - a->p, AO), b->p - a->p);
			return;
		}
		if (glm::dot(glm::cross(n, c->p - a->p), AO) > 0) { //Closest to edge AC
			*b = *a;
			//simp_dim = 2;
			*pSearch_dir = glm::cross(glm::cross(c->p - a->p, AO), c->p - a->p);
			return;
		}

		*simp_dim = 3;
		if (glm::dot(n, AO) > 0) { //Above triangle
			*d = *c;
			*c = *b;
			*b = *a;
			//simp_dim = 3;
			*pSearch_dir = n;
			return;
		}
		//else //Below triangle
		*d = *b;
		*b = *a;
		//simp_dim = 3;
		*pSearch_dir = -n;
		return;
	}

	bool update_simplex4(Point* a, Point* b, Point* c, Point* d, int* simp_dim, glm::vec3 *pSearch_dir)
	{
		// a is peak/tip of pyramid, BCD is the base (counterclockwise winding order)
		//We know a priori that origin is above BCD and below a

		//Get normals of three new faces
		glm::vec3 ABC = glm::cross(b->p - a->p, c->p - a->p);
		glm::vec3 ACD = glm::cross(c->p - a->p, d->p - a->p);
		glm::vec3 ADB = glm::cross(d->p - a->p, b->p - a->p);

		glm::vec3 AO = -a->p; //dir to origin
		*simp_dim = 3; //hoisting this just cause

		//Plane-test origin with 3 faces
		/*
		// Note: Kind of primitive approach used here; If origin is in front of a face, just use it as the new simplex.
		// We just go through the faces sequentially and exit at the first one which satisfies dot product. Not sure this
		// is optimal or if edges should be considered as possible simplices? Thinking this through in my head I feel like
		// this method is good enough. Makes no difference for AABBS, should test with more complex colliders.
		*/
		if (glm::dot(ABC, AO) > 0) { //In front of ABC
			*d = *c;
			*c = *b;
			*b = *a;
			*pSearch_dir = ABC;
			return false;
		}

		if (glm::dot(ACD, AO) > 0) { //In front of ACD
			*b = *a;
			*pSearch_dir = ACD;
			return false;
		}
		if (glm::dot(ADB, AO) > 0) { //In front of ADB
			*c = *d;
			*d = *b;
			*b = *a;
			*pSearch_dir = ADB;
			return false;
		}

		//else inside tetrahedron; enclosed!
		return true;
	}
	
	// GJK
	bool SearchHits_ConvexConvex(structHits *pHits, structRigidBody structRigidBody1/*only dynamic*/, structRigidBody structRigidBody2/*static or dynamic*/, glm::mat4 T1, glm::mat4 T2, structBVHNodeTriangleOffset offset1, structBVHNodeTriangleOffset offset2, structBVHNodeTriangle* pListBVHNodeTriangles1, structBVHNodeTriangle* pListBVHNodeTriangles2)
	{
		//Vector3* mtv;

		glm::vec3 coll1Pos = ToVector3(structRigidBody1.m_v3Position);//coll1->GetTransform().GetPosition();
		glm::vec3 coll2Pos = ToVector3(structRigidBody2.m_v3Position);//coll2->GetTransform().GetPosition();

		if (coll1Pos == coll2Pos) 
		{
			return false;
			//coll2Pos.x += 0.001f;
		}

		Point a, b, c, d; //Simplex: just a set of points (a is always most recently added)
		glm::vec3 search_dir = coll1Pos - coll2Pos; //initial search direction between colliders

		 //Get initial point for simplex
		//Point c;
		CalculateSearchPoint(&c, search_dir, structRigidBody1, structRigidBody2, T1, T2, offset1, offset2, pListBVHNodeTriangles1, pListBVHNodeTriangles2);
		search_dir = -c.p; //search in direction of origin

		//Get second point for a line segment simplex
		//Point b;
		CalculateSearchPoint(&b, search_dir, structRigidBody1, structRigidBody2, T1, T2, offset1, offset2, pListBVHNodeTriangles1, pListBVHNodeTriangles2);

		if (glm::dot(b.p, search_dir) < 0) {
			return false;
		}//we didn't reach the origin, won't enclose it

		search_dir = glm::cross(glm::cross(c.p - b.p, -b.p), c.p - b.p); //search perpendicular to line segment towards origin
		if (search_dir == glm::vec3(0, 0, 0)) { //origin is on this line segment
			return false;
			////Apparently any normal search vector will do?
			//search_dir = glm::cross(c.p - b.p, glm::vec3(1, 0, 0)); //normal with x-axis
			//if (search_dir == glm::vec3(0, 0, 0))
			//	search_dir = glm::cross(c.p - b.p, glm::vec3(0, 0, -1)); //normal with z-axis
		}
		int simp_dim = 2; //simplex dimension

		for (int iterations = 0; iterations < GJK_MAX_NUM_ITERATIONS; iterations++)
		{
			//Point a;
			CalculateSearchPoint(&a, search_dir, structRigidBody1, structRigidBody2, T1, T2, offset1, offset2, pListBVHNodeTriangles1, pListBVHNodeTriangles2);

			if (glm::dot(a.p, search_dir) < 0) {
				return false;
			}//we didn't reach the origin, won't enclose it

			simp_dim++;
			if (simp_dim == 3) {
				update_simplex3(&a, &b, &c, &d, &simp_dim, &search_dir);
			}
			else if (update_simplex4(&a, &b, &c, &d, &simp_dim, &search_dir)) {
				EPA(pHits, a, b, c, d, structRigidBody1, structRigidBody2, T1, T2, offset1, offset2, pListBVHNodeTriangles1, pListBVHNodeTriangles2);
				return true;
			}
		}//endfor

		return false;
	}

	bool SearchHits_ConvexTriangle(structHits* pHits, structRigidBody structRigidBody1, structTriangle triangle, glm::mat4 T1, structBVHNodeTriangleOffset offset1, structBVHNodeTriangle* pListBVHNodeTriangles) 
	{
		//Vector3* mtv;

		glm::vec3 coll1Pos = ToVector3(structRigidBody1.m_v3Position);//coll1->GetTransform().GetPosition();
		glm::vec3 coll2Pos = glm::vec3(0,0,0);//coll2->GetTransform().GetPosition();

		Point a, b, c, d; //Simplex: just a set of points (a is always most recently added)
		glm::vec3 search_dir = coll1Pos - coll2Pos; //initial search direction between colliders

		 //Get initial point for simplex
		//Point c;
		CalculateSearchPoint(&c, search_dir, structRigidBody1, triangle, T1, offset1, pListBVHNodeTriangles);
		search_dir = -c.p; //search in direction of origin

		//Get second point for a line segment simplex
		//Point b;
		CalculateSearchPoint(&b, search_dir, structRigidBody1, triangle, T1, offset1, pListBVHNodeTriangles);

		if (glm::dot(b.p, search_dir) < 0) {
			return false;
		}//we didn't reach the origin, won't enclose it

		search_dir = glm::cross(glm::cross(c.p - b.p, -b.p), c.p - b.p); //search perpendicular to line segment towards origin
		if (search_dir == glm::vec3(0, 0, 0)) { //origin is on this line segment
			return false;
			////Apparently any normal search vector will do?
			//search_dir = glm::cross(c.p - b.p, glm::vec3(1, 0, 0)); //normal with x-axis
			//if (search_dir == glm::vec3(0, 0, 0))
			//	search_dir = glm::cross(c.p - b.p, glm::vec3(0, 0, -1)); //normal with z-axis
		}
		int simp_dim = 2; //simplex dimension

		for (int iterations = 0; iterations < GJK_MAX_NUM_ITERATIONS; iterations++)
		{
			//Point a;
			CalculateSearchPoint(&a, search_dir, structRigidBody1, triangle, T1, offset1, pListBVHNodeTriangles);

			if (glm::dot(a.p, search_dir) < 0) {
				return false;
			}//we didn't reach the origin, won't enclose it

			simp_dim++;
			if (simp_dim == 3) {
				update_simplex3(&a, &b, &c, &d, &simp_dim, &search_dir);
			}
			else if (update_simplex4(&a, &b, &c, &d, &simp_dim, &search_dir)) {
				EPA(pHits, a, b, c, d, structRigidBody1, triangle, T1, offset1, pListBVHNodeTriangles);
				return true;
			}
		}//endfor

		return false;
	}

	// hordo - palya
	void SearchHits_ConvexConcave(structHits* pHits, structRigidBody structRigidBody1/*only dynamic*/, structRigidBody structRigidBody2/*static or dynamic*/, glm::mat4 T1, glm::mat4 T2, structBVHNodeTriangleOffset offset1, structBVHNodeTriangleOffset offset2, structBVHNodeTriangle* pListBVHNodeTriangles) 
	{
		//structBVHNodeTriangle structNodeOrTriangle2 = pListBVHNodeTriangles[nId2];
		//structTriangle triangle2 = structNodeOrTriangle2.m_Triangle;
		//glm::vec3 tri2_a = glm::vec3(T2 * glm::vec4(ToVector3(triangle2.m_v3PosA), 1.0f));
		//glm::vec3 tri2_b = glm::vec3(T2 * glm::vec4(ToVector3(triangle2.m_v3PosB), 1.0f));
		//glm::vec3 tri2_c = glm::vec3(T2 * glm::vec4(ToVector3(triangle2.m_v3PosC), 1.0f));
		//glm::vec3 tri2_n = glm::vec3(T2 * glm::vec4(ToVector3(triangle2.m_v3Normal), 0.0f));
		
		//structNodeOrTriangle2.m_BBox = TransformBBox(T2, structNodeOrTriangle2.m_BBox);
		
		int nTop = -1;
		int arrStack[64];
		
		nTop++;
		arrStack[nTop] = offset2.m_nOffset;
		
		int nId2;
		structBVHNodeTriangle structNodeOrTriangle2;
		
		while (nTop > -1)
		{
			if (pHits->m_nNumHits >= MAX_HITS_OBJECT_OBJECT) 
			{
				return;
			}

			nId2 = arrStack[nTop];
			nTop--;
		
			structNodeOrTriangle2 = pListBVHNodeTriangles[nId2];
			//structNodeOrTriangle.m_BBox = TransformBBox(T2, structNodeOrTriangle.m_BBox);
		
			if (true == IsLeaf(structNodeOrTriangle2)) 
			{
				if (true == IsCollide(structRigidBody1.m_BBox, structNodeOrTriangle2.m_BBox))
				{
					structTriangle triangle = structNodeOrTriangle2.m_Triangle;

					SearchHits_ConvexTriangle(pHits, structRigidBody1, triangle, T1, offset1, pListBVHNodeTriangles);
				}
			}
			else 
			{
				if (true == IsCollide(structRigidBody1.m_BBox, structNodeOrTriangle2.m_BBox))
				{
					if (structNodeOrTriangle2.m_nLeft != -1)
					{
						nTop++;
						arrStack[nTop] = structNodeOrTriangle2.m_nLeft;
					}
		
					if (structNodeOrTriangle2.m_nRight != -1)
					{
						nTop++;
						arrStack[nTop] = structNodeOrTriangle2.m_nRight;
					}
				}
			}
		}
	}

	bool Physics::CollisionDetectionAndResponse(float dt) 
	{
		//cl_int err = 0;
		//
		//int32_t nCount = (int32_t)m_listRigidBodies.size();
		//size_t nLocal = m_nCollisionDetection_Local;
		//size_t nGlobal = ((nCount + nLocal - 1) / nLocal) * nLocal;
		//
		//// calc
		//err |= clSetKernelArg(m_kernelCollisionDetection, 0, sizeof(cl_mem), &m_clmem_inoutRigidBodies);
		//err |= clSetKernelArg(m_kernelCollisionDetection, 1, sizeof(int32_t), &nCount);
		//err |= clSetKernelArg(m_kernelCollisionDetection, 2, sizeof(cl_mem), &m_clmem_inoutBVHObjects);
		//err |= clSetKernelArg(m_kernelCollisionDetection, 3, sizeof(cl_mem), &m_clmem_inBVHNodeTriangles);
		//err |= clSetKernelArg(m_kernelCollisionDetection, 4, sizeof(cl_mem), &m_clmem_inBVHNodeTrianglesOffsets);
		//err |= clSetKernelArg(m_kernelCollisionDetection, 5, sizeof(cl_mem), &m_clmem_inoutHits);
		//err |= clSetKernelArg(m_kernelCollisionDetection, 6, sizeof(cl_mem), &m_clmem_inoutIsCollisionResponse);
		//
		//err |= clEnqueueNDRangeKernel(m_command_queue, m_kernelCollisionDetection, 1, NULL, &nGlobal, &nLocal, 0, NULL, NULL);
		//clFinish(m_command_queue);
		//
		//err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutHits, CL_TRUE, 0, sizeof(structHits) * m_listHits.size(), &(m_listHits[0]), 0, NULL, NULL);
		//err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutIsCollisionResponse, CL_TRUE, 0, sizeof(int32_t) * m_listIsCollisionResponse.size(), &(m_listIsCollisionResponse[0]), 0, NULL, NULL);
		//
		//if (err != CL_SUCCESS)
		//{
		//	return false;
		//}
		//
		//return true;















		// DEBUG: EZEK MAJD NEM KELLENEK, CSAK MOST A CPU-NAK
		cl_int err = 0;
		std::vector<structBVHObject> inoutBVHObjects;
		inoutBVHObjects.resize(m_listBVHObjects.size());
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutBVHObjects, CL_TRUE, 0, sizeof(structBVHObject) * m_listBVHObjects.size(), &(inoutBVHObjects[0]), 0, NULL, NULL);
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), &(m_listRigidBodies[0]), 0, NULL, NULL);
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutHits, CL_TRUE, 0, sizeof(structHits) * m_listHits.size(), &(m_listHits[0]), 0, NULL, NULL);
		err |= clEnqueueReadBuffer(m_command_queue, m_clmem_inoutIsCollisionResponse, CL_TRUE, 0, sizeof(int32_t) * m_listIsCollisionResponse.size(), &(m_listIsCollisionResponse[0]), 0, NULL, NULL);
		
		if (err != CL_SUCCESS)
		{
			return false;
		}
		
		// hits-ek törlése
		for (int32_t i = 0; i < m_listHits.size(); i++)
		{
			// 1 - EZ MEGY MAJD AZ OPENCL FUGGVENYBE
			structHits hits = m_listHits[i];
			hits.m_nNumHits = 0;
			m_listHits[i] = hits;
		
			m_listIsCollisionResponse[i] = 0;
		}
		
		// ütközés keresés
		//for (int32_t id1 = 0; id1 < m_listRigidBodies.size(); id1++)
		std::for_each(std::execution::par_unseq, std::begin(m_listRigidBodies), std::end(m_listRigidBodies), [&](structRigidBody structRigidBody1)
		{
			// 2 - EZ MEGY MAJD AZ OPENCL FUGGVENYBE
			//structRigidBody structRigidBody1 = m_listRigidBodies.at(id1);
			int32_t id1 = structRigidBody1.m_nRigidBodyId;
		
			// isEnabled == false, akkor nem kell
			if (0 == structRigidBody1.m_nIsEnabled) 
			{
				//continue;
				return;
			}
		
			// ha static, akkor nem kell
			if (structRigidBody1.m_nIsConvex != 1)
			{
				//continue;
				return;
			}
		
			structHits allHits;
			allHits.m_nNumHits = 0;
			m_listHits[id1] = allHits;
		
			// TRANSFORM 1
			glm::vec3 v3Rotate1 = ToVector3(structRigidBody1.m_v3Rotate);
			glm::vec3 v3Position1 = ToVector3(structRigidBody1.m_v3Position);
			glm::mat4 T1 = glm::translate(glm::mat4(1.0f), v3Position1) * glm::eulerAngleXYZ(v3Rotate1.x, v3Rotate1.y, v3Rotate1.z);
		
			int nTop = -1;
			int arrStack[64];
		
			nTop++;
			arrStack[nTop] = 0;
		
			int nOtherId;
			structBVHObject structBVHObject;
		
			while (nTop > -1)
			{
				nOtherId = arrStack[nTop];
				nTop--;
		
				if (allHits.m_nNumHits >= MAX_HITS)
				{
					continue;
				}
		
				structBVHObject = inoutBVHObjects[nOtherId];
		
				if (structBVHObject.m_nLeft == -1 && structBVHObject.m_nRight == -1)
				{
					// saját magával nem kell ütközésvizsgálatot csinálni
					int id2 = structBVHObject.m_nRigidBodyId;
					structRigidBody structRigidBody2 = m_listRigidBodies.at(id2);
		
					if ( (id1 /*<*/!= id2) || (structRigidBody2.m_nIsConvex != 1))
					{
						structBBox bboxRigidBody2 = structRigidBody2.m_BBox;
		
						if (true == IsCollide(structRigidBody1.m_BBox, bboxRigidBody2))
						{
							// TRANSFORM 2
							glm::vec3 v3Rotate2 = ToVector3(structRigidBody2.m_v3Rotate);
							glm::vec3 v3Position2 = ToVector3(structRigidBody2.m_v3Position);
							glm::mat4 T2 = glm::translate(glm::mat4(1.0f), v3Position2) * glm::eulerAngleXYZ(v3Rotate2.x, v3Rotate2.y, v3Rotate2.z);
		
							structHits hits;
							hits.m_nNumHits = 0;
							
							if (1 == structRigidBody1.m_nIsConvex && 1 != structRigidBody2.m_nIsConvex)
							{
								SearchHits_ConvexConcave(&hits, structRigidBody1, structRigidBody2, T1, T2, m_listBVHNodeTrianglesOffsets[structRigidBody1.m_nTriMeshId], m_listBVHNodeTrianglesOffsets[structRigidBody2.m_nTriMeshId], &m_listBVHNodeTriangles[0]);
							}
							else if (1 == structRigidBody1.m_nIsConvex && 1 == structRigidBody2.m_nIsConvex)
							{
								SearchHits_ConvexConvex(&hits, structRigidBody1, structRigidBody2, T1, T2, m_listBVHNodeTrianglesOffsets[structRigidBody1.m_nTriMeshId], m_listBVHNodeTrianglesOffsets[structRigidBody2.m_nTriMeshId], &m_listBVHNodeTriangles[0], &m_listBVHNodeTriangles[0]);
							}

							for (int i = 0; i < hits.m_nNumHits; i++) 
							{
								hits.m_hits[i].m_nRigidBodyAId = id1;
								hits.m_hits[i].m_nRigidBodyBId = id2;
							}

							SeparateObjects(hits, dt);
							CollisionResponse(hits, dt);

							//for (int i = 0; i < hits.m_nNumHits; i++)
							//{
							//	if (allHits.m_nNumHits >= MAX_HITS)
							//	{
							//		continue;
							//	}
							//
							//	//structHit hit = hits.m_hits[i];
							//	//if (isnan(hit.m_v3Normal.x) || isnan(hit.m_v3Normal.y) || isnan(hit.m_v3Normal.z))
							//	//{
							//	//	std::cout << "normal" << std::endl;
							//	//}
							//	//if (isnan(hit.m_v3HitPointInWorldA.x) || isnan(hit.m_v3HitPointInWorldA.y) || isnan(hit.m_v3HitPointInWorldA.z))
							//	//{
							//	//	std::cout << "hit a" << std::endl;
							//	//}
							//	//if (isnan(hit.m_v3HitPointInWorldB.x) || isnan(hit.m_v3HitPointInWorldB.y) || isnan(hit.m_v3HitPointInWorldB.z))
							//	//{
							//	//	std::cout << "hit b" << std::endl;
							//	//}
							//
							//	allHits.m_hits[allHits.m_nNumHits] = hits.m_hits[i];
							//
							//	allHits.m_hits[allHits.m_nNumHits].m_nRigidBodyAId = id1;
							//	allHits.m_hits[allHits.m_nNumHits].m_nRigidBodyBId = id2;
							//
							//	allHits.m_nNumHits++;
							//}
		
						}
					}
				}
				else
				{
					if (true == IsCollide(structRigidBody1.m_BBox, structBVHObject.m_BBox))
					{
						if (structBVHObject.m_nLeft != -1)
						{
							nTop++;
							arrStack[nTop] = structBVHObject.m_nLeft;
						}
		
						if (structBVHObject.m_nRight != -1)
						{
							nTop++;
							arrStack[nTop] = structBVHObject.m_nRight;
						}
					}
				}
		
			}
		
			m_listHits[id1] = allHits;
		
			if (allHits.m_nNumHits == 0) // nincs utkozes
			{
				m_listIsCollisionResponse[id1] = 0;
			}
			else // van utkozes => szettolas
			{
				m_listIsCollisionResponse[id1] = 1;
			}
		});
		
		// DEBUG EZ MAJD NEM KELL
		//cl_int err = 0;
		err |= clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), m_listRigidBodies.data(), 0, NULL, NULL);
		err |= clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutHits, CL_TRUE, 0, sizeof(structHits) * m_listHits.size(), m_listHits.data(), 0, NULL, NULL);
		if (err != CL_SUCCESS) 
		{ 
			return false; 
		}

		return true;
	}

	std::vector < structHits >* Physics::GetHits() 
	{
		return &m_listHits;
	}

	bool IsEqualNormals(structVector3 v3Normal1, structVector3 v3Normal2)
	{
		if (glm::angle(ToVector3(v3Normal1), ToVector3(v3Normal2)) < (3.141592f / 180.0f)) // 1 degree 
		{
			return true;
		}

		return false;
	}

	bool IsContainsNormal(structHits separateDirs, structVector3 v3Normal)
	{
		for (int i = 0; i < separateDirs.m_nNumHits; i++)
		{
			if (true == IsEqualNormals(separateDirs.m_hits[i].m_v3Normal, v3Normal)) 
			{
				return true;
			}
		}

		return false;
	}

	glm::vec3 GetPointVelocity(structRigidBody rigidBodyA, glm::vec3 v3Point)
	{
		return (ToVector3(rigidBodyA.m_v3LinearVelocity) + glm::cross(ToVector3(rigidBodyA.m_v3AngularVelocity), v3Point));
	}

	void Physics::SeparateObjects(structHits hits, float dt) 
	{
		if (0 == hits.m_nNumHits)
		{
			//continue;
			return;
		}

		for (int i = 0; i < hits.m_nNumHits; i++)
		{
			structHit hit = hits.m_hits[i];

			structRigidBody rigidBodyA = m_listRigidBodies[hit.m_nRigidBodyAId];
			structRigidBody rigidBodyB = m_listRigidBodies[hit.m_nRigidBodyBId];

			if (1 == rigidBodyB.m_nIsConvex) 
			{
				// separate
				glm::vec3 v3PositionA = ToVector3(rigidBodyA.m_v3Position);
				glm::vec3 v3SeparateA = ToVector3(hit.m_v3SeparateVelocity) * 2.5f * dt;
				glm::vec3 v3NewPosA = v3PositionA + (v3SeparateA);
				m_listRigidBodies[hit.m_nRigidBodyAId].m_v3Position = ToVector3(v3NewPosA);

				glm::vec3 v3PositionB = ToVector3(rigidBodyB.m_v3Position);
				glm::vec3 v3SeparateB = -ToVector3(hit.m_v3SeparateVelocity) * 2.5f * dt;
				glm::vec3 v3NewPosB = v3PositionB + (v3SeparateB);
				m_listRigidBodies[hit.m_nRigidBodyBId].m_v3Position = ToVector3(v3NewPosB);
			}
			else 
			{
				// separate
				glm::vec3 v3PositionA = ToVector3(rigidBodyA.m_v3Position);
				glm::vec3 v3SeparateA = ToVector3(hit.m_v3SeparateVelocity) * 5.0f * dt;
				glm::vec3 v3NewPosA = v3PositionA + (v3SeparateA);
				m_listRigidBodies[hit.m_nRigidBodyAId].m_v3Position = ToVector3(v3NewPosA);
			}
		}
	}

	void Physics::CollisionResponse(structHits hits, float dt)
	{
		//for (int32_t id = 0; id < m_listHits.size(); id++)
		//std::for_each(std::execution::par_unseq, std::begin(m_listHits), std::end(m_listHits), [&](structHits hits)
		{
			// EZ MEGY AZ OPENCL FUGGVENYBE
			//structHits hits = m_listHits[id];
			
			if (0 == hits.m_nNumHits) 
			{
				//continue;
				return;
			}

			for (int i = 0; i < hits.m_nNumHits; i++)
			{
				structHit hit = hits.m_hits[i];

				structRigidBody rigidBodyA = m_listRigidBodies[hit.m_nRigidBodyAId];
				glm::vec3 v3LinearVelocityA = ToVector3(rigidBodyA.m_v3LinearVelocity);
				glm::vec3 v3AngularVelocityA = ToVector3(rigidBodyA.m_v3AngularVelocity);

				structRigidBody rigidBodyB = m_listRigidBodies[hit.m_nRigidBodyBId];
				glm::vec3 v3LinearVelocityB = ToVector3(rigidBodyB.m_v3LinearVelocity);
				glm::vec3 v3AngularVelocityB = ToVector3(rigidBodyB.m_v3AngularVelocity);

				if (1 == rigidBodyB.m_nIsConvex)
				{
					// calc contact velocity
					glm::vec3 rA = ToVector3(hit.m_v3HitPointInWorldA) - ToVector3(m_listRigidBodies[hit.m_nRigidBodyAId].m_v3Position);
					glm::vec3 rB = ToVector3(hit.m_v3HitPointInWorldB) - ToVector3(m_listRigidBodies[hit.m_nRigidBodyBId].m_v3Position);
					glm::vec3 v3ARelVelocity = GetPointVelocity(rigidBodyA, rA);
					glm::vec3 v3BRelVelocity = GetPointVelocity(rigidBodyB, rB);
					glm::vec3 v3RelVelocity = v3ARelVelocity - v3BRelVelocity;
					float fProjVelocity = glm::dot(v3RelVelocity, ToVector3(hit.m_v3Normal));

					//if (fProjVelocity >= 0.0f)
					//{
					//	continue;
					//}

					float fRestitution = rigidBodyA.m_fRestitution * rigidBodyB.m_fRestitution;
					if (isnan(fRestitution)) { fRestitution = 0.0f; }

					// calc inertia
					float fNominator = -(1.0f + fRestitution) * fProjVelocity;

					glm::vec3 v3Denom1 = (ToVector3(hit.m_v3Normal) / rigidBodyA.m_fMass) + glm::cross(glm::cross(rA, ToVector3(hit.m_v3Normal)), rA) / rigidBodyA.m_fMass;
					glm::vec3 v3Denom2 = (ToVector3(hit.m_v3Normal) / rigidBodyB.m_fMass) + glm::cross(glm::cross(rB, ToVector3(hit.m_v3Normal)), rB) / rigidBodyB.m_fMass;
					float fDenominator = glm::dot(v3Denom1 + v3Denom2, ToVector3(hit.m_v3Normal));

					float J = fNominator / fDenominator;
					J /= (float)hits.m_nNumHits;

					J *= 0.5f;

					// apply velocity
					v3LinearVelocityA += (J * ToVector3(hit.m_v3Normal)) / (rigidBodyA.m_fMass * 5.0f);
					v3AngularVelocityA += glm::cross(rA, J * ToVector3(hit.m_v3Normal)) / (rigidBodyA.m_fMass * 5.0f);

					v3LinearVelocityB -= (J * ToVector3(hit.m_v3Normal)) / (rigidBodyB.m_fMass * 5.0f);
					v3AngularVelocityB -= glm::cross(rB, J * ToVector3(hit.m_v3Normal)) / (rigidBodyB.m_fMass * 5.0f);

					m_listRigidBodies[hit.m_nRigidBodyAId].m_v3LinearVelocity = ToVector3(v3LinearVelocityA);
					m_listRigidBodies[hit.m_nRigidBodyAId].m_v3AngularVelocity = ToVector3(v3AngularVelocityA);

					m_listRigidBodies[hit.m_nRigidBodyBId].m_v3LinearVelocity = ToVector3(v3LinearVelocityB);
					m_listRigidBodies[hit.m_nRigidBodyBId].m_v3AngularVelocity = ToVector3(v3AngularVelocityB);
				}
				else
				{
					// calc contact velocity
					glm::vec3 rA = ToVector3(hit.m_v3HitPointInWorldA) - ToVector3(m_listRigidBodies[hit.m_nRigidBodyAId].m_v3Position);
					glm::vec3 v3RelVelocity = GetPointVelocity(rigidBodyA, rA);
					float fProjVelocity = glm::dot(v3RelVelocity, ToVector3(hit.m_v3Normal));

					if (fProjVelocity >= 0.0f)
					{
						continue;
					}

					// calc inertia
					float fNominator = -(1.0f + rigidBodyA.m_fRestitution) * fProjVelocity;
					glm::vec3 v3Denom = (ToVector3(hit.m_v3Normal) / rigidBodyA.m_fMass) + glm::cross(glm::cross(rA, ToVector3(hit.m_v3Normal)), rA) / rigidBodyA.m_fMass;
					float fDenominator = glm::dot(v3Denom, ToVector3(hit.m_v3Normal));

					float J = fNominator / fDenominator;
					J /= (float)hits.m_nNumHits;

					// apply velocity
					v3LinearVelocityA += (J * ToVector3(hit.m_v3Normal)) / (rigidBodyA.m_fMass * 5.0f);
					v3AngularVelocityA += glm::cross(rA, J * ToVector3(hit.m_v3Normal)) / (rigidBodyA.m_fMass * 5.0f);

					m_listRigidBodies[hit.m_nRigidBodyAId].m_v3LinearVelocity = ToVector3(v3LinearVelocityA);
					m_listRigidBodies[hit.m_nRigidBodyAId].m_v3AngularVelocity = ToVector3(v3AngularVelocityA);
				}

			}
		}/*);*/
		
		// DEBUG EZ MAJD NEM KELL
		//cl_int err = 0;
		//err |= clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutRigidBodies, CL_TRUE, 0, sizeof(structRigidBody) * m_listRigidBodies.size(), m_listRigidBodies.data(), 0, NULL, NULL);
		//err |= clEnqueueWriteBuffer(m_command_queue, m_clmem_inoutHits, CL_TRUE, 0, sizeof(structHits) * m_listHits.size(), m_listHits.data(), 0, NULL, NULL);
		//if (err != CL_SUCCESS) 
		//{ 
		//	return; 
		//}
	}
}