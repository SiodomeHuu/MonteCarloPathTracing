#include "bvhtest.h"

#include "auxiliary.h"

#include <deque>
#include <unordered_set>
#include "config.h"

#include "tiny_obj_loader.h"


#include "BVH/hlbvh.h"
#include "BVH/treeletBVH.h"
#include "BVH/simpleQuad.h"
#include "BVH/sahbvh.h"

#include "treeletTemp.h"

#include <iostream>

using namespace MCPT;
using namespace MCPT::Auxiliary;


namespace {
	bool intersectBox(const Ray& r, const BoundingBox& box, float tmin, float& tans) {
		cl_float4 off1 = (box.bbmin - r.origin) / r.direction;
		cl_float4 off2 = (box.bbmax - r.origin) / r.direction;

		cl_float4 tempMin = min(off1, off2);
		cl_float4 tempMax = max(off1, off2);
		cl_float tnear = std::max(std::max(tempMin.x, tempMin.y), tempMin.z);
		cl_float tfar = std::min(std::min(tempMax.x, tempMax.y), tempMax.z);
		if (tfar < tnear || tfar < tmin) return false;
		tans = tnear;
		return true;
	}

	bool intersectPlane(const cl_float4& normal, Ray* ray, float* tans, float tmin) {
		float dotans = dot(ray->direction, normal);

		if (dotans == 0) return false;
		float t = (normal.w - dot(normal, ray->origin)) / dotans;

		if (t <= tmin) return false;
		*tans = t;
		return true;
	}


	std::vector<Triangle> loadObj(const std::string& directory, const std::string& objname) {
		std::vector<Triangle> ansTrg;

		tinyobj::attrib_t attrib;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;

		tinyobj::LoadObj(&attrib, &shapes, &materials, nullptr, nullptr, (directory + objname).c_str(), directory.c_str());

		auto packFloat = [&](size_t* tid) -> Triangle {
			Triangle ans = { 0 };
			for (size_t i = 0; i < 3; ++i) {
				for (size_t j = 0; j < 3; ++j) {
					ans.v[i].s[j] = attrib.vertices[tid[i] * 3 + j];
				}
			}
			return ans;
		};
		auto packTriangle = [&](const tinyobj::shape_t& x) -> void {
			for (size_t i = 0; i < x.mesh.num_face_vertices.size(); ++i) {
				size_t offset = i * 3;

				size_t tid[3];
				tid[0] = x.mesh.indices[offset].vertex_index;
				tid[1] = x.mesh.indices[offset + 1].vertex_index;
				tid[2] = x.mesh.indices[offset + 2].vertex_index;

				ansTrg.push_back(packFloat(tid));
			}
		};

		for (auto& shape : shapes) {
			packTriangle(shape);
		}

		return ansTrg;
	}

}



namespace {

	cl::Program prog;
	cl::Kernel kernel;
	cl::Kernel quadKernel;
	cl::Kernel multi_kernel;

	cl::Kernel lcvKernel;
	cl::Kernel quadLCVKernel;
	cl::Kernel lcvKernel_Multi;
	bool programInited = false;

	cl::Program rayGenProg;
	cl::Kernel rayGenerator;
	bool rayGenInited = false;
}







namespace MCPT::BVH::TEST {

	
	
	std::vector<float> epoTValue;
	std::vector<float> bgData;

	double EPO(const std::vector<BVHNode>& node, const std::vector<Triangle>& triangles) {
		float EPO = 0.0f;
		std::unordered_set<size_t> ancestor;

		auto GETGEO = [&](size_t leafID) -> const Triangle& {
			return triangles[node[leafID].left];
		};
		
		auto POINTINBOX = [](const cl_float4& p, const BoundingBox& box) -> bool {
			if ((p.x >= box.bbmin.x && p.x <= box.bbmax.x)
				&& (p.y >= box.bbmin.y && p.y <= box.bbmax.y)
				&& (p.z >= box.bbmin.z && p.z <= box.bbmax.z))
			{
				return true;
			}
			return false;
		};
		auto TRIANGLEAREA = [](const Triangle& geo) -> float {
			return len(cross(geo.v[1] - geo.v[0], geo.v[2] - geo.v[0])) / 2;
		};

		auto ROUNDTR = [](std::vector<cl_float4>& points, int axis, float pos, int sign) -> void {
			std::vector<cl_float4> ans;
			std::vector<int> inside;

			float tans;

			if (sign > 0) {
				for (auto& p : points) {
					inside.push_back(p.s[axis] >= pos);
				}
			}
			else {
				for (auto& p : points) {
					inside.push_back(p.s[axis] <= pos);
				}
			}
			if (points.empty()) return;
			for (int i = 0; i < points.size(); ++i) {
				auto i_1 = (i + 1 == points.size()) ? (0) : (i + 1);
				if (!inside[i] && !inside[i_1]) {
					continue;
				}
				else if (inside[i] && inside[i_1]) {
					ans.push_back(points[i]);
					continue;
				}
				else {
					if (inside[i]) {
						ans.push_back(points[i]);
					}
					cl_float4 direction = points[i_1] - points[i];
					tans = (pos - points[i].s[axis]) / direction.s[axis];
					ans.push_back(points[i] + tans * direction);
				}
			}

			points.swap(ans);
		};
		auto pArea = [](const std::vector<cl_float4>& points) -> float {
			float ans = 0.0f;
			int sz = points.size();
			if (sz < 2) {
				return ans;
			}
			for (int i = 1; i < sz - 1; ++i) {
				auto x1 = points[i] - points[0];
				auto x2 = points[i + 1] - points[0];
				ans += len(cross(x1, x2)) / 2;
			}
			return ans;
		};
		auto INTERSECTAREA = [&](const Triangle& geo, BoundingBox& box) -> float {
			Triangle tr = geo;
			tr.v[0].w = 1;
			tr.v[1].w = 1;
			tr.v[2].w = 1;

			bool inside[3];
			inside[0] = POINTINBOX(tr.v[0], box);
			inside[1] = POINTINBOX(tr.v[1], box);
			inside[2] = POINTINBOX(tr.v[2], box);

			if (inside[0] && inside[1] && inside[2]) {
				return TRIANGLEAREA(tr);
			}

			std::vector< cl_float4 > points = { tr.v[0],tr.v[1],tr.v[2] };
			

			ROUNDTR(points, 0, box.bbmin.x, 1);
			ROUNDTR(points, 1, box.bbmin.y, 1);
			ROUNDTR(points, 2, box.bbmin.z, 1);

			ROUNDTR(points, 0, box.bbmax.x, -1);
			ROUNDTR(points, 1, box.bbmax.y, -1);
			ROUNDTR(points, 2, box.bbmax.z, -1);

			return pArea(points);
		};

		std::deque<size_t> toBeDone;
		auto TopDown = [&](size_t leafID, size_t rootID) -> void {
			auto& geo = GETGEO(leafID);
			auto box = BoundingBox({ node[rootID].bbmin,node[rootID].bbmax });

			if (ancestor.find(rootID) == ancestor.end()) {
				float epovalue = INTERSECTAREA(geo, box);
				if (epovalue > 0) {

					EPO += epovalue * ((rootID >= (node.size() >> 1)) ? Ctri : Cinn); // / AREA2(box);   // / areas[rootID];

					if (node[rootID].left != node[rootID].right) {
						toBeDone.push_back(node[rootID].left);
						toBeDone.push_back(node[rootID].right);
					}
				}
			}
			else {
				if (node[rootID].left != node[rootID].right) {
					toBeDone.push_back(node[rootID].left);
					toBeDone.push_back(node[rootID].right);
				}
			}
		};

		float percent = 0.05f;
		float step = 0.05f;
		int sz = (node.size() >> 1);

		float prevEPO = 0.0f;
		for (size_t i = sz; i < node.size(); ++i) {
			ancestor.insert(i);

			size_t j = i;
			while (node[j].parent != -1) {
				ancestor.insert(node[j].parent);
				j = node[j].parent;
			}

			toBeDone.push_back(0);
			while (!toBeDone.empty()) {
				auto front = toBeDone.front();
				toBeDone.pop_front();
				TopDown(i, front);
			}

			epoTValue.push_back(EPO - prevEPO);
			prevEPO = EPO;

			ancestor.clear();
			
			if ((i - sz) > (sz + 1) * percent) {
				std::cout << "Now EPO Test: " << (i - sz) << std::endl;
				percent += step;
			}
		}
		std::cout << "epo total " << EPO << std::endl;

		float totalAREA = 0.0;
		for (size_t i = (node.size() >> 1); i < node.size(); ++i) {
			totalAREA += TRIANGLEAREA(GETGEO(i));
		}
		EPO /= totalAREA;
		std::cout << "Total Area: " << totalAREA << std::endl;
		return EPO;
	}

	// with bugs
	[[deprecated]]
	float EPOQuad(const std::vector<QuadBVHNode>& node, const std::vector<Triangle>& triangles) {
		float EPO = 0.0f;
		std::unordered_set<size_t> ancestor;
		auto innerNodeCount = node.size() - triangles.size();

		auto GETGEO = [&](size_t leafID) -> const Triangle& {
			return triangles[node[leafID].children.x];
		};

		auto POINTINBOX = [](const cl_float4& p, const BoundingBox& box) -> bool {
			if ((p.x >= box.bbmin.x && p.x <= box.bbmax.x)
				&& (p.y >= box.bbmin.y && p.y <= box.bbmax.y)
				&& (p.z >= box.bbmin.z && p.z <= box.bbmax.z))
			{
				return true;
			}
			return false;
		};
		auto TRIANGLEAREA = [](const Triangle& geo) -> float {
			return len(cross(geo.v[1] - geo.v[0], geo.v[2] - geo.v[0])) / 2;
		};

		auto ROUNDTR = [](std::vector<cl_float4>& points, int axis, float pos, int sign) -> void {
			std::vector<cl_float4> ans;
			std::vector<int> inside;

			float tans;

			if (sign > 0) {
				for (auto& p : points) {
					inside.push_back(p.s[axis] >= pos);
				}
			}
			else {
				for (auto& p : points) {
					inside.push_back(p.s[axis] <= pos);
				}
			}
			if (points.empty()) return;
			for (int i = 0; i < points.size(); ++i) {
				auto i_1 = (i + 1 == points.size()) ? (0) : (i + 1);
				if (!inside[i] && !inside[i_1]) {
					continue;
				}
				else if (inside[i] && inside[i_1]) {
					ans.push_back(points[i]);
					continue;
				}
				else {
					if (inside[i]) {
						ans.push_back(points[i]);
					}
					cl_float4 direction = points[i_1] - points[i];
					tans = (pos - points[i].s[axis]) / direction.s[axis];
					ans.push_back(points[i] + tans * direction);
				}
			}

			points.swap(ans);
		};
		auto pArea = [](const std::vector<cl_float4>& points) -> float {
			float ans = 0.0f;
			int sz = points.size();
			if (sz < 2) {
				return ans;
			}
			for (int i = 1; i < sz - 1; ++i) {
				auto x1 = points[i] - points[0];
				auto x2 = points[i + 1] - points[0];
				ans += len(cross(x1, x2)) / 2;
			}
			return ans;
		};
		auto INTERSECTAREA = [&](const Triangle& geo, BoundingBox& box) -> float {
			Triangle tr = geo;
			tr.v[0].w = 1;
			tr.v[1].w = 1;
			tr.v[2].w = 1;

			bool inside[3];
			inside[0] = POINTINBOX(tr.v[0], box);
			inside[1] = POINTINBOX(tr.v[1], box);
			inside[2] = POINTINBOX(tr.v[2], box);

			if (inside[0] && inside[1] && inside[2]) {
				return TRIANGLEAREA(tr);
			}

			std::vector< cl_float4 > points = { tr.v[0],tr.v[1],tr.v[2] };


			ROUNDTR(points, 0, box.bbmin.x, 1);
			ROUNDTR(points, 1, box.bbmin.y, 1);
			ROUNDTR(points, 2, box.bbmin.z, 1);

			ROUNDTR(points, 0, box.bbmax.x, -1);
			ROUNDTR(points, 1, box.bbmax.y, -1);
			ROUNDTR(points, 2, box.bbmax.z, -1);

			return pArea(points);
		};
	
		std::deque<size_t> toBeDone;
		auto TopDown = [&](size_t leafID, size_t rootID) -> void {
			auto& geo = GETGEO(leafID);
			auto box = BoundingBox({ node[rootID].bbmin,node[rootID].bbmax });

			if (ancestor.find(rootID) == ancestor.end()) {
				float epovalue = INTERSECTAREA(geo, box);
				if (epovalue > 0) {
					auto isInner = (rootID < innerNodeCount);
					if (!isInner) {
						EPO += epovalue * Ctri;
					}
					else {
						auto notEmptyChildCount = 4;
						if (node[rootID].children.y <= 0) --notEmptyChildCount;
						if (node[rootID].children.w <= 0) --notEmptyChildCount;
						EPO += epovalue * (Cinn / 2) * notEmptyChildCount;
					}

					if (node[rootID].children.x != node[rootID].children.y) {
						toBeDone.push_back(node[rootID].children.x);
						if (node[rootID].children.y > 0)
							toBeDone.push_back(node[rootID].children.y);
						toBeDone.push_back(node[rootID].children.z);
						if (node[rootID].children.w > 0)
							toBeDone.push_back(node[rootID].children.w);
					}
				}
			}
			else {
				if (node[rootID].children.x != node[rootID].children.y) {
					toBeDone.push_back(node[rootID].children.x);
					if (node[rootID].children.y > 0)
						toBeDone.push_back(node[rootID].children.y);
					toBeDone.push_back(node[rootID].children.z);
					if (node[rootID].children.w > 0)
						toBeDone.push_back(node[rootID].children.w);
				}
			}
		};

		float percent = 0.1f;
		float step = 0.1f;
		int sz = (node.size() - triangles.size());

		float prevEPO = 0.0f;
		for (size_t i = sz; i < node.size(); ++i) {
			ancestor.insert(i);

			size_t j = i;
			while (node[j].parent.w != -1) {
				ancestor.insert(node[j].parent.w);
				j = node[j].parent.w;
			}

			toBeDone.push_back(0);
			while (!toBeDone.empty()) {
				auto front = toBeDone.front();
				toBeDone.pop_front();
				TopDown(i, front);
			}

			epoTValue.push_back(EPO - prevEPO);
			prevEPO = EPO;

			ancestor.clear();

			if ((i - sz) > (sz + 1)* percent) {
				std::cout << "Now EPO Test: " << (i - sz) << std::endl;
				percent += step;
			}
		}
		std::cout << "epo total " << EPO << std::endl;

		float totalAREA = 0.0;
		for (size_t i = (node.size() >> 1); i < node.size(); ++i) {
			totalAREA += TRIANGLEAREA(GETGEO(i));
		}
		EPO /= totalAREA;
		std::cout << "Total Area: " << totalAREA << std::endl;
		return EPO;
	}

	
	


	double LCV(cl::Buffer nodes, cl::Buffer triangles, json camera) {
		static auto initer = [&]() {
			if (!programInited) {
				prog = OpenCLBasic::createProgramFromFileWithHeader("./kernels/EPO.cl", "objdef.h");
				programInited = true;
			}
			lcvKernel = OpenCLBasic::createKernel(prog, "LCV");

			if (!rayGenInited) {
				rayGenProg = OpenCLBasic::createProgramFromFileWithHeader("./kernels/rayGenerator.cl","objdef.h");
				rayGenerator = OpenCLBasic::createKernel(rayGenProg, "generateRay");
				rayGenInited = true;
			}
			return 0;
		}();


		int width = camera["resolution"][0];
		int height = camera["resolution"][1];
		Camera cmr = parseCamera(camera);

		auto cmrB = OpenCLBasic::newBuffer<Camera>(1, (void*)(&cmr));
		auto rayB = OpenCLBasic::newBuffer<Ray>(width * height);
		OpenCLBasic::setKernelArg(rayGenerator, cmrB, rayB);
		OpenCLBasic::enqueueNDRange(rayGenerator, { (size_t)width,(size_t)height }, cl::NullRange);

		//
		auto countB = OpenCLBasic::newBuffer<int>(width * height);
		auto innerCountB = OpenCLBasic::newBuffer<int>(width * height);
		auto primCountB = OpenCLBasic::newBuffer<int>(width * height);

		OpenCLBasic::setKernelArg(lcvKernel, rayB, nodes, triangles, countB, innerCountB, primCountB);
		OpenCLBasic::enqueueNDRange(lcvKernel, width * height, cl::NullRange);

		auto finalC = OpenCLBasic::readBuffer<int>(countB);

		auto innerCounts = OpenCLBasic::readBuffer<int>(innerCountB);
		auto primCounts = OpenCLBasic::readBuffer<int>(primCountB);

		double averageInner = 0, averagePrim = 0;
		for (auto& i : innerCounts) {
			averageInner += i;
		}
		averageInner /= (width * height);
		std::cout << "******** Average Inner: " << averageInner << std::endl;
		for (auto& i : primCounts) {
			averagePrim += i;
		}
		averagePrim /= (width * height);
		std::cout << "******** Average Prim: " << averagePrim << std::endl;

		double e = 0.0, e2 = 0.0;
		for (auto& f : finalC) {
			e += f;
			e2 += f * f;
		}
		e /= (width * height);
		e2 /= (width * height);

		std::cout << "******** Average Leaf Node: " << e << std::endl;
		return sqrt(e2 - e * e);
	}

	double LCV_Multi(cl::Buffer nodes, cl::Buffer indices, cl::Buffer triangles, json camera) {
		static auto initer = [&]() {
			if (!programInited) {
				prog = OpenCLBasic::createProgramFromFileWithHeader("./kernels/EPO.cl", "objdef.h");
				programInited = true;
			}
			lcvKernel_Multi = OpenCLBasic::createKernel(prog, "LCV_Multi");

			if (!rayGenInited) {
				rayGenProg = OpenCLBasic::createProgramFromFileWithHeader("./kernels/rayGenerator.cl", "objdef.h");
				rayGenerator = OpenCLBasic::createKernel(rayGenProg, "generateRay");
				rayGenInited = true;
			}
			return 0;
		}();
		int width = camera["resolution"][0];
		int height = camera["resolution"][1];
		Camera cmr = parseCamera(camera);

		auto cmrB = OpenCLBasic::newBuffer<Camera>(1, (void*)(&cmr));
		auto rayB = OpenCLBasic::newBuffer<Ray>(width * height);
		OpenCLBasic::setKernelArg(rayGenerator, cmrB, rayB);
		OpenCLBasic::enqueueNDRange(rayGenerator, { (size_t)width,(size_t)height }, cl::NullRange);


		////
		auto countB = OpenCLBasic::newBuffer<int>(width * height);

		auto innerCountB = OpenCLBasic::newBuffer<int>(width * height);
		auto primCountB = OpenCLBasic::newBuffer<int>(width * height);

		OpenCLBasic::setKernelArg(lcvKernel_Multi, rayB, nodes, indices, triangles, countB, innerCountB, primCountB);
		OpenCLBasic::enqueueNDRange(lcvKernel_Multi, width * height, cl::NullRange);

		auto finalC = OpenCLBasic::readBuffer<int>(countB);
		auto innerCounts = OpenCLBasic::readBuffer<int>(innerCountB);
		auto primCounts = OpenCLBasic::readBuffer<int>(primCountB);

		double averageInner = 0, averagePrim = 0;
		for (auto& i : innerCounts) {
			averageInner += i;
		}
		averageInner /= (width * height);
		std::cout << std::endl << "******** Average Inner: " << averageInner << std::endl;
		for (auto& i : primCounts) {
			averagePrim += i;
		}
		averagePrim /= (width * height);
		std::cout << "******** Average Prim: " << averagePrim << std::endl;

		double e = 0.0, e2 = 0.0;
		for (auto& f : finalC) {
			e += f;
			e2 += f * f;
		}
		e /= (width * height);
		e2 /= (width * height);

		std::cout << "******** Average Leaf Node: " << e << std::endl;
		return sqrt(e2 - e * e);
	}



	double QuadLCV(cl::Buffer nodes, json camera) {
		static auto initer = [&]() {
			if (!programInited) {
				prog = OpenCLBasic::createProgramFromFileWithHeader("./kernels/EPO.cl", "objdef.h");
				programInited = true;
			}
			quadLCVKernel = OpenCLBasic::createKernel(prog, "QuadLCV");

			if (!rayGenInited) {
				rayGenProg = OpenCLBasic::createProgramFromFileWithHeader("./kernels/rayGenerator.cl", "objdef.h");
				rayGenerator = OpenCLBasic::createKernel(rayGenProg, "generateRay");
				rayGenInited = true;
			}
			return 0;
		}();
		
		int width = camera["resolution"][0];
		int height = camera["resolution"][1];
		Camera cmr = parseCamera(camera);

		auto cmrB = OpenCLBasic::newBuffer<Camera>(1, (void*)(&cmr));
		auto rayB = OpenCLBasic::newBuffer<Ray>(width * height);
		OpenCLBasic::setKernelArg(rayGenerator, cmrB, rayB);
		OpenCLBasic::enqueueNDRange(rayGenerator, { (size_t)width,(size_t)height }, cl::NullRange);

		auto countB = OpenCLBasic::newBuffer<int>(width * height);
		OpenCLBasic::setKernelArg(quadLCVKernel, rayB, nodes, countB);
		OpenCLBasic::enqueueNDRange(quadLCVKernel, width * height, cl::NullRange);

		auto finalC = OpenCLBasic::readBuffer<int>(countB);

		double e = 0.0, e2 = 0.0;
		for (auto& f : finalC) {
			e += f;
			e2 += f * f;
		}
		e /= (width * height);
		e2 /= (width * height);
		return sqrt(e2 - e * e);
	}


	

	double LCV(const std::vector<BVHNode>& node, const Camera& camera) {
		std::vector<int> stack;
		stack.reserve(256);

		auto countLeafIntersect = [&](const Ray& r) -> size_t {
			size_t ans = 0;
			auto* pt = node.data();
			stack.clear();
			cl_float temp;
		AGAIN:
			auto box = BoundingBox({ pt->bbmin, pt->bbmax });
			if (intersectBox(r, box, 0.001f, temp)) {
				if (pt->left == pt->right) {
					++ans;
					if (stack.empty()) goto END;
					pt = node.data() + stack.back();
					stack.pop_back();
					goto AGAIN;
				}
				else {
					stack.push_back(pt->right);
					pt = node.data() + pt->left;
					goto AGAIN;
				}
			}
			if (!stack.empty()) {
				pt = node.data() + stack.back();
				stack.pop_back();
				goto AGAIN;
			}
		END:
			return ans;
		};

		auto GENERATERAY = [&](std::vector<Ray>& rays) {
			int WIDTH = Config::WIDTH();
			int HEIGHT = Config::HEIGHT();
			//Camera camera;
			//switch (modelID) {
			//case 0:
			//	camera = Camera(glm::vec3(-470.0f, 359.0f, 316.0f), glm::vec3(0.0f, 1.0f, 0.0f), -40.0f, -26.2f);
			//	break;
			//case 1:
			//	camera = Camera(glm::vec3(0.0f, 0.1f, 2.7f));
			//	break;
			//case 2:
			//	camera = Camera(glm::vec3(0.0f, 0.2f, 0.7f));
			//	break;
			//case 3:
			//	camera = Camera(glm::vec3(-0.6f, -1.0f, 10.0f));
			//	break;
			//case 4:
			//	//22.857025 16.192675 12.377654   -148.400024 -22.00001122.857025 16.192675 12.377654   -148.400024 -22.000011
			//	camera = Camera(glm::vec3(22.857025f, 16.192675f, 12.377654f), glm::vec3(0.0f, 1.0f, 0.0f), -148.0f, -22.0f);
			//	break;
			//case 5://Sponza
			//	camera = Camera(glm::vec3(302.353668f, 410.221863f, -135.232559f), glm::vec3(0.0f, 1.0f, 0.0f), -209.0f, -20.0f);
			//	break;
			//case 6://San_Miguel
			//	camera = Camera(glm::vec3(25.0f, 2.2f, 1.9f), glm::vec3(0.0f, 1.0f, 0.0f), -209.0f, -20.0f);
			//	break;
			//}

			/*cl_float4 pos = { camera.Position.x,camera.Position.y,camera.Position.z,1.0f };
			cl_float4 up = { camera.Up.x,camera.Up.y,camera.Up.z,0 };
			cl_float4 right = { camera.Right.x,camera.Right.y,camera.Right.z,0 };

			cl_float4 front;
			auto Yaw = camera.Yaw;
			auto Pitch = camera.Pitch;

			front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
			front.y = sin(glm::radians(Pitch));
			front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
			front.w = 0;*/

			/*for (int i = 0; i < WIDTH; ++i) {
				for (int j = 0; j < HEIGHT; ++j) {
					float temp1 = (i + 0.5f) / WIDTH - 0.5f;
					float temp2 = (j + 0.5f) / HEIGHT - 0.5f;
					Ray r;
					r.origin = pos;
					r.direction = front + temp1 * right + temp2 * up;
					rays.push_back(std::move(r));
				}
			}*/

			for (int i = 0; i < WIDTH; ++i) {
				for (int j = 0; j < HEIGHT; ++j) {
					float temp1 = (i + 0.5f) / WIDTH - 0.5f;
					float temp2 = (j + 0.5f) / HEIGHT - 0.5f;
					float distance = 0.5f / tan(camera.arg / 2);
					Ray r;
					r.origin = camera.center;
					r.direction = distance * camera.direction  + temp1 * camera.horizontal + temp2 * camera.up;
					rays.push_back(std::move(r));
				}
			}

		};



		std::vector< Ray > rays;
		std::vector< size_t > leafCounts;
		GENERATERAY(rays);
		for (auto& r : rays) {
			leafCounts.push_back(countLeafIntersect(r));
		}

		double En = 0.0f;
		double En2 = 0.0f;
		for (auto& c : leafCounts) {
			En += c;
			En2 += c * c;
		}
		En /= leafCounts.size();
		En2 /= leafCounts.size();

		return sqrt(En2 - En * En);
	}


}



namespace MCPT::BVH::TEST {
	double SAH(const std::vector<BVHNode>& node) {
		double sahAns = 0.0f;
		for (size_t i = 0; i < (node.size() >> 1); ++i) {
			sahAns += Cinn * AREA(node[i].bbmin, node[i].bbmax);
		}
		for (size_t i = (node.size() >> 1); i < node.size(); ++i) {
			sahAns += Ctri * AREA(node[i].bbmin, node[i].bbmax);
		}
		sahAns /= AREA(node[0].bbmin, node[0].bbmax);

		return sahAns;
	}
	double SAH(cl::Buffer nodeB) {
		static auto initer = []() {
			OpenCLBasic::init();
			return 0;
		}();
		auto node = OpenCLBasic::readBuffer<BVHNode>(nodeB);
		double sahAns = 0.0f;
		for (size_t i = 0; i < (node.size() >> 1); ++i) {
			sahAns += Cinn * AREA(node[i].bbmin, node[i].bbmax);
		}
		for (size_t i = (node.size() >> 1); i < node.size(); ++i) {
			sahAns += Ctri * AREA(node[i].bbmin, node[i].bbmax);
		}
		sahAns /= AREA(node[0].bbmin, node[0].bbmax);

		return sahAns;
	}
	double SAH(const std::vector<QuadBVHNode>& node, int objCount) {
		double sahAns = 0.0f;
		auto innerNodeCount = node.size() - objCount;
		for (size_t i = 0; i < innerNodeCount; ++i) {
			int childCount = 4;
			if (node[i].children.y < 0) --childCount;
			if (node[i].children.w < 0) --childCount;
			sahAns += childCount * (Cinn / 2.0) * AREA(node[i].bbmin, node[i].bbmax);
		}
		for (size_t i = innerNodeCount; i < node.size(); ++i) {
			sahAns += Ctri * AREA(node[i].bbmin, node[i].bbmax);
		}
		sahAns /= AREA(node[0].bbmin, node[0].bbmax);

		return sahAns;
	}

	double SAH_Multi(const std::vector<MultiPrimBVHNode>& node) {
		double sahAns = 0.0f;
		/*for (auto i = 0; i < node.size(); ++i) {
			if (node[i].isLeaf) {
				sahAns += Ctri * AREA(node[i].bbmin, node[i].bbmax) * (node[i].right - node[i].left);
			}
			else {
				sahAns += Cinn * AREA(node[i].bbmin, node[i].bbmax);
			}
		}*/
		std::function<double(int)> sahFunc;
		sahFunc = [&](int id) -> double {
			if (node[id].isLeaf) {
				auto ans = Ctri * AREA(node[id].bbmin, node[id].bbmax) * (node[id].right - node[id].left);
				if (isinf(ans)) {
					std::cout << "INF in " << id << " ";
					for (int i = 0; i < 3; ++i)
						std::cout << node[id].bbmin.s[i] << " ";
					for (int i = 0; i < 3; ++i)
						std::cout << node[id].bbmax.s[i] << " ";
					std::cout << std::endl;
				}
				return Ctri * AREA(node[id].bbmin, node[id].bbmax) * (node[id].right - node[id].left);
			}
			else {
				auto l = sahFunc(node[id].left);
				auto r = sahFunc(node[id].right);
				return l + r + Cinn * AREA(node[id].bbmin, node[id].bbmax);
			}
		};
		sahAns = sahFunc(0);
		std::cout << "----" << sahAns << " " << AREA(node[0].bbmin, node[0].bbmax) << std::endl;
		sahAns /= AREA(node[0].bbmin, node[0].bbmax);
		return sahAns;
	}

	double EPO(cl::Buffer bvh, cl::Buffer triangles) {
		static auto initer = []() {
			if (!programInited) {
				prog = OpenCLBasic::createProgramFromFileWithHeader("./kernels/EPO.cl", "objdef.h");
				programInited = true;
			}
			kernel = OpenCLBasic::createKernel(prog, "calculateEPO");
			return 0;
		}();

		int triCount = triangles.getInfo<CL_MEM_SIZE>() / sizeof(Triangle);
		cl::Buffer epoBuffer = OpenCLBasic::newBuffer<float>(triCount);
		cl::Buffer triAreaBuffer = OpenCLBasic::newBuffer<float>(triCount);

		OpenCLBasic::setKernelArg(kernel, bvh, triangles, epoBuffer, triAreaBuffer, triCount);
		OpenCLBasic::enqueueNDRange(kernel, triCount, cl::NullRange);

		std::vector<float> epoAns;
		std::vector<float> areaAns;
		epoAns.resize(triCount);
		areaAns.resize(triCount);
		OpenCLBasic::readBuffer(epoBuffer, epoAns.data());
		OpenCLBasic::readBuffer(triAreaBuffer, areaAns.data());


		double count = 0.0;
		for (int i = 0; i < epoAns.size(); ++i) {
			count += epoAns[i];
		}
		double area = 0.0;
		for (int i = 0; i < areaAns.size(); ++i) {
			area += areaAns[i];
		}
		count /= area;
		return count;
	}
	double QuadEPO(cl::Buffer bvh, cl::Buffer triangles) {
		static auto initer = []() {
			if (!programInited) {
				prog = OpenCLBasic::createProgramFromFileWithHeader("./kernels/EPO.cl", "objdef.h");
				programInited = true;
			}
			quadKernel = OpenCLBasic::createKernel(prog, "calculateQuadEPO");
			return 0;
		}();

		int triCount = triangles.getInfo<CL_MEM_SIZE>() / sizeof(Triangle);
		int offset = bvh.getInfo<CL_MEM_SIZE>() / sizeof(QuadBVHNode) - triCount;
		cl::Buffer epoBuffer = OpenCLBasic::newBuffer<float>(triCount);
		cl::Buffer triAreaBuffer = OpenCLBasic::newBuffer<float>(triCount);

		OpenCLBasic::setKernelArg(quadKernel, bvh, triangles, epoBuffer, triAreaBuffer, offset);
		OpenCLBasic::enqueueNDRange(quadKernel, triCount, cl::NullRange);

		std::vector<float> epoAns;
		std::vector<float> areaAns;
		epoAns.resize(triCount);
		areaAns.resize(triCount);
		OpenCLBasic::readBuffer(epoBuffer, epoAns.data());
		OpenCLBasic::readBuffer(triAreaBuffer, areaAns.data());

		double count = 0.0;
		for (int i = 0; i < epoAns.size(); ++i) {
			count += epoAns[i];
		}
		double area = 0.0;
		for (int i = 0; i < areaAns.size(); ++i) {
			area += areaAns[i];
		}
		count /= area;
		return count;
	}

	double EPO_Multi(const std::vector<MultiPrimBVHNode>& node, const std::vector<unsigned int>& indices, cl::Buffer triangles) {
		static auto initer = []() {
			if (!programInited) {
				prog = OpenCLBasic::createProgramFromFileWithHeader("./kernels/EPO.cl", "objdef.h");
				programInited = true;
			}
			multi_kernel = OpenCLBasic::createKernel(prog, "calculateEPO_Multi");
			return 0;
		}();

		std::vector<unsigned int> leaves;
		for (int i = 0; i < node.size(); ++i) {
			if (node[i].isLeaf) leaves.push_back(i);
		}
		auto leafCount = leaves.size();

		auto bvhB = OpenCLBasic::newBuffer< MultiPrimBVHNode >(node.size(), (void*)node.data());
		auto indexB = OpenCLBasic::newBuffer< uint>(indices.size(), (void*)indices.data());
		auto leafB = OpenCLBasic::newBuffer<uint>(leaves.size(), (void*)leaves.data());
		auto ansB = OpenCLBasic::newBuffer<float>(leafCount);
		auto areaB = OpenCLBasic::newBuffer<float>(leafCount);
		OpenCLBasic::setKernelArg(multi_kernel, bvhB, indexB, leafB, triangles, ansB, areaB);
		OpenCLBasic::enqueueNDRange(multi_kernel, leafCount, cl::NullRange);

		std::vector<float> epoAns = OpenCLBasic::readBuffer<float>(ansB);
		std::vector<float> areaAns = OpenCLBasic::readBuffer<float>(areaB);

		double count = 0.0;
		for (int i = 0; i < epoAns.size(); ++i) {
			count += epoAns[i];
		}
		double area = 0.0;
		for (int i = 0; i < areaAns.size(); ++i) {
			area += areaAns[i];
		}
		count /= area;
		return count;
	}


	void singleTest(cl::Buffer bvh, cl::Buffer triangles) {
		auto config = Config::getConfig();
		if ((bool)config["usequad"]) {
			auto node = OpenCLBasic::readBuffer<QuadBVHNode>(bvh);
			auto tri = OpenCLBasic::readBuffer<Triangle>(triangles);
			std::cout << "SAH: " << SAH(node, tri.size()) << std::endl;
			std::cout << "EPO: " << QuadEPO(bvh, triangles) << std::endl;

			if (!config["camera"].empty) {
				std::cout << "LCV: " << QuadLCV(bvh, config["camera"].obj);
			}
		}
		else {
			std::cout << "SAH: " << SAH(bvh) << std::endl;
			std::cout << "EPO: " << EPO(bvh, triangles) << std::endl;

			if (!config["camera"].empty) {
				std::cout << "LCV: " << LCV(bvh, triangles, config["camera"].obj);
			}
		}
	}
	void singleTest(const std::vector<MultiPrimBVHNode>& node, const std::vector<unsigned int>& indices, cl::Buffer triangles) {
		auto config = Config::getConfig();

		std::cout << "SAH(M): " << SAH_Multi(node) << std::endl;
		std::cout << "EPO(M): " << EPO_Multi(node,indices,triangles) << std::endl;

		auto bvhB = OpenCLBasic::newBuffer<MultiPrimBVHNode>(node.size(), (void*)node.data());
		auto indexB = OpenCLBasic::newBuffer<uint>(indices.size(), (void*)indices.data());

		if (!config["camera"].empty) {
			std::cout << "LCV(M): " << LCV_Multi(bvhB, indexB, triangles, config["camera"].obj);
		}
	}



	void testall() {
		OpenCLBasic::init();
		auto dir = Config::GETDIRECTORY();
		auto objs = Config::GETOBJS();

		for (const auto& obj : objs) {
			auto objStr = obj["name"].get<std::string>();
			auto triangles = loadObj(dir, objStr);

			for (auto& tr : triangles) {
				tr.normal = normalize(cross(tr.v[1] - tr.v[0], tr.v[2] - tr.v[0]));
			}


			std::cout << "Object Name: " << objStr << " " << triangles.size() << std::endl;
			
			auto bvhsToTest = obj["bvhs"];
			auto cameras = obj["cameras"];
			
			std::unique_ptr< HLBVH<CPU> > hlbvh;
			std::unique_ptr< SAHBVH<CPU> > sahbvh;
			std::unique_ptr< TreeletBVH<CPU> > treelet;
			std::unique_ptr< TreeletBVH<GPU> > treeletGPU;
			std::unique_ptr< SimpleBVHQuad<CPU> > quad_hlbvh;
			std::unique_ptr< SimpleBVHQuad<CPU> > quad_treelet;

			for (const auto& bvhtest : bvhsToTest) {
				auto bvhStr = bvhtest.get<std::string>();
				std::cout << "BVH type: " << bvhStr << std::endl;
				if (bvhStr == "hlbvh") {
					if (hlbvh == nullptr)
						hlbvh = std::make_unique< HLBVH<CPU> >(triangles);

					auto& node = hlbvh->getBVH();
					cl::Buffer bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(node.size(), (void*)node.data());
					cl::Buffer triBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());

					//std::cout << "SAH: " << SAH(node) << std::endl;
					//std::cout << "EPO: " << EPO(bvhBuffer, triBuffer) << std::endl;
					
					std::vector<double> lcvs;
					for (const auto& camera : cameras) {
						auto lcv = LCV(bvhBuffer, triBuffer, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera1: " << lcv << std::endl;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}
				/*else if (bvhStr == "sah") {
					sahbvh = std::make_unique< SAHBVH<CPU> >(triangles);
					auto& node = sahbvh->getBVH();

					cl::Buffer bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(node.size(), (void*)node.data());
					cl::Buffer triBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());

					std::cout << "SAH: " << SAH(node) << std::endl;
					std::cout << "EPO: " << EPO(bvhBuffer, triBuffer) << std::endl;

					std::vector<double> lcvs;
					int i = 0;
					for (const auto& camera : cameras) {
						auto lcv = LCV(bvhBuffer, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera" << i << " " << lcv << std::endl;
						++i;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}*/
				else if (bvhStr.substr(0, 3) == "sah") {
					const auto boxes = [&]() {
						std::vector< BBox > ans;
						ans.resize(triangles.size());
						for (int i = 0; i < triangles.size(); ++i) {
							ans[i].unionBBoxCentroid(triangles[i].v[0]);
							ans[i].unionBBoxCentroid(triangles[i].v[1]);
							ans[i].unionBBoxCentroid(triangles[i].v[2]);
						}
						return ans;
					}();
					const auto indices = [&]() {
						std::vector< uint32_t > ans;
						ans.resize(triangles.size());
						for (int i = 0; i < triangles.size(); ++i) {
							ans[i] = i;
						}
						return ans;
					}();

					std::tuple<bool, uint32_t, int> arg;
					switch (bvhStr[3]) {
					case '1': { // 1axis binning
						arg = { false, 16 ,0 };
						break;
					}
					case '2': { // 1axis full
						arg = { false, 0, 0 };
						break;
					}
					case '3': { // 3axis binning
						arg = { true, 16, 0 };
						break;
					}
					case '4': { // 3axis full
						arg = { true, 0, 0 };
						break;
					}
					default:
						throw "Not Implemented";
					}
					SAHBVH<CPU> sahbvh{ boxes,indices,arg };
					auto& node = sahbvh.getBVH();
					auto& sortedIndices = sahbvh.getIndices();

					cl::Buffer bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(node.size(), (void*)node.data());
					cl::Buffer triBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());
					cl::Buffer indicesBuffer = OpenCLBasic::newBuffer<uint>(sortedIndices.size(), (void*)sortedIndices.data());

					std::cout << "SAH: " << SAH_Multi(node) << std::endl;
					std::cout << "EPO: " << EPO_Multi(node, sortedIndices, triBuffer) << std::endl;

					std::vector<double> lcvs;
					int c = 0;
					for (const auto& camera : cameras) {
						auto lcv = LCV_Multi(bvhBuffer, indicesBuffer, triBuffer, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera " << c << ": " << lcv << std::endl;
						++c;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}
				else if (bvhStr == "treelet") {
					if (treelet == nullptr) {
						if (hlbvh == nullptr) {
							hlbvh = std::make_unique< HLBVH<CPU> >(triangles);
						}
						treelet = std::make_unique< TreeletBVH<CPU> >(hlbvh->getBVH());
					}

					auto& node = treelet->getBVH();
					cl::Buffer bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(node.size(), (void*)node.data());
					cl::Buffer triBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());

					std::cout << "SAH: " << SAH(treelet->getBVH()) << std::endl;
					std::cout << "EPO: " << EPO(bvhBuffer, triBuffer) << std::endl;

					std::vector<double> lcvs;
					for (const auto& camera : cameras) {
						auto lcv = LCV(bvhBuffer, triBuffer, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera1: " << lcv << std::endl;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}
				else if (bvhStr == "treelet2") {
					std::unique_ptr< TreeletBVH_<CPU> > treelet2;
					if (hlbvh == nullptr) {
						hlbvh = std::make_unique< HLBVH<CPU> >(triangles);
					}
					{
						auto nodes = hlbvh->getBVH();
						treelet2 = std::make_unique< TreeletBVH_<CPU> >(std::move(nodes));
					}
					auto node = treelet2->getBVH();
					
					cl::Buffer bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(node.size(), (void*)node.data());
					cl::Buffer triBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());
					cl::Buffer indicesBuffer = OpenCLBasic::newBuffer<uint>(treelet2->getIndices().size(), (void*)treelet2->getIndices().data());

					std::cout << "SAH: " << SAH_Multi(node) << std::endl;
					std::cout << "EPO: " << EPO_Multi(node, treelet2->getIndices(), triBuffer) << std::endl;

					std::vector<double> lcvs;
					int c = 0;
					for (const auto& camera : cameras) {
						auto lcv = LCV_Multi(bvhBuffer, indicesBuffer, triBuffer, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera " << c << ": " << lcv << std::endl;
						++c;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}
				else if (bvhStr == "treeletTemp") {
					std::unique_ptr< TreeletTemp > treeletTemp;
					if (hlbvh == nullptr) {
						hlbvh = std::make_unique< HLBVH<CPU> >(triangles);
					}
					{
						Indices indices;
						for (int i = 0; i < triangles.size(); ++i) {
							indices.push_back(i);
						}
						auto& nodes = hlbvh->getBVH();
						treeletTemp = std::make_unique< TreeletTemp >(nodes, indices, 1);
					}
					auto node = treeletTemp->getBVH();
					auto leafIDs = treeletTemp->getLeafIDs();
					double primAverage = 0;
					for (const auto& lid : leafIDs) {
						primAverage += node[lid].right - node[lid].left;
					}
					primAverage /= leafIDs.size();
					

					cl::Buffer bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(node.size(), (void*)node.data());
					cl::Buffer triBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());
					cl::Buffer indicesBuffer = OpenCLBasic::newBuffer<uint>(treeletTemp->getIndices().size(), (void*)treeletTemp->getIndices().data());

					//std::cout << "SAH: " << treeletTemp->sah() << " " << SAH_Multi(treeletTemp->getBVH()) << std::endl;
					//std::cout << "EPO: " << EPO_Multi(node, treeletTemp->getIndices(), triBuffer) << std::endl;


					//std::cout << "******** Average Prim in leaf: " << primAverage << std::endl;
					std::vector<double> lcvs;
					int c = 0;
					for (const auto& camera : cameras) {
						auto lcv = LCV_Multi(bvhBuffer, indicesBuffer, triBuffer, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera " << c << ": " << lcv << std::endl;
						++c;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}
				else if (bvhStr == "treeletGPU") {
					if (treeletGPU == nullptr) {
						if (hlbvh == nullptr) {
							hlbvh = std::make_unique< HLBVH<CPU> >(triangles);
						}
						auto& node = hlbvh->getBVH();
						cl::Buffer bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(node.size(), (void*)node.data());
						cl::Buffer triBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());
						treeletGPU = std::make_unique< TreeletBVH<GPU> >(bvhBuffer, triBuffer);
					}
					auto [bvhB, triB] = treeletGPU->getBuffer();
					std::cout << "SAH: " << SAH(bvhB) << std::endl;
					std::cout << "EPO: " << EPO(bvhB,triB) << std::endl;
					
					std::vector<double> lcvs;
					for (const auto& camera : cameras) {
						auto lcv = LCV(bvhB, triB, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera1: " << lcv << std::endl;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}
				else if (bvhStr == "quadhlbvh") {
					if (quad_hlbvh == nullptr) {
						if (hlbvh == nullptr)
							hlbvh = std::make_unique< HLBVH<CPU> >(triangles);
						quad_hlbvh = std::make_unique< SimpleBVHQuad<CPU> >(hlbvh->getBVH());
					}
					auto& quadBVHNode = quad_hlbvh->getQuadBVH();
					auto objCount = quad_hlbvh->getObjectCount();
					std::cout << "SAH: " << SAH(quadBVHNode, objCount) << std::endl;
					
					auto quadbvhB = OpenCLBasic::newBuffer<QuadBVHNode>(quadBVHNode.size(), (void*)quadBVHNode.data());
					auto tB = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());
					std::cout << "EPO: " << QuadEPO(quadbvhB, tB) << std::endl;

					std::vector<double> lcvs;
					for (const auto& camera : cameras) {
						auto lcv = QuadLCV(quadbvhB, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera1: " << lcv << std::endl;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}
				else if (bvhStr == "quadtreelet") {
					if (quad_treelet == nullptr) {
						if (treelet == nullptr && treeletGPU == nullptr) {
							if (hlbvh == nullptr)
								hlbvh = std::make_unique< HLBVH<CPU> >(triangles);
							auto& node = hlbvh->getBVH();
							cl::Buffer bvhBuffer = OpenCLBasic::newBuffer<BVHNode>(node.size(), (void*)node.data());
							cl::Buffer triBuffer = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());
							treeletGPU = std::make_unique< TreeletBVH<GPU> >(bvhBuffer, triBuffer);
						}
						if (treeletGPU != nullptr) {
							auto [bF, tF] = treeletGPU->getBuffer();
							auto node = OpenCLBasic::readBuffer<BVHNode>(bF);
							auto tri = OpenCLBasic::readBuffer<Triangle>(tF);
							quad_treelet = std::make_unique< SimpleBVHQuad<CPU> >(node);
						}
						else {
							quad_treelet = std::make_unique< SimpleBVHQuad<CPU> >(treelet->getBVH());
						}
					}

					auto& quadBVHNode = quad_treelet->getQuadBVH();
					auto objCount = quad_treelet->getObjectCount();
					std::cout << "SAH: " << SAH(quadBVHNode, objCount) << std::endl;

					auto quadbvhB = OpenCLBasic::newBuffer<QuadBVHNode>(quadBVHNode.size(), (void*)quadBVHNode.data());
					auto tB = OpenCLBasic::newBuffer<Triangle>(triangles.size(), (void*)triangles.data());
					std::cout << "EPO: " << QuadEPO(quadbvhB, tB) << std::endl;

					std::vector<double> lcvs;
					for (const auto& camera : cameras) {
						auto lcv = QuadLCV(quadbvhB, camera);
						lcvs.push_back(lcv);
						std::cout << "Camera1: " << lcv << std::endl;
					}
					if (!lcvs.empty()) {
						double sum = 0.0;
						for (auto i : lcvs) sum += i;
						std::cout << "LCV: " << sum / lcvs.size() << std::endl;
					}
				}
			}
			std::cout << std::endl;
		}
		
	}
}