#include "thirdpartywrapper.h"
#include "config.h"
#include "stb_image_write.h"
#include <memory>

#include "tiny_obj_loader.h"


using namespace MCPT;
using namespace Config;


namespace MCPT::ThirdPartyWrapper {
	void outputPicture(const std::string& str,cl::Buffer buffer) {
		int width = WIDTH();
		int height = HEIGHT();

		auto x = OpenCLBasic::readBuffer<cl_float4>(buffer);
		

		stbi_flip_vertically_on_write(true);
		stbi_write_hdr(str.c_str(), width, height, 4, x[0].s);
	}

	std::tuple<std::vector<Triangle>, std::vector<Material>, std::vector<int> > loadObject(const std::string& directory, const std::string& objname) {
		std::vector<Triangle> ansTrg;
		std::vector<Material> ansMat;
		std::vector<int> ansIndex;

		tinyobj::attrib_t attrib;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;

		tinyobj::LoadObj(&attrib, &shapes, &materials, nullptr, nullptr, (directory+objname).c_str(), directory.c_str());


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
				//int lowRange = x.mesh.num_face_vertices.size() / 8 * 7;
				//int highRange = lowRange + x.mesh.num_face_vertices.size() / 8;
				//if (i<lowRange || i > highRange) continue;
				//if (i % 15 != 1) continue;

				size_t offset = i * 3;

				size_t tid[3];
				tid[0] = x.mesh.indices[offset].vertex_index;
				tid[1] = x.mesh.indices[offset + 1].vertex_index;
				tid[2] = x.mesh.indices[offset + 2].vertex_index;

				ansTrg.push_back(packFloat(tid));
				ansIndex.push_back(x.mesh.material_ids[i]);
			}
		};

		for (auto& shape : shapes) {
			packTriangle(shape);
			//break;
		}
		
		for (const auto& mat : materials) {
			if (mat.ior != 1.0f) {
				Material tempMat = {};
				tempMat.type = MaterialType::MCPT_TRANSPARENT;
				tempMat.Ni = mat.ior;
				ansMat.push_back(tempMat);
			}
			else if (mat.ambient[0] > 0.0f || mat.ambient[1] > 0.0f || mat.ambient[2] > 0.0f) {
				float4 amb = { mat.ambient[0],mat.ambient[1],mat.ambient[2],0.0f };
				Material tempMat = {};
				tempMat.type = MaterialType::MCPT_LIGHT;
				tempMat.ka = amb;
				ansMat.push_back(tempMat);
			}

			else if(mat.shininess != 1.0f) {
				Material tempMat = {};
				tempMat.type = MaterialType::MCPT_GLOSSY;
				tempMat.Ns = mat.shininess;
				tempMat.ks = { mat.specular[0],mat.specular[1],mat.specular[2],0.0f };
				tempMat.ks = (mat.shininess + 2) * (2.0 / M_PI) * tempMat.ks;
				tempMat.kd = { mat.diffuse[0],mat.diffuse[1],mat.diffuse[2],0.0f };
				tempMat.kd = (1.0 / M_PI) * tempMat.kd;
				ansMat.push_back(tempMat);
			}
			else {
				Material tempMat = {};
				tempMat.type = MaterialType::MCPT_DIFFUSE;
				tempMat.kd = { mat.diffuse[0],mat.diffuse[1],mat.diffuse[2],0.0f };
				tempMat.kd = (1.0 / M_PI) * tempMat.kd;
				ansMat.push_back(tempMat);
			}
		}
		return { ansTrg,ansMat,ansIndex };
	}


}