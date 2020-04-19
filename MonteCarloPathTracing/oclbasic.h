#pragma once

#define __CL_ENABLE_EXCEPTIONS
#include <CL/cl.hpp>
#include <vector>
#include <algorithm>

#define PT_USE_NVIDIA
#define PT_OPENGL_COOP

namespace MCPT {

	class OpenCLBasic {
	public:

		static void init();

		static cl::Context& getContext();
		static cl::CommandQueue& getQueue();

		static cl::Program createProgramFromFile(const std::string& code, std::string option = "");
		static cl::Program createProgramFromFileWithHeader(const std::string& code, const std::string& header, std::string option = "");
		static cl::Program createProgram(const std::string& code, std::string option = "");

		static cl::Kernel createKernel(cl::Program& program, const std::string& entry);

		static cl::Buffer createReadBuffer(size_t size, void* pos);
		static cl::Buffer createWriteBuffer(size_t size);
		static cl::Buffer createRWBuffer(size_t size, void* pos);
		static void enqueueReadBuffer(const cl::Buffer& buffer, size_t offset, size_t size, void* pt);
		static void enqueueWriteBuffer(const cl::Buffer& buffer, size_t offset, size_t size, void* pt);


		static void enqueueNDRange(cl::Kernel& kernel, cl::NDRange global, cl::NDRange local, const std::vector<cl::Event>* evs = nullptr, cl::Event* ev = nullptr);
		static float timeCost(const cl::Event& ev, int arg = 0);

		template<unsigned int COUNT, typename T>
		static inline void setKernelArg(cl::Kernel& kn, const T& argEnd) {
			kn.setArg(COUNT, argEnd);
		}
		template<unsigned int COUNT, typename T, typename... ARG>
		static inline void setKernelArg(cl::Kernel& kn, const T& argNow, ARG... args) {
			kn.setArg(COUNT, argNow);
			setKernelArg<COUNT + 1, ARG...>(kn, args...);
		}
		template<typename... ARG>
		static inline void setKernelArg(cl::Kernel& kn, ARG... args) {
			setKernelArg<0, ARG...>(kn, args...);
		}

		template<typename T>
		static cl::Buffer newReadBuffer(size_t len, void* pt) {
			return cl::Buffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, len * sizeof(T), pt);
		}
		template<typename T>
		static cl::Buffer newBuffer(size_t len, void* pt) {
			return cl::Buffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, len * sizeof(T), pt);
		}
		template<typename T>
		static cl::Buffer newBuffer(size_t len) {
			return cl::Buffer(context, CL_MEM_READ_WRITE, len * sizeof(T));
		}
		template<typename T>
		static cl::Buffer newWriteBuffer(size_t len) {
			return cl::Buffer(context, CL_MEM_WRITE_ONLY, len * sizeof(T), nullptr);
		}
		template<typename T>
		static std::vector<T> readBuffer(const cl::Buffer& buffer) {
			size_t size = buffer.getInfo<CL_MEM_SIZE>();
			std::vector<T> ans;
			ans.resize(size / sizeof(T));
			enqueueReadBuffer(buffer, 0, size, ans.data());
			return ans;
		}


		static void readBuffer(const cl::Buffer& buffer, void* pt);
		static void writeBuffer(const cl::Buffer& buffer, void* pt);


		static void enqueue1DKernelWithGroupCount(cl::Kernel& kernel, size_t workGroupCount, size_t singleGroupWorkItemCount, const std::vector<cl::Event>* evs = nullptr, cl::Event* ev = nullptr);

		static void printDeviceInformation(cl::Device& device);



		static std::vector< cl::Platform > platforms;
		static std::vector< cl::Device > devices;

	private:
		static cl::Context context;
		static cl::CommandQueue queue;
		static bool has_init;
	};

	typedef cl_float2 float2;
	typedef cl_float4 float4;
	typedef cl_float8 float8;
	typedef cl_float16 float16;
	typedef cl_int2 int2;
	typedef cl_uint2 uint2;
	typedef cl_int4 int4;
	typedef cl_uint4 uint4;
	typedef cl_uint uint;




	
	template<class T,class T2 = float>
	struct GetVecSize {
		typedef decltype(T::s[0]) basetype;
		typedef T2 floattype;
		constexpr static size_t len = sizeof(T::s) / sizeof(T::s[0]);
		constexpr static bool succ = ((len == 3) || (len == 4));
	};
	
	
	template<class T>
	T cross(const T& a, const T& b) {
		static_assert(GetVecSize<T>::succ);
		T ans = { 0 };
		ans.s[0] = a.s[1] * b.s[2] - a.s[2] * b.s[1];
		ans.s[1] = -a.s[0] * b.s[2] + a.s[2] * b.s[0];
		ans.s[2] = a.s[0] * b.s[1] - a.s[1] * b.s[0];
		return ans;
	}

	template<class T>
	typename GetVecSize<T>::floattype dot(const T& a, const T& b) {
		static_assert(GetVecSize<T>::succ);
		typename GetVecSize<T>::floattype ans = 0;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans += a.s[i] * b.s[i];
		}
		return ans;
	}
	
	template<class T>
	T normalize(const T& a) {
		T b = a;
		typename GetVecSize<T>::floattype ans = dot(a, a);
		ans = sqrt(ans);
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			b.s[i] /= ans;
		}
		return b;
	}
	template<class T>
	typename GetVecSize<T>::floattype len(const T& a) {
		return sqrt(dot(a, a));
	}
	
	
	
	template<class T>
	T operator+(const T& a, const T& b) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = a.s[i] + b.s[i];
		}
		return ans;
	}
	
	template<class T>
	T operator-(const T& a, const T& b) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = a.s[i] - b.s[i];
		}
		return ans;
	}
	
	template<class T>
	T operator*(const T& a, const T& b) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = a.s[i] * b.s[i];
		}
		return ans;
	}
	
	template<class T>
	T operator/(const T& a, const T& b) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = a.s[i] / b.s[i];
		}
		return ans;
	}

	constexpr double M_PI = 3.14159265358;
	constexpr float EPSILON = 0.001f;

	template<class T>
	T min(const T& a, const T& b) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = std::min(a.s[i], b.s[i]);
		}
		return ans;
	}
	template<class T>
	T fmin(const T& a, const T& b) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = std::min(a.s[i], b.s[i]);
		}
		return ans;
	}
	

	template<class T>
	T max(const T& a, const T& b) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = std::max(a.s[i], b.s[i]);
		}
		return ans;
	}
	template<class T>
	T fmax(const T& a, const T& b) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = std::max(a.s[i], b.s[i]);
		}
		return ans;
	}

	template<class T,class T2 = float>
	T operator*(T2 sc, const T& a) {
		T ans;
		for (size_t i = 0; i < GetVecSize<T>::len; ++i) {
			ans.s[i] = sc * a.s[i];
		}
		return ans;
	}

	
	

}
