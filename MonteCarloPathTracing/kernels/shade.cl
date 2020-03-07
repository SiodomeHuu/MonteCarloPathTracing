inline uint random(uint* x) {
	uint temp = (*x);
	temp = temp * 1103515245 + 12345;
	*x = temp;
	return((temp >> 16) & 0x00007FFF);
}

/*
uint random(uint* seed)
{
    *seed ^= *seed << 7;
    *seed ^= *seed >> 9;
    *seed ^= *seed << 8;
    ++*seed;
    return (*seed & 0x7FFF);
}*/


float4 mirrorDirection(float4 normal, float4 incoming) {
	normal.w = 0;
	incoming.w = 0;
	float4 ans = incoming - 2 * dot(normal, incoming) * normal;
	ans.w = 0;
	return normalize(ans);
}

bool transmittedDirection(float4 normal, float4 incoming,
	float index_i, float index_t, float4* transmitted) {

	normal.w = 0;
	incoming.w = 0;
	float ita_r = index_i / index_t;
	float n_dot_i = -dot(normal, incoming);
	float temp = 1.0f - ita_r * ita_r*(1 - n_dot_i * n_dot_i);
	if (temp < 0.0f) return false;
	*transmitted = normalize((ita_r*n_dot_i - sqrt(temp)) *normal + ita_r * incoming);
	return true;
}

float4 randomDirection(float4 normal, uint* seed) {
	float deg = 0, deg2 = 0;
	float4 axis1;
	float4 axis2;
	normal.w=0;

	deg = 2 * M_PI / 32768 * (random(seed));
	deg2 =  random(seed)*1.0f/32768;
	float s = sqrt(deg2);

	if(normal.z == 0) {
		axis1 = (float4)(0, 0 , 1.0f ,0);
	}
	else {
		axis1 = (float4)(1,0,0,0);
	}
	axis2 = normalize(cross(axis1,normal));
	axis1 = normalize(cross(axis2,normal));
	return normalize(cos(deg) * s * axis1 + sin(deg) * s * axis2 + (1-deg2) * normal);
}



__kernel void shade(
	__global Material* materials,
	__global Ray* rays,
	__global Hit* hits,
	__global float4* colorBuffer,

	__global uint* randomNum
) {
	size_t id = get_global_id(0);
	
	Hit myHit = hits[id];
	int materialID = myHit.materialID;


	if(rays[id].term_depth.w & 0xFF000000) {
		return;
	}
	else if(myHit.t >= FLT_MAX) { // not intersected, then out
		colorBuffer[id] = (float4)(0.0f,0.0f,0.0f,0.0f);
		rays[id].term_depth.w |= 0xFF000000;
		return;
	}

	Ray myRay = rays[id];
	Ray newRay;

	uint myRandSeed = randomNum[id];

	__global Material* pMat = materials + materialID;
	
	float4 reflect;
	bool ifTransmit;
	bool ifInObj;
	float i;
	float t;
	

	switch(pMat->type) {
	case MCPT_DIFFUSE:
		newRay.direction = randomDirection(myHit.normal,&myRandSeed);
		newRay.origin = myHit.intersectPoint + EPSILON * newRay.direction;

		newRay.term_depth.w = myRay.term_depth.w + 1;
		newRay.id.w = myRay.id.w;
		colorBuffer[id] = colorBuffer[id] * pMat->kd * dot(newRay.direction.s012, myHit.normal.s012) / (float)(2*M_PI);

		randomNum[id] = myRandSeed;
		break;
	case MCPT_GLOSSY:
		if(random(&myRandSeed) & 0x00000001) {
			//phong
			reflect = mirrorDirection(myHit.normal,myRay.direction);
			newRay.direction = randomDirection(reflect,&myRandSeed);

			while( dot(newRay.direction.s012,myHit.normal.s012) <= 0) {
				newRay.direction = randomDirection(reflect,&myRandSeed);
			}

			newRay.origin = myHit.intersectPoint + EPSILON * newRay.direction;
			newRay.term_depth.w = myRay.term_depth.w + 1;
			newRay.id.w = myRay.id.w;

			colorBuffer[id] = colorBuffer[id] * pMat->ks
				* pow(dot(newRay.direction.s012,reflect.s012),materials[materialID].Ns)
				/** dot(newRay.direction.s012,myHit.normal.s012)*/ / (float)(2*M_PI);
			randomNum[id] = myRandSeed;
		}
		else {
			//diffuse
			newRay.direction = randomDirection(myHit.normal,&myRandSeed);
			newRay.origin = myHit.intersectPoint + EPSILON * newRay.direction;

			newRay.term_depth.w = myRay.term_depth.w + 1;
			newRay.id.w = myRay.id.w;
			colorBuffer[id] = colorBuffer[id] * pMat->kd * dot(newRay.direction.s012,myHit.normal.s012) / (float)(2*M_PI);

			randomNum[id] = myRandSeed;
		}
		break;
	case MCPT_LIGHT:
		rays[id].term_depth.w |= 0xFF000000;
		colorBuffer[id] = colorBuffer[id] * materials[materialID].ka;
		return;
	case MCPT_TRANSPARENT:
		ifInObj = myRay.term_depth.w & 0x00FF0000;
		if(ifInObj){
			i = pMat->Ni;
			t = 1.0f;
		}
		else {
			i = 1.0f;
			t = pMat->Ni;
		}
		ifTransmit = transmittedDirection(myHit.normal,myRay.direction,i,t,&newRay.direction);
		if(!ifTransmit) {
			newRay.origin = myHit.intersectPoint;
			//newRay.direction = mirrorDirection(myHit.normal,myRay.direction);
			newRay.direction = randomDirection(myHit.normal,&myRandSeed);
			newRay.id.w = myRay.id.w;
			newRay.term_depth.w = myRay.term_depth.w + 1;
			
			colorBuffer[id] = colorBuffer[id] * dot(myRay.direction.s012,myHit.normal.s012) / (float)(2*M_PI);
			//rays[id].term_depth.w |= 0xFF000000;
			//colorBuffer[id] = (float4)(0,0,0,0);
			return;
		}
		else {
			newRay.origin = myHit.intersectPoint + EPSILON * newRay.direction;
			newRay.id.w = myRay.id.w;
			newRay.term_depth.w = myRay.term_depth.w + 1;
			newRay.term_depth.w ^= 0x00FF0000;
		}
		break;
	default:
		printf("Crash!!!\n");
		break;
	}
	
	if((newRay.term_depth.w & 0x0000FFFF) >= MAX_DEPTH && !(myRay.term_depth.w & 0xFF000000)) {
		colorBuffer[id] = (float4)(0,0,0,0);
		myRay.term_depth.w |= 0xFF000000;
	}
	rays[id] = newRay;

	return;
}