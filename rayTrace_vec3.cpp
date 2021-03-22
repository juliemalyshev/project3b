//CSCI 5607 HW3 - Rays & Files
//This HW has three steps:
// 1. Compile and run the program (the program takes a single command line argument)
// 2. Understand the code in this file (rayTrace_vec3.cpp), in particular be sure to understand:
//     -How ray-sphere intersection works
//     -How the rays are being generated
//     -The pipeline from rays, to intersection, to pixel color
//    After you finish this step, and understand the math, take the HW quiz on canvas
// 3. Update the file parse_vec3.h so that the function parseSceneFile() reads the passed in file
//     and sets the relevant global variables for the rest of the code to product to correct image

//To Compile: g++ -fsanitize=address -std=c++11 rayTrace_vec3.cpp

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

//Images Lib includes:
#define STB_IMAGE_IMPLEMENTATION //only place once in one .cpp file
#define STB_IMAGE_WRITE_IMPLEMENTATION //only place once in one .cpp files
#include "image_lib.h" //Defines an image class and a color class
#include <iostream>
//#Vec3 Library
#include "vec3.h"

//High resolution timer
#include <chrono>

//Scene file parser
#include "parse_vec3.h"
vec3 intPoint;
bool intersect = false;

//Tests is the ray intersects the sphere
float raySphereIntersect(vec3 start, vec3 dir, vec3 center, float radius,int q){
  float a = dot(dir,dir); //TODO: What do we know about "a" if "dir" is normalized on creation?
  vec3 toStart = (start - center);
  float b = 2 * dot(dir,toStart);
  float c = dot(toStart,toStart) - radius*radius;
  float discr = b*b - (4*a*c);
  float tPosMin=0;
  spheres[q].hit=false;
  if (discr < 0){
    spheres[q].hit = false;
    //intPoint = toStart + (tPosMin*dir);
    return -1;
  }
  else{
    float t0 = (-b + (sqrt(discr))/(2*a));
    float t1 = (-b - (sqrt(discr))/(2*a));

    if (t0 > 0 && t1 > 0){
      tPosMin = std::fmin(t0,t1);
      //std::cout<<tPosMin<<"\n";
      spheres[q].hit = true;
    }
    else if (t0 < 0 && t1 < 0){
      spheres[q].hit = false;
    }
    else if (t0 > 0 && t1 < 0 || t0 < 0 && t1 > 0){
      tPosMin = std::fmax(t0,t1);
      spheres[q].hit = true;
    }
      //intPoint = toStart + (tPosMin*dir);
      //std::cout<<"HELLOOO"<<tPosMin<<"\n";
      //spheres[q].intersection = intPoint;
      return tPosMin;
    }
  //intPoint = toStart + (tPosMin*dir);
  //spheres[q].hit = false;
  return -1;
}
bool raySphereIntersectBool(vec3 start, vec3 dir, vec3 center, float radius){
  float a = dot(dir,dir); //TODO: What do we know about "a" if "dir" is normalized on creation?
  vec3 toStart = (start - center);
  float b = 2 * dot(dir,toStart);
  float c = dot(toStart,toStart) - radius*radius;
  float discr = b*b - 4*a*c;
  if (discr < 0) return false;
  else{
    float t0 = (-b + sqrt(discr))/(2*a);
    float t1 = (-b - sqrt(discr))/(2*a);
    if (t0 > 0 || t1 > 0) return true;
  }
  return false;
}
int closestSphere(vec3 start, vec3 dir, hitInfo centers[]){
  float min = 10000;
  int indexToUse = -1;

  for(int q = 0; q < i;q++){
      float hit = raySphereIntersect(start,dir,centers[q].pos,centers[q].radius,q);
      //std::cout<<hit<<"\n";
      if(hit>0 && hit<min){
        indexToUse = q;
        min = hit;

        spheres[q].intersection = start+(hit*dir);
    }

  }
    if(indexToUse>=0){
      return indexToUse;
    }

  return -1;
}
/*int recursiveDone = 0;
int r,g,b = 0;
Color recursiveReflection(vec3 ray, int max,int closest){

  struct hitInfo closestSphere = spheres[closest];
  vec3 lightDirP = (lightPosP - intPoint).normalized();

  float attenuatedLightIntensityR = rs / (pow(lightDirP.length(), 1.5));
  float attenuatedLightIntensityG = gs / (pow(lightDirP.length(),1.5));
  float attenuatedLightIntensityB = bs / (pow(lightDirP.length(), 1.5));

  vec3 normal = (closestSphere.pos-intPoint ).normalized();

  float ndotl = std::fmax(dot(vec3(0,lightDirP.y/3,0), normal), 0.0);
  recursiveDone++;
  vec3 reflect = (2*dot( normal,-1*ray) * normal+ray).normalized();
  vec3 shadowRayOrigin =  ((0.001*normal)-intPoint);
  /*vec3 shadowRay = (lightPosP-shadowRayOrigin);
  int closestSphereShadow = closestSphere(shadowRayOrigin, shadowRay, spheres);
  //bool shadowIdx = raySphereIntersectBool(shadowRayOrigin,shadowRay,spheres[closestSphereShadow].pos,spheres[closestSphereShadow].radius);
  if(closestSphereShadow>=0){

    if(shadowIdx){
      return Color(0,0,0);
    }

  }
  if(!shadowIdx){

    r += (attenuatedLightIntensityR*materials[closestSphere.materialID][3] * ndotl);
    g += (attenuatedLightIntensityG*materials[closestSphere.materialID][4] * ndotl);
    b += (attenuatedLightIntensityB*materials[closestSphere.materialID][5] * ndotl);
  //}
  if(maxDepth!=0){
    r += (materials[closestSphere.materialID][10]*recursiveReflection(reflect,maxDepth,closest).r);
    g += (materials[closestSphere.materialID][11]*recursiveReflection(reflect,maxDepth,closest).b);
    b += (materials[closestSphere.materialID][12]*recursiveReflection(reflect,maxDepth,closest).g);
  }else{
  return Color(r,g,b);
}
}*/
bool SameSide(vec3 p1,vec3 p2,vec3 a,vec3 b){
    vec3 cp1 = cross(b-a, p1-a);
    vec3 cp2 = cross(b-a, p2-a);
    if (dot(cp1, cp2) >= 0){
      return true;
    }
    else{
       return false;
     }
   }

bool triangleIntersect(vec3 p, vec3 a,vec3 b, vec3 c){
    if (SameSide(p,a, b,c) && SameSide(p,b, a,c) && SameSide(p,c, a,b)){
      return true;
    }
    else{
       return false;
     }
}
bool planeIntersect(vec3 n, vec3 p0, vec3 l0, vec3 l)
{
     float t;
    // assuming vectors are all normalized
    float denom = dot(n, l);
    if (denom > .000001) {
        vec3 p0l0 = p0 - l0;
        t = dot(p0l0, n) / denom;
        return (t >= 0);
    }
    return false;
}
int main(int argc, char** argv){

  //Read command line paramaters to get scene file
  if (argc != 2){
     std::cout << "Usage: ./a.out scenefile\n";
     return(0);
  }
  std::string secenFileName = argv[1];

  //Parse Scene File
  parseSceneFile(secenFileName);

  float imgW = img_width, imgH = img_height;
  float halfW = imgW/2, halfH = imgH/2;
  float d = halfH / tanf(halfAngleVFOV * (M_PI / 180.0f));

  Image outputImg = Image(img_width,img_height);
  auto t_start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < img_width; i++){
    for (int j = 0; j < img_height; j++){
      Color color;
      //TODO: In what way does this assumes the basis is orthonormal?
      //for(int q = 0; q < 10; q++){
        float u = (halfW - (imgW)*(i/imgW));
        float v = (halfH - (imgH)*(j/imgH));
        vec3 p = eye - d*forward + u*right + v*up;

        vec3 rayDir = (p - eye).normalized();  //Normalizing here is optional
        int closestIdx = closestSphere(eye,rayDir,spheres);

        vec3 triNormal = cross(triangles[1]-triangles[0],triangles[2]-triangles[0]).normalized();
        vec3 planeNormal = cross(planes[1]-planes[0],planes[2]-planes[0]).normalized();
        //std::cout<<triangles[1].x<<triangles[1].y;
        float calc = -1*dot(triNormal, triangles[0]);
        float triT =  ((dot(triNormal, eye) + calc) / (dot(triNormal, rayDir)));
        vec3 thit = eye + (triT * rayDir);
        bool triangleInt = triangleIntersect(thit,triangles[0],triangles[1],triangles[2]);
        bool planeInt = planeIntersect(planeNormal,planePos,eye,rayDir);
        //triangles

        if(closestIdx>=0){
          struct hitInfo closest = spheres[closestIdx];
          closest.intersection = -1 *closest.intersection;
          intPoint = -1 *intPoint;
          vec3 rayToLight = (lightPosP-closest.intersection);
          if (closest.hit){

            vec3 normal = (closest.pos-closest.intersection).normalized();


            //std::cout<<closest.intersection.x<<closest.intersection.y<<closest.intersection.z<<"\n";
            //vec3 lightDir = vec3(0,1,0);
            //rayToLight = rayToLight.normalized();
            vec3 lightDirP = (lightPosP - closest.intersection).normalized();
            //vec3 lightDirO = (closest.intersection - lightPosP).normalized();
            vec3 reflect = (2*dot( normal,lightDirP) * normal+lightDirP).normalized();
            float ndotl = std::fmax(dot(vec3(0,lightDirP.y/3,0), normal), 0.0);
            float ndotlS = std::fmax(dot(vec3(0,lightDirS.y/3,0), normal), 0.0);
            vec3 ve =  (eye-closest.intersection).normalized();

            //std::cout<< lightDirP.y<<"\n";
            float diffuseR;
            float attenuatedLightIntensityR = rs / (pow(lightDirP.length(), 1.5));
            float attenuatedLightIntensityG = gs / (pow(lightDirP.length(),1.5));
            float attenuatedLightIntensityB = bs / (pow(lightDirP.length(), 1.5));

            //spotlight calculation
            vec3 rayToInt = (lightPosS-closest.intersection);
            float spotFactor = dot(rayToInt,lightDirS);
            //L = normalize( light.position.xyz/light.position.w - v_eyeCoords );
            //if (10 > 0.0) { // the light is a spotlight
            vec3 d = lightDirS.normalized();  // unit vector!
            float spotCosine = dot(d,-1*rayToInt);
            float spotlight_diffuseR,spotlight_diffuseG,spotlight_diffuseB;
            spotCosine = 49;

            if (spotCosine >= angle1 && spotCosine<= angle2) {

                spotlight_diffuseR = sIntensityr*(rayToInt.length() * ndotlS);
                spotlight_diffuseG = sIntensityg*(rayToInt.length() * ndotlS);
                spotlight_diffuseB = sIntensityb*(rayToInt.length() * ndotlS);
                //std::cout<<spotlight_diffuseG<<"\n";
            }
            //}
            //std::cout<<pow(std::fmax(0,-1*dot(ve,reflect)), highlight)<<"\n";
            //std::cout<<spotCosine<<"\n";
            //std::cout<< (materials[closest.materialID][6] * ioR* pow(std::fmax(0,dot(ve,-1*reflect)), highlight))<< " ";
            float r = (materials[closest.materialID][0] * ra) + (attenuatedLightIntensityR*materials[closest.materialID][3] * ndotl) + ((materials[closest.materialID][6] * pow(std::fmax(0,dot(reflect,ve)), highlight))) ;//+ spotlight_diffuseR;// + sIntensityr;
            float g = (materials[closest.materialID][1] * ga) + (attenuatedLightIntensityG*materials[closest.materialID][4] * ndotl) + ((materials[closest.materialID][7] * pow(std::fmax(0,dot(reflect,ve)), highlight)));// +spotlight_diffuseG;//+ sIntensityg;
            float b = (materials[closest.materialID][2] * ba) + (attenuatedLightIntensityB*materials[closest.materialID][5] * ndotl) + ((materials[closest.materialID][8] * pow(std::fmax(0,dot(reflect,ve)), highlight)));// + spotlight_diffuseB;//+ sIntensityb;


            color = Color(r,g,b);



            //std::cout<<shadowRay.x<<" " <<shadowRay.y<<"\n";
            //std::cout<<shadowIdx<<"\n";
            //std::cout<<" "<<shadowRayOrigin.x<<" "<<shadowRayOrigin.y<<" "<<shadowRayOrigin.z;
            vec3 shadowRayOrigin =  ((0.00001*normal)-closest.intersection);
            vec3 shadowRay = (lightPosP-shadowRayOrigin);
            int closestSphereShadow = closestSphere(shadowRayOrigin, shadowRay,spheres);

            if(closestSphereShadow>=0){
              //bool shadowIdx = raySphereIntersectBool(shadowRayOrigin,shadowRay,spheres[closestSphereShadow].pos,spheres[closestSphereShadow].radius);
              if(spheres[closestSphereShadow].hit){
                color = Color(0,0,0);
              }
              for(int i = 0; i < 4;i++){
                spheres[i].hit = false;
              }
            }
          }

        }else{
          color = Color(red,green,blue);
        }
        if(triangleInt){
          vec3 lightDirP = (lightPosP - thit).normalized();
          float attenuatedLightIntensityR = (pow(lightDirP.length(), 0.5));
          float attenuatedLightIntensityG = (pow(lightDirP.length(),0.5));
          float attenuatedLightIntensityB = (pow(lightDirP.length(), 0.5));
          float ndotl = std::fmax(-1*dot(lightDirP, triNormal),0.0);
          std::cout<<ndotl<<"\n";
          color = Color(0.4,0.2,0);
        }

      outputImg.setPixel(i,j, color);

    }
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  printf("Rendering took %.2f ms\n",std::chrono::duration<double, std::milli>(t_end-t_start).count());

  outputImg.write(imgName.c_str());
  return 0;
} 
