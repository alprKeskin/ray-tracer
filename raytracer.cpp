#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
#include <limits>

using namespace parser;
using namespace std;

enum Shape {
    SPHERE, TRIANGLE, MESH
};

struct PixelColor {
    float red;
    float green;
    float blue;

    PixelColor(float red, float green, float blue) {
        this -> red = red;
        this -> green = green;
        this -> blue = blue;
    }
};

struct Hit {
    bool isHit;
    int materialId;
    Vec3f hitPoint;
    float t;
    Shape objectShape;
    Vec3f normal;
    int objectId;
    Hit (bool isHit, int materialId, Vec3f hitPoint, float t, Shape objectShape, Vec3f normal, int objectId) {
        this -> isHit = isHit;
        this -> materialId = materialId;
        this -> hitPoint = hitPoint;
        this -> t = t;
        this -> objectShape = objectShape;
        this -> normal = normal;
        this -> objectId = objectId;
    }
};

class Math {
    public:
        static float dotProduct(const Vec3f &vector1, const Vec3f &vector2) {
	        return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
        }
        
        static Vec3f addVectors(const Vec3f &vector1, const Vec3f &vector2) {
            Vec3f result;
            result.x = vector1.x + vector2.x;
            result.y = vector1.y + vector2.y;
            result.z = vector1.z + vector2.z;
            return result;
        }

        static Vec3f subtractVectors(const Vec3f &vector1, const Vec3f &vector2) {
            Vec3f result;
            result.x = vector1.x - vector2.x;
            result.y = vector1.y - vector2.y;
            result.z = vector1.z - vector2.z;
            return result;
        }

        static Vec3f scalarMultiplication(const Vec3f &vector, float num) {
            Vec3f result;
            result.x = vector.x * num;
            result.y = vector.y * num;
            result.z = vector.z * num;
            return result;
        }

        static Vec3f crossProduct(const Vec3f &vector1, const Vec3f &vector2) {
            Vec3f result;
            result.x = vector1.y * vector2.z - vector1.z * vector2.y;
            result.y = vector1.z * vector2.x - vector1.x * vector2.z;
            result.z = vector1.x * vector2.y - vector1.y * vector2.x;
            return result;
        }

        static Vec3f normalizeVector(const Vec3f &vector) {
            float vectorLength = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
	        Vec3f normalizedVector;
	        normalizedVector.x = vector.x / vectorLength;
	        normalizedVector.y = vector.y / vectorLength;
	        normalizedVector.z = vector.z / vectorLength;
	        return normalizedVector;
        }

        static float determinantRow(const Vec3f& vector1, const Vec3f& vector2, const Vec3f& vector3) {
            Vec3f crossProduct;
            crossProduct.x = vector2.y * vector3.z - vector2.z * vector3.y;
            crossProduct.y = vector2.z * vector3.x - vector2.x * vector3.z;
            crossProduct.z = vector2.x * vector3.y - vector2.y * vector3.x;

            float determinant = vector1.x * crossProduct.x + vector1.y * crossProduct.y + vector1.z * crossProduct.z;

            return determinant;
        }
        
        static float determinantColumn(const Vec3f& vector1, const Vec3f& vector2, const Vec3f& vector3){
            float determinant = vector1.x * (vector2.y * vector3.z - vector2.z * vector3.y) - vector1.y * (vector2.x * vector3.z - vector2.z * vector3.x) + vector1.z * (vector2.x * vector3.y - vector2.y * vector3.x);
            return determinant;
        }

        static float discriminant(float a, float b, float c) {
            return b * b - 4 * a * c;
        }

        static float calculateDistanceBetweenTwoPoint(Vec3f startPoint, Vec3f endPoint) {
            return sqrt(pow(endPoint.x - startPoint.x, 2) + pow(endPoint.y - startPoint.y, 2) + pow(endPoint.z - startPoint.z, 2));
        }
};

class Ray {
    public:
        Vec3f o;
        Vec3f d;
        Ray(Vec3f o, Vec3f d) {
            this -> o = o;
            this -> d = d;
        }
        
        Vec3f getPoint(float t) {
            return Math::addVectors(this -> o, Math::scalarMultiplication(this -> d, t));
        }
};

float calculateS_u(float left, float right, float image_width, int i) {
    return (right - left)*(i + 0.5)/image_width;
}

float calculateS_v(float bottom, float top, float image_height, int j) {
    return (top - bottom)*(j + 0.5)/image_height;
}

Vec3f calculateM(Vec3f e, Vec3f w, float distance) {
    return Math::addVectors(e, Math::scalarMultiplication(w, -distance));
}

Vec3f calculateQ(Vec3f e, Vec3f w, float distance, Vec3f u, Vec3f v, float left, float top) {
    return Math::addVectors(calculateM(e, w, distance), (Math::addVectors(Math::scalarMultiplication(u, left), Math::scalarMultiplication(v, top)))); 
}

Vec3f calculateS(Vec3f e, Vec3f w, float distance, Vec3f u, Vec3f v, float left, float top, float s_u, float s_v) {
    return Math::addVectors(Math::addVectors(calculateQ(e, w, distance, u, v, left, top), Math::scalarMultiplication(u, s_u)), Math::scalarMultiplication(v, -s_v));
}

Vec3f calculateD(Vec3f e, Vec3f w, float distance, Vec3f u, Vec3f v, float left, float top, float s_u, float s_v) {
    return Math::subtractVectors(calculateS(e, w, distance, u, v, left, top, s_u, s_v), e);
}

Ray* createRay(Vec3f o, Vec3f d) {
    return new Ray(o, d);
}

Hit sphereIntersection(const Scene& scene, const Sphere& sphere, Ray* ray){
    Vec3f center;
    center = scene.vertex_data[sphere.center_vertex_id - 1];
    float radius = sphere.radius;
    Vec3f o = ray -> o;
    Vec3f d = ray -> d;

    Vec3f oMinusCenter = Math::subtractVectors(o, center);
    float A = Math::dotProduct(d, d);
    float B = 2 * Math::dotProduct(d, oMinusCenter);
    float C = Math::dotProduct(oMinusCenter, oMinusCenter) - radius * radius;
    
    float discriminant = Math::discriminant(A, B, C);
    if (discriminant < 0) {
        return Hit(false, 0, Vec3f{0, 0, 0}, 0, SPHERE, Vec3f{0, 0, 0}, -1);
    }
    else if (discriminant == 0) {
        float t = -B / (2*A);
        Vec3f intersectionPoint = ray -> getPoint(t);
        Vec3f normal = Math::normalizeVector(Math::subtractVectors(intersectionPoint, center));
        return Hit(true, sphere.material_id, intersectionPoint, t, SPHERE, normal, -1);
    }
    else {
        float t = (-B - sqrtf(discriminant)) / (2 * A);
        Vec3f intersectionPoint = ray -> getPoint(t);
        Vec3f normal = Math::normalizeVector(Math::subtractVectors(intersectionPoint, center));
        return Hit(true, sphere.material_id, intersectionPoint, t, SPHERE, normal, -1);
    }
}

Hit triangleIntersection(const Scene& scene, const Triangle& triangle, Ray* ray){
    Vec3f v0 = scene.vertex_data[triangle.indices.v0_id - 1];
    Vec3f v1 = scene.vertex_data[triangle.indices.v1_id - 1];
    Vec3f v2 = scene.vertex_data[triangle.indices.v2_id - 1];
    Vec3f o = ray->o;
    Vec3f d = ray->d;

    Vec3f aMinusB = Math::subtractVectors(v0, v1);
    Vec3f aMinusC = Math::subtractVectors(v0, v2);
    Vec3f aMinusO = Math::subtractVectors(v0, o);

    float determinantA = Math::determinantColumn(aMinusB, aMinusC, d);
    float t = Math::determinantColumn(aMinusB, aMinusC, aMinusO) / determinantA;
    float beta = Math::determinantColumn(aMinusO, aMinusC, d) / determinantA;
    float gamma = Math::determinantColumn(aMinusB, aMinusO, d) / determinantA;
    if (beta >= 0 && gamma >= 0 && beta + gamma <= 1) {
        Vec3f intersectionPoint = ray->getPoint(t);
        Vec3f normal = Math::normalizeVector(Math::crossProduct(aMinusB, aMinusC));
        Hit hit(true, triangle.material_id, intersectionPoint, t, TRIANGLE, normal, -1);
        return hit;
    }
    else { 
        return Hit(false, triangle.material_id, Vec3f{0, 0, 0}, t, TRIANGLE, Vec3f{0, 0, 0}, -1);
    }
}
    
Hit faceIntersection(const Scene& scene, const Mesh& mesh ,const Face& face, Ray* ray) {
    Vec3f v0 = scene.vertex_data[face.v0_id - 1];
    Vec3f v1 = scene.vertex_data[face.v1_id - 1];
    Vec3f v2 = scene.vertex_data[face.v2_id - 1];
    Vec3f o = ray->o;
    Vec3f d = ray->d;

    Vec3f aMinusB = Math::subtractVectors(v0, v1);
    Vec3f aMinusC = Math::subtractVectors(v0, v2);
    Vec3f aMinusO = Math::subtractVectors(v0, o);

    Vec3f faceNormal = Math::normalizeVector(Math::crossProduct(aMinusB, aMinusC));

    float determinantA = Math::determinantColumn(aMinusB, aMinusC, d);
    float t = Math::determinantColumn(aMinusB, aMinusC, aMinusO) / determinantA;
    float beta = Math::determinantColumn(aMinusO, aMinusC, d) / determinantA;
    float gamma = Math::determinantColumn(aMinusB, aMinusO, d) / determinantA;
    if (beta >= 0 && gamma >= 0 && beta + gamma <= 1) {
        Vec3f intersectionPoint = ray->getPoint(t);
        Hit hit(true, mesh.material_id, intersectionPoint, t, TRIANGLE, faceNormal,-1);
        return hit;
    }
    else { 
        return Hit(false, mesh.material_id, Vec3f{0, 0, 0}, t, TRIANGLE, Vec3f{0, 0, 0}, -1);;
    }
}

Hit getClosestObjectHitForTheRay(Ray* ray, const Scene &scene) {
    int spheresSize = scene.spheres.size();
    int trianglesSize = scene.triangles.size();
    int meshesSize = scene.meshes.size();
    float t_min = (float) INT16_MAX;

    Hit closestHit = Hit(false, 0, Vec3f{0, 0, 0}, 0, SPHERE, Vec3f{0, 0, 0},-1);

    for (int currentSphereIndex = 0; currentSphereIndex < spheresSize; currentSphereIndex++) {
        Sphere currentSphere = scene.spheres[currentSphereIndex];
        Hit currentHit = sphereIntersection(scene, currentSphere, ray);
        currentHit.objectId = currentSphereIndex;
        if (currentHit.isHit) {
            if (currentHit.t < t_min && currentHit.t > 0) {
                t_min = currentHit.t;
                closestHit = currentHit;
            }
        }
    }

    for (int currentTriangleIndex = 0; currentTriangleIndex < trianglesSize; currentTriangleIndex++) {
        Triangle currentTriangle = scene.triangles[currentTriangleIndex];
        Hit currentHit = triangleIntersection(scene, currentTriangle, ray);
        currentHit.objectId = currentTriangleIndex;
        if (currentHit.isHit) {
            if (currentHit.t < t_min && currentHit.t > 0) {
                t_min = currentHit.t;
                closestHit = currentHit;
            }
        }
    }
    
    for (int currentMeshIndex = 0; currentMeshIndex < meshesSize; currentMeshIndex++) {
        Mesh currentMesh = scene.meshes[currentMeshIndex];
        int facesSize = scene.meshes[currentMeshIndex].faces.size();
        for (int currentfacesIndex = 0; currentfacesIndex < facesSize; currentfacesIndex++) {
            Face currentFace = currentMesh.faces[currentfacesIndex];
            Hit currentHit = faceIntersection(scene, currentMesh, currentFace, ray);
            currentHit.objectId = currentMeshIndex;
            if (currentHit.isHit) {
                if (currentHit.t < t_min && currentHit.t > 0) {
                    t_min = currentHit.t;
                    closestHit = currentHit;
                }
            }
        }
    }
    return closestHit;
}

Vec3f calculateAmbient(const Scene& scene, const Hit& hit) {
    int materialId = hit.materialId;
    Vec3f sceneAmbientLight = scene.ambient_light;

    float pixelX = scene.materials[materialId - 1].ambient.x * sceneAmbientLight.x;
	float pixelY = scene.materials[materialId - 1].ambient.y * sceneAmbientLight.y;
	float pixelZ = scene.materials[materialId - 1].ambient.z * sceneAmbientLight.z;

    return Vec3f{pixelX, pixelY, pixelZ};
}

Vec3f calculateIrradiance(const PointLight& pointLight, const float distance) {
    return Math::scalarMultiplication(pointLight.intensity, 1 / (distance * distance));
}

Vec3f calculateDiffuse(const Scene& scene, const Hit& hit, const PointLight& pointLight) {
    Vec3f irradiance = calculateIrradiance(pointLight, Math::calculateDistanceBetweenTwoPoint(hit.hitPoint, pointLight.position));

    Vec3f directionFromIntersectionPointToPointLight = Math::normalizeVector(Math::subtractVectors(pointLight.position, hit.hitPoint));
    
    float cosine = fmax(0, Math::dotProduct(directionFromIntersectionPointToPointLight, hit.normal));
    
    return Vec3f{
        scene.materials[hit.materialId - 1].diffuse.x * irradiance.x * cosine,
        scene.materials[hit.materialId - 1].diffuse.y * irradiance.y * cosine,
        scene.materials[hit.materialId - 1].diffuse.z * irradiance.z * cosine,
    };
}

Vec3f calculateSpecular(const Scene& scene, const Hit& hit, const PointLight& pointLight, Ray* ray) {
    Vec3f directionFromIntersectionPointToPointLight = Math::normalizeVector(Math::subtractVectors(pointLight.position, hit.hitPoint));
    Vec3f h = Math::normalizeVector(Math::subtractVectors(directionFromIntersectionPointToPointLight, ray -> d));
    Vec3f irradiance = calculateIrradiance(pointLight, Math::calculateDistanceBetweenTwoPoint(hit.hitPoint, pointLight.position));
    float phong_exponent = scene.materials[hit.materialId - 1].phong_exponent;
    float cosine = fmax(0, Math::dotProduct(hit.normal, h));

    return Vec3f{
        scene.materials[hit.materialId - 1].specular.x * irradiance.x * pow(cosine, phong_exponent),
        scene.materials[hit.materialId - 1].specular.y * irradiance.y * pow(cosine, phong_exponent),
        scene.materials[hit.materialId - 1].specular.z * irradiance.z * pow(cosine, phong_exponent),
    };
}

bool isMirror(const Material& material) {
    return (material.mirror.x > 0 || material.mirror.y > 0 || material.mirror.z > 0);
}

Vec3f calculatePixelColor(const Scene& scene, const Hit hit, Ray* ray, int recursionCount) {
    Vec3f pixelColor = calculateAmbient(scene, hit);
    vector<PointLight> pointLights = scene.point_lights;
    int numberOfPointLight = pointLights.size();

    for (int currentPointLight = 0; currentPointLight < numberOfPointLight; currentPointLight++) {
        Vec3f wi = Math::subtractVectors(pointLights[currentPointLight].position, hit.hitPoint);
        Vec3f wiEpsilon = {
            wi.x * scene.shadow_ray_epsilon,
		    wi.y * scene.shadow_ray_epsilon,
		    wi.z * scene.shadow_ray_epsilon,
        };
		
        Ray* shadowRay = new Ray(Math::addVectors(hit.hitPoint, wiEpsilon), wi);

        Hit hitInBetween = getClosestObjectHitForTheRay(shadowRay, scene);
        delete shadowRay;
        // There is an object between the intersection point and light source 
        if (hitInBetween.isHit) {
            // point is in shadow – no contribution from this light
            float hitPointToPointLight = Math::calculateDistanceBetweenTwoPoint(hit.hitPoint, pointLights[currentPointLight].position);
            float hitPointToHitInBetween = Math::calculateDistanceBetweenTwoPoint(hit.hitPoint, hitInBetween.hitPoint);
            if (hitPointToHitInBetween < hitPointToPointLight) {
                continue;
            }
        }
        pixelColor = Math::addVectors(pixelColor, calculateDiffuse(scene, hit, pointLights[currentPointLight]));
        pixelColor = Math::addVectors(pixelColor, calculateSpecular(scene, hit, pointLights[currentPointLight], ray));
    }

    Vec3f reflectedPixelColor{0,0,0};
    if (isMirror(scene.materials[hit.materialId-1]) && recursionCount > 0) {
        Vec3f w_o = Math::scalarMultiplication(ray -> d, -1);
        float dotProduct = Math::dotProduct(hit.normal, w_o);
        Vec3f temp = Math::scalarMultiplication(hit.normal, dotProduct);
        temp = Math::scalarMultiplication(temp, 2);
        Vec3f w_r = Math::addVectors(ray -> d, temp);
        w_r = Math::normalizeVector(w_r);
        Vec3f wiNewEpsilon = {
            w_r.x * scene.shadow_ray_epsilon,
            w_r.y * scene.shadow_ray_epsilon,
            w_r.z * scene.shadow_ray_epsilon,
        };
        Ray* reflectionRay = new Ray(Math::addVectors(hit.hitPoint, wiNewEpsilon), w_r);
        Hit reflectionHit = getClosestObjectHitForTheRay(reflectionRay, scene);
        if(reflectionHit.isHit && !(reflectionHit.objectId == hit.objectId && reflectionHit.objectShape == hit.objectShape)) {
            reflectedPixelColor = calculatePixelColor(scene, reflectionHit, reflectionRay, recursionCount-1);
            delete reflectionRay;                
        }
        pixelColor.x += reflectedPixelColor.x * scene.materials[hit.materialId - 1].mirror.x;
        pixelColor.y += reflectedPixelColor.y * scene.materials[hit.materialId - 1].mirror.y;
        pixelColor.z += reflectedPixelColor.z * scene.materials[hit.materialId - 1].mirror.z;
    }
    return pixelColor;
}

int roundChannel(float value) {
    int valueInt = round(value);
    if (valueInt > 255) return 255;
    else return valueInt;
}


int main(int argc, char* argv[]) {
    parser::Scene scene;
    scene.loadFromXml(argv[1]);

    int cameraCount = scene.cameras.size();

    for (int cameraNumber = 0; cameraNumber < cameraCount; cameraNumber++) {
        Camera camera = scene.cameras[cameraNumber];

        // image width and height in pixels.
        int width = camera.image_width;
        int height = camera.image_height;

        unsigned char* image = new unsigned char [width * height * 3];
        int columnWidth = width / 8;
        int x = 0;

        // distance from camera to the image plane.
        float distance = camera.near_distance;

        float imagePlaneLeft = camera.near_plane.x;
        float imagePlaneRight = camera.near_plane.y;
        float imagePlaneBottom = camera.near_plane.z;
        float imagePlaneTop = camera.near_plane.w;

        Vec3f v = Math::normalizeVector(camera.up);
        Vec3f gaze = Math::normalizeVector(camera.gaze);
        Vec3f u = Math::normalizeVector(Math::crossProduct(gaze, v));
        Vec3f w = Math::scalarMultiplication(gaze, -1);
        Vec3f cameraPosition = camera.position;

        // for all rows
        for (int j = 0; j < height; j++) {
            // for all pixels in row j
            for (int i = 0; i < width; i++) {
                
                float s_u = calculateS_u(imagePlaneLeft, imagePlaneRight, width, i);
                float s_v = calculateS_v(imagePlaneBottom, imagePlaneTop, height, j);
                
                Ray* ray = createRay(cameraPosition, calculateD(cameraPosition, w, distance, u, v, imagePlaneLeft, imagePlaneTop, s_u, s_v));

                Hit closestHit = getClosestObjectHitForTheRay(ray, scene);

                if (closestHit.isHit) {
                    // pixel color calculate
                    Vec3f pixelColor = calculatePixelColor(scene, closestHit, ray, scene.max_recursion_depth);

                    if(pixelColor.x > 255) pixelColor.x = 255;
                    if(pixelColor.y > 255) pixelColor.y = 255;
                    if(pixelColor.z > 255) pixelColor.z = 255;

                    image[x++] = roundChannel(pixelColor.x);
                    image[x++] = roundChannel(pixelColor.y);
                    image[x++] = roundChannel(pixelColor.z);
                    delete ray;
                }
                else {
                    image[x++] = scene.background_color.x;
                    image[x++] = scene.background_color.y;
                    image[x++] = scene.background_color.z;
                }
            }
        }
        string imageName = camera.image_name;
        write_ppm(imageName.c_str(), image, width, height);
        
        delete[] image;
    } 
   return 0;
}