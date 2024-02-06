#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

inline float deg2rad(const float &deg)
{
    return deg * M_PI / 180.0;
}

// Compute reflection direction
// 计算反射方向 I:入射方向 N：法线方向
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
}

// [comment]
// Compute refraction direction using Snell's law
//
// We need to handle with care the two possible situations:
//
//    - When the ray is inside the object
//
//    - When the ray is outside.
//
// If the ray is outside, you need to make cosi positive cosi = -N.I
//
// If the ray is inside, you need to invert the refractive indices and negate the normal N
// [/comment]
// 计算返回折射方向
//        I:入射向量
//        N:法线向量
//        ior:折射率
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    Vector3f n = N;
    if (cosi < 0)
    {
        cosi = -cosi;
    }
    else
    {
        std::swap(etai, etat);
        n = -N;
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

// [comment]
// Compute Fresnel equation
// (菲涅尔方程)计算反射率
//
// \param I is the incident view direction
//
// \param N is the normal at the intersection point
//
// \param ior is the material refractive index
// [/comment]
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0)
    {
        std::swap(etai, etat);
    }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1)
    {
        return 1;
    }
    else
    {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}

// [comment]
// Returns true if the ray intersects an object, false otherwise.
// （返回光线与物体的交点(距离光源最近的交点)）

// \param orig is the ray origin
// （orig:光线的发射位置）

// \param dir is the ray direction
// （dir:光线方向）

// \param objects is the list of objects the scene contains
// （objects:场景中的所有物体）

// \param[out] tNear contains the distance to the cloesest intersected object.
// （tNear:光线与相交的物体的最近的距离）

// \param[out] index stores the index of the intersect triangle if the interesected object is a mesh.
// (index:如果相交的物体是mesh,index存储了相交三角形的索引)

// \param[out] uv stores the u and v barycentric coordinates of the intersected point
// (uv:交点的uv坐标)

// \param[out] *hitObject stores the pointer to the intersected object (used to retrieve material information, etc.)
// (hitObject:与光线相交的物体的指针)

// \param isShadowRay is it a shadow ray. We can return from the function sooner as soon as we have found a hit.
// (isShadowRay:是否是shadowRay)
// [/comment]

// （返回从orig向dir方向发射的射线与场景中物体的交点(距离光源最近的交点)）
std::optional<hit_payload> trace(
    const Vector3f &orig, const Vector3f &dir,
    const std::vector<std::unique_ptr<Object>> &objects)
{
    float tNear = kInfinity; //最近的交点距离，默认为无穷大
    std::optional<hit_payload> payload;//御用存储交点信息的可选类型

    // 遍历场景中的所有物体，判断光线与物体是否有交点（没有采用加速结构）
    for (const auto &object : objects)
    {
        float tNearK = kInfinity; //物体与光线的交点距离，默认为无穷大
        uint32_t indexK; //物体的Triangle数据的索引
        Vector2f uvK;   // 交点在Triangle中的重心坐标
        
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)
        {
            //找到距离光线最近的交点    
            payload.emplace();//创建一个新对象
            payload->hit_obj = object.get();
            payload->tNear = tNearK;
            payload->index = indexK;
            payload->uv = uvK;
            tNear = tNearK;
        }
    }

    return payload;
}

// [comment]
// Implementation of the Whitted-style light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refraction direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refraction depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is diffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
// [/comment]
// 		
// 使用光线追踪算法计算从原点orig沿着方向dir发出的光线与场景scene的交点，并返回交点处的颜色
Vector3f castRay(
    const Vector3f &orig, const Vector3f &dir, const Scene &scene,
    int depth)
{
    if (depth > scene.maxDepth)
    {
        //scene.maxDepth = 5：只计算光线最大发生5次反射（或折射）产生的光照效果
        return Vector3f(0.0, 0.0, 0.0);
    }

    //光照背景色
    Vector3f hitColor = scene.backgroundColor;
    

    if (auto payload = trace(orig, dir, scene.get_objects()); payload) // 这段代码是使用 C++17 的结构化绑定（Structured Bindings）特性来简化代码
    {                                                                  // 光线与场景中的物体有交点
        //获得距离光照最近的交点信息:payload



        // 光线与物体的交点
        Vector3f hitPoint = orig + dir * payload->tNear;

        // hitPoint的法线向量
        Vector3f N; // normal

        // hitPoint的纹理坐标
        Vector2f st; // st coordinates

        // 通过交点信息、光照信息计算出
        //      交点的法线向量N和纹理坐标st
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st);

        switch (payload->hit_obj->materialType)
        {
        case REFLECTION_AND_REFRACTION: // 物体材质：能反射、能折射
        {
            // 反射方向
            Vector3f reflectionDirection = normalize(reflect(dir, N));

            // 折射方向
            Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));




            // 新的反射光线原点
            // 新的反射光线的原点不设置为原来交点的原因：
            //       为了避免光线与自身相交，需要将起始点稍微偏移离开交点。
            //   反射方向和法线方向点积 < 0 : 交点在物体的内表面，新的反射光线的起始点向内偏移，所以 新的反射光线的原点 = 原来的交点 - N*scene.epsilon
            //   反射方向和法线方向点积 >= 0 : 交点在物体的外表面，新的反射光线的起始点向外偏移，所以 新的反射光线的原点 = 原来的交点 + N*scene.epsilon
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? hitPoint - N * scene.epsilon : hitPoint + N * scene.epsilon;

            // 折射光线原点
            Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ? hitPoint - N * scene.epsilon : hitPoint + N * scene.epsilon;

            // 递归调用得到反射的颜色
            Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);

            // 递归调用得到折射的颜色
            Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);

            // 计算反射率
            float kr = fresnel(dir, N, payload->hit_obj->ior);

            // 反射颜色和折射颜色加权
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);
            break;
        }
        case REFLECTION: // 物体材质：只能反射
        {
            // 计算反射率
            float kr = fresnel(dir, N, payload->hit_obj->ior);

            // 反射方向
            Vector3f reflectionDirection = reflect(dir, N);

            // 反射光线原点
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? hitPoint + N * scene.epsilon : hitPoint - N * scene.epsilon;
            // 递归调用得到反射颜色
            hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
            break;
        }
        default: // 物体材质： 既不能发射也不能折射   Phong illumation模型
        {
            // [comment]
            // We use the Phong illumation model int the default case. The phong model
            // is composed of a diffuse and a specular reflection component.
            // [/comment]
            Vector3f lightAmt = 0, specularColor = 0;

            // 阴影点
            Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ? hitPoint + N * scene.epsilon : hitPoint - N * scene.epsilon;

            // [comment]
            // Loop over all lights in the scene and sum their contribution up
            // We also apply the lambert cosine law
            // [/comment]
            for (auto &light : scene.get_lights())
            {
                // 光线方向
                Vector3f lightDir = light->position - hitPoint;

                // square of the distance between hitPoint and the light
                float lightDistance2 = dotProduct(lightDir, lightDir);
                
                lightDir = normalize(lightDir);

                float LdotN = std::max(0.f, dotProduct(lightDir, N));

                // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?

                //从shadowPointOrig向光源方向发射射线，找交点
                auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                
                // inShadow  true:光线无法直接照到当前hitPoint位置  false:光线可以直接照到当前hitPoint位置
                bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);

                //环境光
                lightAmt += inShadow ? 0 : light->intensity * LdotN;

                Vector3f reflectionDirection = reflect(-lightDir, N);

                // 高光颜色
                specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                                      payload->hit_obj->specularExponent) *
                                 light->intensity;
            }

            hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks;
            break;
        }
        }
    }

    return hitColor;
}

// [comment]
// The main render function. This where we iterate over all pixels in the image, generate
// primary rays and cast these rays into the scene. The content of the framebuffer is
// saved to a file.
// [/comment]
void Renderer::Render(const Scene &scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;

    // Use this variable as the eye position to start your rays.
    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            float x;
            float y;
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.
            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio*

            // x:(0,scene.width)  =>  x：(0,1)
            x = (i + 0.5) / scene.width;
            // y:(0,scene.height)  =>  x：(0,1)
            y = (j + 0.5) / scene.height;

            // x:(0,1)   =>  x:(-1,1)
            x = 2 * x - 1;
            // y:(0,1)   =>  y:(1,-1)
            y = -2 * y + 1;

            // 对于宽高比！=1的屏幕，像素不是正方形，x坐标乘上宽高比
            x = x * imageAspectRatio;

            // scale = tan(fov/2) ,摄像机到近裁剪平面的距离 = 1 ，所以 宽高需要乘缩放比例 scale
            x = x * scale;
            y = y * scale;

            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
