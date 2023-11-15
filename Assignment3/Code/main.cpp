#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

using namespace std;

// V矩阵： 世界空间 ->  观察空间
// eye_pos：摄像机位置 {0，0，10}（在世界空间下的位置）
//  摄像机坐标和世界坐标一样使用右手坐标系,但是摄像机的 视锥体 朝向-z方向
//
// 两种思考方式：
//     方式1： 1.计算观察空间的三个坐标轴在世界空间的表示，以及观察空间原点的位置，构建从观察空间变换到世界空间的变换矩阵
//            2. 对该矩阵求逆得到世界空间到观察空间的变换矩阵V
//
//     方式2：1.想象平移整个观察空间，让摄像机原点位于世界坐标的原点，坐标轴与世界空间中的坐标轴重合
//           2.这种方法得到的矩阵和方式1得到的矩阵是一样的，即为矩阵V
//             (这里观察空间位置是{0，0，10}，平移到原点按{0，0，-10}平移)
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

// M矩阵：模型空间  ->  世界空间
//  "/models/spot/spot_griangulated_good.obj"模型使用的是右手坐标系
//  此处世界坐标和模型坐标一样使用右手坐标系，世界坐标原点和模型原点重合
//  xyz轴绕y轴旋转140度(x'与x相差140度 z'与z相差140度) 得到世界坐标的xyz朝向

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    cout << "M矩阵 绕Y轴旋转：" << angle << "度" << endl;

    // 角度转换成弧度
    angle = angle * MY_PI / 180.f;

    // 绕y轴旋angle
    rotation << cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f scale;
    // 放大 2.5倍
    scale << 2.5, 0, 0, 0,
        0, 2.5, 0, 0,
        0, 0, 2.5, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return translate * rotation * scale;
    // 上面是通过第二种方式得到的M矩阵

    // M = translate * rotation * scale;
    // M = 2.5*cos(140度)  ,0    ,2.5*sin(140度) ,0
    //     0               ,2.5  ,0             ,0,
    //     -2.5*sin(140度) ,0     ,2.5*cos(140度),0
    //     0               ,0    ,0             ,1
    // M去除最后一行的矩阵：前三列分别是模型坐标系的x,y,z基向量在世界坐标系的表示，最后一列是模型坐标系原点在世界坐标系的位置
}

// 投影矩阵(裁剪矩阵)
// 摄像机空间 ->  齐次裁剪空间
//  在裁剪空间之前，虽然使用了齐次坐标表示点和向量，但第四个分量都是固定的：
//       点的w分量是1，方向矢量的w分量是0。
// 经过投影矩阵的变换后，顶点的w分量将会具有特殊的意义。用于齐次除法。
//  齐次裁剪坐标：左手坐标系
//  NDC：左手坐标系

// 观察空间使用右手坐标系，齐次裁剪空间（NDC）使用左手坐标系、右手坐标系，
// 透视投影矩阵将会有所不同，参考https://zhuanlan.zhihu.com/p/618620569
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // 传来的参数 eye_fov:45度  apect_ratio:1  zNear:0.1  zFar:50

    //  Use the same projection matrix from the previous assignments

    Eigen::Matrix4f projection;
    float radian = eye_fov / 2 / 180 * acos(-1);
    projection << cos(radian) / sin(radian) / aspect_ratio, 0, 0, 0,
        0, cos(radian) / sin(radian), 0, 0,
        0, 0, -(zFar + zNear) / (zFar - zNear), -2 * zNear * zFar / (zFar - zNear),
        0, 0, -1, 0;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload &payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload &payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f &vec, const Eigen::Vector3f &axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    // 位置
    Eigen::Vector3f position;

    // 强度
    Eigen::Vector3f intensity;
};

// Blinn-Phong + texure 着色模型
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment

        // 取纹理坐标的颜色
        auto tex_coord = payload.tex_coords;
        return_color = payload.texture->getColor(tex_coord.x(), tex_coord.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10}; // 环境光强度
    Eigen::Vector3f eye_pos{0, 0, 10};               // 眼睛位置

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto &light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.

        auto distance = (light.position - point).norm();
        auto lightDirection = (light.position - point).normalized();
        auto n = normal.normalized();
        Eigen::Vector3f diffuse = kd.cwiseProduct((light.intensity / (distance * distance))) * max(0.0f, n.dot(lightDirection));

        // 高光反射
        //  Ls = ks*(I/(r*r))*max(0,n·h)
        auto lightVect = light.position - point;
        auto eyeVect = eye_pos - point;
        auto h = (lightVect + eyeVect).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct((light.intensity / (distance * distance))) * pow((0.0f, n.dot(lightDirection)), p);

        auto ambient = ka.cwiseProduct(amb_light_intensity);

        auto lightEffect = diffuse + specular + ambient;
        result_color += lightEffect;
    }

    return result_color * 255.f;

}

// Blinn-Phong着色模型
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload &payload)
{
    // ka:环境光系数（三维向量） a:ambient coefficient
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);

    // kd:漫反射系数（三维向量）  d：diffuse  coefficient
    Eigen::Vector3f kd = payload.color;

    // ks:高光反射系数（三维向量）  s: specular coefficient
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{
        {20, 20, 20},   // 位置
        {500, 500, 500} // 强度
    };
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};

    // 环境光强度
    Eigen::Vector3f amb_light_intensity{10, 10, 10};

    // 眼睛位置
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos; // 顶点在摄像机坐标坐标系下的位置
    Eigen::Vector3f normal = payload.normal;
    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto &light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.

        // 漫反射
        // Ld = kd*(I/(r*r))*max(0,n·l)
        auto distance = (light.position - point).norm();
        auto lightDirection = (light.position - point).normalized();
        auto n = normal.normalized();
        Eigen::Vector3f diffuse = kd.cwiseProduct((light.intensity / (distance * distance))) * max(0.0f, n.dot(lightDirection));

        // 高光反射
        //  Ls = ks*(I/(r*r))*max(0,n·h)
        auto lightVect = light.position - point;
        auto eyeVect = eye_pos - point;
        auto h = (lightVect + eyeVect).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct((light.intensity / (distance * distance))) * pow((0.0f, n.dot(lightDirection)), p);

        auto ambient = ka.cwiseProduct(amb_light_intensity);

        auto lightEffect = diffuse + specular + ambient;
        result_color += lightEffect;
    }

    return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload &payload)
{

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto &light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.
    }

    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload &payload)
{

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char **argv)
{
    // 三角形个数
    std::vector<Triangle *> TriangleList;

    // 弧度：140
    float angle = 140.0;

    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for (auto mesh : Loader.LoadedMeshes)
    {
        // 遍历mesh包含的所有顶点，获取三角形列表
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            Triangle *t = new Triangle();
            for (int j = 0; j < 3; j++)
            {
                // 顶点位置 :Vector4f w默认是1
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
            }

            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    // 观察点位置（摄像机位置）
    Eigen::Vector3f eye_pos = {0, 0, 10};

    // 设置顶点着色器
    r.set_vertex_shader(vertex_shader);

    // 设置片元着色器
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));

        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));

        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        // r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a')
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }
    }
    return 0;
}
