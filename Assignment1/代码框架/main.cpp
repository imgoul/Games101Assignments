#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // 旋转矩阵 绕z轴
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    float radian = rotation_angle / 180 * acos(-1);
    trans << cos(radian), -sin(radian), 0, 0,
        sin(radian), cos(radian), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    model = model * trans;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    //透视投影矩阵
    float radian = eye_fov / 2 / 180 * acos(-1);
    projection << cos(radian) / sin(radian) / aspect_ratio, 0, 0, 0,
        0, cos(radian) / sin(radian), 0, 0,
        0, 0, -(zFar + zNear) / (zFar - zNear), -2 * zNear * zFar / (zFar - zNear),
        0, 0, -1, 0;
    return projection;
}

// argc 执行程序传递的参数个数
int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);//创建一个700x700的画布

    Eigen::Vector3f eye_pos = {0, 0, 5};//摄像机坐标

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};//三角形坐标

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}}; //顶点下标

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        //保存成一个png
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //设置M矩阵：模型坐标转换到世界坐标
        r.set_model(get_model_matrix(angle));

        //设置V矩阵：世界坐标转换到观察坐标
        r.set_view(get_view_matrix(eye_pos));

        //设置P矩阵：观察坐标转换到齐次裁剪坐标
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        //画三角形
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);


        
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
