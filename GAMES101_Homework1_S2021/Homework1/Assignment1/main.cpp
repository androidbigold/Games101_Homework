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
    double angle_in_pi = rotation_angle / 180.0 * MY_PI;
    Eigen::Vector4f axis_z(0.0f, 0.0f, 1.0f, 0.0f);

    Eigen::Matrix4f p1;
    p1 << 1.0f, 0.0f, 0.0f, 0.0f,
          0.0f, 1.0f, 0.0f, 0.0f,
          0.0f, 0.0f, 1.0f, 0.0f,
          0.0f, 0.0f, 0.0f, 0.0f;
    p1 = std::cos(angle_in_pi) * p1;
    p1(3, 3) = 1.0f;

    Eigen::Matrix4f p2;
    p2 = (1 - std::cos(angle_in_pi)) * axis_z * axis_z.transpose();

    Eigen::Matrix4f p3;
    p3 << 0.0f,         -axis_z.z(),    axis_z.y(),     0.0f,
          axis_z.z(),   0.0f,           -axis_z.x(),    0.0f,
          -axis_z.y(),  axis_z.x(),     0.0f,           0.0f,
          0.0f,         0.0f,           0.0f,           0.0f;
    p3 = std::sin(angle_in_pi) * p3;

    Eigen::Matrix4f translate;
    translate = p1 + p2 + p3;

    model = translate * model;

    return model;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around a any axis.
    // Then return it.
    double angle_in_pi = angle / 180.0 * MY_PI;
    Eigen::Vector4f axis_w(axis.x(), axis.y(), axis.z(), 0.0f);

    Eigen::Matrix4f p1;
    p1 << 1.0f, 0.0f, 0.0f, 0.0f,
          0.0f, 1.0f, 0.0f, 0.0f,
          0.0f, 0.0f, 1.0f, 0.0f,
          0.0f, 0.0f, 0.0f, 0.0f;
    p1 = std::cos(angle_in_pi) * p1;
    p1(3, 3) = 1.0f;

    Eigen::Matrix4f p2;
    p2 = (1 - std::cos(angle_in_pi)) * axis_w * axis_w.transpose();

    Eigen::Matrix4f p3;
    p3 << 0.0f,         -axis_w.z(),    axis_w.y(),     0.0f,
          axis_w.z(),   0.0f,           -axis_w.x(),    0.0f,
          -axis_w.y(),  axis_w.x(),     0.0f,           0.0f,
          0.0f,         0.0f,           0.0f,           0.0f;
    p3 = std::sin(angle_in_pi) * p3;

    Eigen::Matrix4f translate;
    translate = p1 + p2 + p3;

    model = translate * model;

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
    float width_near;
    float height_near;
    float width_far;
    float height_far;

    height_near = 2 * zNear * std::tan(eye_fov / 2 / 180.0 * MY_PI);
    width_near = aspect_ratio * height_near;

    height_far = 2 * zFar * std::tan(eye_fov / 2 / 180.0 * MY_PI);
    width_far = aspect_ratio * height_far;    

    Eigen::Matrix4f persp;
    persp << zNear,     0.0f,   0.0f,           0.0f,
             0.0f,      zNear,  0.0f,           0.0f,
             0.0f,      0.0f,   zNear + zFar,   -zNear * zFar,
             0.0f,      0.0f,   1.0f,           0.0f;

    Eigen::Matrix4f translate;
    translate << 1.0f, 0.0f, 0.0f, 0.0f,
                 0.0f, 1.0f, 0.0f, 0.0f,
                 0.0f, 0.0f, 1.0f, -(zFar + zNear) / 2,
                 0.0f, 0.0f, 0.0f, 1.0;
    
    Eigen::Matrix4f scale;
    scale << 2 / width_near,    0.0f,               0.0f,                   0.0f,
             0.0f,              2 / height_near,    0.0f,                   0.0f,
             0.0f,              0.0f,               2 / (zNear - zFar),     0.0f,
             0.0f,              0.0f,               0.0f,                   1.0f;

    Eigen::Matrix4f ortho;
    ortho = scale * translate;

    projection = ortho * persp;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    Eigen::Vector3f rotate_axis = {1.0f, 1.0f, 0.0f};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
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

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(rotate_axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
