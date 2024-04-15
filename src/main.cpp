#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

float angleToRadians(float angle) { return MY_PI*angle/180; }

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

// Create the model matrix for rotating the triangle around the Z axis.
// Then return it.
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // transform angle to radians
    float cosValue = cos(angleToRadians(rotation_angle));
    float sinValue = sin(angleToRadians(rotation_angle));
    Eigen::Matrix4f rotate;
    rotate << cosValue, -sinValue, 0, 0,
              sinValue, cosValue,  0, 0,
              0,        0,         1, 0,
              0,        0,         0, 1;
    return rotate;
}

// Create the projection matrix for the given parameters.
// Then return it.
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // eye_fov: viewing angle in the range of [-eye_fov, eye_fov]
    // aspect_ratio: the height:width of viewing plane
    Eigen::Matrix4f squish;
    Eigen::Matrix4f translation;
    Eigen::Matrix4f scale;

    float n = -zNear;
    float f = -zFar;

    squish << n, 0, 0,   0,
              0, n, 0,   0,
              0, 0, n+f, -n*f,
              0, 0, 1,   0;
    
    float top = abs(n)*tan(angleToRadians(eye_fov/2));
    float bottom = -top;

    float right = top*aspect_ratio;
    float left = -right;

    translation << 1, 0, 0, -(left+right)/2,
                   0, 1, 0, -(top+bottom)/2,
                   0, 0, 1, -(n+f)/2,
                   0, 0, 0, 1;

    scale << 2/(right-left), 0,              0,       0,
             0,              2/(top-bottom), 0,       0,
             0,              0,              2/(n-f), 0,
             0,              0,              0,       1;

    return scale*translation*squish;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f K = Eigen::Matrix4f::Identity();
    float sinValue = sin(angleToRadians(angle));
    float cosValue = cos(angleToRadians(angle));
    float kx = axis[0];
    float ky = axis[1];
    float kz = axis[2];
    K << 0,   -kz, ky,
         kz,  0,   -kx,
         -ky, kx, 0;
    return Eigen::Matrix4f::Identity() + sinValue*K + (1-cosValue)*K*K;
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
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

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

        r.set_model(get_model_matrix(angle));
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
        else if (key == 'w') {
            eye_pos[2] -= 1;
        }
        else if (key == 's') {
            eye_pos[2] += 1;
        }
    }

    return 0;
}
