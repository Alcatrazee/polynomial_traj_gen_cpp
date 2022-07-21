// trajectory_generator.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <vector>
#include <poly_traj_gen_class.h>
#include <fstream>

using namespace std;

//void linspace(float start, float end, uint32_t steps, Eigen::VectorXf &res) {
//    float delta = end - start;
//    float delta_step = delta / (steps-1);
//    for (int i = 0; i < steps; i++) {
//        res (i) =  (start + i * delta_step);
//    }
//}
//
//void generate_3rd_polynomial_coff(float x_start, float initial_k, float y_end, float final_k, float x_end,Eigen::Matrix<float,4,1> &cof_vector) {
//    Eigen::Matrix<float, 4, 4> A_matrix;
//    A_matrix << 1, 0, 0, 0,
//        0, 1, 0, 0,
//        1, x_end, pow(x_end, 2), pow(x_end, 3),
//        0, 1, 2 * x_end, 3 * pow(x_end, 2);
//    Eigen::Matrix<float, 4, 1> b;
//    b << x_start, initial_k, y_end, final_k;
//    cof_vector = A_matrix.inverse() * b;
//}
//
//void get_x_array_3rd_polynomial(float x_start, float x_end, float T, uint32_t steps_per_sec, Eigen::VectorXf & x_array) {
//    Eigen::Matrix<float, 4, 1> cof;
//    generate_3rd_polynomial_coff(x_start, 0, x_end, 0, T, cof);
//    uint32_t steps = T * steps_per_sec;
//    Eigen::VectorXf t(steps);
//    linspace(0, T, steps, t);
//    cout << t << endl;
//    cout << cof << endl;
//    for (int i = 0; i < steps; i++) {
//        x_array(i) = (cof(0, 0) + cof(1, 0) * t[i] + cof(2, 0) * pow(t[i], 2) + cof(3, 0) * pow(t[i], 3));
//    }
//}
//
//#define pi 3.1415926
//int main()
//{
//    float final_yaw =  pi / 6;
//    float initial_yaw = 0;
//    float x_start = 0; 
//    float x_end = 0.5;
//    float y_start = 0;
//    float y_end = 0.2;
//    float T = 5;
//    uint32_t steps_per_sec = 100;
//
//    uint32_t steps = steps_per_sec * T;
//    float initial_k = atan(initial_yaw);
//    float final_k = atan(final_yaw);
//
//    Eigen::Matrix<float, 4, 1> cof_vector;
//    generate_3rd_polynomial_coff(x_start, initial_k, y_end, final_k,x_end, cof_vector);
//
//    Eigen::VectorXf x_pose(steps);
//    get_x_array_3rd_polynomial(x_start, x_end, T, steps_per_sec, x_pose);
//    Eigen::VectorXf t(steps);
//    linspace(0, T, steps, t);
//    Eigen::VectorXf y_pose(steps);
//
//    cout << t << endl;
//
//    for (int i = 0; i < steps; i++) {
//        y_pose(i) = (cof_vector(0, 0) + cof_vector(1, 0) * x_pose(i) + cof_vector(2, 0) * pow(x_pose(i), 2) + cof_vector(3, 0) * pow(x_pose(i), 3));
//        cout << x_pose(i) << "\t" << y_pose(i) << endl;
//    }
//
//    return 0;
//}

Eigen::MatrixXf test_function() {
	Eigen::MatrixXf test_mat(2, 2);
	test_mat << 1, 2,
		3, 4;
	return test_mat;
}

int main() {
	poly_traj_generator trajectory_generator;
	
	Eigen::VectorXf constrains(8);
	Eigen::VectorXf viapoint_profile(3);
	Eigen::Vector2f T;

	constrains << 0, 0, 2, 0.4, 0, 3.14159 / 2.5, 0.4, 0;
	viapoint_profile << 0.5, 0, 0;
	T << 3, 4;
	Eigen::MatrixXf path = trajectory_generator.generate_polynomial_traj(constrains, viapoint_profile, T);

	ofstream fout("matrix_test.txt", ios::binary);
	fout << path << endl;
	fout.flush();

	//cout << path << endl;
	return 0;
}	

