#include <poly_traj_gen_class.h>
#include <iostream>

using namespace std;

poly_traj_generator::poly_traj_generator() {
	cout << "polynimial trajectory generator initializing" << endl;
	steps_per_sec_ = 100;
}

//this function is to create a 3rd ordered polynomial curve with 1 viapoint, parameters:
//constrains: [x_start, y_start, x_end, y_end, initial_direction, final_direction, x_offset, y_offset]
//viapoint_profile : [x_pose, y_pose, via_point_dir, ]
//T : [T_phase1, T_phase2]
Eigen::MatrixXf poly_traj_generator::generate_polynomial_traj(Eigen::VectorXf &constrains,Eigen::VectorXf &viapoint_profile,Eigen::Vector2f &T) 
{
	float x_start, y_start, x_end, y_end, initial_direction, final_direction, x_offset, y_offset;
	float via_point_x_pose, via_point_y_pose, via_point_dir,T_phase1,T_phase2;

	x_start = constrains(0);
	y_start = constrains(1);
	x_end = constrains(2) - constrains(6);
	y_end = constrains(3);
	initial_direction = constrains(4);
	final_direction = constrains(5);
	x_offset = constrains(6);
	y_offset = constrains(7);

	via_point_x_pose = viapoint_profile(0) - x_offset;
	via_point_y_pose = viapoint_profile(1) - y_offset;
	via_point_dir = viapoint_profile(2);

	T_phase1 = T(0);
	T_phase2 = T(1);

	Eigen::Matrix<float, 4, 1> coff1, coff2;

	coff1 = generate_3rd_polynomial_coff(y_start, atan(initial_direction), via_point_y_pose, atan(via_point_dir), via_point_x_pose);
	coff2 = generate_3rd_polynomial_coff(via_point_y_pose, atan(via_point_dir), y_end, atan(final_direction), x_end-via_point_x_pose);

	const uint32_t steps = steps_per_sec_ * (T_phase1 + T_phase2);
	

	Eigen::VectorXf x_pose1(uint32_t(T_phase1* steps_per_sec_)), x_pose2(uint32_t(T_phase2 * steps_per_sec_));
	
	x_pose1  = get_x_array_3rd_polynomial(x_start, 0, via_point_x_pose, atan(0.1), T_phase1, steps_per_sec_);
	x_pose2 = get_x_array_3rd_polynomial(via_point_x_pose, atan(0.1), x_end, 0, T_phase2, steps_per_sec_);
	Eigen::VectorXf x_pose(x_pose1.size() + x_pose2.size());
	x_pose << x_pose1, x_pose2;

	Eigen::VectorXf y_pose(x_pose.size());

	Eigen::RowVectorXf coff_1 = coff1.col(0);
	Eigen::RowVectorXf coff_2 = coff2.col(0);

	for (int i = 0; i < steps; i++) {
		if (i < T_phase1 * steps_per_sec_) {
			y_pose(i) = polyval(x_pose(i),3, coff_1);
		}else{
			y_pose(i) = polyval(x_pose(i) - x_pose(T_phase1*steps_per_sec_), 3, coff_2);;
		}
	}
	Eigen::MatrixXf poses(steps, 2);

	x_pose = x_pose + Eigen::VectorXf::Ones(steps) * x_offset;
	y_pose = y_pose + Eigen::VectorXf::Ones(steps) * y_offset;

	poses << x_pose, y_pose;

	return poses;
}

Eigen::Matrix<float,4,1> poly_traj_generator::generate_3rd_polynomial_coff(float x_start, float initial_k, float y_end, float final_k, float x_end) {
    Eigen::Matrix<float, 4, 4> A_matrix;
    A_matrix << 1, 0, 0, 0,
        0, 1, 0, 0,
        1, x_end, pow(x_end, 2), pow(x_end, 3),
        0, 1, 2 * x_end, 3 * pow(x_end, 2);
    Eigen::Matrix<float, 4, 1> b;
    b << x_start, initial_k, y_end, final_k;
	Eigen::Matrix<float, 4, 1> cof_vector = A_matrix.inverse() * b;
	return cof_vector;
}

Eigen::VectorXf poly_traj_generator::get_x_array_3rd_polynomial(float x_start,float k_initial, float x_end,float final_k, float T, uint32_t steps_per_sec) {
    Eigen::Matrix<float, 4, 1> cof = generate_3rd_polynomial_coff(x_start, k_initial, x_end, final_k, T);
    uint32_t steps = T * steps_per_sec;
    Eigen::VectorXf t(steps);
    t = linspace(0, T, steps);
	Eigen::VectorXf x_array(steps);
    for (int i = 0; i < steps; i++) {
        x_array(i) = (cof(0, 0) + cof(1, 0) * t[i] + cof(2, 0) * pow(t[i], 2) + cof(3, 0) * pow(t[i], 3));
    }
	return x_array;
}

Eigen::VectorXf poly_traj_generator::linspace(float start, float end, uint32_t steps) {
    float delta = end - start;
    float delta_step = delta / (steps-1);
	Eigen::VectorXf res(steps);
    for (int i = 0; i < steps; i++) {
        res (i) =  (start + i * delta_step);
    }
	return res;
}

float poly_traj_generator::polyval(float x, uint32_t order, Eigen::RowVectorXf& coff) {
	float value = 0;
	for (int i = 0; i < order + 1; i++)
		value += coff(i) * pow(x, i);
	return value;
}