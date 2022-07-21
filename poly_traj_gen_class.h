#pragma once
#ifndef POLY_TRAJ_GEN_CLASS__H
#define POLY_TRAJ_GEN_CLASS__H

#include <Eigen/Core>
#include <Eigen/Dense>

class poly_traj_generator {
public:
	poly_traj_generator();
	Eigen::MatrixXf generate_polynomial_traj(Eigen::VectorXf& constrains, Eigen::VectorXf& viapoint_profile, Eigen::Vector2f& T);
	Eigen::Matrix<float, 4, 1> generate_3rd_polynomial_coff(float x_start, float initial_k, float y_end, float final_k, float x_end);
	Eigen::VectorXf get_x_array_3rd_polynomial(float x_start, float k_initial, float x_end, float final_k, float T, uint32_t steps_per_sec) ;
	Eigen::VectorXf linspace(float start, float end, uint32_t steps);
	float polyval(float x, uint32_t order, Eigen::RowVectorXf& coff);
private:
	uint32_t steps_per_sec_;

};

#endif