#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"


class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	Eigen::MatrixXd calcWeights();
	void pre_draw();
	bool check_collision_cond(Eigen::AlignedBox<double, 3> box_1, Eigen::AlignedBox<double, 3> box_2, Eigen::Matrix3d& R_mat, Eigen::Matrix4d& A_trans, Eigen::Matrix4d& B_trans, Eigen::Matrix3d& A_rot, Eigen::Matrix3d& B_rot);
	~SandBox();
	bool check_collision_rec(igl::AABB<Eigen::MatrixXd, 3>* node1, igl::AABB<Eigen::MatrixXd, 3>* node2, Eigen::Matrix3d& R_mat, Eigen::Matrix4d& A_trans, Eigen::Matrix4d& B_trans, Eigen::Matrix3d& A_rot, Eigen::Matrix3d& B_rot);
	void check_collision();
	void new_check_collision(int i, Eigen::Vector3d head_loc);
	void Init(const std::string& config);
	void init_skinning_stuff();
	void scale_snake();
	void draw_skeleton();
	double doubleVariable;

	const Eigen::Vector3d sea_green = Eigen::Vector3d(70. / 255., 252. / 255., 167. / 255.);



	enum direction { up, down, right, left, NONE };

	bool dir_change = false;
	direction _dir = NONE;
	double xangle = 0.0;
	double yangle = 0.0;
	void update_next_pose();
	void set_dir(direction dir) {
		_dir = dir;
	};

	typedef
		std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
		RotationList;
	// W - weights matrix
	// BE - Edges between joints
	// C - joints positions
	// P - parents
	// M - weights per vertex per joint matrix
	// U - new vertices position after skinning
	Eigen::MatrixXd V, W, C, U, M;
	Eigen::MatrixXd CT;
	Eigen::MatrixXd skeleton_joint_color;
	Eigen::MatrixXd skeleton_bone_color;
	Eigen::MatrixXi F, BE;
	Eigen::MatrixXi BET;
	Eigen::VectorXi P;
	RotationList rest_pose, curr_pose, next_pose;

	std::vector<RotationList > poses; // rotations of joints for animation
	std::vector<Eigen::Vector3d> new_pos;
	double anim_t = 0;
	double anim_t_dir = 0.2;
	Eigen::Vector4d our_vec;
	Eigen::Vector3d move_dir;
	Eigen::Vector3d head_loc;
	void move_balls();
	int border = 0;
	int sign = 1;

	Eigen::Quaterniond rot_quaternion = Eigen::Quaterniond::Identity();

	void PauseMusic();
	void ResumeMusic();
private:
	// Prepare array-based edge data structures and priority queue
	
	
	void Animate(igl::opengl::ViewerCore &core);
};

