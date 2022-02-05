#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include <igl/deform_skeleton.h>
#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/dqs.h>
#include <igl/forward_kinematics.h>
#include <igl/lbs_matrix.h>
#include <igl/PI.h>


SandBox::SandBox()
{
	

}
Eigen::MatrixXd SandBox:: calcWeights()
{
	std::cout << "CALC\n";

	int num_of_verticies = data().V.rows();
	std::cout << num_of_verticies << std::endl;
	Eigen::MatrixXd W = Eigen::MatrixXd(num_of_verticies, 16);
	std::cout << "CALC1\n";

	for (int i = 0; i < W.rows(); ++i)
	{
		for (int j = 0; j < W.row(i).cols(); ++j)
		{
			W(i, j) = 0;
		
		}
		
	}
	for (int i = 0; i < num_of_verticies; ++i)
	{
		
		double loc = (data().V(i,2) / 0.1) + 8;
		if(loc < 0)
		{
			loc = 0;
		}

		double loc_floor = floor(loc);

		//std::cout << "loc: " << loc << std::endl;
		//std::cout << "loc floor: " << loc_floor<< std::endl;
		if(loc == 0){
			W(i, 0) = 1;
		}
		else if(loc >= 16)
		{
			W(i, 15) = 1;
		}
		else if(loc - loc_floor == 0.0)
		{

			W(i, loc ) = 1;

		}
		else if(loc - loc_floor <= 0.5) //TAKE 1 LEFT 1 MIDDLE 1 RIGHT                 J1  |  J2 *|  J3 | J4       TAKE J1 J2 J3
		{
			if(loc_floor == 0.0)
			{
				W(i, 0) = (1 - loc);
				W(i, 1) = loc ;
			}
			
			else if (loc_floor == 15)
			{
				W(i, 15) = 1;
			}
			else
			{
				double left = (loc - (loc_floor - 1))*10;
				double middle = (loc - (loc_floor))* 10;
				double right = (loc_floor + 1 - (loc))* 10;
				middle = 1/(middle / left);
				right = 1/(right / left);
				left = 1;
				double sum = 1/(middle + right + left);
				W(i, (loc_floor)  - 1) = left * sum;
				W(i, (loc_floor)  ) = middle * sum;
				W(i, (loc_floor)  + 1) = right * sum;

			}
		}
		else //TAKE 1 MIDDLE 2 RIGHT                                  J1  |  J2  |* J3 | J4       TAKE  J2 J3 J4
		{
			if(loc_floor == 15)
			{

				W(i, 15) =1;
			}
			else if(loc_floor == 14)
			{
				W(i, 14) = loc - 14;
				W(i, 15) = 15-loc;
			}
			else  {
				double middle = (loc - (loc_floor)) * 10;

				double righ1 = (loc_floor + 1 - (loc)) * 10;
				double right2 = (loc_floor + 2 - (loc)) * 10;
				middle = 1 / (middle / right2);
				righ1 = 1 / (righ1 / right2);
				right2 = 1;
				double sum = 1 / (middle + righ1 + right2);
				W(i, (loc_floor)) = middle * sum;
				W(i, (loc_floor)+1) = righ1 * sum;
				W(i, (loc_floor)+2) = right2 * sum;
			}
		}
	}
	std::cout << "CALC2\n";

	return W;
}
void SandBox::pre_draw()
{

	// Interpolate pose and identity
	 RotationList anim_pose(curr_pose.size());
	 //std::cout << "anim_t : " << anim_t << std::endl;
	 
	for (int e = 0; e < curr_pose.size(); e++)
	{
		anim_pose[e] = rest_pose[e].slerp(anim_t, curr_pose[e]);
	}
	// Propagate relative rotations via FK to retrieve absolute transformations
	// vQ - rotations of joints
	// vT - translation of joints
	RotationList vQ;
	std::vector<Eigen::Vector3d> vT;
	
	
	
	 igl::forward_kinematics(C, BE, P, anim_pose, vQ, vT);
	 igl::dqs(V, W, vQ, vT, U);



	const int dim = C.cols();
	Eigen::MatrixXd T(BE.rows() * (dim + 1), dim);
	for (int e = 0; e < BE.rows(); e++)
	{
		Eigen::Affine3d a = Eigen::Affine3d::Identity();
		a.translate(vT[e]);
		a.rotate(vQ[e]);
		T.block(e * (dim + 1), 0, dim + 1, dim) =
			a.matrix().transpose().block(0, 0, dim + 1, dim);
	}

	//Also deform skeleton edges
	//move joints according to T, returns new position in CT and BET

	igl::deform_skeleton(C, BE, T, CT, BET);

	data().set_vertices(U);
	data().set_points(CT, skeleton_joint_color);
	data().set_edges(CT, BET, skeleton_bone_color);
	data().compute_normals();
		
	anim_t += anim_t_dir;
		
	
}

bool SandBox::check_collision_cond(Eigen::AlignedBox<double, 3> box_1, Eigen::AlignedBox<double, 3> box_2, Eigen::Matrix3d& R_mat, Eigen::Matrix4d& A_trans, Eigen::Matrix4d& B_trans, Eigen::Matrix3d& A_rot, Eigen::Matrix3d& B_rot)
{
	double R, R_0, R_1;

	Eigen::Vector4d A_vec_center = Eigen::Vector4d(box_1.center()[0], box_1.center()[1], box_1.center()[2], 1);
	Eigen::Vector4d A_vec_center_trans = A_trans * A_vec_center;
	Eigen::RowVector3d P_A = Eigen::RowVector3d(A_vec_center_trans.x(), A_vec_center_trans.y(), A_vec_center_trans.z());

	Eigen::RowVector3d A_x = A_rot * Eigen::Vector3d(1, 0, 0);
	Eigen::RowVector3d A_y = A_rot * Eigen::Vector3d(0, 1, 0);
	Eigen::RowVector3d A_z = A_rot * Eigen::Vector3d(0, 0, 1);
	double W_A = box_1.sizes()[0] / 2;
	double H_A = box_1.sizes()[1] / 2;
	double D_A = box_1.sizes()[2] / 2;



	Eigen::Vector4d B_vec_center = Eigen::Vector4d(box_2.center()[0], box_2.center()[1], box_2.center()[2], 1);
	Eigen::Vector4d B_vec_center_trans = B_trans * B_vec_center;
	Eigen::RowVector3d P_B = Eigen::RowVector3d(B_vec_center_trans.x(), B_vec_center_trans.y(), B_vec_center_trans.z());

	Eigen::RowVector3d B_x = B_rot * Eigen::Vector3d(1, 0, 0);
	Eigen::RowVector3d B_y = B_rot * Eigen::Vector3d(0, 1, 0);
	Eigen::RowVector3d B_z = B_rot * Eigen::Vector3d(0, 0, 1);
	double W_B = box_2.sizes()[0] / 2;
	double H_B = box_2.sizes()[1] / 2;
	double D_B = box_2.sizes()[2] / 2;



	Eigen::RowVector3d T = P_B - P_A;
	//Eigen::Matrix3d R_mat = A * B;

	//std::cout << T << std::endl;

	//CASE 1:
	R_0 = W_A;
	R_1 = W_B * abs(R_mat(0, 0)) + H_B * abs(R_mat(0, 1)) + D_B * abs(R_mat(0, 2));
	R = abs(T.dot(A_x));
	if (R > R_0 + R_1)
		return false;

	//CASE 2:
	R_0 = H_A;
	R_1 = W_B * abs(R_mat(1, 0)) + H_B * abs(R_mat(1, 1)) + D_B * abs(R_mat(1, 2));
	R = abs(T.dot(A_y));
	if (R > R_0 + R_1)
		return false;

	//CASE 3:
	R_0 = D_A;
	R_1 = W_B * abs(R_mat(2, 0)) + H_B * abs(R_mat(2, 1)) + D_B * abs(R_mat(2, 2));
	R = abs(T.dot(A_z));
	if (R > R_0 + R_1)
		return false;

	//CASE 4:
	R_0 = W_A * abs(R_mat(0, 0)) + H_A * abs(R_mat(1, 0)) + D_A * abs(R_mat(2, 0));
	R_1 = W_B;
	R = abs(T.dot(B_x));
	if (R > R_0 + R_1)
		return false;

	//CASE 5:
	R_0 = W_A * abs(R_mat(0, 1)) + H_A * abs(R_mat(1, 1)) + D_A * abs(R_mat(2, 1));
	R_1 = H_B;
	R = abs(T.dot(B_y));
	if (R > R_0 + R_1)
		return false;

	//CASE 6:
	R_0 = W_A * abs(R_mat(0, 2)) + H_A * abs(R_mat(1, 2)) + D_A * abs(R_mat(2, 2));
	R_1 = D_B;
	R = abs(T.dot(B_z));
	if (R > R_0 + R_1)
		return false;

	//CASE 7:
	R_0 = H_A * abs(R_mat(2, 0)) + D_A * abs(R_mat(1, 0));
	R_1 = H_B * abs(R_mat(0, 2)) + D_B * abs(R_mat(0, 1));
	R = abs(R_mat(1, 0) * T.dot(A_z) - R_mat(2, 0) * T.dot(A_y));
	if (R > R_0 + R_1)
		return false;

	//CASE 8:
	R_0 = H_A * abs(R_mat(2, 1)) + D_A * abs(R_mat(1, 1));
	R_1 = W_B * abs(R_mat(0, 2)) + D_B * abs(R_mat(0, 0));
	R = abs(R_mat(1, 1) * T.dot(A_z) - R_mat(2, 1) * T.dot(A_y));
	if (R > R_0 + R_1)
		return false;

	//CASE 9:
	R_0 = H_A * abs(R_mat(2, 2)) + D_A * abs(R_mat(1, 2));
	R_1 = W_B * abs(R_mat(0, 1)) + H_B * abs(R_mat(0, 0));
	R = abs(R_mat(1, 2) * T.dot(A_z) - R_mat(2, 2) * T.dot(A_y));
	if (R > R_0 + R_1)
		return false;

	//CASE 10:
	R_0 = W_A * abs(R_mat(2, 0)) + D_A * abs(R_mat(0, 0));
	R_1 = H_B * abs(R_mat(1, 2)) + D_B * abs(R_mat(1, 1));
	R = abs(R_mat(2, 0) * T.dot(A_x) - R_mat(0, 0) * T.dot(A_z));
	if (R > R_0 + R_1)
		return false;

	//CASE 11:
	R_0 = W_A * abs(R_mat(2, 1)) + D_A * abs(R_mat(0, 1));
	R_1 = W_B * abs(R_mat(1, 2)) + D_B * abs(R_mat(1, 0));
	R = abs(R_mat(2, 1) * T.dot(A_x) - R_mat(0, 1) * T.dot(A_z));
	if (R > R_0 + R_1)
		return false;

	//CASE 12:
	R_0 = W_A * abs(R_mat(2, 2)) + D_A * abs(R_mat(0, 2));
	R_1 = W_B * abs(R_mat(1, 1)) + H_B * abs(R_mat(1, 0));
	R = abs(R_mat(2, 2) * T.dot(A_x) - R_mat(0, 2) * T.dot(A_z));
	if (R > R_0 + R_1)
		return false;

	//CASE 13:
	R_0 = W_A * abs(R_mat(1, 0)) + H_A * abs(R_mat(0, 0));
	R_1 = H_B * abs(R_mat(2, 2)) + D_B * abs(R_mat(2, 1));
	R = abs(R_mat(0, 0) * T.dot(A_y) - R_mat(1, 0) * T.dot(A_x));
	if (R > R_0 + R_1)
		return false;

	//CASE 14:
	R_0 = W_A * abs(R_mat(1, 1)) + H_A * abs(R_mat(0, 1));
	R_1 = W_B * abs(R_mat(2, 2)) + D_B * abs(R_mat(2, 0));
	R = abs(R_mat(0, 1) * T.dot(A_y) - R_mat(1, 1) * T.dot(A_x));
	if (R > R_0 + R_1)
		return false;

	//CASE 15:
	R_0 = W_A * abs(R_mat(1, 2)) + H_A * abs(R_mat(0, 2));
	R_1 = W_B * abs(R_mat(2, 1)) + H_B * abs(R_mat(2, 0));
	R = abs(R_mat(0, 2) * T.dot(A_y) - R_mat(1, 2) * T.dot(A_x));
	if (R > R_0 + R_1)
		return false;

	return true;
}

bool SandBox::check_collision_rec(igl::AABB<Eigen::MatrixXd, 3>* node1, igl::AABB<Eigen::MatrixXd, 3>* node2, Eigen::Matrix3d& R_mat, Eigen::Matrix4d& A_trans, Eigen::Matrix4d& B_trans, Eigen::Matrix3d& A_rot, Eigen::Matrix3d& B_rot)
{
	if (check_collision_cond(node1->m_box, node2->m_box, R_mat, A_trans, B_trans, A_rot, B_rot))
	{
		if (node1->is_leaf() && node2->is_leaf())
		{
			data_list[0].drawBox(node1->m_box);
			data_list[1].drawBox(node2->m_box);


			dir = -1 * dir;
			return true;
		}

		igl::AABB<Eigen::MatrixXd, 3>* left1 = node1->is_leaf() ? node1 : node1->m_left;
		igl::AABB<Eigen::MatrixXd, 3>* right1 = node1->is_leaf() ? node1 : node1->m_right;
		igl::AABB<Eigen::MatrixXd, 3>* left2 = node2->is_leaf() ? node2 : node2->m_left;;
		igl::AABB<Eigen::MatrixXd, 3>* right2 = node2->is_leaf() ? node2 : node2->m_right;

		if (check_collision_rec(left1, left2, R_mat, A_trans, B_trans, A_rot, B_rot))
			return true;
		else if (check_collision_rec(left1, right2, R_mat, A_trans, B_trans, A_rot, B_rot))
			return true;
		else if (check_collision_rec(right1, left2, R_mat, A_trans, B_trans, A_rot, B_rot))
			return true;
		else if (check_collision_rec(right1, right2, R_mat, A_trans, B_trans, A_rot, B_rot))
			return true;
		else return false;

	}
	return false;
}

void SandBox::check_collision()
{
	igl::AABB<Eigen::MatrixXd, 3>* node1 = &data_list[0].tree;
	igl::AABB<Eigen::MatrixXd, 3>* node2 = &data_list[1].tree;

	Eigen::Matrix4d A_trans = data_list[0].MakeTransd();
	Eigen::Matrix3d A_rot = data_list[0].GetRotation();

	Eigen::Matrix4d B_trans = data_list[1].MakeTransd();
	Eigen::Matrix3d B_rot = data_list[1].GetRotation();

	Eigen::RowVector3d A_x = A_rot * Eigen::Vector3d(1, 0, 0);
	Eigen::RowVector3d A_y = A_rot * Eigen::Vector3d(0, 1, 0);
	Eigen::RowVector3d A_z = A_rot * Eigen::Vector3d(0, 0, 1);


	Eigen::Matrix3d A;
	A << A_x[0], A_x[1], A_x[2],
		A_y[0], A_y[1], A_y[2],
		A_z[0], A_z[1], A_z[2];


	Eigen::RowVector3d B_x = B_rot * Eigen::Vector3d(1, 0, 0);
	Eigen::RowVector3d B_y = B_rot * Eigen::Vector3d(0, 1, 0);
	Eigen::RowVector3d B_z = B_rot * Eigen::Vector3d(0, 0, 1);

	Eigen::Matrix3d B;
	B << B_x[0], B_y[0], B_z[0],
		B_x[1], B_y[1], B_z[1],
		B_x[2], B_y[2], B_z[2];

	Eigen::Matrix3d R_mat = A * B;





	if (check_collision_rec(node1, node2, R_mat, A_trans, B_trans, A_rot, B_rot)) {
		std::cout << "Collision detected! " << std::endl;
		isMoving = false;
	}
}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 1;
			data().line_width = 2;
			data().set_visible(false, 1);
			//data().SetCenterOfRotation(Eigen::Vector3d(10,0, 0));
				

			
		}
		nameFileout.close();
	}
	append_joint();
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	
	init_skinning_stuff();
	scale_snake();
	draw_skeleton();
	
	//DRAW A BOX
	data().tree.init(data().V, data().F);
	igl::AABB<Eigen::MatrixXd, 3> tree = data().tree;
	Eigen::AlignedBox<double, 3> box = tree.m_box;
	data().drawBox(box);
	data(0).MyTranslate(Eigen::Vector3d(0, 0, -19), true);
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	//data().MyScale(Eigen::Vector3d(1, 1, 16));

}

void SandBox::init_skinning_stuff()
{
	V = data().V;
	U = V;
	F = data().F;
	int num_of_joints = joint().joint_pos.size();
	C = Eigen::MatrixXd(num_of_joints, 3);
	for(int i = 0; i < num_of_joints;i++)
	{
			C.row(i)= joint().joint_pos[i];
		
	}
	BE = Eigen::MatrixXi(num_of_joints-1, 2);
	for (int i = 0; i < num_of_joints-1; i++)
	{
		BE(i,0) = i;
		BE(i,1) = i+1;

	}

	W = calcWeights();

	igl::directed_edge_parents(BE, P);
	igl::directed_edge_orientations(C, BE, rest_pose);

	for (int i = 0; i < rest_pose.size(); ++i)
	{
		std::cout << rest_pose[i].matrix() << std::endl;
	}
	

	curr_pose = rest_pose;
	next_pose = curr_pose;

	CT = C;
}



void SandBox::scale_snake()
{
	C = C * 16;
	int number_of_v = V.rows();
	Eigen::Vector3d curr_v;
	for (int i = 0; i < number_of_v; ++i)
	{
		curr_v =  V.row(i);

		V.row(i) = Eigen::Vector3d(curr_v[0], curr_v[1], curr_v[2] * 16);
	}
	data().set_vertices(V);
	data().compute_normals();
}


void SandBox::draw_skeleton()
{
	skeleton_joint_color.resize(C.rows(), 3);
	skeleton_bone_color.resize(BE.rows(), 3);
	for (int i = 0; i < C.rows(); ++i)
	{
		skeleton_joint_color.row(i) << 1, 1, 1;
	}
	for (int i = 0; i < BE.rows(); ++i)
	{
		skeleton_bone_color.row(i) = sea_green;
	}
	data().add_points(C, skeleton_joint_color);
	data().set_edges(C,BE , skeleton_bone_color);
}

void SandBox::update_next_pose()
{
	Eigen::Vector3d axis;
	double angle = 0;


	switch (_dir)
	{
	case up:
		axis << 1, 0, 0;
		angle = igl::PI / 10;
		xangle += angle;
		break;
	case down:
		axis << 1, 0, 0;
		angle = -igl::PI / 10;
		xangle += angle;
		break;
	case right:
		axis << 0, 1, 0;
		angle = igl::PI / 10;
		yangle += angle;
		break;
	case left:
		axis << 0, 1, 0;
		angle = -igl::PI / 10;
		yangle += angle;
		break;
	case NONE:
		return;
	}
	
	// std::cout << "next_pose mat: \n" << next_pose[0].toRotationMatrix().matrix() << std::endl;

	Eigen::Quaterniond turn_x(Eigen::AngleAxisd(xangle, Eigen::Vector3d::UnitX()));

	Eigen::Quaterniond turn_y(Eigen::AngleAxisd(yangle, Eigen::Vector3d::UnitY()));
	//std::cout << "turn          " <<turn.matrix() << std::endl;
	Eigen::Quaterniond turn1(Eigen::AngleAxisd(angle, curr_pose[0].toRotationMatrix().transpose()*axis));
	Eigen::Quaterniond turn2(Eigen::AngleAxisd(-angle, curr_pose[0].toRotationMatrix().transpose()*axis));
	next_pose[0] = curr_pose[0]*turn1;
	next_pose[1] = turn2;
	dir_change = true;
}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{
		//check_collision();
		pre_draw();
		data().MyTranslate((CT.row(BE(0, 0)) - CT.row(BE(0, 1))).normalized() * 0.2, true);

		
		if (anim_t > 1)
		{
			update_next_pose();
			_dir = NONE;
			anim_t = 0;
			rest_pose = curr_pose;
			curr_pose = next_pose;
			for (int i = next_pose.size() - 1; i > 1; --i)
			{
				// std::cout <<"next pose: \n" <<next_pose[i].matrix()<<std::endl;
				next_pose[i] = curr_pose[i - 1];
			}

			if (dir_change)
			{
				next_pose[1] = Eigen::Quaterniond::Identity();
				dir_change = false;
			}
			
		}

	}
}


