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

#include <windows.h>
#include <Mmsystem.h>
#include <mciapi.h>

#pragma comment(lib, "Winmm.lib")



SandBox::SandBox()
{
	

}
Eigen::MatrixXd SandBox:: calcWeights()
{

	int num_of_verticies = data(snake_pos).V.rows();
	Eigen::MatrixXd W = Eigen::MatrixXd(num_of_verticies, 16);

	for (int i = 0; i < W.rows(); ++i)
	{
		for (int j = 0; j < W.row(i).cols(); ++j)
		{
			W(i, j) = 0;
		
		}
		
	}
	for (int i = 0; i < num_of_verticies; ++i)
	{
		
		double loc = (data(snake_pos).V(i,2) / 0.1) + 8;
		if(loc < 0)
		{
			loc = 0;
		}

		double loc_floor = floor(loc);

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

	return W;
}
void SandBox::pre_draw()
{

	// Interpolate pose and identity
	 RotationList anim_pose(curr_pose.size());

	
	for (int e = 0; e < curr_pose.size(); e++)
	{
		anim_pose[e] = rest_pose[e].slerp(anim_t, curr_pose[e]);
	}
	rot_quaternion = anim_pose[0] ;
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

	data(snake_pos).set_vertices(U);
	data(snake_pos).set_points(CT, skeleton_joint_color);
	data(snake_pos).set_edges(CT, BET, skeleton_bone_color);
	data(snake_pos).compute_normals();

	
	our_vec<< CT(0, 0), CT(0, 1), CT(0, 2), 1;
	our_vec = data(snake_pos).MakeTransd() * our_vec;
	head_loc = our_vec.head<3>();
	
	anim_t += anim_t_dir;
		
	
}


void SandBox::new_check_collision(int i, Eigen::Vector3d head_loc)
{
	Eigen::Vector4d ball_loc_vec4= data_list[i].MakeTransd() * Eigen::Vector4d::UnitW();
	Eigen::Vector3d ball_loc_vec3 = ball_loc_vec4.head<3>();
	if((head_loc-ball_loc_vec3).norm()<1)
	{
		if (i <= 3) {
			std::cout << "HIT\n";
		}
		else {
			std::cout << "GAME OVER\n";
			setStart();
			SetAnimation();
			setMainMenu();
			isCameraUp = true;
			betweenLevelsOrganize();

			insertScore(playerName, score);
		}
		
		if(isSecondLevel)
			score = score + 200;
		else {
			score = score + 100;
			number_of_balls++;
		}

		int sign1 = (rand() % 2) * 2 - 1;
		int sign2 = (rand() % 2) * 2 - 1;
		data(i).MyTranslate(Eigen::Vector3d(sign1*(7 + (std::rand() % 4)),0, sign2*(7 + (std::rand() % 4))),true) ;

		if (score == 400) {
			setStart();
			SetAnimation();
			betweenLevelsOrganize();
			setBetweenLevels();
			isCameraUp = true;

		}
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
			//std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 1;
			data().line_width = 2;
			data().set_visible(false, 1);
			//data().SetCenterOfRotation(Eigen::Vector3d(10,0, 0));
			data().cubemapTexture = data().loadCubemap(data().faces);

			
		}
		nameFileout.close();
	}
	snake_pos = data_list.size() - 1;
	append_joint();
	
	
	init_skinning_stuff();
	scale_snake();
	draw_skeleton();


	organizeLevel();

	
	data(0).MyScale(Eigen::Vector3d(200, 200, 200));

	//mciSendString("open \"C:\\Users\\elikk\\Desktop\\ASS4\\EngineForAnimationCourse\\tutorial\\data\\musicforgame.wav\" type mpegvideo alias wav", NULL, 0, NULL);
	mciSendString("open \"musicforgame.wav\" type mpegvideo alias wav", NULL, 0, NULL);

	mciSendString("play wav", NULL, 0, NULL);
	
}

void SandBox::init_skinning_stuff()
{
	V = data(snake_pos).V;
	U = V;
	F = data(snake_pos).F;
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

		V.row(i) = Eigen::Vector3d(curr_v[0], curr_v[1], curr_v[2]*16 );
	}
	data(snake_pos).set_vertices(V);
	data(snake_pos).compute_normals();
	

}


void SandBox::draw_skeleton()
{
	data().point_size = 7;
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
	data(snake_pos).add_points(C, skeleton_joint_color);
	data(snake_pos).set_edges(C,BE , skeleton_bone_color);
}

void SandBox::update_next_pose()
{
	Eigen::Vector3d axis;
	double angle = 0;

	if (isCameraUp) {
		switch (_dir)
		{
		case up:
			axis << 1, 0, 0;
			angle = igl::PI / 10;
			break;
		case down:
			axis << 1, 0, 0;
			angle = -igl::PI / 10;
			break;
		case right:
			axis << 0, 1, 0;
			angle = igl::PI / 10;
			break;
		case left:
			axis << 0, 1, 0;
			angle = -igl::PI / 10;
			break;
		case NONE:
			return;
		}
	}
	else {
		switch (_dir)
		{
		case up:
			axis << 1, 0, 0;
			angle = -igl::PI / 10;
			break;
		case down:
			axis << 1, 0, 0;
			angle = igl::PI / 10;
			break;
		case right:
			axis << 0, 1, 0;
			angle = -igl::PI / 10;
			break;
		case left:
			axis << 0, 1, 0;
			angle = igl::PI / 10;
			break;
		case NONE:
			return;
		}

	}
	

	Eigen::Quaterniond turn1(Eigen::AngleAxisd(angle, curr_pose[0].toRotationMatrix().transpose()*axis));
	Eigen::Quaterniond turn2(Eigen::AngleAxisd(-angle, curr_pose[0].toRotationMatrix().transpose()*axis));
	next_pose[0] = curr_pose[0]*turn1;
	next_pose[1] = turn2;
	dir_change = true;
}

SandBox::~SandBox()
{

}

void SandBox::move_balls()
{
	if (border == 40  || border == -40)
	{
		sign *= -1;
		
	}
	for (int i = 1; i < data_list.size()-1; ++i)
	{
		if(i%3==0)
		{
			move_dir = Eigen::Vector3d::UnitY();
		}
		else if (i % 3 == 1)
		{
			move_dir = Eigen::Vector3d::UnitX();

		}
		else
			move_dir = Eigen::Vector3d::UnitZ();
		data_list[i].MyTranslate(move_dir * ( sign * 0.35), true);
	}
	border += sign;
}

void SandBox::PauseMusic()
{
	mciSendString("pause wav", NULL, 0, NULL);

}

void SandBox::ResumeMusic()
{
	mciSendString("resume wav", NULL, 0, NULL);

}

void SandBox::betweenLevelsOrganize()
{	
	_dir = NONE;
	anim_t = anim_t_dir;
	for (int i = 0; i < rest_pose.size(); ++i)
	{
		rest_pose[i] = Eigen::Quaterniond::Identity();
	}
	curr_pose = rest_pose;
	next_pose = curr_pose;

	dir_change = false;

	Eigen::Vector3d trans;
	
	for (int i = 1; i < data_list.size(); i++) {
		trans = (data(i).MakeTransd() * Eigen::Vector4d::UnitW()).head<3>();
		data(i).MyTranslate(-trans, true);
	}
	organizeLevel();

}

void SandBox::organizeLevel()
{
	int sign1;
	int sign2;
	for (int i = 1; i < snake_pos; i++) {
		sign1 = (rand() % 2) * 2 - 1;
		sign2 = (rand() % 2) * 2 - 1;

		if (i > 0 && i < 4) {
			data(i).MyTranslate(Eigen::Vector3d(sign1 * (7 + (std::rand() % 4)), 0, sign2 * (7 + (std::rand() % 4))), true);
			data(i).set_colors(Eigen::RowVector3d(0.9, 0.9, 0.1));
		}
		else {
		data(i).MyTranslate(Eigen::Vector3d(sign1 * (10 + (std::rand() % 4)), 0, sign2 * (10 + (std::rand() % 4))), true);
		data(i).set_colors(Eigen::RowVector3d(0.1, 0.9, 0.1));
		}
	}
}





void SandBox::Animate(igl::opengl::ViewerCore &core)
{
	

	if (isActive)
	{
		if(isCameraUp)
		{
			core.camera_eye = Eigen::Vector3f(0, 20, 5);
			core.camera_zoom = 5.f;
			core.camera_translation = Eigen::Vector3f::Zero();
			core.camera_up = Eigen::Vector3f(0, -1, 0);
			core.density = 0.05f;
			core.gradient = 1.5f;
		}
		else
		{
			Eigen::Matrix4d transMat = data(snake_pos).MakeTransd();
			core.camera_eye = (rot_quaternion.toRotationMatrix()  *Eigen::Vector3d(0, 0, 0.8)).block(0, 0, 3, 1).cast<float>();
			core.camera_up = Eigen::Vector3d(0, 1, 0).cast<float>();
			core.camera_translation = ((transMat * Eigen::Vector4d(0, 0, 13.5, -1)).block(0,0,3,1)).block(0, 0, 3, 1).cast<float>();
			core.camera_zoom = 1.f;
			core.density = 0.75f;
			core.gradient = 5.f;
		}
		if (betweenLevelRestore) {
			betweenLevelsOrganize();
			betweenLevelRestore = false;
		}
		if(isSecondLevel)
			move_balls();
		for (int i = 1; i < data_list.size()-1; ++i)
		{
			new_check_collision(i, head_loc);

		}
		pre_draw();
		data(snake_pos).MyTranslate((CT.row(BE(0, 0)) - CT.row(BE(0, 1))).normalized() * 0.24, true);
		
		if (anim_t > 1)
		{
			update_next_pose();
			_dir = NONE;
			anim_t = anim_t_dir;
			rest_pose = curr_pose;
			curr_pose = next_pose;
			for (int i = next_pose.size() - 1; i > 1; --i)
			{
				next_pose[i] = curr_pose[i-1];
			}

			if (dir_change)
			{
				next_pose[1] = Eigen::Quaterniond::Identity();
				dir_change = false;
			}
			
		}
		

	}
	
}


