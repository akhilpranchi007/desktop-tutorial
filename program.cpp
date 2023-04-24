
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#define pi 3.141592

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

#include "myfunctions.h"

extern robot_system S1;

int main() 
{
	double x0, y0, theta0, x0_o, y0_o, theta0_o, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	const double i_center = 320, j_center = 240;

	// TODO: it might be better to put this model initialization
	// section in a separate function

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	N_obs = 2;

	x_obs[1] = 135; // pixels
	y_obs[1] = 135 ; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 400; // pixels
	y_obs[2] = 400; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)

	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;

	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file

	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp", "obstacle_blue.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent) 
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	level = 1;
	set_simulation_mode(mode, level);

	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0, y0, theta0);

	x0_o = 150;
	y0_o = 375;
	theta0_o = 3.14159 / 4;
	set_opponent_position(x0_o, y0_o, theta0);
	

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1500; // pulse width for stop for left wheelus
	pw_r = 1500; // pulse width for stop for right wheel
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)

	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// set initial inputs
	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	
	// opponent inputs
	pw_l_o = 1500; // pulse width for left wheel servo (us)
	pw_r_o = 1500; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)
	

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
		opponent_max_speed);
	

	// regular vision program ////////////////////////////////

	// Step 1: Define the Parameters and allocate the image
	image rgb0, rgb1, a, b, v, c;
	int height, width, t, nlabels, nl = 2;
	
	//measure initial clock time
	tc0 = high_resolution_time();
	
	double ic, jc, Rc, Gc, Bc; // ic and jc --> Centroid Position
							   // Rc, Gc, and Bc --> Centroid Colour
	
	double xa_f, xa_r, ya_f, ya_r, x_a, y_a, theta_a; // Robot A tracker
								   // f --> Front; r --> Rear
	
	double xb_f, xb_r, yb_f, yb_r, x_b, y_b, theta_b; // Robot B tracker
								   // f --> Front; r --> Rear
	
	 double x, y, theta, dist;		

	 double vel_a, w_a; // Linear and angular velocity of robot a.
	 int sensor_flag;
	 double vel_b, w_b; // Linear and angular velocity of robot b.

	 double x_coord[20], y_coord[20];
	 int init = 0;

	 int obstacle_flag_f_a = 0, obstacle_flag_r_a = 0;
	 int obstacle_flag_f_b = 0, obstacle_flag_r_b = 0;

	 const int Pw_0 = 15000; // Reference Speed


	fstream f1, f2;
	f1.open("position_robot_a.csv", ios::out);
	f2.open("position_robot_b.csv", ios::out);
	

	// Standard...!! Do not change this.
	height = 480;
	width = 640;

	// Define Image Parameters
	// rgb0 --> Original Image.
	rgb0.type = RGB_IMAGE;
	rgb0.width = width;
	rgb0.height = height;

	// rgb1 --> Copy of Original Image.
	rgb1.type = RGB_IMAGE;
	rgb1.width = width;
	rgb1.height = height;

	// a --> Greysacle Image: Used for doing the image processing.
	a.type = GREY_IMAGE;
	a.width = width;
	a.height = height;

	// b --> Greyscale Image: Temporary Image for storing the results.
	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;

	// v --> RGB Image: Used for viewing the output.
	v.type = RGB_IMAGE;
	v.width = width;
	v.height = height;

	// c --> Label Image: For storing the labels of the objects
	c.type = LABEL_IMAGE;
	c.width = width;
	c.height = height;


	// Allocates Space in the memory for the image
	allocate_image(rgb0);
	allocate_image(rgb1);
	allocate_image(a);
	allocate_image(b);
	allocate_image(v);
	allocate_image(c);

	cout << "\n Press space to load the image.";
	pause();

	t = 202; // Threshold value
	
	//fstream f;
	//f.open("Distance.csv", ios::out);
	

	while (1)
	{
		tc = high_resolution_time() - tc0;
		// Gets image from simulation and stores it in rgb0.
		acquire_image_sim(rgb0);
		
		copy(rgb0, rgb1); // Copying the original image to do image processing.
		
		// Fix the colour intensity of the objects
		colour_correction(rgb1);
		//save_rgb_image("rgb_out.bmp", rgb1);
		//view_rgb_image(rgb1);
		
		// Prepare the image for further analysis
		prepare_image(rgb1, a, b, v, t);

		if (init == 0)
		{
			init = 1;
			object_tracker(rgb1, a, c, x_coord, y_coord);
		}
		//object_tracker(rgb1, a, c, x_coord, y_coord);

		// Tracks the centroids of Robot B
		track_orange_target(rgb1, a, c, v, xb_f, yb_f,x_coord,y_coord);
		track_blue_target(rgb1, a, c, v, xb_r, yb_r, x_coord, y_coord);
		position_robot(x_b, y_b, theta_b, xb_f, yb_f, xb_r, yb_r);
		pulsewidth_to_velocity(pw_l_o, pw_r_o, vel_b, w_b);
		obstacle_detection(v, a, c, xb_f, yb_f, xb_r, yb_r, theta_b, obstacle_flag_f_b, obstacle_flag_r_b, vel_b, w_b);
		
		//cout << "\n Vel = " << vel_b << "\t w_b = " << w_b;
		
		if ( (obstacle_flag_f_b >= 1) || (obstacle_flag_r_b >= 1) )
		{
			
			// Stop the robot
			vel_b = 0;
			w_b = 0;

			velocity_to_pulsewidth(vel_b, w_b, pw_l_o, pw_r_o);
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
				opponent_max_speed);
		}
		//cout << "\n Obstacle Flag = " << obstacle_flag;
		f2 << tc << "," << x_b << "," << y_b << "," << theta_b << "\n";

		// Tracks the centroids of Robot A
		track_green_target(rgb1, a, c, v, xa_f, ya_f, x_coord, y_coord);
		track_red_target(rgb1, a, c, v, xa_r, ya_r, x_coord, y_coord);
		position_robot(x_a, y_a, theta_a, xa_f, ya_f, xa_r, ya_r);
		pulsewidth_to_velocity(pw_l, pw_r, vel_a, w_a);
		obstacle_detection(v, a, c, xa_f, ya_f, xa_r, ya_r, theta_a, obstacle_flag_f_a, obstacle_flag_r_a, vel_a, w_a);
				
		//cout << "\n Vel = " << vel_a << "\t w_a = " << w_a;
		
		if (obstacle_flag_f_a == 1) // Front end of the robot is going off the screen
		{
			// Reverse the robot
			vel_a = -405;
			w_a = 0;
			velocity_to_pulsewidth(vel_a, w_a, pw_l, pw_r);
			set_inputs(pw_l, pw_r, pw_laser, laser,
				light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);
		}
		else if (obstacle_flag_r_a == 1) // Rear end of the robot is going off the screen
		{
			// Drive the robot forward
			vel_a = 405;
			w_a = 0;
			velocity_to_pulsewidth(vel_a, w_a, pw_l, pw_r);
			set_inputs(pw_l, pw_r, pw_laser, laser,
				light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);
		}
		else if ((obstacle_flag_f_a == 2) || (obstacle_flag_r_a == 2))
		{
			// stop the car
			vel_a = 0;
			w_a = 0;
			velocity_to_pulsewidth(vel_a, w_a, pw_l, pw_r);
			set_inputs(pw_l, pw_r, pw_laser, laser,
				light, light_gradient, light_dir, image_noise,
				max_speed, opponent_max_speed);
		}
		
		// Steer Robot A towards the target
		Steer_A(x_b, y_b, x_a, y_a, theta_a, vel_a, w_a);
		velocity_to_pulsewidth(vel_a, w_a, pw_l, pw_r);
		set_inputs(pw_l, pw_r, pw_laser, laser,
			light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		//cout << "\n Obstacle Flag = " << obstacle_flag_a;
		f1 << tc <<"," << x_a << "," << y_a << "," << theta_a << "\n";
		

		// set_inputs(pw_l, pw_r, pw_laser, laser,
		//			light, light_gradient, light_dir, image_noise,
		//		max_speed, opponent_max_speed);
		

		// Used for calculating the distance between the two targets
		// dist = pow((x_orange - x_blue),2) + pow((y_orange - y_blue),2);
		// dist = pow(dist,0.5);
		// f << dist << "\n";
					
		// view_rgb_image(rgb0); // Dispalys the image on image view
		
		view_rgb_image(rgb0);
	    
		// Controlling Robot B manually
		if (KEY(VK_UP))
		{
			vel_b += 20;
		}
		else if (KEY(VK_DOWN))
		{
			vel_b -= 20;
		}
		// Saturate linear velocity
		if (vel_b > 810.0) vel_b = 810.0;
		if (vel_b < -810.0) vel_b = -810.0;

		if (KEY(VK_LEFT))
		{
			w_b += 0.5;
		}
		else if (KEY(VK_RIGHT))
		{
			w_b -= 0.5;
		}
		// Saturate angular velocity
		if (w_b > 12.5) w_b = 12.5;
		if (w_b < -12.5) w_b = -12.5;

		velocity_to_pulsewidth(vel_b, w_b, pw_l_o, pw_r_o);
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
			opponent_max_speed);

		
		// Controlling Robot A manually
		if (KEY(VK_UP))
		{
			vel_a += 20;
		}
		else if (KEY(VK_DOWN))
		{
			vel_a -= 20;
		}
		// Saturate linear velocity
		if (vel_a > 810.0) vel_a = 810.0;
		if (vel_a < -810.0) vel_a = -810.0;

		if (KEY(VK_LEFT))
		{
			w_a += 0.5;
		}
		else if (KEY(VK_RIGHT))
		{
			w_a -= 0.5;
		}
		// Saturate angular velocity
		if (w_a > 12.5) w_a = 12.5;
		if (w_a < -12.5) w_a = -12.5;

		velocity_to_pulsewidth(vel_a, w_a, pw_l, pw_r);
		set_inputs(pw_l, pw_r, pw_laser, laser,
			light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);
		

		Sleep(2); // Add some delay, Otherwise the simulation is too fast
	}
}