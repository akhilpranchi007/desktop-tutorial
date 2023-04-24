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


// Prepare the image for further analysis
void prepare_image(image& rgb1, image& a, image& b, image& v, int& t)
{
	/*
	// Initialising Parameters for the histogram
	double hist[70], hmin = 0, hmax = 0, h;
	int nhist = 60, init_1 = 1;
	fstream f;
	f.open("Histogram_Data.csv", ios::out);
	*/

	// Convert to grey scale image
	copy(rgb1, a);
	//save_rgb_image("a_out.bmp", v);

	// Scale the Image
	scale(a, b);
	copy(b, a);

	// Add Filter to smoothen the Image
	lowpass_filter(a, b);
	copy(b, a);

	// Compute the Threshold of the Image
	threshold(a, b, t);
	copy(b, a);

	/*
	// Making a Histogram to find the threshold value.
	histogram(a, hist, nhist, hmin, hmax);
	while (init_1)
	{
		for (int i = 0; i < nhist; i++)
		{
			// x --> Starting value for the bin values
			h = hmin + ((hmax - hmin) / nhist) * i;
			f << h << "," << hist[i] << "\n";
		}
		init_1--;
		f.close();
	}
	*/

	/*
	// Adjusting the threshold with Keyboard
	if (KEY(VK_UP))
	{
		t++;
	}

	if (KEY(VK_DOWN))
	{
		t--;
	}

	cout << "\n Threshold Value: "<<t;
	*/

	// Invert the Image
	invert(a, b);
	copy(b, a);

	// Performing erosions to remove small objects
	// Two erosions seemed to work pretty Well
	erode(a, b);
	copy(b, a);

	// Dialting the Image to close small holes
	dialate(a, b);
	copy(b, a);

	dialate(a, b);
	copy(b, a);
	copy(a, v);
}
// Pulse Width to Velocity
double convert(double pw)
{
	double pw_0 = 1500, pw_range = 500, pw_max, pw_min;
	double vmax = 100, v;

	// Saturate the pulse width value
	pw_max = pw_0 + pw_range;
	pw_min = pw_0 - pw_range;

	if (pw < pw_min) pw = pw_min;
	if (pw > pw_max) pw = pw_max;

	// Convert ppulse width to velocity
	v = (pw - pw_0) / pw_range;
	v *= vmax;
	return v;
}

// Euler Simulation
void sim_step(double pw_l, double pw_r, double dt, double x[4], double D)
{
	static int init = 1;
	const int n = 3; //  Number of state variables

	double dx[n + 1];
	double vr, vl, v, omega;

	// Initialise the required parameters
	while (init)
	{
		for (int i = 1; i < n; i++)
		{
			dx[i] = 0;
		}
		init = 0;
	}

	vr = convert(pw_r);
	vl = -convert(pw_l); // -ve because the servos are flipped

	v = (vr + vl) / 2;
	omega = (vr - vl) / D;

	dx[1] = v * cos(x[3]);
	dx[2] = v * sin(x[3]);
	dx[3] = omega;

	for (int i = 1; i <= n; i++)
	{
		x[i] = x[i] + dx[i] * dt;
	}
}

void colour_correction(image& rgb1)
{
	int h, w;
	unsigned char* p;

	h = rgb1.height;
	w = rgb1.width;
	p = rgb1.pdata;

	for (int i = 0; i < w; i++)
	{
		for (int j = 0; j < h; j++)
		{
			
			// Green
			if ((*p >= 118) && (*p <= 140)) // Blue Intensity
				if ((*(p + 1) >= 169) && (*(p + 1) <= 185)) // Green Intensity
					if ((*(p + 2) >= 55) && (*(p + 2) <= 80)) // Red Intensity
					{
						*p = 129;
						*(p + 1) = 177;
						*(p + 2) = 68;
					}

			// Red
			if ((*p >= 55) && (*p <= 90)) // Blue Intensity
				if ((*(p + 1) >= 80) && (*(p + 1) <= 92)) // Green Intensity
					if ((*(p + 2) >= 213) && (*(p + 2) <= 235)) // Red Intensity
					{
						*p = 72;
						*(p + 1) = 86;
						*(p + 2) = 224;
					}

			// Blue
			if ((*p >= 220) && (*p <= 235)) // Blue Intensity
				if ((*(p + 1) >= 150) && (*(p + 1) <= 163)) // Green Intensity
					if ((*(p + 2) >= 30) && (*(p + 2) <= 55)) // Red Intensity
					{
						*p = 227;
						*(p + 1) = 156;
						*(p + 2) = 43;
					}

			// Orange
			if ((*p >= 110) && (*p <= 170)) // Blue Intensity
				if ((*(p + 1) >= 180) && (*(p + 1) <= 205)) // Green Intensity
					if ((*(p + 2) >= 250) && (*(p + 2) <= 255)) // Red Intensity
					{
						// New Colour: Brown(R:102, G:51, B:0)
						*p = 0;
						*(p + 1) = 51;
						*(p + 2) = 102;
					}
			p += 3;
		}
	}
}

void centroid_colour(image& rgb, double ic, double jc,
	double& R, double& G, double& B)
{
	int w, h;
	unsigned char* p;

	w = rgb.width;
	h = rgb.height;
	p = rgb.pdata;

	for (int j = 0; j < h; j++)
	{
		for (int i = 0; i < w; i++)
		{
			if ((i == (int)ic) && (j == (int)jc))
			{
				B = *p;
				G = *(p + 1);
				R = *(p + 2);
			}
			p += 3;
		}

	}
}

void object_tracker(image& rgb1, image& a, image& label, double x[], double y[])
{
	int nlabels;
	double R, G, B;

	label_image(a, label, nlabels);
	for (int i = 1; i <= nlabels; i++)
	{
		centroid(a, label, i, x[i], y[i]);
		//cout << "\n i = " << i << "  x = " << x[i] << "  y = " << y[i];
	}
}

int find_targets(double& R, double& G, double& B)
{
	// Green Target
	if ((R == 68) && (G == 177) && (B == 129))
		return 1;
	
	// Red Target
	else if ((R == 224) && (G == 86) && (B == 72))
		return 2;

	// Orange Target
	else if ((R == 102) && (G == 51) && (B == 0))
		return 3;

	// Blue Target
	else if ((R == 43) && (G == 156) && (B == 227))
		return 4;

	// No target detected
	else return 0;
}

int search_target(image& a, image& label, image& rgb, int nlabel, int& n_target, int flag)
{
	unsigned short int* pl;
	int w, h, i, j, dr = 3, ds = 3;
	double ic, jc, smax, theta, x, y, i_obj, j_obj;

	int flag_1 = 0;

	double x_orange, y_orange;
	double Rc, Gc, Bc;

	double rmin, rmax;

	rmin = 77; // Min radius of search
	rmax = 82; // Max radius of search

	w = label.width;
	h = label.height;
	pl = (i2byte*)label.pdata;

	// Take the Centroid as the initial position
	centroid(a, label, nlabel, ic, jc);
	i = (int)ic;
	j = (int)jc;

	// Start the Search Pattern
	for (double r = rmin; r < rmax; r += dr)
	{
		smax = 2 * 3.1416 * r;
		for (double s = 0; s < smax; s += ds)
		{
			theta = s / r;
			x = ic + r * cos(theta);
			y = jc + r * sin(theta);
			i = (int)x;
			j = (int)y;

			
			// limit i and j to stay within the range
			if (i < 0) i = 0;
			if (i > label.width - 1) i = label.width - 1;
			if (j < 0) j = 0;
			if (j > label.height - 1) j = label.height - 1;
			
			nlabel = *(pl + i + j * w);
			if ((nlabel > 0))
			{
				if (flag == 1) // Finding Red Target
				{
					centroid(a, label, nlabel, i_obj, j_obj);
					centroid_colour(rgb, i_obj, j_obj, Rc, Gc, Bc);
					flag_1 = find_targets(Rc, Gc, Bc);
					if (flag_1 == 2)
					{
						n_target = nlabel;
						return 1;
					}
				}

				if (flag == 2) // Finding Green Target
				{
					centroid(a, label, nlabel, i_obj, j_obj);
					centroid_colour(rgb, i_obj, j_obj, Rc, Gc, Bc);
					flag_1 = find_targets(Rc, Gc, Bc);
					if (flag_1 == 1)
					{
						n_target = nlabel;
						return 1;
					}
				}

				if (flag == 3) // Finding Blue Target
				{
					centroid(a, label, nlabel, i_obj, j_obj);
					centroid_colour(rgb, i_obj, j_obj, Rc, Gc, Bc);
					flag_1 = find_targets(Rc, Gc, Bc);
					if (flag_1 == 4)
					{
						n_target = nlabel;
						return 1;
					}
				}

				if (flag == 4) // Finding Orange Target
				{
					centroid(a, label, nlabel, i_obj, j_obj);
					centroid_colour(rgb, i_obj, j_obj, Rc, Gc, Bc);
					flag_1 = find_targets(Rc, Gc, Bc);
					if (flag_1 == 3)
					{
						n_target = nlabel;
						return 1;
					}
				}
			}
		}
	}
	return 0;
}


int position_robot(double& x, double& y, double& theta,
	double& x1, double& y1, double& x2, double& y2)
{
	x = (x2 + x1) / 2;
	y = (y2 + y1) / 2;
	theta = atan2( (y1 - y2), (x1- x2) );
	return 0;

}

int track_orange_target(image& rgb1, image& a, image& label, image& v,
	double& x_orange, double& y_orange, double x_coord[], double y_coord[])
{
	int nl, nlabels, flag_1 = 0, n_target = 0, flag_2 = 0, counter = 0, i, k;
	double ic, jc, Rc, Gc, Bc;
	int n_object = 0, x, y;

	label_image(a, label, nlabels);

	// See if there are multiple orange objects with a blue target in its vicinity
	for (nl = 1; nl <= nlabels; nl++)
	{
		flag_2 = 0;
		centroid(a, label, nl, ic, jc);
		centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
		flag_1 = find_targets(Rc, Gc, Bc);
		if (flag_1 == 3)
		{
			counter = search_target(a, label, rgb1, nl, n_target, flag_1);
			if (counter == 1) // There is a blue target in the vicinity.
			{
				x_orange = ic;
				y_orange = jc;
				//cout << "\n ic = " << ic << "\t jc = " << jc;
				n_object++; // Number of orange objects with a blue target in the vicinity.
			}
		}
	}
	//cout << "\n Number of orange object = " << n_object;
	if (n_object == 1)
	{
		centroid_colour(rgb1, x_orange, y_orange, Rc, Gc, Bc);
		draw_point_rgb(v, x_orange, y_orange, Rc, Gc, Bc);
		return 0;
	}
	else if (n_object > 1)
	{
		for (nl = 1; nl <= nlabels; nl++)
		{
			flag_2 = 0;
			centroid(a, label, nl, ic, jc);
			for (i = 0; i <= nlabels; i++)
			{
				centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
				flag_1 = find_targets(Rc, Gc, Bc);
				if (flag_1 == 3)
				{
					counter = search_target(a, label, rgb1, nl, n_target, flag_1);
					if (counter == 1)
					{	
						if ((abs(ic - x_coord[i]) <= 1) && (abs(jc - y_coord[i]) <= 1))
						{
							flag_2 = 1;
						}
					}

				}
			}
			if ((flag_2 == 0) && (flag_1 == 3))
			{
				//cout << "\n ic = " << ic << "  jc: " << jc;
				counter = search_target(a, label, rgb1, nl, n_target, flag_1);
				if (counter == 1)
				{
					//cout << "\n ic = " << ic << "  jc: " << jc;
					x_orange = ic;
					y_orange = jc;
					draw_point_rgb(v, ic, jc, Rc, Gc, Bc);

				}
			}
		}
	}
	return 0;
}

int track_blue_target(image& rgb1, image& a, image& label, image& v,
	double& x_blue, double& y_blue, double x_coord[], double y_coord[])
{
	int nl, nlabels, flag_1 = 0, n_target = 0, flag_2 = 0, counter = 0, i, k;
	double ic, jc, Rc, Gc, Bc;
	int n_object = 0;

	label_image(a, label, nlabels);

	// See if there are multiple blue objects with a orange target in its vicinity
	for (nl = 1; nl <= nlabels; nl++)
	{
		flag_2 = 0;
		centroid(a, label, nl, ic, jc);
		centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
		flag_1 = find_targets(Rc, Gc, Bc);
		if (flag_1 == 4)
		{
			counter = search_target(a, label, rgb1, nl, n_target, flag_1);
			if (counter == 1) // There is a orange target in the vicinity.
			{
				x_blue = ic;
				y_blue = jc;
				//cout << "\n ic = " << ic << "\t jc = " << jc;
				n_object++; // Number of blue objects with a blue target in the vicinity.
			}
		}
	}
	//cout << "\n Number of blue object = " << n_object;
	if (n_object == 1)
	{
		centroid_colour(rgb1, x_blue, y_blue, Rc, Gc, Bc);
		draw_point_rgb(v, x_blue, y_blue, Rc, Gc, Bc);
		return 0;
	}
	else if (n_object > 1)
	{
		for (nl = 1; nl <= nlabels; nl++)
		{
			flag_2 = 0;
			centroid(a, label, nl, ic, jc);
			for (i = 0; i <= nlabels; i++)
			{
				centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
				flag_1 = find_targets(Rc, Gc, Bc);
				if (flag_1 == 4)
				{
					counter = search_target(a, label, rgb1, nl, n_target, flag_1);
					if (counter == 1)
					{
						if ((abs(ic - x_coord[i]) <= 1) && (abs(jc - y_coord[i]) <= 1))
						{
							flag_2 = 1;
						}
					}

				}
			}
			if ((flag_2 == 0) && (flag_1 == 4))
			{
				counter = search_target(a, label, rgb1, nl, n_target, flag_1);
				if (counter == 1)
				{
					x_blue = ic;
					y_blue = jc;
					draw_point_rgb(v, ic, jc, Rc, Gc, Bc);

				}
			}
		}
	}
	return 0;
}

int track_green_target(image& rgb1, image& a, image& label, image& v,
	double& x_green, double& y_green, double x_coord[], double y_coord[])
{
	int nl, nlabels, flag_1 = 0, n_target = 0, flag_2 = 0, counter = 0, i, k;
	double ic, jc, Rc, Gc, Bc;
	int n_object = 0;

	label_image(a, label, nlabels);

	// See if there are multiple green objects with a red target in its vicinity
	for (nl = 1; nl <= nlabels; nl++)
	{
		flag_2 = 0;
		centroid(a, label, nl, ic, jc);
		centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
		flag_1 = find_targets(Rc, Gc, Bc);
		if (flag_1 == 1)
		{
			counter = search_target(a, label, rgb1, nl, n_target, flag_1);
			if (counter == 1) // There is a orange target in the vicinity.
			{
				x_green = ic;
				y_green = jc;
				//cout << "\n ic = " << ic << "\t jc = " << jc;
				n_object++; // Number of grene objects with a red target in the vicinity.
			}
		}
	}
	//cout << "\n Number of green object = " << n_object;
	if (n_object == 1)
	{
		centroid_colour(rgb1, x_green, y_green, Rc, Gc, Bc);
		draw_point_rgb(v, x_green, y_green, Rc, Gc, Bc);
		return 0;
	}
	else if (n_object > 1)
	{
		for (nl = 1; nl <= nlabels; nl++)
		{
			flag_2 = 0;
			centroid(a, label, nl, ic, jc);
			for (i = 0; i <= nlabels; i++)
			{
				centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
				flag_1 = find_targets(Rc, Gc, Bc);
				if (flag_1 == 1)
				{
					counter = search_target(a, label, rgb1, nl, n_target, flag_1);
					if (counter == 1)
					{
						if ((abs(ic - x_coord[i]) <= 1) && (abs(jc - y_coord[i]) <= 1))
						{
							flag_2 = 1;
						}
					}

				}
			}
			if ((flag_2 == 0) && (flag_1 == 1))
			{
				counter = search_target(a, label, rgb1, nl, n_target, flag_1);
				if (counter == 1)
				{
					x_green = ic;
					y_green = jc;
					draw_point_rgb(v, ic, jc, Rc, Gc, Bc);

				}
			}
		}
	}
	return 0;
}

int track_red_target(image& rgb1, image& a, image& label, image& v,
	double& x_red, double& y_red, double x_coord[], double y_coord[])
{
	int nl, nlabels, flag_1 = 0, n_target = 0, flag_2 = 0, counter = 0, i, k;
	double ic, jc, Rc, Gc, Bc;
	int n_object = 0;

	label_image(a, label, nlabels);

	// See if there are multiple red objects with a green target in its vicinity
	for (nl = 1; nl <= nlabels; nl++)
	{
		flag_2 = 0;
		centroid(a, label, nl, ic, jc);
		centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
		flag_1 = find_targets(Rc, Gc, Bc);
		if (flag_1 == 2)
		{
			counter = search_target(a, label, rgb1, nl, n_target, flag_1);
			if (counter == 1) // There is a green target in the vicinity.
			{
				x_red = ic;
				y_red = jc;
				//cout << "\n ic = " << ic << "\t jc = " << jc;
				n_object++; // Number of red objects with a green target in the vicinity.
			}
		}
	}
	//cout << "\n Number of red object = " << n_object;
	if (n_object == 1)
	{
		centroid_colour(rgb1, x_red, y_red, Rc, Gc, Bc);
		draw_point_rgb(v, x_red, y_red, Rc, Gc, Bc);
		return 0;
	}
	else if (n_object > 1)
	{
		for (nl = 1; nl <= nlabels; nl++)
		{
			flag_2 = 0;
			centroid(a, label, nl, ic, jc);
			for (i = 0; i <= nlabels; i++)
			{
				centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
				flag_1 = find_targets(Rc, Gc, Bc);
				if (flag_1 == 2)
				{
					counter = search_target(a, label, rgb1, nl, n_target, flag_1);
					if (counter == 1)
					{
						if ((abs(ic - x_coord[i]) <= 1) && (abs(jc - y_coord[i]) <= 1))
						{
							flag_2 = 1;
						}
					}

				}
			}
			if ((flag_2 == 0) && (flag_1 == 2))
			{
				counter = search_target(a, label, rgb1, nl, n_target, flag_1);
				if (counter == 1)
				{
					x_red = ic;
					y_red = jc;
					draw_point_rgb(v, ic, jc, Rc, Gc, Bc);

				}
			}
		}
	}
	return 0;
}

void pulsewidth_to_velocity(int pw_l, int pw_r, double& v, double& w)
{
	const double D = 121.0; // Distance between the wheels.
	double vl, vr;
	const int vmax = 810;
	const int pw_0 = 1500, pw_range = 500;

	// left wheel velocity
	vl = (double)((pw_0 - pw_l) * vmax) / pw_range;

	// right wheel velocity
	vr = (double)((pw_r - pw_0) * vmax) / pw_range;

	// Linear Velocity
	v = (vl + vr) / 2;

	// angular velocity
	w = (vr - vl) / D;
}

void velocity_to_pulsewidth(double v, double w, int& pw_l, int& pw_r)
{
	const double D = 121.0; // Distance between the wheels.
	double vl, vr;
	const double vmax = 810.0;
	const int pw_0 = 1500, pw_range = 500;

	// Right wheel velocity
	vr = v + (w * D) / 2;

	// Left wheel velocity
	vl = v - (w * D) / 2;

	// Left Servo pulse width
	pw_l = pw_0 - (vl * pw_range) / vmax;

	// Right Servo pulse width
	pw_r = pw_0 + (vr * pw_range) / vmax;
}

int Steer_A(double x_b, double y_b, double x_a, double y_a, double theta_a, double& vel, double& w)
{
	double diff, theta_diff;
	double laser_min, laser_max;


	theta_diff = atan2((y_b - y_a), (x_b - x_a));
	//cout << "\n Theta_A = " << theta_a << " Theta_diff = " << theta_diff;


	laser_min = theta_a - (0.3 * pi / 2);
	laser_max = theta_a + (0.3 * pi / 2);
	//cout << "\n laser_min = " << laser_min << " laser_max = " << laser_max;

	// Two possible cases
	//Case 1. Theta is positive (0 included)
	if (theta_a >= 0)
	{
		if ((theta_diff >= laser_min) && (theta_diff <= laser_max))
		{
			// Opponent robot is within the firing range
			// no steering required
			//vel = 0;
			w = 0;
			return 1;
		}
		else if ((theta_diff >= theta_a - pi) && (theta_diff <= laser_min))
		{
			// The robot should turn right
			// Angular velocity is negative
			vel = 0;
			w = -10;
		}
		else if ((theta_diff >= laser_max) && (theta_diff <= pi))
		{
			// The robot should turn left
			// Angular velocity is positive
			vel = 0;
			w = 10;
		}
		else if ((theta_diff >= -pi) && (theta_diff <= theta_a - pi))
		{
			// The robot should turn left
			// Angular velocity is positive
			vel = 0;
			w = 10;
		}
	}

	//Case 2. Theta_a is negative.
	else if (theta_a < 0)
	{
		if ((theta_diff >= laser_min) && (theta_diff <= laser_max))
		{
			// Opponent robot is within the firing range
			// no steering required
			//vel = 0;
			w = 0;
			return 1;
		}
		else if ((theta_diff >= laser_max) && (theta_diff <= theta_a + pi))
		{
			// The robot should turn left
			// Angular velocity is positive
			vel = 0;
			w = 10;
		}
		else if ((theta_diff >= -pi) && (theta_diff <= laser_min))
		{
			// The robot should turn right
			// Angular velocity is negative
			vel = 0;
			w = -10;
		}
		else if ((theta_diff >= theta_a + pi) && (theta_diff <= pi))
		{
			// The robot should turn right
			// Angular velocity is negative
			vel = 0;
			w = -10;
		}
	}

	return 0;
}

int front_obstacle_sensor(image& v, image& a, image& label, double x_f, double y_f, double theta, int sensor_flag)
{
	int nlabel, i, j;
	unsigned short int* pl;
	double theta_min, theta_max, d_theta, ic, jc;
	double r;

	if ((x_f > 1) && (y_f > 1)) // 
	{
		r = 46; //Radius of search
		if (sensor_flag == 1)
		{
			theta_min = theta;
			theta_max = theta + pi / 2;
		}
		else if (sensor_flag == -1)
		{
			theta_min = theta - pi / 2;
			theta_max = theta;
		}
		else
		{
			theta_min = theta - pi / 2;
			theta_max = theta + pi / 2;
		}
		/*
		theta_min = theta - pi / 2;
		theta_max = theta + pi / 2; // Total angle of search = 180 degrees
		// Note: This can be reduced. 180 degrees is not needed.*/

		d_theta = 0.1;
		nlabel = -1;

		// Assign pointer to label image
		pl = (unsigned short int*)label.pdata;

		// search states from ( x + r*cos(theta_min), y + r*sin(theta_min) )
		for (double theta = theta_min; theta <= theta_max; theta += d_theta)
		{
			i = (int)x_f + r * cos(theta);
			j = (int)y_f + r * sin(theta);

			// limit i and j from going off the screen
			/*
			if (i < 0) i = 0;
			if (i > label.width - 1) i = label.width - 1;

			if (j < 0) j = 0;
			if (j > label.height - 1) j = label.height - 1;*/

			// limit robot from going off the screen
			if (i < 0) return 1;
			if (i > label.width - 1) return 1;

			if (j < 0) return 1;
			if (j > label.height - 1) return 1;   // Returns 1 if the robot is heading outside the border.

			// Check if there is an object at (i, j)
			nlabel = *(pl + i + j * label.width);
			{
				//centroid(a, label, nlabel, ic, jc);
				//draw_point_rgb(v, ic, jc, 0, 0, 0);

				if (nlabel > 0) return 2;	// Returns 2 if there is an obstacle in the way.
			}
		}
	}
	return 0;
}

int rear_obstacle_sensor(image& v, image& a, image& label, double x_r, double y_r, double theta, int sensor_flag)
{
	int nlabel, i, j;
	unsigned short int* pl;
	double theta_min, theta_max, d_theta, ic, jc;
	double r;

	theta += pi;

	if ((x_r > 1) && (y_r > 1))
	{
		r = 43.0; // Radius of search

		if (sensor_flag == 1)
		{
			theta_min = theta;
			theta_max = theta + pi / 2;
		}
		else if (sensor_flag == -1)
		{
			theta_min = theta - pi / 2;
			theta_max = theta;
		}
		else
		{
			theta_min = theta - pi / 2;
			theta_max = theta + pi / 2;
			//cout << "\n Theta_min = " << theta_min << " \t Theta_max = " << theta_max;
		}
		/*
		theta_min = theta - pi / 2;
		theta_max = theta + pi / 2; // Total angle of search = 180 degrees
		// Note: This can be reduced. 180 degrees is not needed.
		*/

		d_theta = 0.1;
		nlabel = -1;

		// Assign pointer to label image
		pl = (unsigned short int*)label.pdata;

		// search states from ( x + r*cos(theta_min), y + r*sin(theta_min) )
		for (double theta = theta_min; theta <= theta_max; theta += d_theta)
		{
			i = (int)x_r + r * cos(theta);
			j = (int)y_r + r * sin(theta);

			// limit i and j from going off the screen
			/*
			if (i < 0) i = 0;
			if (i > label.width - 1) i = label.width - 1;

			if (j < 0) j = 0;
			if (j > label.height - 1) j = label.height - 1;*/

			// limit robot from going off the screen
			if (i < 0) return 1;
			if (i > label.width - 1) return 1;

			if (j < 0) return 1;
			if (j > label.height - 1) return 1;   // Returns 1 if the robot is heading outside the border.

			// Check if there is an object at (i, j)
			nlabel = *(pl + i + j * label.width);
			{
				//centroid(a, label, nlabel, ic, jc);
				//draw_point_rgb(v, ic, jc, 0, 0, 0);

				if (nlabel > 0)
				{
					//cout << "\n nalbel = " << nlabel;
					//cout << i << "\t" << j << "\n";
					return 2;	// Returns 2 if there is an obstacle in the way.
				}
			}
		}
	}
	return 0;
}

void obstacle_detection(image& v, image& a, image& label, double x_f, double y_f, double x_r, double y_r,
	double theta, int& obstacle_flag_f, int& obstacle_flag_r, double& vel, double& w)
{
	int sensor_flag = 0;

	obstacle_flag_r = 0;
	obstacle_flag_f = 0;

	if (vel > 0)
	{
		sensor_flag = 0;
		// The Robot is heading forward
		// Therefore check the front sensor
		obstacle_flag_f = front_obstacle_sensor(v, a, label, x_f, y_f, theta, sensor_flag);

		// Activate rear sensor based on angular velocity
		if (w > 0)
		{
			sensor_flag = 1;
			obstacle_flag_r = rear_obstacle_sensor(v, a, label, x_r, y_r, theta, sensor_flag);
		}
		else if (w < 0)
		{
			sensor_flag = -1;
			obstacle_flag_r = rear_obstacle_sensor(v, a, label, x_r, y_r, theta, sensor_flag);
		}

	}
	else if (vel <= 0)
	{
		sensor_flag = 0;
		// The Robot is heading backward
		// Therefore check the rear sensor
		obstacle_flag_r = rear_obstacle_sensor(v, a, label, x_r, y_r, theta, sensor_flag);

		if (w > 0)
		{
			sensor_flag = 1;
			obstacle_flag_f = front_obstacle_sensor(v, a, label, x_f, y_f, theta, sensor_flag);
		}
		else if (w < 0)
		{
			sensor_flag = -1;
			obstacle_flag_f = front_obstacle_sensor(v, a, label, x_f, y_f, theta, sensor_flag);
		}
	}
}

// Previous Versions of tracking functions --- Just for reference
/*
int track_orange_target(image& rgb1, image& a, image& label, image& v,
	double& x_orange, double& y_orange, double x_coord[], double y_coord[])
{


	for (nl = 0; nl <= nlabels; nl++)
	{
		flag_2 = 0;
		centroid(a, label, nl, ic, jc);
		for (i = 1; i <= nlabels; i++)
		{
			centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
			flag_1 = find_targets(Rc, Gc, Bc);
			//cout << "\n ic = " << ic << "  jc: " << jc;
			if (flag_1 == 3)
			{
				counter = search_target(a, label, rgb1, nl, n_target, flag_1);
				if (counter == 1)
				{
					if ((abs(ic - x_coord[i]) <= 0.7) && (abs(jc - y_coord[i]) <= 0.7))
					{
						flag_2 = 1;
						return 0;
					}
				}

			}
		}
		if((flag_2 == 0)&&(flag_1==3))
		{
			counter = search_target(a, label, rgb1, nl, n_target, flag_1);
			if (counter == 1)
			{

				x_orange = ic;
				y_orange = jc;
				draw_point_rgb(v, ic, jc, Rc, Gc, Bc);

			}
		}
	}
	
}
*/

/*
void track_robot_A(image& rgb1, image& a, image& label, image& v,
				   double& x_red, double& y_red, double& x_green, double& y_green)
{
	int nl, nlabels, flag_1 = 0, n_target, counter;
	double ic, jc, Rc, Gc, Bc;

	label_image(a, label, nlabels);

	for (nl = 0; nl <= nlabels; nl++)
	{
		centroid(a, label, nl, ic, jc);
		centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
		flag_1 = find_targets(Rc, Gc, Bc);

		if (flag_1 == 1) // Tracking Green Target
		{
			counter = search_target(a, label, rgb1, nl, n_target, flag_1);
			if (counter == 1)
			{
				
				x_red = ic;
				y_red = jc;
				draw_point_rgb(v, ic, jc, Rc, Gc, Bc);
				centroid(a, label, n_target, ic, jc);
				centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
				draw_point_rgb(v, ic, jc, Rc, Gc, Bc);
				x_green = ic;
				y_green = jc;
			}
		}

		
		if (flag_1 == 2) // Tracking Red Target
		{
			counter = search_target(a, label, rgb1, nl, n_target, flag_1);
			if (counter == 1)
			{
				centroid(a, label, n_target, ic, jc);
				centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
				draw_point_rgb(v, ic, jc, Rc, Gc, Bc);
			}
		}
		

	}
}
*/

/*
void track_robot_B(image& rgb1, image& a, image& label, image& v,
					double &x_orange, double &y_orange, double &x_blue, double &y_blue)
{
	int nl, nlabels, flag_1 = 0, n_target, counter = 0;
	double ic, jc, Rc, Gc, Bc;

	const int n_obs = 6;

	static int p_obs = 0;

	static double x_obs[n_obs], y_obs[n_obs];

	label_image(a, label, nlabels);

	for (nl = 0; nl <= nlabels; nl++)
	{
		centroid(a, label, nl, ic, jc);
		centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
		flag_1 = find_targets(Rc, Gc, Bc);

		if (flag_1 == 3) // Tracking Blue Target
		{
				counter = search_target(a, label, rgb1, nl, n_target, flag_1);
				if (counter == 1)
				{
					x_orange = ic;
					y_orange = jc;
					draw_point_rgb(v, ic, jc, Rc, Gc, Bc);
					centroid(a, label, n_target, ic, jc);
					centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
					draw_point_rgb(v, ic, jc, Rc, Gc, Bc);
					x_blue = ic;
					y_blue = jc;
				}
		}


		if (flag_1 == 4) // Tracking Orange target
		{
				counter = search_target(a, label, rgb1, nl, n_target, flag_1);
				if (counter == 1)
				{
					centroid(a, label, n_target, ic, jc);
					centroid_colour(rgb1, ic, jc, Rc, Gc, Bc);
					draw_point_rgb(v, ic, jc, Rc, Gc, Bc);
				}
		}
	}
}
*/
