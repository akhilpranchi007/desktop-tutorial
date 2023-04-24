// Header Files for the custom functions used in the project

void prepare_image(image& rgb1, image& a, image& b, image& v, int& t);

// Converts the pulse widths to linear velocity
double convert(double pw);

void colour_correction(image& a);

void centroid_colour(image& rgb, double ic, double jc,
	double& R, double& G, double& B);

void object_tracker(image& rgb1, image& a, image& label, double x[], double y[]);

int find_targets(double& R, double& G, double& B);

int search_target(image& a, image& label, image& rgb, int nlabel, int& n_target, int flag);

int position_robot(double& x, double& y, double& theta,
	double& x1, double& y1, double& x2, double& y2);

int track_orange_target(image& rgb1, image& a, image& label, image& v,
	double& x_orange, double& y_orange, double x_coord[], double y_coord[]);

int track_blue_target(image& rgb1, image& a, image& label, image& v,
	double& x_blue, double& y_blue, double x_coord[], double y_coord[]);

int track_red_target(image& rgb1, image& a, image& label, image& v,
	double& x_red, double& y_red, double x_coord[], double y_coord[]);

int track_green_target(image& rgb1, image& a, image& label, image& v,
	double& x_green, double& y_green, double x_coord[], double y_coord[]);

void pulsewidth_to_velocity(int pw_l, int pw_r, double& v, double& w);

void velocity_to_pulsewidth(double v, double w, int& pw_l, int& pw_r);

int Steer_A(double x_b, double y_b, double x_a, double y_a, double theta_a,
	double& vel, double& w);

int front_obstacle_sensor(image& v, image& a, image& label, double x_f, 
	double y_f, double theta, int sensor_flag);

int rear_obstacle_sensor(image& v, image& a, image& label, double x_r, 
	double y_r, double theta, int sensor_flag);

void obstacle_detection(image& v, image& a, image& label, double x_f, double y_f, double x_r,
	double y_r,	double theta, int& obstacle_flag_f, int& obstacle_flag_r, double& vel, double& w);
