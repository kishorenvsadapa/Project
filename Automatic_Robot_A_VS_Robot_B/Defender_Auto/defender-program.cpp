
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "update_simulation.h"

#include "timer.h"


using namespace std;
#define PI 3.14159265

extern robot_system* S1;

// class to deal with (x,y) points in plane
class position;
static vector<position> obstacles_obstruction_points;

// declare vectors
vector<position> next_destination_queue;
vector<position> shifted_destinations;
static vector<position> obstacle_cent;
static vector<position> robot_cent;

image label, rgb0, rgb_aux, labeled;
image a, b, c;

int nlabels, B, G, R;
static double height = 480;
static double width = 640;
int	tvalue = 65; // threshold value

double green_x, green_y, red_x, red_y, orange_x, orange_y, blue_x, blue_y;

static int cur_recursive_calls;
bool display_checks;

ibyte* p;

void initialize_vision(image& rgb, image& rgb0, image& rgb_aux, image& a, image& b, image& c, image& label, image& labeled);
void deactivate_vision(image& rgb, image& rgb0, image& rgb_aux, image& a, image& b, image& c, image& label, image& labeled);
void manipulation_green_pixels(image& rgb0);
void manipulation_red_pixels(image& rgb0);
void manipulation_orange_pixels(image& rgb0);
void manipulation_blue_pixels(image& rgb0);
void highlight_greens(image& rgb0);
void highlight_reds(image& rgb0);
void highlight_oranges(image& rgb0);
void highlight_blues(image& rgb0);
void average_colour(image& rgb, image& label_image, int label_num, double& R, double& G, double& B);

void obstacle_points();
void obstacle_centroids(image& rgb);
void robot_centroids(image& rgb, double& green_x, double& green_y, double& red_x, double& red_y, double& orange_x, double& orange_y, double& blue_x, double& blue_y);
void calculate_next_position(position cur, position destination);
int* shooting(position cur, position target, double angle_cur, double alpha);
int* robot_control(position cur, double angle_cur);
int* rotation_control(position cur, position dest, double angle_cur);

double constrain_angle_negpi_pi(double angle);
double constrain_angle_0_2pi(double angle);

// Begin vision

void initialize_vision(image& rgb, image& rgb0, image& rgb_aux, image& a, image& b, image& c, image& label, image& labeled)

{
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	rgb0.type = RGB_IMAGE;
	rgb0.width = width;
	rgb0.height = height;

	rgb_aux.type = RGB_IMAGE;
	rgb_aux.width = width;
	rgb_aux.height = height;

	a.type = GREY_IMAGE;
	a.width = width;
	a.height = height;

	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;

	c.type = GREY_IMAGE;
	c.width = width;
	c.height = height;

	labeled.type = LABEL_IMAGE;
	labeled.width = width;
	labeled.height = height;

	label.type = LABEL_IMAGE;
	label.width = width;
	label.height = height;

	// images memory allocation
	allocate_image(rgb);
	allocate_image(rgb0);
	allocate_image(rgb_aux);
	allocate_image(a);
	allocate_image(b);
	allocate_image(c);
	allocate_image(label);
	allocate_image(labeled);
}


void deactivate_vision(image& rgb, image& rgb0, image& rgb_aux, image& a, image& b, image& c, image& label, image& labeled)

{

	// free images memory
	free_image(rgb);
	free_image(rgb0);
	free_image(rgb_aux);
	free_image(a);
	free_image(b);
	free_image(c);
	free_image(label);
	free_image(labeled);

}

// Extract green pixels

void manipulation_green_pixels(image& rgb0)
{
	p = rgb0.pdata;
	for (int k = 0; k < width * height; k++)
	{
		B = *(p);
		G = *(p + 1);
		R = *(p + 2);
		if (B < 140 && R < 140 && G > 160)
		{
			B = 255;
			G = 255;
			R = 255;
		}
		else
		{
			B = 0;
			G = 0;
			R = 0;
		}
		*p = B;
		*(p + 1) = R;
		*(p + 2) = G;
		p += 3;
	}
}

// Extract red pixels

void manipulation_red_pixels(image& rgb0)
{
	p = rgb0.pdata;
	for (int k = 0; k < width * height; k++)
	{
		B = *(p);
		G = *(p + 1);
		R = *(p + 2);
		if (R > 200 && B < 100 && G < 100)
		{
			B = 255;
			G = 255;
			R = 255;
		}
		else
		{
			B = 0;
			G = 0;
			R = 0;
		}
		*p = B;
		*(p + 1) = R;
		*(p + 2) = G;
		p += 3;
	}
}


// Extract orange pixels

void manipulation_orange_pixels(image& rgb0)
{
	p = rgb0.pdata;
	for (int k = 0; k < width * height; k++)
	{
		B = *(p);
		G = *(p + 1);
		R = *(p + 2);
		if (R > 200 && B < 50 && G < 200)
		{
			B = 255;
			G = 255;
			R = 255;
		}
		else
		{
			B = 0;
			G = 0;
			R = 0;
		}
		*p = B;
		*(p + 1) = R;
		*(p + 2) = G;
		p += 3;
	}
}

// Extract blue pixels


void manipulation_blue_pixels(image& rgb0)
{
	p = rgb0.pdata;
	for (int k = 0; k < width * height; k++)
	{
		B = *(p);
		G = *(p + 1);
		R = *(p + 2);
		if (R < 30 && B > 220 && G < 30)
		{
			B = 255;
			G = 255;
			R = 255;
		}
		else
		{
			B = 0;
			G = 0;
			R = 0;
		}
		*p = B;
		*(p + 1) = R;
		*(p + 2) = G;
		p += 3;
	}
}


void highlight_greens(image& rgb0) {

	p = rgb0.pdata;
	for (int z = 0; z < (int)width * height; z++)
	{
		B = *(p);
		G = *(p + 1);
		R = *(p + 2);
		if (125 < B < 140 && G > 175 && 65 < R < 75)
		{
			B = 0;
			G = 255;
			R = 0;
		}

		*p = B;
		*(p + 1) = G;
		*(p + 2) = R;
		p += 3;
	}

}


void highlight_reds(image& rgb0) {

	p = rgb0.pdata;
	for (int z = 0; z < (int)width * height; z++)
	{
		B = *(p);
		G = *(p + 1);
		R = *(p + 2);
		if (70 < B < 80 && 85 < G < 92 && R > 220)
		{
			B = 0;
			G = 0;
			R = 255;
		}

		*p = B;
		*(p + 1) = G;
		*(p + 2) = R;
		p += 3;
	}

}


void highlight_oranges(image& rgb0) {

	p = rgb0.pdata;
	for (int z = 0; z < (int)width * height; z++)
	{
		B = *(p);
		G = *(p + 1);
		R = *(p + 2);
		if (115 < B < 135 && 180 < G < 195 && R > 245)
		{
			B = 30;
			G = 190;
			R = 225;
		}

		*p = B;
		*(p + 1) = G;
		*(p + 2) = R;
		p += 3;
	}

}

void highlight_blues(image& rgb0) {

	p = rgb0.pdata;
	for (int z = 0; z < (int)width * height; z++)
	{
		B = *(p);
		G = *(p + 1);
		R = *(p + 2);
		if (B > 215 && 150 < G < 165 && R < 70)
		{
			B = 240;
			G = 20;
			R = 25;
		}

		*p = B;
		*(p + 1) = G;
		*(p + 2) = R;
		p += 3;
	}

}


void average_colour(image& rgb, image& label_image, int label_num, double& R, double& G, double& B)
{
	int i, j, k, label, N;
	int height, width; // ints are 4 bytes on the PC
	ibyte* p; // pointer to colour components in the rgb image
	i2byte* pl; // pointer to the label image

	height = rgb.height;
	width = rgb.width;

	p = rgb.pdata;
	pl = (i2byte*)label_image.pdata;

	// initialize the summation varibles to compute average colour
	R = 0.0;
	G = 0.0;
	B = 0.0;
	N = 0; // number of pixels with the label number of interest

	for (k = 0; k < width * height; k++) { // loop for kth pixel

		// how to get j and i from k ?
		i = k % width;
		j = (k - i) / width;
		label = *pl;

		// collect data if the pixel has the label of interest
		if (label == label_num) {
			N++;
			// 3 bytes per pixel -- colour in order BGR
			B += *p; // 1st byte in pixel
			G += *(p + 1); // 2nd byte in pixel
			R += *(p + 2); // 3rd
		}

		// increment pointers
		p += 3; // 3 bytes per pixel
		pl++;

	}

	// compute average colour
	R = R / N;
	G = G / N;
	B = B / N;

}


// Operations on points p1, p2 --> current position, desired position

class position {
public:
	double x;
	double y;
	position() {}
	position(double x_pos, double y_pos)
	{
		x = x_pos;
		y = y_pos;
	}

	static double distance_between_points(position p1, position p2)
	{
		return sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));
	}

	static double angle_between_points(position p1, position p2)
	{
		return atan2(p2.y - p1.y, p2.x - p1.x);
	}

	// check the presence of an obstacle between the two points p1, p2

	static pair <bool, position> obs_bool(position p1, position p2) {
		vector <position> obs_cent_critical;
		vector <double> obs_cent_critical_distances;
		double p1_p2_angle = angle_between_points(p1, p2);
		double safety_coefficient = 75.0;
		bool obs_bool = 0;
		bool obs_bool_aux = 0;
		double distance_line_center;
		double m; // angular coefficient of the line connecting the two points p1, p2
		double b; // bias term for the line connecting p1, p2
		double distance, distance_0, distance_cent_p1;
		double dist_max;
		position obs_critical;
		double difference_angles_aux;
		double difference_angles;
		//cout << "\np1 --> " << "(" << p1.x << "," << p1.y << ")";
		//cout << "\np2 --> " << "(" << p2.x << "," << p2.y << ")";
		p1_p2_angle = angle_between_points(p1, p2);

		if ((p2.x == p1.x) && (p2.y >= p1.y)) {
			m = 1000.0;
		}
		else if ((p2.x == p1.x) && (p2.y < p1.y)) {
			m = -1000.0;
		}
		else {
			m = (p2.y - p1.y) / (p2.x - p1.x);
		}
		b = p1.y - (m * p1.x);

		//cout << "\nm: " << m << ", b: " << b;
		for (int j = 0; j < obstacle_cent.size(); j++) // check if the line connecting the two points (cur and destination) contains the centroid of an obstacle
		{
			distance_line_center = abs((m * obstacle_cent[j].x) - obstacle_cent[j].y + b) / sqrt(pow(m, 2) + 1);

			dist_max = (position::distance_between_points(p1, p2)) + safety_coefficient;

			/*
			if ((display_checks == 1) || (display_checks == 0)) {
				cout << "\nDistance from the obstacle to the line: " << distance_line_center;
				cout << "\nCondition checked for the distance? " << distance_between_points(p1, obstacle_cent[j]) << " <= " << dist_max << " ? ";
				cout << "\nAngle of interest: " << abs(constrain_angle_0_2pi(angle_between_points(p1, obstacle_cent[j])) - constrain_angle_0_2pi(p1_p2_angle));
				cout << "\nAngle between p1 and obstacle: " << constrain_angle_0_2pi(angle_between_points(p1, obstacle_cent[j]));
				cout << "\nAngle between p1 and p2: " << constrain_angle_0_2pi(p1_p2_angle);
				cout << "\nMargin: " << safety_coefficient;
			}
			*/

			difference_angles_aux = abs(constrain_angle_0_2pi(angle_between_points(p1, obstacle_cent[j])) - constrain_angle_0_2pi(p1_p2_angle));
			if (difference_angles_aux >= PI) {
				difference_angles = 2 * PI - difference_angles_aux;
			}
			else {
				difference_angles = difference_angles_aux;
			}

			if ((abs(distance_line_center) < safety_coefficient) && ((distance_between_points(p1, obstacle_cent[j]) * cos(difference_angles)) <= dist_max) && (difference_angles <= PI / 2)) {
				obs_bool = 1;
				obs_bool_aux = 1;
				obs_cent_critical.push_back(obstacle_cent[j]);
				obs_cent_critical_distances.push_back(position::distance_between_points(p1, obstacle_cent[j]));

			}

			else {
				obs_bool_aux = 0;
			}
			//cout << "\nCurrent obstacle analyzed -->" << "(" << obstacle_cent[j].x << "," << obstacle_cent[j].y << ")";
			//cout << "\nObstacle # " << j + 1 << ": detection --> " << obs_bool_aux;
			//cout << "\n";
		}

		if ((display_checks == 1) || (display_checks == 0)) {
			//cout << "\nIs there an obstacle on the way? " << obs_bool << " | 0 --> No, 1 --> Yes";
			//cout << "\n";
		}


		//cout << "\n Obstacle critical vector size: " << obs_cent_critical.size();
		//cout << "\n Critical obstacle distances vector size: " << obs_cent_critical_distances.size();

		if (obs_cent_critical.size() != 0) {
			double min_distance = obs_cent_critical_distances[0];
			for (int k = 0; k < obs_cent_critical.size(); k++) {
				if (obs_cent_critical_distances[k] <= min_distance) {
					min_distance = obs_cent_critical_distances[k];
					obs_critical = obs_cent_critical[k];
				}
			}
		}


		//cout << "\nCritical obstacle --> " << "(" << obs_critical.x << "," << obs_critical.y << ")";
		obs_cent_critical.clear();
		obs_cent_critical_distances.clear();
		return make_pair(obs_bool, obs_critical);
	}


	static position best_hiding_position(position cur, position opponent) {
		double margin = 120.0; // do not want to collide with obstacle
		double min = 0;
		double threshold = 10;
		bool pos_found = 0;
		position optimal_obs_cent;
		position hiding_point;
		position hiding_point_aux;
		for (auto& element : obstacle_cent)
		{
			double angle_obs_aux = position::angle_between_points(opponent, element);
			hiding_point_aux = position(element.x + cos(angle_obs_aux) * margin, element.y + sin(angle_obs_aux) * margin);
			double obs_dist = position::distance_between_points(element, opponent); // find the furthest obstacle from the opponent
			if ((obs_dist > min) && (position::bool_edge(hiding_point_aux) == 0))
			{
				min = obs_dist;
				optimal_obs_cent = element;
				pos_found = 1;
			}
		}

		if (pos_found == 0) {
			
			position hiding_point_first_quadrant = position(480, 360);
			position hiding_point_second_quadrant = position(160, 360);
			position hiding_point_third_quadrant = position(160, 120);
			position hiding_point_fourth_quadrant = position(480, 120);
			bool obs_bool_hiding_point_first_quadrant = (position::obs_bool(cur, hiding_point_first_quadrant)).first;
			bool obs_bool_hiding_point_second_quadrant = (position::obs_bool(cur, hiding_point_second_quadrant)).first;
			bool obs_bool_hiding_point_third_quadrant = (position::obs_bool(cur, hiding_point_third_quadrant)).first;
			bool obs_bool_hiding_point_fourth_quadrant = (position::obs_bool(cur, hiding_point_fourth_quadrant)).first;

			if ((opponent.x <= 320) && (opponent.y <= 240) && (obs_bool_hiding_point_first_quadrant == 0)) {
				hiding_point.x = 480;
				hiding_point.y = 360;
			}
			else if ((opponent.x <= 320) && (opponent.y >= 240) && (obs_bool_hiding_point_fourth_quadrant == 0)) {
				hiding_point.x = 480;
				hiding_point.y = 120;
			}
			else if ((opponent.x >= 320) && (opponent.y <= 240) && (obs_bool_hiding_point_second_quadrant == 0)) {
				hiding_point.x = 160;
				hiding_point.y = 360;
			}
			else if ((opponent.x >= 320) && (opponent.y >= 240) && (obs_bool_hiding_point_third_quadrant == 0)) {
				hiding_point.x = 160;
				hiding_point.y = 120;
			}
			else {
				hiding_point.x = cur.x;
				hiding_point.y = cur.y;
			}
		}

			if (pos_found == 1) {
				double angle_obs = position::angle_between_points(opponent, optimal_obs_cent);

				hiding_point = position(optimal_obs_cent.x + cos(angle_obs) * margin, optimal_obs_cent.y + sin(angle_obs) * margin);
			}

			//cout << "\nBest hiding point found: " << " ( " << hiding_point.x << " , " << hiding_point.y << " ) ";

			return hiding_point;
	}

	static vector<position> shifted_destinations_computation(position p1, position p2, double shift_margin)
	{
		vector<position> destinations;
		position obs_critical;
		double obs_critical_x;
		double obs_critical_y;
		double p1_p2_angle = angle_between_points(p1, p2);
		double m; // angular coefficient of the line connecting the two points p1, p2
		double b; // bias term for the line connecting p1, p2
		double distance, distance_0, distance_cent_p1;
		double eps = 0.1; // to account for small angles
		position sh_dest_1, sh_dest_2;

		if ((p2.x == p1.x) && (p2.x >= p1.x)) {
			m = 1000.0;
		}
		else if ((p2.x == p1.x) && (p2.x < p1.x)) {
			m = -1000.0;
		}
		else {
			m = (p2.y - p1.y) / (p2.x - p1.x);
		}

		b = p1.y - (m * p1.x);

		//cout << "\nm value: " << m;
		//cout << "\nb value: " << b;

		/*
		cout << "\nAngle between p1, p2: " << p1_p2_angle;
		*/


		obs_critical = position::obs_bool(p1, p2).second;
		obs_critical_x = obs_critical.x;
		obs_critical_y = obs_critical.y;


		//cout << "\nCritical obstacle -->" << "(" << obs_critical_x << "," << obs_critical_y << ")";
		if (((0 <= p1_p2_angle) && (p1_p2_angle <= eps)) || ((-eps <= p1_p2_angle) && (p1_p2_angle <= 0)) || ((-PI <= p1_p2_angle) && (p1_p2_angle <= (-PI + eps))) || ((PI - eps <= p1_p2_angle) && (p1_p2_angle <= PI))) // avoid having the arctangent equal to inf
		{
			//cout << "\nCritical angle!";
			sh_dest_1 = position(obs_critical_x, obs_critical_y + shift_margin);
			sh_dest_2 = position(obs_critical_x, obs_critical_y - shift_margin);
		}


		else
		{
			//cout << "\nNormal perpendicular calculation";
			double perpendicular = atan(-1.0 / m);
			if (p2.y > p1.y) {
				perpendicular += PI;
			}

			//cout << "\nPerpendicular value: " << perpendicular;
			sh_dest_1 = position(obs_critical_x + (cos(perpendicular) * shift_margin), obs_critical_y + (sin(perpendicular) * shift_margin));
			//cout << "\nShifted destination #1: " << "(" << sh_dest_1.x << "," << sh_dest_1.y << ")";
			sh_dest_2 = position(obs_critical_x - (cos(perpendicular) * shift_margin), obs_critical_y - (sin(perpendicular) * shift_margin));
			//cout << "\nShifted destination #2: " << "(" << sh_dest_2.x << "," << sh_dest_2.y << ")";
		}


		shifted_destinations.push_back(sh_dest_1);
		shifted_destinations.push_back(sh_dest_2);


		//cout << "\nShifted positions: (" << shifted_destinations[0].x << "," << shifted_destinations[1].x << "," << shifted_destinations[0].y << "," << shifted_destinations[1].y << ")";
		return shifted_destinations; // return both shifted positions and the obstacle
	}

	static float area_between_3_points(position p1, position p2, position p3)
	{
		return abs((p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) / 2.0);
	}

	static bool bool_triangle(position p, position p1, position p2, position p3)
	{
		float a0 = area_between_3_points(p1, p2, p3);
		float a1 = area_between_3_points(p, p1, p2);
		float a2 = area_between_3_points(p, p2, p3);
		float a3 = area_between_3_points(p, p1, p3);
		return (a0 == a1 + a2 + a3);
	}

	static bool bool_rectangle(position p, position p1, position p2, position p3, position p4)
	{
		int x_max = max(p1.x, p2.x, p3.x, p4.x);
		int x_min = min(p1.x, p2.x, p3.x, p4.x);
		int y_max = max(p1.y, p2.y, p3.y, p4.y);
		int y_min = min(p1.y, p2.y, p3.y, p4.y);
		if (p.x <= x_max && p.x >= x_min) {
			if (p.y <= y_max && p.y >= y_min) {
				return 1;
			}
		}
		else {
			return 0;
		}
	}

	static bool bool_edge(position p1)
	{
		//cout << "\nPoint of interest bool_edge: " << "(" << p1.x << "," << p1.y << ")";
		double margin = 20;
		double y_margin = margin, x_margin = margin;
		//cout << "\nMargins bool_edge: " << width - x_margin << "," << height - y_margin;
		if ((p1.x >= (width - x_margin)) || (p1.x <= x_margin) || (p1.y >= (height - y_margin)) || (p1.y <= y_margin)) {
			return 1;
		}
		else {
			return 0;
		}
	}
};


// Obstacle detection

void obstacle_points()
{
	// compute a square of points that the 
	int radius = 30; // 2 is used as a safety factor
	for (auto& center : obstacle_cent)
		for (int i = -1 * radius; i <= radius; i++)
		{
			for (int j = -1 * radius; j <= radius; j++)
			{
				obstacles_obstruction_points.push_back(position(center.x + i, center.y + j));
			}
		}
}


void obstacle_centroids(image& rgb) {
	double ic, jc;
	int nlabels;
	//cout << "check 1";
	copy(rgb, a);
	scale(a, b);
	lowpass_filter(b, a);
	threshold(a, b, 170);
	invert(b, a); // obstacles --> white
	erode(a, b);
	dialate(b, a);
	copy(a, rgb_aux);
	label_image(b, labeled, nlabels);
	//cout << "check2";
	ibyte* pointer, * pointer0;
	pointer0 = a.pdata;
	int n_pixels = 0;
	i4byte i, j, k;

	for (int n = 1; n <= nlabels; n++)
	{
		n_pixels = 0;
		centroid(a, labeled, n, ic, jc); // calculates all the centroids
		for (j = jc - 27; j <= jc + 27; j++)
		{
			for (i = ic - 30; i <= ic + 30; i++)
			{
				k = i + j * 640;
				pointer = pointer0 + k; //1 byte per pixel --> grey_scale image
				if (*pointer != 0) n_pixels++; //(n is the number of white pixels)
			}
		}
		if (2900 <= n_pixels) {  // obstacle sufficiently high, remove noise, 2900 found by trial and error
			obstacle_cent.push_back(position(ic, jc));
		}
	}
}



void robot_centroids(image& rgb, double& green_x, double& green_y, double& red_x, double& red_y, double& orange_x, double& orange_y, double& blue_x, double& blue_y) {
	double ic, jc;
	int nlabels;
	double R_ave, G_ave, B_ave;
	highlight_oranges(rgb);
	highlight_greens(rgb);
	highlight_reds(rgb);
	highlight_blues(rgb);
	copy(rgb, a);
	scale(a, b);
	lowpass_filter(b, a);
	threshold(a, b, 170);
	invert(b, a); // obstacles --> white
	erode(a, b);
	dialate(b, a);
	label_image(b, labeled, nlabels);

	ibyte* pointer, * pointer0;
	pointer0 = a.pdata;
	int n_pixels = 0;
	i4byte i, j, k;

	for (int n = 1; n <= nlabels; n++)
	{
		n_pixels = 0;
		centroid(a, labeled, n, ic, jc); // calculates all the centroids
		average_colour(rgb, labeled, n, R_ave, G_ave, B_ave);
		//cout << "\nR_ave --> " << R_ave;
		//cout << "\nG_ave --> " << G_ave;
		//cout << "\nB_ave --> " << B_ave;
		//cout << "\n";
		for (j = jc - 27; j <= jc + 27; j++)
		{
			for (i = ic - 30; i <= ic + 30; i++)
			{
				k = i + j * 640;
				pointer = pointer0 + k; //1 byte per pixel --> grey_scale image
				if (*pointer != 0) n_pixels++; //(n is the number of white pixels)
			}
		}
		if ((2900 > n_pixels) && (R_ave < 45) && (B_ave > 40) && (B_ave < 50)) {
			green_x = ic;
			green_y = jc;
			robot_cent.push_back(position(ic, jc));
		}

		else if ((2900 > n_pixels) && (R_ave > 90)) {
			red_x = ic;
			red_y = jc;
			robot_cent.push_back(position(ic, jc));
		}

		else if ((2900 > n_pixels) && (R_ave > 45) && (B_ave > 40)) {
			orange_x = ic;
			orange_y = jc;
			robot_cent.push_back(position(ic, jc));
		}

		else if ((2900 > n_pixels) && (B_ave > 80)) {
			blue_x = ic;
			blue_y = jc;
			robot_cent.push_back(position(ic, jc));
		}

	}
}

int label_objects(int tvalue)
{
	int nlabels;

	// convert RGB image to a greyscale image
	copy(rgb0, a);

	// scale the image to enhance contrast
	scale(a, a);

	// use threshold function to make a binary image (0,255)
	threshold(a, a, tvalue);

	// invert the image
	invert(a, a);

	// perform an erosion function to remove noise (small objects)
	erode(a, b);

	// perform a dialation function to fill in 
	// and grow the objects
	dialate(b, a);

	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(a, label, nlabels);

	return 0; // no errors
}

void trackcentroids(image& a, image& label, i2byte& nlabel, double& color_x, double& color_y) {
	label_objects(tvalue);
	i2byte* pl;
	double r, rmax, dr, s, smax, ds, theta;
	int i, j;
	int k;

	// pointer to a label image
	pl = (i2byte*)label.pdata;
	//cout << "\ntracking now!";
	// check if an object exists at the current location
	nlabel = *(pl + (int)color_y * label.width + (int)color_x);
	/*if (nlabel != 0) {
		cout << "returned";
		return;
	}*/

	// search for green object
	rmax = 60.0; // maximum radius of search (pixels)
	dr = 3.0; // radius divisions (pixels)
	ds = 3.0; // arc-length divisions (pixels)
	//cout << "\nsearching for the object";
	for (r = 1.0; r <= rmax; r += dr) {

		smax = 2 * 3.1416 * r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta
			i = (int)(color_x + r * cos(theta));
			j = (int)(color_y + r * sin(theta));

			// limit (i,j) from going off the image
			if (i < 0) i = 0;
			if (i > label.width - 1) i = label.width - 1;
			if (j < 0) j = 0;
			if (j > label.height - 1) j = label.height - 1;

			// check if there is an object at location (i,j)
			nlabel = *(pl + j * label.width + i);
			if (nlabel != 0) {
				centroid(a, label, nlabel, color_x, color_y);
				return;
			}

		}
		//if (k % 10 == 0) {
			//cout << "\n i = " << i;
			//cout << "\n j = " << j;
		//}k += 1;
	}
	//centroid(a, label, nlabel, color_x, color_y);
	//cout << "\ncentroid: ic = " << color_x << " " << color_y;

	// draw a point at centroid location (ic,jc) with intensity 128
	copy(rgb0, b);
	//		draw_point(b,(int)ic,(int)jc,255);
	//copy(b, rgb);    // convert to RGB image format

	//draw_point_rgb(rgb, (int)color_x, (int)color_y, 0, 0, 255);
	//draw_point_rgb(rgb, color_x, color_y, R, G, B);


}


// Path planning


void calculate_next_position(position cur, position destination)
{
	position destination_modified_1;
	position destination_modified_2;
	double step = 10;
	double intermediates_shift_margin_initial = 90.0;
	double shift_position_margin = intermediates_shift_margin_initial;
	int decision = 3; // initialize the variable as an out of range number
	int cur_recursive_calls = 0;
	static int counter_calculations = 0;
	if (cur_recursive_calls == 0) {
		position destination_0 = destination;
	}

	bool obs_on_way_shifted_1 = position::obs_bool(cur, destination).first;
	bool obs_on_way_shifted_2 = position::obs_bool(cur, destination).first;
	display_checks = 0;
	bool sh_dest_1_on_edge = position::bool_edge(destination);
	bool sh_dest_2_on_edge = position::bool_edge(destination);
	bool sh_dest_1_in_triangle = 0;
	bool sh_dest_2_in_triangle = 0;

	vector<position> sh_destinations;

	if (((obs_on_way_shifted_1 == 1) || (sh_dest_1_on_edge == 1)) && ((obs_on_way_shifted_2 == 1) || (sh_dest_2_on_edge == 1))) {
		counter_calculations += 1;
		while (((obs_on_way_shifted_1 == 1) || (sh_dest_1_on_edge == 1)) && ((obs_on_way_shifted_2 == 1) || (sh_dest_2_on_edge == 1)) && (cur_recursive_calls < 100))
		{

			shifted_destinations.clear();
			if (cur_recursive_calls == 0) {
				//cout << "\nRecursive calls: ";
			}
			cur_recursive_calls += 1;
			//cout << "\n";
			//cout << cur_recursive_calls << "...";
			shift_position_margin += 5.0;
			sh_destinations = position::shifted_destinations_computation(cur, destination, shift_position_margin);
			obs_on_way_shifted_1 = position::obs_bool(cur, sh_destinations[0]).first;
			obs_on_way_shifted_2 = position::obs_bool(cur, sh_destinations[1]).first;
			sh_dest_1_on_edge = position::bool_edge(sh_destinations[0]);
			sh_dest_2_on_edge = position::bool_edge(sh_destinations[1]);
			//cout << "\nShifted dest. 1 --> " << "(" << sh_destinations[0].x << "," << sh_destinations[0].y << ")" << " --> " << obs_on_way_shifted_1 << "," << sh_dest_1_on_edge;
			//cout << "\nShifted dest. 2 --> " << "(" << sh_destinations[1].x << "," << sh_destinations[1].y << ")" << " --> " << obs_on_way_shifted_2 << "," << sh_dest_2_on_edge;
		}

		cur_recursive_calls = 0;
		shift_position_margin = intermediates_shift_margin_initial;

		if (obstacle_cent.size() == 2) { // operations done in case of 2 obstacles in the pixel space
			if ((obs_on_way_shifted_1 == 1) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 0)) {
				decision = 1; // go to shifted destination 2
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 1) && (sh_dest_1_on_edge == 0)) {
				decision = 0; // go to shifted destination 1
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_1_on_edge == 0) && (sh_dest_2_on_edge == 1)) {
				decision = 0;
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_1_on_edge == 1) && (sh_dest_2_on_edge == 0)) {
				decision = 1;
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_1_on_edge == 0) && (sh_dest_2_on_edge == 0)) {
				if (position::distance_between_points(cur, sh_destinations[0]) <= position::distance_between_points(cur, sh_destinations[1])) {
					decision = 0;
				}
				else if (position::distance_between_points(cur, sh_destinations[0]) > position::distance_between_points(cur, sh_destinations[1])) {
					decision = 1;
				}
				else {
					srand(time(NULL));
					decision = rand() % 2; // generate random number, either 0 or 1, so randomly go
				}
			}
			else {
				decision = 2; // stay still
			}
		}

		if (obstacle_cent.size() == 3) {

			sh_dest_1_in_triangle = position::bool_triangle(sh_destinations[0], obstacle_cent[0], obstacle_cent[1], obstacle_cent[2]);
			sh_dest_2_in_triangle = position::bool_triangle(sh_destinations[1], obstacle_cent[0], obstacle_cent[1], obstacle_cent[2]);

			if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 1) && (sh_dest_1_on_edge == 0) && (sh_dest_1_in_triangle == 0)) {
				decision = 0; // go to shifted destination 1
			}
			else if ((obs_on_way_shifted_1 == 1) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 0) && (sh_dest_2_in_triangle == 0)) {
				decision = 1; // go to shifted destination 2
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 1) && (sh_dest_2_in_triangle == 0) && (sh_dest_1_on_edge == 0) && (sh_dest_1_in_triangle == 0)) {
				decision = 0; // go to shifted destination 1
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 0) && (sh_dest_2_in_triangle == 1) && (sh_dest_1_on_edge == 0) && (sh_dest_1_in_triangle == 0)) {
				decision = 0; // go to shifted destination 1
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 1) && (sh_dest_2_in_triangle == 1) && (sh_dest_1_on_edge == 0) && (sh_dest_1_in_triangle == 0)) {
				decision = 0; // go to shifted destination 1
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 0) && (sh_dest_2_in_triangle == 0) && (sh_dest_1_on_edge == 1) && (sh_dest_1_in_triangle == 0)) {
				decision = 1; // go to shifted destination 2
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 0) && (sh_dest_2_in_triangle == 0) && (sh_dest_1_on_edge == 0) && (sh_dest_1_in_triangle == 1)) {
				decision = 1; // go to shifted destination 2
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 0) && (sh_dest_2_in_triangle == 0) && (sh_dest_1_on_edge == 1) && (sh_dest_1_in_triangle == 1)) {
				decision = 1; // go to shifted destination 2
			}
			else if ((obs_on_way_shifted_1 == 0) && (obs_on_way_shifted_2 == 0) && (sh_dest_2_on_edge == 0) && (sh_dest_2_in_triangle == 0) && (sh_dest_1_on_edge == 0) && (sh_dest_1_in_triangle == 0)) {
				if (position::distance_between_points(cur, sh_destinations[0]) <= position::distance_between_points(cur, sh_destinations[1])) {
					decision = 0;
				}
				else if (position::distance_between_points(cur, sh_destinations[0]) > position::distance_between_points(cur, sh_destinations[1])) {
					decision = 1;
				}
				else {
					srand(time(NULL));
					decision = rand() % 2; // generate random number, either 0 or 1, so randomly go
				}
			}
			else {
				decision = 2; // stay still
			}
		}
	}

	if ((decision == 0) || (decision == 1)) {
		position decided_position = sh_destinations.at(decision); // pick the shifted destination
		next_destination_queue.insert(next_destination_queue.begin(), decided_position);
		//cout << "\nDecided position --> " << "(" << decided_position.x << "," << decided_position.y << ")";
		counter_calculations = 0;
	}
	if (decision == 2) {
		shifted_destinations.clear(); // do nothing
	}

	if (decision == 3) {
		next_destination_queue.insert(next_destination_queue.begin(), destination); // add the opponent-destination to the destination_queue
		counter_calculations = 0;
	}

	if (counter_calculations >= 2) {
		while ((position::bool_edge(destination_modified_1) == 1) && (position::bool_edge(destination_modified_1) == 1)) {
			destination_modified_1.x = destination.x;
			destination_modified_1.y = destination.y + step;
			destination_modified_2.x = destination.x;
			destination_modified_2.y = destination.y - step;
			cout << "\nStuck calculating desired positions...";
		}
		if (position::bool_edge(destination_modified_1) == 0) {
			destination = destination_modified_1;
		}
		else if (position::bool_edge(destination_modified_2) == 0) {
			destination = destination_modified_2;
		}
		calculate_next_position(cur, destination);
	}

}


// Control

int* robot_control(position cur, double angle_cur)
{

	// pw[0] ==> pwl (left)
	// pw[1] ==> pwr (right)

	static int pw[2];
	int pw_0 = 1500;
	int pw_max = 2000;
	int pw_min = 1000;
	int pw_delta = 500;

	if (next_destination_queue.size() == 0)
	{
		pw[0] = pw_0; // left wheel not moving
		pw[1] = pw_0; // right wheel not moving
		return pw;
	}

	position destin = next_destination_queue.front();
	//cout << "\nThe on-going destination point is: " << destin.x << " " << destin.y << endl;

	double xdes = destin.x;
	double ydes = destin.y;
	double xcur = cur.x;
	double ycur = cur.y;

	double angle_threshold = PI / 2;
	int position_error = 25; // do not get too close to destination, otherwise overlapping


	// POSITION:

	// when the robot is close to desired position --> pw = 1500
	if (sqrt(pow((ycur - ydes), 2) + pow((xcur - xdes), 2)) <= position_error) // position_error is high so that the two robots do not collide with each other
	{
		pw[0] = pw_0;
		pw[1] = pw_0;
		if (next_destination_queue.size() > 0)
			next_destination_queue.erase(next_destination_queue.begin()); // eliminate the first element of the vector
		return pw;
	}

	// ORIENTATION:

	double angle_error = 0.04; // set a minimum orientation error threshold for the robot
	// angle between the two centroids
	double angle_des = position::angle_between_points(cur, destin);
	double angle_diff = angle_des - angle_cur;

	// constraint the angle to be in the range -pi to pi
	angle_diff = constrain_angle_negpi_pi(angle_diff);

	if (abs(angle_diff) < angle_error) // the robot is aligned --> do not turn anymore
	{
		// forward translation:
		pw[1] = pw_max;
		pw[0] = pw_min;
		return pw;
	}

	// reverse is not implemented

	// The desired position is in front of the Robot
	if (abs(angle_diff) <= angle_threshold)
	{
		// counter-clockwise rotation and moving forward+
		if (0 < angle_diff && angle_diff < PI / 10)
		{
			pw[1] = pw_max;
			pw[0] = pw_0 - (0.2) * pw_delta;
		}

		// clockwise rotation and moving forward+
		else if (-PI / 10 < angle_diff && angle_diff < 0)
		{
			pw[1] = pw_0 + (0.2) * pw_delta;
			pw[0] = pw_min;
		}

		// counter-clockwise rotation and moving forward++
		else if (PI / 10 <= angle_diff && angle_diff < PI / 5)
		{
			pw[1] = pw_max;
			pw[0] = pw_0 - (0.25) * pw_delta;
		}

		// clockwise rotation and moving forward++
		else if (-PI / 5 < angle_diff && angle_diff <= -PI / 10)
		{
			pw[1] = pw_0 + (0.25) * pw_delta;
			pw[0] = pw_min;
		}
		
		// counter-clockwise rotation and moving forward+++
		else if (PI / 5 <= angle_diff && angle_diff < 3 * PI / 10)
		{
			pw[1] = pw_max;
			pw[0] = pw_0 + (0.6) * pw_delta;
		}

		// clockwise rotation and moving forward+++
		else if (-3 * PI / 10 < angle_diff && angle_diff <= -PI / 5)
		{
			pw[1] = pw_0 - (0.6) * pw_delta;
			pw[0] = pw_min;
		}

		// counter-clockwise rotation and moving forward++++
		else if (3 * PI / 10 <= angle_diff && angle_diff < 2 * PI / 5)
		{
			pw[1] = pw_max;
			pw[0] = pw_0 + (0.8) * pw_delta;
		}

		// clockwise rotation and moving forward++++
		else if (-2 * PI / 5 < angle_diff && angle_diff <= -3 * PI / 10)
		{
			pw[1] = pw_0 - (0.8) * pw_delta;
			pw[0] = pw_min;
		}


		// counter-clockwise rotation 
		else if (2 * PI / 5 <= angle_diff && angle_diff < PI / 2)
		{
			pw[1] = pw_max;
			pw[0] = pw_max;
		}

		// clockwise rotation
		else if (-PI / 2 < angle_diff && angle_diff <= -2 * PI / 5)
		{
			pw[1] = pw_min;
			pw[0] = pw_min;
		}

		//cout << "Left wheel input: " << pw[1] << ", Right wheel input: " << pw[0] << endl;

		return pw;
	}



	else  // just pure turning
	{
		// counter-clockwise rotation 
		if (-PI <= angle_diff && angle_diff < -PI / 2)
		{
			pw[1] = pw_min;
			pw[0] = pw_min;
		}

		// clockwise rotation
		else if (PI / 2 < angle_diff && angle_diff <= PI)
		{
			pw[1] = pw_max;
			pw[0] = pw_max;
		}

		//cout << "Left wheel input: " << pw[1] << ", Right wheel input: " << pw[0] << endl;

		return pw;
	}

}


// Angle manipulation

double constrain_angle_negpi_pi(double angle) {
	angle = fmod(angle + PI, 2 * PI);
	if (angle < 0)
		angle += 2 * PI;
	return angle - PI;
}

double constrain_angle_0_2pi(double angle) {
	angle = fmod(angle, 2 * PI);
	if (angle < 0)
		angle += 2 * PI;
	return angle;
}

int main()
{
	static int iterations_number = 0;
	int initial = 0;
	int counter_destinations = 0;
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int n_robot = 2;
	double x_obs[50] = { 0.0 }, y_obs[50] = { 0.0 };
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	int* result;
	int capture = 0;
	int v_mode;
	position opponent_pos;
	i2byte nlabel = 0;
	
	display_checks = 0;
	// image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	//const int N_obs = 2;
	const int N_obs = 3;


	x_obs[0] = 180; // pixels
	y_obs[0] = 150; // pixels

	x_obs[1] = 300; // pixels
	y_obs[1] = 300; // pixels

	x_obs[2] = 370; // pixels
	y_obs[2] = 180; // pixels


	/*
	char obstacle_file[N_obs][S_MAX] = {
	"obstacle_black.bmp" , "obstacle_blue.bmp"
	};
	*/

	char obstacle_file[N_obs][S_MAX] = {
	"obstacle_black.bmp" , "obstacle_blue.bmp", "obstacle_red.bmp"
	};


	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	cout << "\npress space key to begin program.";
	pause();

	activate_vision();

	activate_simulation(width1, height1, x_obs, y_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp", obstacle_file, D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// set simulation mode
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 2;
	set_simulation_mode(mode);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 500;
	y0 = 300;
	theta0 = PI/4;
	set_robot_position(x0,y0,theta0);
	

	// inputs to the robot's wheels --> initial ones (the robot should be not moving initially)

	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	
	// speed paramaters

	max_speed = 100; // max wheel speed of robot (pixels/s)

	// set initial inputs

	set_inputs(pw_l,pw_r,pw_laser,laser,max_speed);
	
	int height, width;

	image rgb;

	width  = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	double x_des;
	double y_des;

	// allocate memory for the images

	initialize_vision(rgb, rgb0, rgb_aux, a, b, c, label, labeled);

	join_player();

	// measure initial clock time

	tc0 = high_resolution_time();

	int R = 0;
	int G = 0;
	int B = 0;

	int R_obs = 128;
	int B_obs = 128;

	Sleep(100);

	while(1) {

		// simulates the robots and acquires the image from simulation

		acquire_image_sim(rgb);


		if (iterations_number >= 100) {

			if (initial == 0) // do it only once, it's an initialization process
			{

				copy(rgb, rgb0);
				highlight_oranges(rgb0);
				obstacle_centroids(rgb0);
				obstacle_points();
				initial = 1;
				cout << "\nThe number of obstacles obtained with vision is:" << obstacle_cent.size();
				cout << "\nThe number of pixels belonging to obstacles:" << obstacles_obstruction_points.size();
				copy(rgb, rgb0);
				robot_centroids(rgb0, green_x, green_y, red_x, red_y, orange_x, orange_y, blue_x, blue_y);
				draw_point_rgb(rgb, green_x, green_y, R, G, B);
				draw_point_rgb(rgb, red_x, red_y, R, G, B);
				draw_point_rgb(rgb, orange_x, orange_y, R, G, B);
				draw_point_rgb(rgb, blue_x, blue_y, R, G, B);
			}

			copy(rgb, rgb0);
			copy(rgb, a);
			//for (auto& element : AllColors) {
			trackcentroids(a, label, nlabel, green_x, green_y);
			//cout << "\ngreenx = " << green_x << " " << "greeny = " << green_y;
			draw_point_rgb(rgb, green_x, green_y, R, G, B);
			trackcentroids(a, label, nlabel, red_x, red_y);
			//cout << "\nredx = " << red_x << " " << "redy = " << red_y;
			draw_point_rgb(rgb, red_x, red_y, R, G, B);
			trackcentroids(a, label, nlabel, orange_x, orange_y);
			//cout << "\norangex = " << orange_x << " " << "orangey = " << orange_y;
			draw_point_rgb(rgb, orange_x, orange_y, R, G, B);
			trackcentroids(a, label, nlabel, blue_x, blue_y);
			//cout << "\nbluex = " << blue_x << " " << "bluey = " << blue_y;
			draw_point_rgb(rgb, blue_x, blue_y, R, G, B);
			//copy(rgb, rgb0);
			//robot_centroids(rgb0, green_x, green_y, red_x, red_y, orange_x, orange_y, blue_x, blue_y);
			for (int i = 0; i < (obstacle_cent.size()); i++) {
				draw_point_rgb(rgb, obstacle_cent[i].x, obstacle_cent[i].y, 120, G, 120);
			}
			

			// cout << "\nThe number of robot centroids obtained with vision is:" << robot_cent.size();

			// compute current position of the robot and current orientation

			tc = high_resolution_time() - tc0;
			double angcur = (atan2(green_y - red_y, green_x - red_x));

			double x_cur = green_x;
			double y_cur = green_y;


			opponent_pos.x = (blue_x + orange_x) / 2;
			opponent_pos.y = (blue_y + orange_y) / 2;

			opponent_pos = position(opponent_pos.x, opponent_pos.y);


			//cout << "\nDestination queue size: " << destination_queue.size();
			position cur(x_cur, y_cur);

			//draw_point_rgb(rgb, cur.x, cur.y, 0, 0, 0);

			position dest = position::best_hiding_position(cur, opponent_pos);

			draw_point_rgb(rgb, dest.x, dest.y,120, 0, 120);

			//cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
			//cout << "\nCurrent destination: " << dest.x << " , " << dest.y;
			/*
			if (available == 1) {
				display_checks = 1;
				//next position computation
				cout << "\n---------------------------------------------------------------------------------------------------------------------------------------------";
				cout << "\nCounter destinations: " << counter_destinations + 1;
				calculate_next_positions(cur, dest);
				cout << "\n |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
				cout << "\nCurrent robot position: " << xcur << " , " << ycur;
				available = 0;
				counter_destinations += 1;
			}
			*/

			calculate_next_position(cur, dest);
			display_checks = 0;

			if (next_destination_queue.size() != 0) {
				result = robot_control(cur, angcur);
				pw_l = result[0];
				pw_r = result[1];
			}

			if (next_destination_queue.size() == 0) {
				pw_l = 1500;
				pw_r = 1500;
			}

			set_inputs(pw_l, pw_r, pw_laser, laser, max_speed);

			// NOTE: only one program can call view_image()
			/*
			v_mode = 1;
			view_rgb_image(rgb, v_mode);
			*/


			// clear the destination queue

			next_destination_queue.clear();
		}

		Sleep(10); // 100 fps max

		robot_cent.clear();

		iterations_number += 1;

		//save_rgb_image("output_defender.bmp", rgb);

	}

	// free the image memory before the program completes

	deactivate_vision(rgb, rgb0, rgb_aux, a, b, c, label, labeled);
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}
