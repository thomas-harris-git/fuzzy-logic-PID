#include <Aria.h>
#include <iostream>

using namespace std;

double baseVel = 200, prev_error = 0, error_i = 0, Speed = 0.5;
string RFS_Fuzzy_Value, RBS_Fuzzy_Value, Sensor_Fuzzy_Value;

double Gradients[10], firing[20], Left_Motor_Speed, Right_Motor_Speed, Final_Left_Speed, Final_Right_Speed;
string Fuzzy_Values[10], fuzzy_prox[20];

int sonar[8], sonarDistance[8];

void right_edge_following_PID(double current_distance) //PID controller 
{
	double kp = 0.96, ki = 0.45, kd = 0, w;
	int desired_distance = 400, d = 260;

	double error = desired_distance - current_distance;

	error_i = error_i + error;

	//limit for the error sum 
	if (error_i < -600)
		error_i = -600;
	if (error_i > 600)
		error_i = 600;

	double error_d = error - prev_error;
	prev_error = error;

	double output = kp*error + ki*error_i + kd*error_d;

	w = d / (2 * output);
	Final_Left_Speed = (baseVel - ((w*d) / 2))*Speed;
	Final_Right_Speed = (baseVel + ((w*d) / 2))*Speed;
	
}

struct membership_function //creates membership function structure 
{
	int a, b, c, d;
	string prox;

	double Gradient(double x);
};

//membership functions for sensors 
membership_function RFS_Close = { 0,0,350,525,"Close" };		//Right Front Sensor
membership_function RFS_Median = { 350,525,575,750,"Median" };
membership_function RFS_Far = { 575,750,5000,5000,"Far" };

membership_function RBS_Close = { 0,0,200,375,"Close" };		//Right Back Sensor
membership_function RBS_Median = { 200,375,425,600,"Median" };
membership_function RBS_Far = { 425,600,5000,5000,"Far" };

membership_function FMS_Close = { 0,0,250,500,"Close" };		//Front Middle Sensor
membership_function FMS_Median = { 250,500,600,850,"Median" };
membership_function FMS_Far = { 600,850,5000,5000,"Far" };

membership_function FSS_Close = { 0,0,200,450,"Close" };		//Front Side Sensors 
membership_function FSS_Median = { 200,450,550,800,"Median" };
membership_function FSS_Far = { 550,800,5000,5000,"Far" };

double membership_function::Gradient(double x) //get the gradient of the line 'x' is on 
{
	double out = 0;
	Sensor_Fuzzy_Value = "";
	if (x >= b && x <= c)
		out = 1;
	if (x >= a && x < b)
		out = (x - a) / (b - a);
	if (x > c && x <= d)
		out = (d - x) / (d - c);
	if (out > 0)
		Sensor_Fuzzy_Value = prox;
	return out;
}

double minimum(double x, double y)//returns the minimum of two values 
{
	if (x < y)
		return(x);
	else
		return(y);
}

void right_edge_rule_set(string Front_Back_Sensor) //Fuzzy rule set for right edge following 
{
	//sets the motor speed values for each rule
	int Left_Motor_Speed_Slow = 40, Left_Motor_Speed_Median = 90, Left_Motor_Speed_Fast = 140,
		Right_Motor_Speed_Slow = 85, Right_Motor_Speed_Median = 135, Right_Motor_Speed_Fast = 185;

	if (Front_Back_Sensor == "CloseClose")
		Left_Motor_Speed = Left_Motor_Speed_Slow, Right_Motor_Speed = Right_Motor_Speed_Fast;

	if (Front_Back_Sensor == "CloseMedian")
		Left_Motor_Speed = Left_Motor_Speed_Slow, Right_Motor_Speed = Right_Motor_Speed_Fast;

	if (Front_Back_Sensor == "CloseFar")
		Left_Motor_Speed = Left_Motor_Speed_Slow, Right_Motor_Speed = Right_Motor_Speed_Fast;

	if (Front_Back_Sensor == "MedianClose")
		Left_Motor_Speed = Left_Motor_Speed_Median, Right_Motor_Speed = Right_Motor_Speed_Slow;

	if (Front_Back_Sensor == "MedianMedian")
		Left_Motor_Speed = Left_Motor_Speed_Median, Right_Motor_Speed = Right_Motor_Speed_Median;

	if (Front_Back_Sensor == "MedianFar")
		Left_Motor_Speed = Left_Motor_Speed_Slow, Right_Motor_Speed = Right_Motor_Speed_Median;

	if (Front_Back_Sensor == "FarClose")
		Left_Motor_Speed = Left_Motor_Speed_Fast, Right_Motor_Speed = Right_Motor_Speed_Slow;

	if (Front_Back_Sensor == "FarMedian")
		Left_Motor_Speed = Left_Motor_Speed_Median, Right_Motor_Speed = Right_Motor_Speed_Slow;

	if (Front_Back_Sensor == "FarFar")
		Left_Motor_Speed = Left_Motor_Speed_Fast, Right_Motor_Speed = Right_Motor_Speed_Slow;
}

void set_steering_speed(string steering, int speed) //used to set the motor speeds for the obstacle avoidance
{
	if (steering == "Left") //turn left 
	{
		Right_Motor_Speed = 100*speed;
		Left_Motor_Speed = 50*speed;
	}
	if (steering == "Right") //turn right 
	{
		Right_Motor_Speed = 50*speed;
		Left_Motor_Speed = 100*speed;
	}
	if (steering == "") //continue forward
	{
		Right_Motor_Speed = 100 * speed;
		Left_Motor_Speed = 100 * speed;
	}
}

void obstacle_avoidance_rule_set(string Sensor) //obstacle avoidance rule set, sets steering and speed for each
{
	int slow = 0.5, median = 1, fast = 1.5;
	string steering = "";
	if (Sensor == "CloseCloseClose")
		set_steering_speed("Left",fast);
	if (Sensor == "CloseCloseMedian")
		set_steering_speed("Right",median);
	if (Sensor == "CloseCloseFar")
		set_steering_speed("Right",median);
	if (Sensor == "CloseMedianClose")
		set_steering_speed("Left",fast);
	if (Sensor == "CloseMedianMedian")
		set_steering_speed("Right",slow);
	if (Sensor == "CloseMedianFar")
		set_steering_speed("Right",median);
	if (Sensor == "CloseFarClose")
		set_steering_speed("",median);
	if (Sensor == "CloseFarMedian")
		set_steering_speed("Right",median);
	if (Sensor == "CloseFarFar")
		set_steering_speed("Right",slow);
	if (Sensor == "MedianCloseClose")
		set_steering_speed("Left",fast);
	if (Sensor == "MedianCloseMedian")
		set_steering_speed("Left",median);
	if (Sensor == "MedianCloseFar")
		set_steering_speed("Right",median);
	if (Sensor == "MedianMedianClose")
		set_steering_speed("left", median);
	if (Sensor == "MedianMedianMedian")
		set_steering_speed("Left", slow);
	if (Sensor == "MedianMedianFar")
		set_steering_speed("Right",slow);
	if (Sensor == "MedianFarClose")
		set_steering_speed("Left",median);
	if (Sensor == "MedianFarMedian")
		set_steering_speed("Left",slow);
	if (Sensor == "MedianFarFar")
		set_steering_speed("Right",median);
	if (Sensor == "FarCloseClose")
		set_steering_speed("Left",median);
	if (Sensor == "FarCloseMedian")
		set_steering_speed("Left",median);
	if (Sensor == "FarCloseFar")
		set_steering_speed("Left",median);
	if (Sensor == "FarMedianClose")
		set_steering_speed("Left",median);
	if (Sensor == "FarMedianMedian")
		set_steering_speed("Left",median);
	if (Sensor == "FarMedianFar")
		set_steering_speed("Left",median);
	if (Sensor == "FarFarClose")
		set_steering_speed("Left",median);
	if (Sensor == "FarFarMedian")
		set_steering_speed("Left",median);
	if (Sensor == "FarFarFar")
		set_steering_speed("",median);
}

void right_edge_firing_strength(double RFS, double RBS) //gets the firing strength, fuzzy values from the sensors and sets the motor speed
{
	int firing_num = 0;
	double Divide = 0;

	//gets the gradient of the sensor readings and if they fire any fuzzy values
	//right front sensor
	Gradients[0] = RFS_Close.Gradient(RFS);
	Fuzzy_Values[0] = Sensor_Fuzzy_Value;
	Gradients[1] = RFS_Median.Gradient(RFS);
	Fuzzy_Values[1] = Sensor_Fuzzy_Value;
	Gradients[2] = RFS_Far.Gradient(RFS);
	Fuzzy_Values[2] = Sensor_Fuzzy_Value;
	//right back sensor
	Gradients[3] = RBS_Close.Gradient(RBS);
	Fuzzy_Values[3] = Sensor_Fuzzy_Value;
	Gradients[4] = RBS_Median.Gradient(RBS);
	Fuzzy_Values[4] = Sensor_Fuzzy_Value;
	Gradients[5] = RBS_Far.Gradient(RBS);
	Fuzzy_Values[5] = Sensor_Fuzzy_Value;

	for (int i = 0; i < 3; i++) {
		for (int x = 3; x < 6; x++) {
			if (Gradients[i] == 0 || Gradients[x] == 0) {	//if rule is not fired then do nothing
			}
			else {
				firing[firing_num] = minimum(Gradients[i], Gradients[x]); //gets firing strength 
				fuzzy_prox[firing_num] = Fuzzy_Values[i] + Fuzzy_Values[x]; //gets fuzzy value of the two sensors
				firing_num++;
			}
		}
	}

	for (int i = 0; i < firing_num; i++) {
		right_edge_rule_set(fuzzy_prox[i]); //inputs the fuzzy values into the rule set
		Final_Left_Speed += Left_Motor_Speed*firing[i];
		Final_Right_Speed += Right_Motor_Speed*firing[i];
		Divide += firing[i];
	}
	//set motor speeds
	Final_Left_Speed = Final_Left_Speed / Divide;
	Final_Right_Speed = Final_Right_Speed / Divide;

	for(int i=0; i<10; i++) //clears vectors 
	{
		Gradients[i] = 0;
		Fuzzy_Values[i] = "";
	}
}

void obstacle_avoidance_firing_strength(double FLS, double FMS, double FRS ) //gets the firing strength, fuzzy values from the sensors and sets the motor speed
{
	int firing_num = 0;
	double Divide = 0;

	//gets the gradient of the sensor readings and if they fire any fuzzy values
	//front left sensor
	Gradients[0] = FSS_Close.Gradient(FLS);
	Fuzzy_Values[0] = Sensor_Fuzzy_Value;
	Gradients[1] = FSS_Median.Gradient(FLS);
	Fuzzy_Values[1] = Sensor_Fuzzy_Value;
	Gradients[2] = FSS_Far.Gradient(FLS);
	Fuzzy_Values[2] = Sensor_Fuzzy_Value;
	//front middle sensor
	Gradients[3] = FMS_Close.Gradient(FMS);
	Fuzzy_Values[3] = Sensor_Fuzzy_Value;
	Gradients[4] = FMS_Median.Gradient(FMS);
	Fuzzy_Values[4] = Sensor_Fuzzy_Value;
	Gradients[5] = FMS_Far.Gradient(FMS);
	Fuzzy_Values[5] = Sensor_Fuzzy_Value;
	//front right sensor
	Gradients[6] = FSS_Close.Gradient(FRS);
	Fuzzy_Values[6] = Sensor_Fuzzy_Value;
	Gradients[7] = FSS_Median.Gradient(FRS);
	Fuzzy_Values[7] = Sensor_Fuzzy_Value;
	Gradients[8] = FSS_Far.Gradient(FRS);
	Fuzzy_Values[8] = Sensor_Fuzzy_Value;

	for (int i = 0; i < 3; i++) {
		for (int x = 3; x < 6; x++) {
			for (int n = 6; n < 9; n++) {
				if (Gradients[i] == 0 || Gradients[x] == 0 || Gradients[n] == 0) {	//if rule is not fired then do nothing
				}
				else {
					firing[firing_num] = Gradients[i] * Gradients[x] * Gradients[n]; //gets firing strength
					fuzzy_prox[firing_num] = Fuzzy_Values[i] + Fuzzy_Values[x] + Fuzzy_Values[n]; //gets fuzzy value of the sensors
					firing_num++;
				}
			}
		}
	}

	for (int i = 0; i < firing_num; i++) {
		obstacle_avoidance_rule_set(fuzzy_prox[i]); //inputs the fuzzy values into the rule set
		Final_Left_Speed += Left_Motor_Speed*firing[i];
		Final_Right_Speed += Right_Motor_Speed*firing[i];
		Divide += firing[i];
	}
	//set motor speeds
	Final_Left_Speed = Final_Left_Speed / Divide;
	Final_Right_Speed = Final_Right_Speed / Divide;

	for (int i = 0; i<10; i++) //clear vector 
	{
		Gradients[i] = 0;
		Fuzzy_Values[i] = "";
	}
}

void PID() { //gets the smallest current disance between sensor 6 and 7 for the PID 
	int Current_Distance;

	sonarDistance[6] = sonarDistance[6]*(sin((40*3.14/180))); //equation to get disance from wall 

	Current_Distance = minimum(sonarDistance[6], sonarDistance[7]);

	right_edge_following_PID(Current_Distance);
}

void Fuzzy_Logic() { // choices between obstacle avoidance and right edge following for the fuzzy logic
	int d1, front_middle;
	front_middle = minimum(sonarDistance[3], sonarDistance[4]); 
	d1 = minimum(sonarDistance[2], sonarDistance[5]);
	d1 = minimum(d1, front_middle);

	if(d1 < 600) //if the minimum distance of the front sensors is less than 600 then do obstacle avoidance
		obstacle_avoidance_firing_strength(sonarDistance[2], front_middle, sonarDistance[5]);
	else
		right_edge_firing_strength(sonarDistance[6], sonarDistance[7]);
	
}

int main(int argc, char **argv)
{
	// create instances 
	Aria::init();
	ArRobot robot;
	ArPose pose;
	// parse command line arguments 
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		std::cout << "Robot connected!" << std::endl;

	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	while (true)
	{
		//gets the sonar readings 
		for (int i = 0; i < 8; i++)
		{
			sonar[i] = robot.getSonarReading(i)->getRange();
			if (0 < sonar[i] < 5000)
				sonarDistance[i] = sonar[i];
			if (sonarDistance[i] > 5000) //anything greater than 5000 is equal to 5000 
				sonarDistance[i] = 5000;
		}

		//Fuzzy_Logic();
		PID();
		
		robot.setVel2(Final_Left_Speed, Final_Right_Speed); //sets motor speeds
		Final_Left_Speed = 0, Final_Right_Speed = 0;

	}

	robot.lock();
	robot.stop();
	robot.unlock();
	// terminate all threads and exit 
	Aria::exit();
}