#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> udacityJMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;
	
	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}
	
    return result;
    
}

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    double si    = start[0];
    double si_d  = start[1];
    double si_dd = start[2];
    double sf    = end[0];
    double sf_d  = end[1];
    double sf_dd = end[2];
    
    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T3*T;
    double T5 = T4*T;
    
    MatrixXd A(3,3);
    A << T3  , T4    , T5  ,
         3*T2, 4*T3  , 5*T4  ,
         6*T , 12*T2 , 20*T3 ;
    
    MatrixXd B(3,1);
    B << sf - (si + si_d*T + 0.5*si_dd*T2),
         sf_d - (si_d + si_dd*T),
         sf_dd - si_dd;
         
    MatrixXd a3_5(3,1);
    a3_5 = A.inverse()*B;

    double a0 = si;
    double a1 = si_d;
    double a2 = si_dd/2;
    double a3 = a3_5(0);
    double a4 = a3_5(1);
    double a5 = a3_5(2);

    //return {a0,a1,a2,a3,a4,a5};
    return {a0,a1,a2,a3,a4,a5};
}

bool close_enough(vector< double > poly, vector<double> target_poly, double eps=0.01) {


	if(poly.size() != target_poly.size())
	{
		cout << "your solution didn't have the correct number of terms" << endl;
		return false;
	}
	for(int i = 0; i < poly.size(); i++)
	{
		double diff = poly[i]-target_poly[i];
		if(abs(diff) > eps)
		{
			cout << "at least one of your terms differed from target by more than " << eps << endl;
			return false;
		}

	}
	return true;
}
	
struct test_case {
	
		vector<double> start;
		vector<double> end;
		double T;
};

vector< vector<double> > answers = {{0.0, 10.0, 0.0, 0.0, 0.0, 0.0},{0.0,10.0,0.0,0.0,-0.625,0.3125},{5.0,10.0,1.0,-3.0,0.64,-0.0432}};

int main_ref() {

	//create test cases

	vector< test_case > tc;

	test_case tc1;
	tc1.start = {0,10,0};
	tc1.end = {10,10,0};
	tc1.T = 1;
	tc.push_back(tc1);

	test_case tc2;
	tc2.start = {0,10,0};
	tc2.end = {20,15,20};
	tc2.T = 2;
	tc.push_back(tc2);

	test_case tc3;
	tc3.start = {5,10,2};
	tc3.end = {-30,-20,-4};
	tc3.T = 5;
	tc.push_back(tc3);

	bool total_correct = true;
	for(int i = 0; i < tc.size(); i++)
	{
		vector< double > jmt = JMT(tc[i].start, tc[i].end, tc[i].T);
		bool correct = close_enough(jmt,answers[i]);
		total_correct &= correct;

	}
	if(!total_correct)
	{
		cout << "Try again!" << endl;
	}
	else
	{
		cout << "Nice work!" << endl;
	}

	return 0;
} 
