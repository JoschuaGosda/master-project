#include <iostream>
#include "gpm.hpp"

#include <Eigen/Eigen>


int main(int, char**) {
	
	Eigen::Matrix<double, 7, 1> joint_angles;
	Eigen::Matrix<double, 7, 1> result;
	Eigen::Matrix<double, 6, 1> desPosition;
	joint_angles << 0, 0, 0, 0, 0, 0, 0;
	desPosition << 0, 0, 0, 0, 0, 0;

	result = gpm(joint_angles, desPosition);
	std::cout << "result: \n" << result << std::endl;
	
	return 0;
}





/* Tasks to be performed within C++
    1. read operational pose from function
        - coordinates should be stored according to how broccoli functions need it to be
    2. implement computation of Jacobian
        - search for existing solutions
    3. use broccoli to compute inverse kinematics
*/


// use broccoli to compute the joint angle via inverse kinematics


// thing about what is input and output of c++ function
