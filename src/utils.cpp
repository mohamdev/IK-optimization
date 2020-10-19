/*
 * utils.cpp
 *
 *  Created on: 19 Oct 2020
 *      Author: aladinedev2
 *  This file contains all the typedefs used in the functions of the project.
 *  It also will contain all the needed includes, and all the useful tool functions that
 *  can be used for conversions between different data types.
 */

#include "utils.h"

void eigen2vector(ScalarMatrix const& eigenMat, std::vector<std::vector<double>> & returnedVect)
{
	/**This function converts an eigen scalar matrix to a std::vector<vector<double>> **/
    int j = 0;
    int i = 0;
    for (i=0; i<eigenMat.rows(); i++)
    {
        for(j=0; j<eigenMat.cols(); j++)
        {
            returnedVect[i][j] = eigenMat(i,j);
        }
    }
}

void rot2quat(ScalarMatrix const & R, ScalarVector & Q)
{
	/*
	 * This function is used to convert a rotation matrix R to a quaternion Q
	 * The quaternion Q is passed by reference and directly modified in the function.
	 * INPUTS:
	 * 	R = 3*3 Rotation matrix
	 * 	Q = quaternion passed by reference and directly modified in the function
	 *
	 * */
    Scalar Tr(R(0,0) + R(1,1) + R(2,2));
    Scalar S = sqrt(Tr+1)*2;
    Q(0) = 0.25 * S;
    Q(1) = (R(2,1) - R(1,2))/S;
    Q(2) = (R(0,2) - R(2,0))/S;
    Q(3) = (R(1,0) - R(0,1))/S;


    /**
        if (Tr>0)
    {
        Scalar S = sqrt(Tr+1)*2;
        Q(0) = 0.25 * S;
        Q(1) = (R(2,1) - R(1,2))/S;
        Q(2) = (R(0,2) - R(2,0))/S;
        Q(3) = (R(1,0) - R(0,1))/S;
    } else if (R(0,0)>R(1,1) && R(0,0)<R(2,2))
    {
            Scalar S = sqrt(1+R(0,0)-R(1,1)-R(2,2))*2;
            Q(0) = (R(2,1) - R(1,2))/S;
            Q(1) = 0.25*S;
            Q(2) = (R(0,1) + R(1,0))/S;
            Q(3) = (R(0,2) + R(2,0))/S;

    }else if (R(1,1)>R(2,2))
    {
            Scalar S = sqrt(1+R(1,1) - R(0,0) - R(2,2))*2;
            Q(0) = (R(0,2)-R(2,0))/S;
            Q(1) = (R(0,1) + R(1,0))/S;
            Q(2) = 0.25*S;
            Q(3) = (R(1,2) + R(2,1))/S;
    }else {
            Scalar S = sqrt(1+R(2,2)-R(0,0)-R(1,1))*2;
            Q(0) = (R(1,0) - R(0,1))/S;
            Q(1) = (R(0,2)+R(2,0))/S;
            Q(2) = (R(1,2)-R(2,1))/S;
            Q(3) = 0.25*S;
    }

    **/

}





