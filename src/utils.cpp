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

void rotMatToQuaternion(ADMatrix const & R, ADVector & Quat)
{
    ADScalar tr; tr = R(0,0) + R(1,1) + R(2,2);
    ADScalar S1; S1 = 0.5 / CppAD::sqrt(tr+1.0);
    ADScalar S2; S2 = 2.0 * CppAD::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
    ADScalar S3; S3 = 2.0 * CppAD::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
    ADScalar S4; S4 = 2.0 * CppAD::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
    ADScalar zero(0);

    //w
    Quat(0) = CondExpOp(CompareLt,zero,tr,0.25 / S1,
                       CondExpOp(CompareLt, R(1,1), R(0,0),
                                CondExpOp(CompareLt, R(2,2), R(0,0), (R(2,1) - R(1,2)) / S2,
                                    CondExpOp(CompareLt, R(2,2),R(1,1), (R(0,2) - R(2,0)) / S3, (R(1,0) - R(0,1)) / S4)),
                                        CondExpOp(CompareLt, R(2,2),R(1,1), (R(0,2) - R(2,0)) / S3, (R(1,0) - R(0,1)) / S4)));

    //x
    Quat(1) = CondExpOp(CompareLt,zero,tr,(R(2,1) - R(1,2))*S1,
                           CondExpOp(CompareLt, R(1,1), R(0,0),
                                    CondExpOp(CompareLt, R(2,2), R(0,0), 0.25*S2,
                                        CondExpOp(CompareLt, R(2,2),R(1,1), (R(0,1)+R(1,0))/S3, (R(0,2)+R(2,0))/S4)),
                                            CondExpOp(CompareLt, R(2,2),R(1,1), (R(0,1)+R(1,0))/S3, (R(0,2)+R(2,0))/S4)));
    //y
    Quat(2) = CondExpOp(CompareLt,zero,tr,(R(0,2) - R(2,0))*S1,
                           CondExpOp(CompareLt, R(1,1), R(0,0),
                                    CondExpOp(CompareLt, R(2,2), R(0,0), (R(0,1) + R(1,0)) / S2,
                                        CondExpOp(CompareLt, R(2,2),R(1,1), 0.25 * S3, (R(1,2) + R(2,1)) / S4)),
                                            CondExpOp(CompareLt, R(2,2),R(1,1), 0.25 * S3, (R(1,2) + R(2,1)) / S4)));
    //z
    Quat(3) = CondExpOp(CompareLt,zero,tr,(R(1,0) - R(0,1))*S1,
                           CondExpOp(CompareLt, R(1,1), R(0,0),
                                    CondExpOp(CompareLt, R(2,2), R(0,0), (R(0,2) + R(2,0)) / S2,
                                        CondExpOp(CompareLt, R(2,2),R(1,1), (R(1,2) + R(2,1)) / S3, 0.25 * S4)),
                                            CondExpOp(CompareLt, R(2,2),R(1,1), (R(1,2) + R(2,1)) / S3, 0.25 * S4)));

}

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
void eigen2vector(std::vector<ScalarVector> const& eigenMat, std::vector<std::vector<Scalar>> & returnedVect)
{

    for(unsigned int i=0; i<eigenMat[0].rows(); i++){
        std::vector<Scalar> newVec;
        for (unsigned int j=0; j<eigenMat.size(); j++)
        {
        	newVec.push_back(eigenMat[j](i));
        }
        returnedVect.push_back(newVec);
    }


}

void vector2eigen(std::vector<ScalarVector> const& vectorMat, ScalarMatrix & eigenMat){
	for (unsigned int i = 0; i < vectorMat.size() -1 ; i++){
		for(unsigned int j = 0; j<vectorMat[i].rows(); j++){
			eigenMat(j,i) = vectorMat[i](j);
		}
	}
}
//void getStateVectAD_vel(ADVector & X, ADVector const & q, ADVector const & dq){
//
//    //ADVector X(q.rows()+dq.rows());
//    int i = 0;
//    int j = 0;
//    for (i=0; i<X.rows(); i += 2){
//        X(i) = q(j);
//        X(i+1) = dq(j);
//        j++;
//    }
//    //return X;
//}


Eigen::Vector4d rot2quat(Eigen::Matrix3d const & R)
{
	/*
	 * This function is used to convert a rotation matrix R to a quaternion Q
	 * The quaternion Q is passed by reference and directly modified in the function.
	 * INPUTS:
	 * 	R = 3*3 Rotation matrix
	 *
	 * */
	Eigen::Vector4d Q = Eigen::MatrixXd::Zero(4,1);
    Scalar Tr(R(0,0) + R(1,1) + R(2,2));
    Scalar S = sqrt(Tr+1.0)*2.0;
    Q(0) = 0.25 * S;
    Q(1) = (R(2,1) - R(1,2))/S;
    Q(2) = (R(0,2) - R(2,0))/S;
    Q(3) = (R(1,0) - R(0,1))/S;

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

    return Q;
}

void rot2quatAD(ADMatrix const & R, ADVector & Q)
{
    ADScalar Tr;
    Tr = R(0,0) + R(1,1) + R(2,2);
    ADScalar S = sqrt(Tr+1.0)*2.0;
    Q(0) = 0.25 * S;
    Q(1) = (R(2,1) - R(1,2))/S;
    Q(2) = (R(0,2) - R(2,0))/S;
    Q(3) = (R(1,0) - R(0,1))/S;


//        if (Tr>0)
//    {
//        ADScalar S;
//        S = sqrt(Tr+1)*2.0;
//        Q(0) = 0.25 * S;
//        Q(1) = (R(2,1) - R(1,2))/S;
//        Q(2) = (R(0,2) - R(2,0))/S;
//        Q(3) = (R(1,0) - R(0,1))/S;
//    } else if (R(0,0)>R(1,1) && R(0,0)<R(2,2))
//    {
//            ADScalar S;
//            S = sqrt(1.0+R(0,0)-R(1,1)-R(2,2))*2;
//            Q(0) = (R(2,1) - R(1,2))/S;
//            Q(1) = 0.25*S;
//            Q(2) = (R(0,1) + R(1,0))/S;
//            Q(3) = (R(0,2) + R(2,0))/S;
//
//    }else if (R(1,1)>R(2,2))
//    {
//            ADScalar S;
//            S = sqrt(1.0+R(1,1) - R(0,0) - R(2,2))*2.0;
//            Q(0) = (R(0,2)-R(2,0))/S;
//            Q(1) = (R(0,1) + R(1,0))/S;
//            Q(2) = 0.25*S;
//            Q(3) = (R(1,2) + R(2,1))/S;
//    }else {
//            ADScalar S;
//            S = sqrt(1.0+R(2,2)-R(0,0)-R(1,1))*2.0;
//            Q(0) = (R(1,0) - R(0,1))/S;
//            Q(1) = (R(0,2)+R(2,0))/S;
//            Q(2) = (R(1,2)-R(2,1))/S;
//            Q(3) = 0.25*S;
//    }

}

ScalarMatrix read_data(std::string const & filePath, std::string const & dataType){
        // File pointer
    fstream fin;
    int nbrows(0);
    ScalarMatrix data_matrix;
    if (dataType == "q"){
        data_matrix.resize(13,1122);
        nbrows = 13;
    }else if (dataType == "measurement"){
        data_matrix.resize(39,1122);
        nbrows = 39;
    }else if (dataType == "SimulatedData"){
        data_matrix.resize(1122, 300);
        nbrows = 1122;
    }
    // Open an existing file
    fin.open(filePath, ios::in);

    // Read the Data from the file
    // as String Vector
    std::vector<string> row;
    string line, word;

    int j = 0;
    for (j = 0; j<nbrows; j++){
        unsigned int i =0;

        row.clear();

        // read an entire row and
        // store it in a string variable 'line'
        getline(fin, line);

        // used for breaking words
        stringstream s(line);

        // read every column data of a row and
        // store it in a string variable, 'word'
        while (getline(s, word, ',')) {

            // add all the column data
            // of a row to a vector
            row.push_back(word);
        }

        ScalarVector q_data(row.size());
        for (i = 0; i<row.size(); i++){
            q_data(i) = std::stod(row[i]);
        }
        data_matrix.row(j) = q_data.transpose();
    }
    return data_matrix;
}

std::fstream& GotoLine(std::fstream& file, unsigned int const & num){
    //std::fstream file(filePath);
    file.seekg(std::ios::beg);
    for(unsigned int i=0; i < num - 1; ++i){
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}

ScalarMatrix readTrajFromCSV(int const & trajIndex, std::string const & typeTraj)
{
    // File pointer
    ScalarMatrix data_matrix(13,1121);

    if (typeTraj == "pos")
    {
        for(int qIndex = 0; qIndex<13; qIndex++)
        {
            fstream fin;
            std::string filePath("./newData/q" + std::to_string(qIndex + 1) + ".csv");
            // Open an existing file
            fin.open(filePath, ios::in);
            GotoLine(fin,trajIndex);

            // Read the Data from the file
            // as String Vector
            std::vector<string> row;
            string line, word;
            row.clear();

            // read an entire row and
            // store it in a string variable 'line'
            getline(fin, line);

            // used for breaking words
            stringstream s(line);

            // used for breaking words
            // read every column data of a row and
            // store it in a string variable, 'word'
            while (getline(s, word, ','))
            {
                // add all the column data
                // of a row to a vector
                row.push_back(word);
            }

            ScalarVector q_data(row.size());
            for (unsigned int i = 0; i<row.size(); i++)
            {
                q_data(i) = std::stod(row[i]);
            }

            data_matrix.row(qIndex) = q_data.transpose();
        }

    } else if (typeTraj == "vel")
    {
        for(int qIndex = 0; qIndex<13; qIndex++)
        {
            fstream fin;
            std::string filePath("./newData/dq" + std::to_string(qIndex + 1) + ".csv");
            // Open an existing file
            fin.open(filePath, ios::in);
            GotoLine(fin,trajIndex);

            // Read the Data from the file
            // as String Vector
            std::vector<string> row;
            string line, word;
            row.clear();

            // read an entire row and
            // store it in a string variable 'line'
            getline(fin, line);

            // used for breaking words
            stringstream s(line);

            // used for breaking words
            // read every column data of a row and
//             store it in a string variable, 'word'
            while (getline(s, word, ','))
            {
            //getline(s, word, ',');
//                // add all the column data
//                // of a row to a vector
                row.push_back(word);
            }

            ScalarVector q_data(row.size());
            for (unsigned int i = 0; i<row.size(); i++)
            {
                q_data(i) = std::stod(row[i]);
            }

            data_matrix.row(qIndex) = q_data.transpose();
        }

    } else if (typeTraj == "acc")
    {
        for(int qIndex = 0; qIndex<13; qIndex++)
            {
                fstream fin;
                std::string filePath("./newData/ddq" + std::to_string(qIndex + 1) + ".csv");
                // Open an existing file
                fin.open(filePath, ios::in);
                GotoLine(fin,trajIndex);

                // Read the Data from the file
                // as String Vector
                std::vector<string> row;
                string line, word;
                row.clear();

                // read an entire row and
                // store it in a string variable 'line'
                getline(fin, line);

                // used for breaking words
                stringstream s(line);

                // used for breaking words
                // read every column data of a row and
                // store it in a string variable, 'word'
            while (getline(s, word, ','))
            {
//            getline(s, word, ',');
//                // add all the column data
//                // of a row to a vector
                row.push_back(word);
            }

                ScalarVector q_data(row.size());
                for (unsigned int i = 0; i<row.size(); i++)
                {
                    q_data(i) = std::stod(row[i]);
                }
                data_matrix.row(qIndex) = q_data.transpose();
            }
    }
    return data_matrix;
}

/**estimatedData refers to estimated angles and q_data refers to reference angles**/
void plotData(
		ScalarMatrix const & Q_estimated,
		ScalarMatrix const & Q_reference,
		ScalarMatrix const & estimatedMeasurement,
		ScalarMatrix const & referenceMeasurement,
		ScalarMatrix const & sensorMeasurement
		)
{
    /**Setting Time vector t**/
    std::vector<std::vector<double>> t(39, std::vector<double>(Q_estimated.cols())); //Initialize time vector
    int i = 0;
    int j = 0;
    for (i=0; i<39; i++) //Set values for time vector
    {
        for (j = 0; j<Q_estimated.cols(); j++)
        {
            if (i==0 && j==0)
            {
                t[i][j] = 1;
            }
            else if (i != 0 && j == 0)
            {
                t[i][j] = t[i-1][Q_estimated.cols()-1];
            }else
            {
                t[i][j] = t[i][j-1] + 1;

            }
        }
    }

    /**Setting Angles q_es and q_ref**/
    std::vector<std::vector<double>> q_ref(Q_estimated.rows(), std::vector<double>(Q_estimated.cols())); //Reference angles
    std::vector<std::vector<double>> q_es(Q_estimated.rows(), std::vector<double>(Q_estimated.cols())); //Estimated angles
    eigen2vector(Q_estimated, q_es); //COnvert estimatedData to vector
    eigen2vector(Q_reference, q_ref); //Convert q_data to vector

    /**Setting Measurements**/
    std::vector<std::vector<double>> sensorMeas(sensorMeasurement.rows(), std::vector<double>(sensorMeasurement.cols())); //Sensor measurements
    std::vector<std::vector<double>> refMeas(referenceMeasurement.rows(), std::vector<double>(referenceMeasurement.cols())); //Reference measurements
    std::vector<std::vector<double>> esMeas(estimatedMeasurement.rows(), std::vector<double>(estimatedMeasurement.cols())); //Estimated measurements
    eigen2vector(estimatedMeasurement, esMeas);
    eigen2vector(referenceMeasurement, refMeas);
    eigen2vector(sensorMeasurement, sensorMeas);

    /**Plotting angles Q **/
    plt::figure(1);
    plt::subplot(5,1,1);
    plt::named_plot("Reference angle", t[0],q_ref[0], "g");
    plt::named_plot("Estimated angle", t[0],q_es[0],"r--");
    for (i=1; i<Q_estimated.rows(); i++)
    {
        plt::plot(t[i],q_ref[i],"g");
        plt::plot(t[i],q_es[i], "r--");
    }
    //plt::title("Q1 / Q2 / Q3 / Q4 / Q5 / Q6 / Q7 / Q8 / Q9 / Q10 / Q11 / Q12 / Q13");
    plt::legend();

/**Plotting measurements **/
    /** Positions **/
    plt::subplot(5,1,2);
    for (i=0; i<9; i+=3)
    {
        if (i == 0){
            plt::named_plot("Reference positions",t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::named_plot("Estimated positions",t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::named_plot("Sensor positions",t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
        }else{
            plt::plot(t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
        }
    }
    //plt::title("IMU1_px/IMU1_py/IMU1_pz ; IMU2_px/IMU2_py/IMU2_pz ; IMU3_px/IMU3_py/IMU3_pz");
    plt::legend();

    i=9;
    plt::subplot(5,1,3);
    for (i=9; i<24; i+=6)
    {
        if (i == 9){
            plt::named_plot("Reference accelerations",t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::named_plot("Estimated accelerations",t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::named_plot("Sensor accelerations",t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");

        }else{
            plt::plot(t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
        }
    }
    //plt::title("IMU1_ACCx/IMU1_ACCy/IMU1_ACCz ; IMU2_ACCx/IMU2_ACCy/IMU2_ACCz ; IMU3_ACCx/IMU3_ACCy/IMU3_ACCz");
    plt::legend();

    i=12;
    plt::subplot(5,1,4);
    for (i=12; i<27; i+=6)
    {
        if (i == 12){
            plt::named_plot("Reference velocities",t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::named_plot("Estimated velocities",t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::named_plot("Sensor velocities",t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");

        }else{
            plt::plot(t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
        }
    }
    //plt::title("IMU1_VELx/IMU1_VELy/IMU1_VELz ; IMU2_VELx/IMU2_VELy/IMU2_VELz ; IMU3_VELx/IMU3_VELy/IMU3_VELz");
    plt::legend();
    i = 27;
    /**Quaternions **/
    plt::subplot(5,1,5);
    for (i=27; i<39; i+=4)
    {
        if (i == 27){
            plt::named_plot("Reference quaternions",t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i+3],refMeas[i+3],"g");
            plt::named_plot("Estimated quaternions",t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i+3],esMeas[i+3], "r--");
            plt::named_plot("Sensor quaternions",t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
            plt::plot(t[i+3],sensorMeas[i+3], "b--");

        }else{
            plt::plot(t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i+3],refMeas[i+3],"g");
            plt::plot(t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i+3],esMeas[i+3], "r--");
            plt::plot(t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
            plt::plot(t[i+3],sensorMeas[i+3], "b--");
        }
    }
    //plt::title("IMU1_w/IMU1_x/IMU1_y/IMU1_z  ; IMU2_w/IMU2_x/IMU1_y/IMU2_z ; IMU3_w/IMU3_x/IMU3_y/IMU3_z");
    plt::legend();

    plt::show();

}


/**estimatedData refers to estimated angles and q_data refers to reference angles**/
void plotData(
		ScalarMatrix const & Q_estimated,
		ScalarMatrix const & Q_reference,
		ScalarMatrix const & dq_estimated,
		ScalarMatrix const & dq_reference,
		ScalarMatrix const & ddq_estimated,
		ScalarMatrix const & ddq_reference,
		ScalarMatrix const & estimatedMeasurement,
		ScalarMatrix const & referenceMeasurement,
		ScalarMatrix const & sensorMeasurement
		)


{
    /**Setting Time vector t**/
    std::vector<std::vector<double>> t(39, std::vector<double>(Q_estimated.cols())); //Initialize time vector
    int i = 0;
    int j = 0;
    for (i=0; i<39; i++) //Set values for time vector
    {
        for (j = 0; j<Q_estimated.cols(); j++)
        {
            if (i==0 && j==0)
            {
                t[i][j] = 1;
            }
            else if (i != 0 && j == 0)
            {
                t[i][j] = t[i-1][Q_estimated.cols()-1];
            }else
            {
                t[i][j] = t[i][j-1] + 1;

            }
        }
    }

    /**Setting Angles q_es and q_ref**/
    std::vector<std::vector<double>> q_ref(Q_estimated.rows(), std::vector<double>(Q_estimated.cols())); //Reference angles
    std::vector<std::vector<double>> q_es(Q_estimated.rows(), std::vector<double>(Q_estimated.cols())); //Estimated angles
    eigen2vector(Q_estimated, q_es); //COnvert estimatedData to vector
    eigen2vector(Q_reference, q_ref); //Convert q_data to vector

    /**Setting Measurements**/
    std::vector<std::vector<double>> sensorMeas(sensorMeasurement.rows(), std::vector<double>(sensorMeasurement.cols())); //Sensor measurements
    std::vector<std::vector<double>> refMeas(referenceMeasurement.rows(), std::vector<double>(referenceMeasurement.cols())); //Reference measurements
    std::vector<std::vector<double>> esMeas(estimatedMeasurement.rows(), std::vector<double>(estimatedMeasurement.cols())); //Estimated measurements
    eigen2vector(estimatedMeasurement, esMeas);
    eigen2vector(referenceMeasurement, refMeas);
    eigen2vector(sensorMeasurement, sensorMeas);

    /**Plotting angles Q **/
    plt::figure(1);
    plt::subplot(5,1,1);
    plt::named_plot("Reference angle", t[0],q_ref[0], "g");
    plt::named_plot("Estimated angle", t[0],q_es[0],"r--");
    for (i=1; i<Q_estimated.rows(); i++)
    {
        plt::plot(t[i],q_ref[i],"g");
        plt::plot(t[i],q_es[i], "r--");
    }
    //plt::title("Q1 / Q2 / Q3 / Q4 / Q5 / Q6 / Q7 / Q8 / Q9 / Q10 / Q11 / Q12 / Q13");
    plt::legend();

/**Plotting measurements **/
    /** Positions **/
    plt::subplot(5,1,2);
    for (i=0; i<9; i+=3)
    {
        if (i == 0){
            plt::named_plot("Reference positions",t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::named_plot("Estimated positions",t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::named_plot("Sensor positions",t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
        }else{
            plt::plot(t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
        }
    }
    //plt::title("IMU1_px/IMU1_py/IMU1_pz ; IMU2_px/IMU2_py/IMU2_pz ; IMU3_px/IMU3_py/IMU3_pz");
    plt::legend();

    i=9;
    plt::subplot(5,1,3);
    for (i=9; i<18; i+=3)
    {
        if (i == 9){
            plt::named_plot("Reference velocities",t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::named_plot("Estimated velocities",t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::named_plot("Sensor velocities",t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");

        }else{
            plt::plot(t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
        }
    }
    //plt::title("IMU1_ACCx/IMU1_ACCy/IMU1_ACCz ; IMU2_ACCx/IMU2_ACCy/IMU2_ACCz ; IMU3_ACCx/IMU3_ACCy/IMU3_ACCz");
    plt::legend();

    i=18;
    plt::subplot(5,1,4);
    for (i=18; i<27; i+=3)
    {
        if (i == 9){
            plt::named_plot("Reference accelerations",t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::named_plot("Estimated accelerations",t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::named_plot("Sensor accelerations",t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");

        }else{
            plt::plot(t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
        }
    }

    i=27;

    plt::legend();
    /**Quaternions **/
    plt::subplot(5,1,5);
    for (i=27; i<39; i+=4)
    {
        if (i == 27){
            plt::named_plot("Reference quaternions",t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i+3],refMeas[i+3],"g");
            plt::named_plot("Estimated quaternions",t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i+3],esMeas[i+3], "r--");
            plt::named_plot("Sensor quaternions",t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
            plt::plot(t[i+3],sensorMeas[i+3], "b--");

        }else{
            plt::plot(t[i],refMeas[i],"g");
            plt::plot(t[i+1],refMeas[i+1],"g");
            plt::plot(t[i+2],refMeas[i+2],"g");
            plt::plot(t[i+3],refMeas[i+3],"g");
            plt::plot(t[i],esMeas[i], "r--");
            plt::plot(t[i+1],esMeas[i+1], "r--");
            plt::plot(t[i+2],esMeas[i+2], "r--");
            plt::plot(t[i+3],esMeas[i+3], "r--");
            plt::plot(t[i],sensorMeas[i], "b--");
            plt::plot(t[i+1],sensorMeas[i+1], "b--");
            plt::plot(t[i+2],sensorMeas[i+2], "b--");
            plt::plot(t[i+3],sensorMeas[i+3], "b--");
        }
    }
    //plt::title("IMU1_w/IMU1_x/IMU1_y/IMU1_z  ; IMU2_w/IMU2_x/IMU1_y/IMU2_z ; IMU3_w/IMU3_x/IMU3_y/IMU3_z");
    plt::legend();


    std::vector<std::vector<double>> dq_ref(dq_estimated.rows(), std::vector<double>(dq_estimated.cols())); //Reference angles
    std::vector<std::vector<double>> dq_es(dq_reference.rows(), std::vector<double>(dq_reference.cols())); //Estimated angles
    eigen2vector(dq_estimated, dq_es); //COnvert estimatedData to vector
    eigen2vector(dq_reference, dq_ref); //Convert q_data to vector
    plt::figure(2);
    plt::subplot(3,1,1);
    plt::named_plot("Reference angle", t[0],q_ref[0], "g");
    plt::named_plot("Estimated angle", t[0],q_es[0],"r--");
    for (i=1; i<Q_estimated.rows(); i++)
    {
        plt::plot(t[i],q_ref[i],"g");
        plt::plot(t[i],q_es[i], "r--");
    }
    //plt::title("Q1 / Q2 / Q3 / Q4 / Q5 / Q6 / Q7 / Q8 / Q9 / Q10 / Q11 / Q12 / Q13");
    plt::legend();

    plt::subplot(3,1,2);
    plt::named_plot("Reference velocity", t[0],dq_ref[0], "g");
    plt::named_plot("Estimated velocity", t[0],dq_es[0],"r--");
    for (i=1; i<dq_estimated.rows(); i++)
    {
        plt::plot(t[i],dq_ref[i],"g");
        plt::plot(t[i],dq_es[i], "r--");
    }
    //plt::title("Q1 / Q2 / Q3 / Q4 / Q5 / Q6 / Q7 / Q8 / Q9 / Q10 / Q11 / Q12 / Q13");
    plt::legend();
    std::vector<std::vector<double>> ddq_ref(ddq_estimated.rows(), std::vector<double>(ddq_estimated.cols())); //Reference angles
    std::vector<std::vector<double>> ddq_es(ddq_reference.rows(), std::vector<double>(ddq_reference.cols())); //Estimated angles
    eigen2vector(ddq_estimated, ddq_es); //COnvert estimatedData to vector
    eigen2vector(ddq_reference, ddq_ref); //Convert q_data to vector
    plt::subplot(3,1,3);
    plt::named_plot("Reference acceleration", t[0],ddq_ref[0], "g");
    plt::named_plot("Estimated acceleration", t[0],ddq_es[0],"r--");
    for (i=1; i<ddq_estimated.rows(); i++)
    {
        plt::plot(t[i],ddq_ref[i],"g");
        plt::plot(t[i],ddq_es[i], "r--");
    }

    plt::show();
}

