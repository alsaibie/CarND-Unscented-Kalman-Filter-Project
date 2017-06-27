#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include <iomanip>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


int main(int argc, char *argv[]) {


    string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    if (!in_file_.is_open()) {
        cout << "Cannot open input file: " << in_file_name_ << endl;
    }

    string out_file_name_ = "../data/obj_pose-laser-radar-ukf-output.txt";
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    if (!out_file_.is_open()) {
        cout << "Cannot open output file: " << in_file_name_ << endl;
    }

    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;

    string line;
    Tools tools;
    // prep the measurement packages (each line represents a measurement at a
    // timestamp)
    while (getline(in_file_, line)) {

        string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        istringstream iss(line);
        long timestamp;

        // reads first element from the current line
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0) {
            // LASER MEASUREMENT

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        } else if (sensor_type.compare("R") == 0) {
            // RADAR MEASUREMENT

            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }

        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        gt_pack_list.push_back(gt_package);
    }

    // Create a Fusion EKF instance
    UKF ukf;

    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    //Call the EKF-based fusion
    size_t N = measurement_pack_list.size();
    for (size_t k = 0; k < N; ++k) {
        // start filtering from the second frame (the speed is unknown in the first
        // frame)
        ukf.ProcessMeasurement(measurement_pack_list[k]);

        // output the estimation
        VectorXd x_cartesian_estimate = ukf.getCartesian();
        out_file_ << x_cartesian_estimate(0) << "\t"; // First Entry
        out_file_ << x_cartesian_estimate(1) << "\t";
        out_file_ << x_cartesian_estimate(2) << "\t";
        out_file_ << x_cartesian_estimate(3) << "\t";


        // output the measurements
        if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
            // output the estimation
            out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
            out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
            out_file_ << "L" << ukf.NIS_lidar_<<"\t";
        } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
            // output the estimation in the cartesian coordinates
            float ro = measurement_pack_list[k].raw_measurements_(0);
            float phi = measurement_pack_list[k].raw_measurements_(1);
            out_file_ << ro * cos(phi) << "\t"; // px_meas
            out_file_ << ro * sin(phi) << "\t"; // py_meas
            out_file_ << "R" << ukf.NIS_radar_<<"\t";
        }

        // output the ground truth packages
        out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
        out_file_ << gt_pack_list[k].gt_values_(3) << "\t";

        estimations.push_back(x_cartesian_estimate);
        ground_truth.push_back(gt_pack_list[k].gt_values_);
        VectorXd rmse(4);
        //rmse.fill(0.0);
        rmse = tools.CalculateRMSE(estimations, ground_truth);

        out_file_ << rmse(0) << "\t";
        out_file_ << rmse(1) << "\t";
        out_file_ << rmse(2) << "\t";
        out_file_ << rmse(3) << "\n";

    }

    // compute the accuracy (RMSE)

    VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
    cout << "Accuracy - RMSE:" << endl << std::setprecision(2) << rmse << endl;

    // close files
    if (out_file_.is_open()) {
        out_file_.close();
    }

    if (in_file_.is_open()) {
        in_file_.close();
    }

    return 0;
}