#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int simulator_main()
{
    uWS::Hub h;
    
    // Create a Kalman Filter instance
    UKF ukf;
    
    // used to compute the RMSE later
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    
    h.onMessage([&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            
            auto s = hasData(std::string(data));
            if (s != "") {
                
                auto j = json::parse(s);
                
                std::string event = j[0].get<std::string>();
                
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    
                    string sensor_measurment = j[1]["sensor_measurement"];
                    
                    MeasurementPackage meas_package;
                    istringstream iss(sensor_measurment);
                    long long timestamp;
                    
                    // reads first element from the current line
                    string sensor_type;
                    iss >> sensor_type;
                    
                    if (sensor_type.compare("L") == 0) {
                        meas_package.sensor_type_ = MeasurementPackage::LASER;
                        meas_package.raw_measurements_ = VectorXd(2);
                        float px;
                        float py;
                        iss >> px;
                        iss >> py;
                        meas_package.raw_measurements_ << px, py;
                        iss >> timestamp;
                        meas_package.timestamp_ = timestamp;
                    } else if (sensor_type.compare("R") == 0) {
                        
                        meas_package.sensor_type_ = MeasurementPackage::RADAR;
                        meas_package.raw_measurements_ = VectorXd(3);
                        float ro;
                        float theta;
                        float ro_dot;
                        iss >> ro;
                        iss >> theta;
                        iss >> ro_dot;
                        meas_package.raw_measurements_ << ro,theta, ro_dot;
                        iss >> timestamp;
                        meas_package.timestamp_ = timestamp;
                    }
                    float x_gt;
                    float y_gt;
                    float vx_gt;
                    float vy_gt;
                    iss >> x_gt;
                    iss >> y_gt;
                    iss >> vx_gt;
                    iss >> vy_gt;
                    VectorXd gt_values(4);
                    gt_values(0) = x_gt;
                    gt_values(1) = y_gt;
                    gt_values(2) = vx_gt;
                    gt_values(3) = vy_gt;
                    ground_truth.push_back(gt_values);
                    
                    //Call ProcessMeasurment(meas_package) for Kalman filter
                    ukf.ProcessMeasurement(meas_package);
                    
                    //Push the current estimated x,y positon from the Kalman filter's state vector
                    
                    VectorXd estimate(4);
                    
                    double p_x = ukf.x_(0);
                    double p_y = ukf.x_(1);
                    double v  = ukf.x_(2);
                    double yaw = ukf.x_(3);
                    
                    double v1 = cos(yaw)*v;
                    double v2 = sin(yaw)*v;
                    
                    estimate(0) = p_x;
                    estimate(1) = p_y;
                    estimate(2) = v1;
                    estimate(3) = v2;
                    
                    estimations.push_back(estimate);
                    
                    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
                    
                    json msgJson;
                    msgJson["estimate_x"] = p_x;
                    msgJson["estimate_y"] = p_y;
                    msgJson["rmse_x"] =  RMSE(0);
                    msgJson["rmse_y"] =  RMSE(1);
                    msgJson["rmse_vx"] = RMSE(2);
                    msgJson["rmse_vy"] = RMSE(3);
                    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
                    // std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    
                }
            } else {
                
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
        
    });
    
    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
    
    return 0;
}

void check_file_arguments(int argc, char* argv[]) {
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";
    
    bool has_valid_args = false;
    
    // make sure the user has provided input and output files
    if (argc == 1) {
        cerr << usage_instructions << endl;
    } else if (argc == 2) {
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    } else if (argc == 3) {
        has_valid_args = true;
    } else if (argc > 3) {
        cerr << "Too many arguments.\n" << usage_instructions << endl;
    }
    
    if (!has_valid_args) {
        exit(EXIT_FAILURE);
    }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }
    
    if (!out_file.is_open()) {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}

int file_main(int argc, char* argv[]) {
    check_file_arguments(argc, argv);
    
    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);
    
    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);
    
    check_files(in_file_, in_file_name_, out_file_, out_file_name_);
    
    vector<MeasurementPackage> measurement_pack_list;
    vector<VectorXd> gt_pack_list;
    
    string line;
    
    // prep the measurement packages (each line represents a measurement at a
    // timestamp)
    while (getline(in_file_, line)) {
        
        string sensor_type;
        MeasurementPackage meas_package;
        istringstream iss(line);
        long long timestamp;
        
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
            
            // read ground truth data to compare later
            float x_gt;
            float y_gt;
            float vx_gt;
            float vy_gt;
            iss >> x_gt;
            iss >> y_gt;
            iss >> vx_gt;
            iss >> vy_gt;
            VectorXd gt_values = VectorXd(4);
            gt_values << x_gt, y_gt, vx_gt, vy_gt;
            gt_pack_list.push_back(gt_values);
            
        } else if (sensor_type.compare("R") == 0) {
            // RADAR MEASUREMENT
            
            // read measurements at this timestamp
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float phi;
            float ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, phi, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
            
            // read ground truth data to compare later
            float x_gt;
            float y_gt;
            float vx_gt;
            float vy_gt;
            iss >> x_gt;
            iss >> y_gt;
            iss >> vx_gt;
            iss >> vy_gt;
            VectorXd gt_values = VectorXd(4);
            gt_values << x_gt, y_gt, vx_gt, vy_gt;
            gt_pack_list.push_back(gt_values);
            
        }
    }
    
    // Create a UKF instance
    UKF ukf;
    
    // used to compute the RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    
    // start filtering from the second frame (the speed is unknown in the first frame)
    size_t number_of_measurements = measurement_pack_list.size();
    
     cout << "Read in " << endl << number_of_measurements << endl;
    
    // column names for output file
    out_file_ << "time_stamp" << "\t";
    out_file_ << "px_state" << "\t";
    out_file_ << "py_state" << "\t";
    out_file_ << "v_state" << "\t";
    out_file_ << "yaw_angle_state" << "\t";
    out_file_ << "yaw_rate_state" << "\t";
    out_file_ << "sensor_type" << "\t";
    out_file_ << "NIS" << "\t";
    out_file_ << "px_measured" << "\t";
    out_file_ << "py_measured" << "\t";
    out_file_ << "px_ground_truth" << "\t";
    out_file_ << "py_ground_truth" << "\t";
    out_file_ << "vx_ground_truth" << "\t";
    out_file_ << "vy_ground_truth" << "\n";
    
    for (size_t k = 0; k < number_of_measurements; ++k) {
        // Call the UKF-based fusion
        ukf.ProcessMeasurement(measurement_pack_list[k]);
        
        // timestamp
        out_file_ << measurement_pack_list[k].timestamp_ << "\t"; // pos1 - est
        
        // output the state vector
        out_file_ << ukf.x_(0) << "\t"; // pos1 - est
        out_file_ << ukf.x_(1) << "\t"; // pos2 - est
        out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
        out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
        out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est
        
        // output lidar and radar specific data
        if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
            // sensor type
            out_file_ << "lidar" << "\t";
            
            // NIS value
            out_file_ << ukf.NIS_laser_ << "\t";
            
            // output the lidar sensor measurement px and py
            out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
            out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
            
        } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
            // sensor type
            out_file_ << "radar" << "\t";
            
            // NIS value
            out_file_ << ukf.NIS_radar_ << "\t";
            
            // output radar measurement in cartesian coordinates
            float ro = measurement_pack_list[k].raw_measurements_(0);
            float phi = measurement_pack_list[k].raw_measurements_(1);
            out_file_ << ro * cos(phi) << "\t"; // px measurement
            out_file_ << ro * sin(phi) << "\t"; // py measurement
        }
        
        // output the ground truth
        out_file_ << gt_pack_list[k](0) << "\t";
        out_file_ << gt_pack_list[k](1) << "\t";
        out_file_ << gt_pack_list[k](2) << "\t";
        out_file_ << gt_pack_list[k](3) << "\n";
        
        // convert ukf x vector to cartesian to compare to ground truth
        VectorXd ukf_x_cartesian_ = VectorXd(4);
        
        float x_estimate_ = ukf.x_(0);
        float y_estimate_ = ukf.x_(1);
        float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
        float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
        
        ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
        
        cout << "index = " << k << endl;
        cout << "x_ = " << ukf_x_cartesian_ << endl;
        cout << "gt = " << gt_pack_list[k] << endl;
        
        estimations.push_back(ukf_x_cartesian_);
        ground_truth.push_back(gt_pack_list[k]);
        
    }
    
    // compute the accuracy (RMSE)
    Tools tools;
    cout << "RMSE" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
    
    // close files
    if (out_file_.is_open()) {
        out_file_.close();
    }
    
    if (in_file_.is_open()) {
        in_file_.close();
    }
    
    cout << "Done!" << endl;
    return 0;
}

int main(int argc, char* argv[]) {
    return file_main(argc, argv);
    //return simulator_main();
}























































































