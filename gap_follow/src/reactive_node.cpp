//C++ library includes
#include <memory> //for smart pointers
//#include <chrono> //for time 
#include <math.h> //for isnan, isinf etc.
#include <string>
#include <cstdlib> //for abs value function
#include <vector> /// CHECK: might cause errors due to double header in linker


//ROS related headers
#include "rclcpp/rclcpp.hpp"
//message header
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


//headers to code written by you


//other macros
#define NODE_NAME "reactive_node" 
#define _USE_MATH_DEFINES
#define ANGLE_B M_PI/2
#define ANGLE_A ANGLE_B - (50*M_PI/180)
#define DESIRED_DISTANCE_LEFT 1.00 // in meters
#define VIEW_MIN_ANGLE -90 
#define VIEW_MAX_ANGLE 90 //min and max angle for looking forward. rest is used to look back and make sure we do not hit a turn
#define WINDOW_AVG 5
//#define K_p 1
//#define K_d 0.01
//#define K_i 0.05

//using namespaces 
//used for bind (uncomment below)
using std::placeholders::_1;
//using namespace std::chrono_literals;


class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node(NODE_NAME)
    {
        // initialise parameters
        this->declare_parameter("go", true);
        drive_flag = this->get_parameter("go").as_bool();
        //set parameter during run time with: ros2 param set <node_name> <parameter_name> <value>
        //set parameter during launch time with: 


        /// TODO: create ROS subscribers and publishers
        //initialise subscriber sharedptr obj
        subscription_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 1000, std::bind(&ReactiveFollowGap::scan_callback, this, _1));

        //initialise publisher sharedptr obj
        publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1000);

        publisher_rviz = this->create_publisher<sensor_msgs::msg::LaserScan>(rviz_debug_topic, 1000);

        RCLCPP_INFO (this->get_logger(), "this node has been launched");

    }

private:

    //global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    struct localData{
        double angle_min, angle_increment, max_range;
        std::vector<float> range_data;
        double max_look = 7;
    };

    int max_gap_indices[2];
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    std::string rviz_debug_topic = "/rviz_custom_debugger";

    //struct initialisation
    localData scan_message_localdata;
    bool drive_flag; //parameter to freeze car
    
    //declare publisher sharedpointer obj
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laserscan;

    //declare publisher sharedpointer obj
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive; 
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_rviz;

    //helper functions --> can turn them into a package library (in ros package template)
    double to_radians(double degrees) {
        double radians;
        return radians = degrees*M_PI/180;
    }

    double to_degrees(double radians) {
        double degrees;
        return degrees = radians*180/M_PI;
    }

    int get_index (double angle_inquiry) {
        //angle_inquiry should follow the convention of the LIDAR and should be in radians
        const int initialindex = 0;//start of array
        int index = floor((abs(angle_inquiry-scan_message_localdata.angle_min)/scan_message_localdata.angle_increment))+initialindex;

        return index;
        
    }

    void preprocess_lidar_eliminate_bubble()
    {   

        //NOTE: vectorise your calculations instead of using a for loop!!

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        
        double closest_distance = scan_message_localdata.max_range;
        int closest_ind;
        int max_angle_ind = get_index(to_radians(VIEW_MAX_ANGLE));// for loop constraints set up like this to optimise memory and time
        //for loop to zero all infs and nans
        for (int i = (get_index(to_radians(VIEW_MIN_ANGLE))); i < max_angle_ind; i++) {
            if (std::isnan(scan_message_localdata.range_data[i])) {
                scan_message_localdata.range_data[i] = 0.0;
            } 
            
            else if (std::isinf(scan_message_localdata.range_data[i]) || (scan_message_localdata.range_data[i] > scan_message_localdata.max_look)) {
                scan_message_localdata.range_data[i] = scan_message_localdata.max_look;
            }
        }

        //setting each value to mean over some window
        for (int i = (get_index(to_radians(VIEW_MIN_ANGLE))); i < max_angle_ind; i++) {
            //windowsize = 5

            scan_message_localdata.range_data[i] = (scan_message_localdata.range_data[i-2]+scan_message_localdata.range_data[i-1]+scan_message_localdata.range_data[i]+scan_message_localdata.range_data[i+1]+scan_message_localdata.range_data[i+2])/5;
            if (scan_message_localdata.range_data[i] < closest_distance) {
                closest_distance = scan_message_localdata.range_data[i];
                closest_ind = i;
            }
        }

        RCLCPP_INFO (this->get_logger(), "closdist %f..... closind %d", closest_distance, closest_ind);
        
        //setting bubble to 0; 
        unsigned int radius = 150;
        for (unsigned int i = closest_ind - radius; i < (closest_ind + radius + 1); i++) {
            scan_message_localdata.range_data[i] = 0.0;
        }

        

    }

    void find_max_gap() //TODO: error is here - compare with lejung
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        
        
        //unsigned int current_start = (get_index(to_radians(VIEW_MIN_ANGLE))) - 1;
        // unsigned int gap = 0;
        // unsigned int longest_gap = 0;
        // unsigned int start;
        // bool index_start = false;

        // //loop variables
        // int max_angle_ind = get_index(to_radians(VIEW_MAX_ANGLE));

        // //RCLCPP_INFO (this->get_logger(), "%d", max_angle_ind);


        // //my algo:
        // for (int i = (get_index(to_radians(VIEW_MIN_ANGLE))); i < max_angle_ind; i++) {
        //     if (index_start == false) {
        //         if (scan_message_localdata.range_data[i] > 0.5) {
        //             index_start = true;
        //             start = i;
        //         }
        //     }


        //     else {
        //         if (scan_message_localdata.range_data[i] <= 0.5) {
        //             index_start = false;

        //             gap = i-start;

        //             if (longest_gap < gap) {
        //                 longest_gap = gap;
        //                 max_gap_indices[0] = start;
        //                 max_gap_indices[1] = i;

        //             }
        //         }
        //     }

        int max_angle_ind = get_index(to_radians(VIEW_MAX_ANGLE));
        int len_local = 0;
        int max_len = 0;
		int end_i = 0;

        for (int i = (get_index(to_radians(VIEW_MIN_ANGLE))); i < max_angle_ind; i++) {
            if (scan_message_localdata.range_data[i] >= 1.35) {
                len_local++;
            }
            else {
                len_local = 0;
            }
            if (len_local > max_len) {
                max_len = len_local;
                end_i = i;
            }
        }

        max_gap_indices[0] = end_i-max_len+1;
        max_gap_indices[1] = end_i;

        //might have to add an if statement outside this loop (look at lejung's code)


        //...............................................................................//

        //Lejung's code

        // unsigned int min_indx = get_index(to_radians(VIEW_MIN_ANGLE));
        // unsigned int max_indx = get_index(to_radians(VIEW_MIN_ANGLE));
        // unsigned int start = min_indx;
        // unsigned int end = min_indx;
        // unsigned int current_start = min_indx - 1;
        // unsigned int duration = 0;
        // unsigned int longest_duration = 0;
        // for (unsigned int i = min_indx; i <= max_indx; i++) {
        //     // ROS_INFO_STREAM("angles " << lidar_info.angle_min + i * lidar_info.angle_increment << " ranges " << ranges[i]);
        //     if (current_start < min_indx) {
        //         if (scan_message_localdata.range_data[i] > 0.0) {
        //             current_start = i;
        //         }
        //     } else if (scan_message_localdata.range_data[i] <= 0.0) {
        //         duration = i - current_start;
        //         if (duration > longest_duration) {
        //             longest_duration = duration;
        //             start = current_start;
        //             end = i - 1;
        //         }
                
        //         current_start = min_indx - 1;
        //     }
        // }
        // if (current_start >= min_indx) {
        //     duration = max_indx + 1 - current_start;
        //     if (duration > longest_duration) {
        //         longest_duration = duration;
        //         start = current_start;
        //         end = max_indx;
        //     }
        // }
        
        
        //max_gap_indices[0] = start;
        //max_gap_indices[1] = end;

    }

    int find_best_point()
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there

        double largest_distance = scan_message_localdata.range_data[max_gap_indices[0]];
        int largest_distance_ind = floor((max_gap_indices[0] + max_gap_indices[1])/2); //set it as mid-point as safety

        //RCLCPP_INFO (this->get_logger(), "%d", largest_distance_ind);

        for(int i = max_gap_indices[0]; i <= max_gap_indices[1]; ++i) {
            if (scan_message_localdata.range_data[i] > largest_distance) {
                largest_distance = scan_message_localdata.range_data[i];
                largest_distance_ind = i;
            }

            //TODO: add else-if block to deal with equality condition and select point with lower steering angle
        }
        
        RCLCPP_INFO (this->get_logger(), "maxgapstart %d..... maxgapend %d", max_gap_indices[0], max_gap_indices[1]);


        return largest_distance_ind;
    }

    double get_velocity(double steering_angle) {

        double velocity;

        if (abs(steering_angle) > to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            velocity = 1.5;
        } 
        else if (abs(steering_angle) > to_radians(10.0) && abs(steering_angle) < to_radians(20.0)) {
            velocity = 1.0;
        } 
        else {
            velocity = 0.5;
        }

        return velocity;
    }

    void publish_message (int largest_distance_ind) {
        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msgObj.drive.steering_angle = scan_message_localdata.angle_min+(largest_distance_ind*scan_message_localdata.angle_increment);//TODO: ask about angle
        

        RCLCPP_INFO (this->get_logger(), "driveflag: %s", (drive_flag ? "true" : "false"));

        if (drive_flag == false) {
            drive_msgObj.drive.speed = 0.0;
        }
        else {drive_msgObj.drive.speed = get_velocity(drive_msgObj.drive.steering_angle);}
        publisher_drive->publish(drive_msgObj);
        //RCLCPP_INFO (this->get_logger(), "%f ..... %f", drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle));
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_submsgObj) {   
        //copy relevant data to struct
        scan_message_localdata.angle_min = scan_submsgObj->angle_min;
        scan_message_localdata.angle_increment = scan_submsgObj->angle_increment;
        scan_message_localdata.range_data = scan_submsgObj->ranges;
        scan_message_localdata.max_range = scan_submsgObj->range_max;
        
        //pack rviz_debug message object with data
        auto rviz_debugObj = sensor_msgs::msg::LaserScan();

        // rviz_debugObj.header = scan_submsgObj->header;
        // rviz_debugObj.angle_min = scan_submsgObj->angle_min;
        // rviz_debugObj.angle_max = scan_submsgObj->angle_max;
        // rviz_debugObj.angle_increment = scan_submsgObj->angle_increment;
        // rviz_debugObj.time_increment = scan_submsgObj->time_increment;
        // rviz_debugObj.scan_time = scan_submsgObj->scan_time;
        // rviz_debugObj.range_min = scan_submsgObj->range_min;
        // rviz_debugObj.range_max = scan_submsgObj->range_max;
        //left intensity field empty


        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 
        preprocess_lidar_eliminate_bubble();

        // Find max length gap 
        find_max_gap();        

        // Find the best point in the gap 
        int largest_index = find_best_point();

        //TODO: might fail here due to index error
        
        //packing rviz debug range data 
        rviz_debugObj.ranges = scan_message_localdata.range_data;
        
        for (int i = 0; i<1079; i++) {
            rviz_debugObj.ranges[i] = 0.0;//might have to mark as NaN
            //rviz_debugObj.intensities[i] = 0.0;
        }

        for (int i = max_gap_indices[0]; i <= max_gap_indices[1]; i++) {
            rviz_debugObj.ranges[i] = scan_message_localdata.range_data[i];
            //rviz_debugObj.intensities[i] = 8.0;
        }
        
        publisher_rviz->publish(rviz_debugObj);

        // Publish Drive message
        publish_message(largest_index);

    }



};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<ReactiveFollowGap>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}

// there is a global queue that stores all the triggers (i.e. everytime a message is recieved, a 'callback' is added to the queue)
// rosspin() runs a loop that dequeues and executes these callbacks at a particular rate.

//why do we find the closest point and set to 0 first? If we're finding the farthest point anyway, how does this change anything?
//what is the 'better method' from the lecture
//how do we convert pid to angle
//how to convert to angle over here? do we send the message in the car's reference frame or the LIDAR's reference frame?
//how to pass smart pointer objects through arguments in functions? by reference? using the <memory> class? 

/* TODO:
    try different variations! 
    - UNC tweak, etc. 

*/

/*
    resources: 
    https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html
    https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html
    https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/


*/