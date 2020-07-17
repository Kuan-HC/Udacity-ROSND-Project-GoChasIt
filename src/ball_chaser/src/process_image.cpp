#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

#define left_boundary  300U
#define right_boundary 500U
#define image_midoint  400U
#define ball_pixels  50000U

enum move_state
{
    Standby,
    Turn_R,
    Turn_L,
    Forward
};

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the drive_bot service and pass the requested velocity and  yaw rate
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    const int white_pixel = 255;
    static bool initialized = false;
    static unsigned int data_langth;

    if (initialized == false)
    {
        ROS_INFO(" Image Height: %i,  Width : %i", img.height, img.width);  
        ROS_INFO(" Step: %i", img.step);  
        data_langth = img.step * img.height;
        ROS_INFO(" Data_length: %i", data_langth);
        initialized = true;  
        ROS_INFO("Ready to receive image from camera");     
    }
    /***********************************************************************************
     * Height:800 Width:800 Step: 2400 (Full row length in byte)
     * Even though this sensor_msgs/Image short of detailed explanation in
     * http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html
     * Presume that step contents three layers(R,G,B) in one row, which means
     * data[0]:first pixel   data[3]: second pixel data[6]: third pixel and so on
     * ********************************************************************************/
        
    /***********************************************************************************
     *  Analyze received image, it shows that white ball would appear only in the middle.
     *  Therefore, upper and bottom part can be neglect.
     *  In this project, rows in range 3/8 to 5/8 are took into account.
     *  Count the number of white pixels and sum of white pixels x positon to calculate
     *  white area center position.
     *  Following instructure implement this
     * *********************************************************************************/
    static const unsigned int start_pixel = data_langth*3/8;
    static const unsigned int end_pixel = data_langth*5/8;
    
    /*************************************************************************************
     * For Gazebo Env, just consider sole RGB layer would be enough
     * **********************************************************************************/
    unsigned long int pixel_x_pos_sum = 0U;
    unsigned int pixel_num_sum = 0U;
    unsigned int center_x_pos = 0U;
    static unsigned char  action;

    for (unsigned int index = start_pixel; index <= end_pixel; index += 3U)
    {
        if(img.data[index] == white_pixel)
        {
            pixel_x_pos_sum += (index%2400U)/3U;
            pixel_num_sum++;
        }
    }
    if(pixel_num_sum != 0U)
    {
        center_x_pos = pixel_x_pos_sum / pixel_num_sum;
        //ROS_INFO("White area center position %i, Number of white pixels: %i ",center_x_pos,pixel_num_sum);  /* for tunning */
        switch(action)
        {
            case Standby:
                if(center_x_pos < left_boundary)
                    action = Turn_L;
                else if(center_x_pos > right_boundary)
                    action = Turn_R;
                else if(pixel_num_sum < ball_pixels)
                     action = Forward;
                else
                    /* Do nothing  stay in this state*/
                break;

            case Turn_R:
                if(center_x_pos <= image_midoint)
                {
                    if(pixel_num_sum < ball_pixels)
                        action = Forward;
                    else
                        action = Standby;
                }
                break;

            case Turn_L:
                if(center_x_pos >= image_midoint)
                {
                    if(pixel_num_sum < ball_pixels)
                        action = Forward;
                    else
                        action = Standby;
                }
                break;

            case Forward:
                if(center_x_pos > right_boundary)
                    action = Turn_R;

                else if(center_x_pos < left_boundary)
                    action = Turn_L;

                else if(pixel_num_sum >= ball_pixels)    
                    action = Standby;
                break;
        }

        switch(action)
        {
            case Standby:
                drive_robot( 0.0f, 0.0f);
                break;
            case Turn_R:
                drive_robot( 0.0f, -0.2f);
                break;
            case Turn_L:
                drive_robot( 0.0f, 0.2f);
                break;
            case Forward:
                drive_robot( 0.5f, 0.0f);
                break;
        }

    }
    else
         drive_robot( 0.0f, 0.0f);  /* whit ball not in image stop*/
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
