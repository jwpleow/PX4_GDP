#include "headers/gdpdrone.h"
#include "Dstar.h"
#include <iostream>

int main(int argc, char **argv) {

        //initialize(argc,argv,drone);

        ros::init(argc, argv, "B00lake_node");

        GDPdrone drone;


    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 50.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 1.5;
    int time_takeoff = 100;
    drone.Commands.request_Takeoff(altitude, time_takeoff);


        Dstar* dstar = new Dstar();
    list<state> mypath;

        float saf=7.0;
        int gx=60;
        int gy=20;
        //float sens[3];
        float k=1.0;
        //sens[0]=drone.Data.lidar.ranges[0];
        //sens[1]=drone.Data.lidar.ranges[1];
        //sens[2]=drone.Data.lidar.ranges[2];

    dstar->init(0, 0, gx, gy);

        int xst=0,yst=0;

        while(!(xst == gx && yst == gy)) {
        dstar->replan();           // plan a path
        mypath = dstar->getPath(); // retrieve path

                for(auto& v : mypath) {
            dstar->updateStart(v.x, v.y); // move start to new path point
            for(int j = 0; j < 10; j++) {
                drone.Commands.move_Position_Local((-v.y+yst), (v.x-xst), 0, 0, "BODY_OFFSET", j);
                ros::spinOnce();
                rate.sleep();
            }
            xst = v.x;
            yst = v.y;

                ROS_INFO("vx [%i]",v.x);
                ROS_INFO("vy [%i]",v.y);
            //std::cout << v.x << " " << v.y << '\n';

                        //if (v.x==3 && v.y==1) sens[0]=0.4;
                        if (drone.Data.lidar.ranges[0]<k) {
                            for(int j = 0; j < 10; j++) {
                                drone.Commands.move_Position_Local(0, 0, 0, 0, "BODY_OFFSET", j);
                                ros::spinOnce();
                                rate.sleep();
                            }
                                while (drone.Data.lidar.ranges[0]<k) {
                    yst++;
                    dstar->updateStart(xst,yst);      // move start to new path point
                    for(int j = 0; j < 10; j++) {
                        drone.Commands.move_Position_Local((-1), 0, 0, 0, "BODY_OFFSET", j);
                        ros::spinOnce();
                        rate.sleep();
                    }//Move the actual drone
                                        //if (yst>3) sens[0]=1;
                }
                                yst+=saf;
                for(int j = 0; j < 10; j++) {
                    drone.Commands.move_Position_Local((-saf), 0, 0, 0, "BODY_OFFSET", j);
                    ros::spinOnce();
                    rate.sleep();
                }
                                dstar->updateStart(xst,yst);
                                for (int i=1;i<=4;i++) {
                                        xst++;
                                        for(int j = 0; j < 10; j++) {
                                            drone.Commands.move_Position_Local(0, 1, 0, 0, "BODY_OFFSET", j);
                                            ros::spinOnce();
                                            rate.sleep();
                                        }
                                        dstar->updateStart(xst,yst);
                                        while (drone.Data.lidar.ranges[0]<k || drone.Data.lidar.ranges[1]<k ||drone.Data.lidar.ranges[2]<k) {
                                                        yst++;
                                                        dstar->updateStart(xst,yst);      // move start to new path point
                                                        for(int j = 0; j < 10; j++) {
                                                            drone.Commands.move_Position_Local((-1), 0, 0, 0, "BODY_OFFSET", j);
                                                            ros::spinOnce();
                                                            rate.sleep();
                                                        }//Move the actual drone
                                        }
                                }
                                break;
                        }
                        else if (drone.Data.lidar.ranges[1]<k) {
                for(int j = 0; j < 10; j++) {
                    drone.Commands.move_Position_Local(0, 0, 0, 0, "BODY_OFFSET", j);
                    ros::spinOnce();
                    rate.sleep();
                }
                while (drone.Data.lidar.ranges[1]<k) {
                    yst++;
                    dstar->updateStart(xst,yst);      // move start to new path point
                    for(int j = 0; j < 10; j++) {
                        drone.Commands.move_Position_Local((-1), 0, 0, 0, "BODY_OFFSET", j);
                        ros::spinOnce();
                        rate.sleep();
                    }//Move the actual drone
                }
                                yst+=saf;
                for(int j = 0; j < 10; j++) {
                    drone.Commands.move_Position_Local((-saf), 0, 0, 0, "BODY_OFFSET", j);
                    ros::spinOnce();
                    rate.sleep();
                }
                                dstar->updateStart(xst,yst);
                                for (int i=1;i<=4;i++) {
                                        xst++;
                                        for(int j = 0; j < 10; j++) {
                                            drone.Commands.move_Position_Local(0, 1, 0, 0, "BODY_OFFSET", j);
                                            ros::spinOnce();
                                            rate.sleep();
                                        }
                                        dstar->updateStart(xst,yst);
                                        while (drone.Data.lidar.ranges[0]<k || drone.Data.lidar.ranges[1]<k ||drone.Data.lidar.ranges[2]<k) {
                                                        yst++;
                                                        dstar->updateStart(xst,yst);      // move start to new path point
                                                        for(int j = 0; j < 10; j++) {
                                                            drone.Commands.move_Position_Local((-1), 0, 0, 0, "BODY_OFFSET", j);
                                                            ros::spinOnce();
                                                            rate.sleep();
                                                        }//Move the actual drone
                                        }
                                }
                break;

            }
                        else if (drone.Data.lidar.ranges[2]<k) {
                for(int j = 0; j < 10; j++) {
                    drone.Commands.move_Position_Local(0, 0, 0, 0, "BODY_OFFSET", j);
                    ros::spinOnce();
                    rate.sleep();
                }
                while (drone.Data.lidar.ranges[2]<k) {
                    yst--;
                    dstar->updateStart(xst,yst);      // move start to new path point
                    for(int j = 0; j < 10; j++) {
                        drone.Commands.move_Position_Local(1, 0, 0, 0, "BODY_OFFSET", j);
                        ros::spinOnce();
                        rate.sleep();
                    }//Move the actual drone
                }
                                yst-=saf;
               for(int j = 0; j < 10; j++) {
                    drone.Commands.move_Position_Local((saf), 0, 0, 0, "BODY_OFFSET", j);
                    ros::spinOnce();
                    rate.sleep();
                }
                                dstar->updateStart(xst,yst);
                                for (int i=1;i<=4;i++) {
                                        xst++;
                                        for(int j = 0; j < 10; j++) {
                                            drone.Commands.move_Position_Local(0, 1, 0, 0, "BODY_OFFSET", j);
                                            ros::spinOnce();
                                            rate.sleep();
                                        }
                                        dstar->updateStart(xst,yst);
                                        while (drone.Data.lidar.ranges[0]<k || drone.Data.lidar.ranges[1]<k ||drone.Data.lidar.ranges[2]<k) {
                                                        yst--;
                                                        dstar->updateStart(xst,yst);      // move start to new path point
                                                        for(int j = 0; j < 10; j++) {
                                                            drone.Commands.move_Position_Local(1, 0, 0, 0, "BODY_OFFSET", j);
                                                            ros::spinOnce();
                                                            rate.sleep();
                                                        }//Move the actual drone
                                        }
                                }
                break;

            }
            }
        }

    // dstar->updateGoal(0,1);        // move goal to (0,1)
    drone.Commands.request_LandingAuto();
    return 0;
}
