#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Eigen>
#include <stdio.h>

/*
 * this function is to get desired states for specific trajectory, just generated, at time dt.
 * input:
 * dT   -> the time
 * hover_pos -> the desired position where you want quadrotor to hover
 * now_vel -> maybe useless
 *
 * output:
 * desired_pos -> desired position at dT
 * desired_vel -> desired velocity at dT
 * desired_acc -> desired acceleration at dT
 * return:
 * true  -> you have alread configured desired states
 * false -> no desired state
 */
bool
trajectory_control( const double dT,
                    const Eigen::Vector3d hover_pos,
                    const Eigen::Vector3d now_vel,
                    double& end_time,
                    Eigen::Vector3d& desired_pos,
                    Eigen::Vector3d& desired_vel,
                    Eigen::Vector3d& desired_acc )
{
    // if you don't want to use Eigen, then you can use these arrays
    // or you can delete them and use Eigen
    double hover_p[3], now_v[3];
    double desired_p[3], desired_v[3], desired_a[3];
    hover_p[0] = hover_pos.x( );
    hover_p[1] = hover_pos.y( );
    hover_p[2] = hover_pos.z( );
    now_v[0]   = now_vel.x( );
    now_v[1]   = now_vel.y( );
    now_v[2]   = now_vel.z( );
    // your code // please use coefficients from matlab to get desired states
  
    const int R = 8; // Using R - 1 th order polynomial for the trajectory
    const int v_limit = 0.5;  // maximum velocity
    const int W = 9; // number of waypoints

    printf( "x = %.2f, y = %.2f, z = %.2f\n", desired_p[0], desired_p[1], desired_p[2] );

    /*---------------------------------------------------------------------------------*/
    /*--- YOUR CODE FROM HERE --- YOUR CODE FROM HERE --- YOUR CODE FROM HERE ---------*/
    /*---------------------------------------------------------------------------------*/
    //	printf("dt= %.2f\n",dT);
    double max_run_time = 25.0; // set your trajectory max run time.
    double end_position[3];     // set your trajectory end point.

    end_position[0] = 0.0;
    end_position[1] = 2.0;
    end_position[2] = 0.5;
    
    const double path1[3*W] = { 0.0, 0.0, 0.5, \    //           /
                               -0.2, 0.2, 0.7, \    //          /
                               -0.5, 0.5, 0.9, \    //         /
                               -0.8, 0.8, 1.0, \    //        /
                               -1.0, 1.0, 1.0, \    //       (
                               -0.8, 1.2, 1.0, \    //        \
                               -0.5, 1.5, 0.9, \    //         \
                               -0.2, 1.8, 0.7, \    //          \
                                0.0, 2.0, 0.5  };   //           \

    const double T[W] = {0.0, 0.125*25, 0.25*25, 0.375*25, 0.5*25, 0.625*25, 0.75*25, 0.875*25, 25.0};
  
    const double P_x[R*(W-1)] = { 0.0000, 0.0000, 0.0000, 0.0000,-0.0104, 0.0045,-0.0007, 0.0000, \
                                 -0.2000,-0.1364,-0.0014, 0.0105,-0.0011,-0.0006, 0.0002, 0.0000, \
                                 -0.5000,-0.0660,-0.0047,-0.0050, 0.0009, 0.0002,-0.0000, 0.0000, \
                                 -0.8000,-0.1115, 0.0060, 0.0041,-0.0002,-0.0001,-0.0000, 0.0000, \
                                 -1.0000, 0.0000, 0.0226, 0.0000,-0.0001, 0.0000,-0.0000, 0.0000, \
                                 -0.8000, 0.1115, 0.0060,-0.0041,-0.0002, 0.0001,-0.0000, 0.0000, \
                                 -0.5000, 0.0660,-0.0047, 0.0050, 0.0009,-0.0002,-0.0000, 0.0000, \
                                 -0.2000, 0.1364,-0.0014,-0.0105,-0.0011, 0.0006, 0.0002, 0.0000, };
  
    const double P_y[R*(W-1)] = { 0.0000, 0.0000, 0.0000, 0.0000, 0.0105,-0.0045, 0.0007, 0.0000, \
                                  0.2000, 0.1358, 0.0010,-0.0105, 0.0012, 0.0006,-0.0002, 0.0000, \
                                  0.5000, 0.0685, 0.0055, 0.0047,-0.0010,-0.0002,-0.0000, 0.0000, \
                                  0.8000, 0.1040,-0.0089,-0.0036, 0.0006, 0.0001,-0.0000, 0.0000, \
                                  1.0000, 0.0348, 0.0000, 0.0041,-0.0000,-0.0001,-0.0000, 0.0000, \
                                  1.2000, 0.1040, 0.0089,-0.0036,-0.0010, 0.0001,-0.0000, 0.0000, \
                                  1.5000, 0.0685,-0.0055, 0.0047, 0.0010,-0.0002,-0.0000, 0.0000, \
                                  1.8000, 0.1358,-0.0010,-0.0105,-0.0012, 0.0006, 0.0002, 0.0000, };
  
    const double P_z[R*(W-1)] = { 0.5000, 0.0000, 0.0000, 0.0000, 0.0113,-0.0050, 0.0008, 0.0000, \
                                  0.7000, 0.1274,-0.0063,-0.0117, 0.0016, 0.0007,-0.0002, 0.0000, \
                                  0.9000, 0.0189, 0.0020, 0.0048,-0.0013,-0.0002, 0.0000, 0.0000, \
                                  1.0000, 0.0242,-0.0115,-0.0014, 0.0010, 0.0000, 0.0000, 0.0000, \
                                  1.0000, 0.0000, 0.0064, 0.0000,-0.0010, 0.0000, 0.0000, 0.0000, \
                                  1.0000,-0.0242,-0.0115, 0.0014, 0.0010, 0.0000, 0.0000, 0.0000, \
                                  0.9000,-0.0189, 0.0020,-0.0048,-0.0013, 0.0002, 0.0000, 0.0000, \
                                  0.7000,-0.1274,-0.0063, 0.0117, 0.0016,-0.0007,-0.0002, 0.0000, };
  
    //int i;  The segment where dT is 0~7
    for (int i = 0; i < W; i++) 
    {
      if (dT >= T[i] && dT < T[i+1]) 
      {
        break;
      }
    }
    for (int j = 0; j < R; j++)
    {
      desired_p[0] = desired_p[0] + P_x[j+i*R]*pow((dT- T[i]), j);
      desired_p[1] = desired_p[1] + P_y[j+i*R]*pow((dT- T[i]), j);
      desired_p[2] = desired_p[2] + P_z[j+i*R]*pow((dT- T[i]), j);
      if (j==0)
      {
        desired_v[0] = 0;
        desired_v[1] = 0;
        desired_v[2] = 0;
      }
      if (j>0)
      {
        desired_v[0] = desired_v[0] + j*P_x[j+i*R]*pow((dT- T[i]), (j-1));
        desired_v[1] = desired_v[1] + j*P_y[j+i*R]*pow((dT- T[i]), (j-1));
        desired_v[2] = desired_v[2] + j*P_z[j+i*R]*pow((dT- T[i]), (j-1));
      }
      desired_a[0] = 0;
      desired_a[1] = 0;
      desired_a[2] = 0;
    
    /*---------------------------------------------------------------------------------*/
    /*---- YOUR CODE END HERE ---- YOUR CODE END HERE ---- YOUR CODE END HERE ---------*/
    /*---------------------------------------------------------------------------------*/
    if ( dT < max_run_time )
    {
        // output trajectory
        desired_pos.x( ) = desired_p[0];
        desired_pos.y( ) = desired_p[1];
        desired_pos.z( ) = desired_p[2];
        desired_vel.x( ) = desired_v[0];
        desired_vel.y( ) = desired_v[1];
        desired_vel.z( ) = desired_v[2];
        desired_acc.x( ) = desired_a[0];
        desired_acc.y( ) = desired_a[1];
        desired_acc.z( ) = desired_a[2];
        end_time         = max_run_time;
      
      
      std::cout << "Time: " << dT << ".\n";
      std::cout << "x: " << desired_p[0] << "\n";
      std::cout << "y: " << desired_p[1] << "\n";
      std::cout << "z: " << desired_p[2] << "\n";
      std::cout << "pos_x: " << hover_p[0] << "\n";
      std::cout << "pos_y: " << hover_p[1] << "\n";
      std::cout << "pos_z: " << hover_p[2] << "\n";
      
      
        return true; // if you have got desired states, true.
    }
    else
    {
        // output end point trajectory
        desired_pos.x( ) = end_position[0];
        desired_pos.y( ) = end_position[1];
        desired_pos.z( ) = end_position[2];
        desired_vel.x( ) = 0.0;
        desired_vel.y( ) = 0.0;
        desired_vel.z( ) = 0.0;
        desired_acc.x( ) = 0.0;
        desired_acc.y( ) = 0.0;
        desired_acc.z( ) = 0.0;
        return false; // if you have got desired states, true.
    }
}

#endif // TRAJECTORY_H
