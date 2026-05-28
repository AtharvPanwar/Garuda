#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/camera.h>
#include "pid_controller.h"

// =========================================================================
// ======                     MISSION CONFIGURATION                   ======
// =========================================================================
#define DEST_X       1.0   // Edit this to change your target X coordinate
#define DEST_Y      -2.0   // Edit this to change your target Y coordinate
#define DEST_Z       0.1   // Edit this to change your final target Z altitude
#define CRUISE_ALT   2.0   // Edit this to change the mid-air flight cruising altitude
// =========================================================================

#define STEP_SIZE    1.0

typedef struct { double x, y, z; } Point;

Point waypoints[50];
int total_waypoints = 0;
int current_wp_idx = 0;

int to_grid(double val) { return (int)round(val); }

void plan_path(double sx, double sy, double ex, double ey, double cruise_alt, double dest_z) {
    total_waypoints = 0;
    waypoints[total_waypoints++] = (Point){sx, sy, cruise_alt};
    double cx = sx;
    while (fabs(cx - ex) > 0.1) {
        cx += (ex > cx) ? STEP_SIZE : -STEP_SIZE;
        waypoints[total_waypoints++] = (Point){cx, sy, cruise_alt};
    }
    double cy = sy;
    while (fabs(cy - ey) > 0.1) {
        cy += (ey > cy) ? STEP_SIZE : -STEP_SIZE;
        waypoints[total_waypoints++] = (Point){ex, cy, cruise_alt};
    }
    waypoints[total_waypoints++] = (Point){ex, ey, dest_z};
}

double clamp(double v, double min, double max) {
    if (v > max) return max;
    if (v < min) return min;
    return v;
}

int main(int argc, char **argv) {
  wb_robot_init();
  const int timestep = (int)wb_robot_get_basic_time_step();

  // Devices
  WbDeviceTag motors[4];
  char m_names[4][10] = {"m1_motor", "m2_motor", "m3_motor", "m4_motor"};
  for (int i = 0; i < 4; i++) {
    motors[i] = wb_robot_get_device(m_names[i]);
    wb_motor_set_position(motors[i], INFINITY);
  }

  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);

  WbDeviceTag cam_up = wb_robot_get_device("camera-up");
  WbDeviceTag cam_down = wb_robot_get_device("camera-down");
  if (cam_up) wb_camera_enable(cam_up, timestep);
  if (cam_down) wb_camera_enable(cam_down, timestep);

  // Configuration Folders
  const char *path_up = "/home/atharv-panwar/my_vision_project/my_project2/controllers/autonomus/images/camera-up";
  const char *path_down = "/home/atharv-panwar/my_vision_project/my_project2/controllers/autonomus/images/camera-down";
  const char *vid_path_up = "/home/atharv-panwar/my_vision_project/my_project2/controllers/autonomus/video/camera-up";
  const char *vid_path_down = "/home/atharv-panwar/my_vision_project/my_project2/controllers/autonomus/video/camera-down";

  wb_robot_step(timestep); 
  const double *start_gps = wb_gps_get_values(gps);
  
  // Path planned using the configurable targets specified at the top
  plan_path(start_gps[0], start_gps[1], DEST_X, DEST_Y, CRUISE_ALT, DEST_Z); 

  double past_time = wb_robot_get_time();
  double last_photo_time = -2.0; // Ensures immediate first photo execution
  double past_x = start_gps[0], past_y = start_gps[1];
  
  // --- VIDEO ENCODER STATE VARIABLES ---
  double video_start_time = 0.0;
  double video_start_x = start_gps[0], video_start_y = start_gps[1], video_start_z = start_gps[2];
  FILE *video_pipe_up = NULL;
  FILE *video_pipe_down = NULL;
  int video_initialized = 0;

  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
  motor_power_t motor_power;
  gains_pid_t gains = { .kp_att_y=1,.kd_att_y=0.5,.kp_att_rp=0.5,.kd_att_rp=0.1,
                        .kp_vel_xy=2,.kd_vel_xy=0.5,.kp_z=6.0,.ki_z=5,.kd_z=5 };
  
  init_pid_attitude_fixed_height_controller();

  // --- PRINT TABLE HEADER ---
  printf("+-------------------+---------+---------+---------+-------------+-------------+------------+---------------------------------------+\n");
  printf("| event_type        | pos_x   | pos_y   | pos_z   | waypoint_gx | waypoint_gy | cam_orient | saved_filename                        |\n");
  printf("+-------------------+---------+---------+---------+-------------+-------------+------------+---------------------------------------+\n");
  
  // --- PRINT FLIGHT START EVENT ---
  printf("| %-17s | %7s | %7s | %7s | %11s | %11s | %-10s | %-37s |\n",
         "FLIGHT_START", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL");

  while (wb_robot_step(timestep) != -1) {
    const double current_time = wb_robot_get_time();
    double dt = current_time - past_time;
    if (dt <= 0) continue;

    const double *gps_val = wb_gps_get_values(gps);
    const double *imu_val = wb_inertial_unit_get_roll_pitch_yaw(imu);
    
    actual_state.altitude = gps_val[2];
    actual_state.roll = imu_val[0];
    actual_state.pitch = imu_val[1];
    actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];
    
    double vx_g = (gps_val[0] - past_x) / dt;
    double vy_g = (gps_val[1] - past_y) / dt;
    actual_state.vx = vx_g * cos(imu_val[2]) + vy_g * sin(imu_val[2]);
    actual_state.vy = -vx_g * sin(imu_val[2]) + vy_g * cos(imu_val[2]);

    // --- CONTINUOUS 10-SECOND VIDEO CHUNKER ---
    if (!video_initialized || (current_time - video_start_time >= 10.0)) {
      // Close out previous active chunks and rename them using precise final metadata values
      if (video_initialized) {
        char old_vpath[1024], new_vpath[1024];
        char video_file_name[512];
        char sx[16], sy[16], sz[16];

        sprintf(sx, "%.2f", gps_val[0]);
        sprintf(sy, "%.2f", gps_val[1]);
        sprintf(sz, "%.2f", gps_val[2]);
        
        if (cam_up && video_pipe_up) {
          pclose(video_pipe_up);
          video_pipe_up = NULL;
          sprintf(old_vpath, "%s/.tmp_up.mp4", vid_path_up);
          sprintf(video_file_name, "camera_up=%.2f,%.2f,%.2f,%.2f_to_%.2f,%.2f,%.2f,%.2f.mp4",
                  video_start_time, video_start_x, video_start_y, video_start_z,
                  current_time, gps_val[0], gps_val[1], gps_val[2]);
          sprintf(new_vpath, "%s/%s", vid_path_up, video_file_name);
          rename(old_vpath, new_vpath);

          printf("| %-17s | %7s | %7s | %7s | %11s | %11s | %-10s | %-37s |\n",
                 "VIDEO_RECORD_CHUNK", sx, sy, sz, "NULL", "NULL", "UP", video_file_name);
        }
        
        if (cam_down && video_pipe_down) {
          pclose(video_pipe_down);
          video_pipe_down = NULL;
          sprintf(old_vpath, "%s/.tmp_down.mp4", vid_path_down);
          sprintf(video_file_name, "camera_down=%.2f,%.2f,%.2f,%.2f_to_%.2f,%.2f,%.2f,%.2f.mp4",
                  video_start_time, video_start_x, video_start_y, video_start_z,
                  current_time, gps_val[0], gps_val[1], gps_val[2]);
          sprintf(new_vpath, "%s/%s", vid_path_down, video_file_name);
          rename(old_vpath, new_vpath);

          printf("| %-17s | %7s | %7s | %7s | %11s | %11s | %-10s | %-37s |\n",
                 "VIDEO_RECORD_CHUNK", sx, sy, sz, "NULL", "NULL", "DOWN", video_file_name);
        }
      }

      // Update baseline coordinate stamps for the next 10-second segment
      video_start_time = current_time;
      video_start_x = gps_val[0];
      video_start_y = gps_val[1];
      video_start_z = gps_val[2];
      video_initialized = 1;

      // Calculate execution frame rates explicitly based on simulator's basic time step
      int fps = (int)round(1000.0 / timestep);
      if (fps <= 0) fps = 30;

      // Launch fresh background asynchronous FFmpeg pipes
      char cmd_buffer[2048];
      if (cam_up) {
        int w = wb_camera_get_width(cam_up);
        int h = wb_camera_get_height(cam_up);
        sprintf(cmd_buffer, "ffmpeg -y -f rawvideo -pixel_format bgra -video_size %dx%d -framerate %d -i - -c:v libx264 -pix_fmt yuv420p \"%s/.tmp_up.mp4\" > /dev/null 2>&1", w, h, fps, vid_path_up);
        video_pipe_up = popen(cmd_buffer, "w");
      }
      if (cam_down) {
        int w = wb_camera_get_width(cam_down);
        int h = wb_camera_get_height(cam_down);
        sprintf(cmd_buffer, "ffmpeg -y -f rawvideo -pixel_format bgra -video_size %dx%d -framerate %d -i - -c:v libx264 -pix_fmt yuv420p \"%s/.tmp_down.mp4\" > /dev/null 2>&1", w, h, fps, vid_path_down);
        video_pipe_down = popen(cmd_buffer, "w");
      }
    }

    // --- STREAM LIVE FRAMES DIRECTLY INTO THE ENCODER PIPES ---
    if (cam_up && video_pipe_up) {
      const unsigned char *img = wb_camera_get_image(cam_up);
      if (img) {
        int w = wb_camera_get_width(cam_up);
        int h = wb_camera_get_height(cam_up);
        fwrite(img, 4, w * h, video_pipe_up);
      }
    }
    if (cam_down && video_pipe_down) {
      const unsigned char *img = wb_camera_get_image(cam_down);
      if (img) {
        int w = wb_camera_get_width(cam_down);
        int h = wb_camera_get_height(cam_down);
        fwrite(img, 4, w * h, video_pipe_down);
      }
    }

    // --- VISION CAPTURE (Every 2.0 Seconds Still-Images) ---
    if (current_time - last_photo_time >= 2.0) {
      char full_path[1024];
      char file_name[256];
      char sx[16], sy[16], sz[16];

      sprintf(sx, "%.2f", gps_val[0]);
      sprintf(sy, "%.2f", gps_val[1]);
      sprintf(sz, "%.2f", gps_val[2]);

      if (cam_up) {
        sprintf(file_name, "camera_up=%.2f,%.2f,%.2f_%.2f.jpeg", gps_val[0], gps_val[1], gps_val[2], current_time);
        sprintf(full_path, "%s/%s", path_up, file_name);
        wb_camera_save_image(cam_up, full_path, 100);
        
        printf("| %-17s | %7s | %7s | %7s | %11s | %11s | %-10s | %-37s |\n",
               "CAMERA_CAPTURE", sx, sy, sz, "NULL", "NULL", "UP", file_name);
      }

      if (cam_down) {
        sprintf(file_name, "camera_down=%.2f,%.2f,%.2f_%.2f.jpeg", gps_val[0], gps_val[1], gps_val[2], current_time);
        sprintf(full_path, "%s/%s", path_down, file_name);
        wb_camera_save_image(cam_down, full_path, 100);
        
        printf("| %-17s | %7s | %7s | %7s | %11s | %11s | %-10s | %-37s |\n",
               "CAMERA_CAPTURE", sx, sy, sz, "NULL", "NULL", "DOWN", file_name);
      }

      // Lock to execution step interval alignment to avoid time drift
      last_photo_time = current_time;
    }

    // --- WAYPOINT NAVIGATION ---
    Point target = waypoints[current_wp_idx];
    double dist = sqrt(pow(target.x - gps_val[0], 2) + pow(target.y - gps_val[1], 2));
    double alt_err = fabs(target.z - gps_val[2]);
    
    if (dist < 0.25 && alt_err < 0.15) {
        if (current_wp_idx < total_waypoints - 1) {
            char sz_alt[16], swgx[16], swgy[16];
            
            sprintf(sz_alt, "%.2f", waypoints[current_wp_idx].z);
            sprintf(swgx, "%d", to_grid(waypoints[current_wp_idx].x));
            sprintf(swgy, "%d", to_grid(waypoints[current_wp_idx].y));

            printf("| %-17s | %7s | %7s | %7s | %11s | %11s | %-10s | %-37s |\n",
                   "WAYPOINT_REACHED", "NULL", "NULL", sz_alt, swgx, swgy, "NULL", "NULL");

            current_wp_idx++;
        }
    }

    desired_state.vx = (target.x - gps_val[0]) * 0.5;
    desired_state.vy = (target.y - gps_val[1]) * 0.5;
    desired_state.altitude = target.z;
    desired_state.yaw_rate = 0;

    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains, dt, &motor_power);
    wb_motor_set_velocity(motors[0], clamp(-motor_power.m1, -600, 600));
    wb_motor_set_velocity(motors[1], clamp(motor_power.m2, -600, 600));
    wb_motor_set_velocity(motors[2], clamp(-motor_power.m3, -600, 600));
    wb_motor_set_velocity(motors[3], clamp(motor_power.m4, -600, 600));

    past_time = current_time;
    past_x = gps_val[0]; past_y = gps_val[1];
  }
  
  // --- SHUTDOWN CLEANUP ---
  const double final_time = wb_robot_get_time();
  const double *final_gps = wb_gps_get_values(gps);
  char final_old_path[1024], final_new_path[1024];
  char final_video_name[512];
  char fsx[16], fsy[16], fsz[16];

  sprintf(fsx, "%.2f", final_gps[0]);
  sprintf(fsy, "%.2f", final_gps[1]);
  sprintf(fsz, "%.2f", final_gps[2]);

  // Only commit if the video was initialized and has actual runtime frames written to it
  if (video_initialized && (final_time > video_start_time)) {
    if (cam_up && video_pipe_up) {
      pclose(video_pipe_up);
      sprintf(final_old_path, "%s/.tmp_up.mp4", vid_path_up);
      sprintf(final_video_name, "camera_up=%.2f,%.2f,%.2f,%.2f_to_%.2f,%.2f,%.2f,%.2f.mp4",
              video_start_time, video_start_x, video_start_y, video_start_z,
              final_time, final_gps[0], final_gps[1], final_gps[2]);
      sprintf(final_new_path, "%s/%s", vid_path_up, final_video_name);
      rename(final_old_path, final_new_path);

      printf("| %-17s | %7s | %7s | %7s | %11s | %11s | %-10s | %-37s |\n",
             "VIDEO_RECORD_CHUNK", fsx, fsy, fsz, "NULL", "NULL", "UP", final_video_name);
    }
    if (cam_down && video_pipe_down) {
      pclose(video_pipe_down);
      sprintf(final_old_path, "%s/.tmp_down.mp4", vid_path_down);
      sprintf(final_video_name, "camera_down=%.2f,%.2f,%.2f,%.2f_to_%.2f,%.2f,%.2f,%.2f.mp4",
              video_start_time, video_start_x, video_start_y, video_start_z,
              final_time, final_gps[0], final_gps[1], final_gps[2]);
      sprintf(final_new_path, "%s/%s", vid_path_down, final_video_name);
      rename(final_old_path, final_new_path);

      printf("| %-17s | %7s | %7s | %7s | %11s | %11s | %-10s | %-37s |\n",
             "VIDEO_RECORD_CHUNK", fsx, fsy, fsz, "NULL", "NULL", "DOWN", final_video_name);
    }
  }

  printf("+-------------------+---------+---------+---------+-------------+-------------+------------+---------------------------------------+\n");
  wb_robot_cleanup();
  return 0;
}