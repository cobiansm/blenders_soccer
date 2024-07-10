#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

class HeadControl {
public:
    HeadControl() {
        ros::NodeHandle nh;
        int robot_id;
        nh.param("robot_id", robot_id, 0);

        error_pub = nh.advertise<geometry_msgs::Point>("robotis_" + std::to_string(robot_id) + "/error", 1);
        position_pub = nh.advertise<geometry_msgs::Point>("robotis_" + std::to_string(robot_id) + "/position", 1);
        search_ball_pub = nh.advertise<std_msgs::Bool>("robotis_" + std::to_string(robot_id) + "/search_ball", 1);
        ball_pub = nh.advertise<std_msgs::Bool>("robotis_" + std::to_string(robot_id) + "/find_ball", 1);
        turnNsearch_pub = nh.advertise<std_msgs::Bool>("robotis_" + std::to_string(robot_id) + "/turnNsearch", 1);

        sub_center = nh.subscribe("robotis_" + std::to_string(robot_id) + "/BallCenter", 1, &HeadControl::callbackCenter, this);

        search_ball.data = false;
        find_ball.data = false;

        ux0 = 0;
        uy0 = 0;
        ux1 = 0;
        uy1 = 0;

        kpx = 0.00018;
        kpy = 0.00018;

        //kpx = 0.00018;
        //kpy = 0.00018;

        errx0 = 0;
        errx1 = 0;
        erry0 = 0;
        erry1 = 0;

        now = 0;
        start_search = 0;
        end_search = false;

        noball_start_time = 0;
        noball_now_time = 0;
        noball_end_time = false;

        t = 0;
        head_direction = 1;
    }

    void callbackCenter(const geometry_msgs::Point& ball_center) {
        center.x = ball_center.x;
        center.y = ball_center.y;
    }

    geometry_msgs::Point cal_err() {
        int x_center = 320;
        int y_center = 240;

        geometry_msgs::Point point;

        if (center.x != 999 && center.y != 999) {
            double errorx = x_center - center.x;
            double errory = y_center - center.y;

            errorx = errorx * 70 / x_center;
            errory = errory * 70 / y_center;

            point.x = errorx;
            point.y = errory;

            error_pub.publish(point);
        }

        return point;
    }

    void control() {
        turn_search.data = false;
        turnNsearch_pub.publish(turn_search);

        geometry_msgs::Point point = cal_err();

        if ((errx0 < -35 || errx0 > 35) || (erry0 < -35 || erry0 > 35)) {
            find_ball.data = false;
        }

        if (center.x == 999 && center.y == 999) {
            if (noball_start_time == 0) {
                noball_start_time = time(0);
            }

            if (!noball_end_time) {
                if (noball_now_time - noball_start_time > 3) {
                    noball_end_time = true;
                }

                noball_now_time = time(0);
            } else {
                noball_start_time = 0;
                noball_end_time = false;
                noball_now_time = 0;

                ux1 = 0;
                uy1 = 0;

                search_ball.data = true;
                search_ball_pub.publish(search_ball);

                t += 1 * head_direction;

                if (t >= 360) {
                    head_direction = -1;
                } else if (t <= -360) {
                    head_direction = 1;
                }

                double ux_noball = 60 * sin(1 * t);
                double uy_noball = 15 * cos(1 * -t * head_direction) - 30;

                ux_noball = (ux_noball * M_PI) / 180;
                uy_noball = (uy_noball * M_PI) / 180;

                geometry_msgs::Point position_point;
                position_point.x = ux_noball;
                position_point.y = uy_noball;

                position_pub.publish(position_point);

                errx1 = 0;
                erry1 = 0;

                if (start_search == 0) {
                    start_search = time(0);
                }

                if (!end_search) {
                    turn_search.data = false;
                    turnNsearch_pub.publish(turn_search);
                    if (now - start_search > 15) {
                        end_search = true;
                    }
                    now = time(0);
                } else {
                    turn_search.data = true;
                    turnNsearch_pub.publish(turn_search);
                    ros::Duration(1.0).sleep();
                    start_search = 0;
                    end_search = false;
                    now = 0;
                }
            }
        } else if (!find_ball.data) {
            search_ball.data = false;
            search_ball_pub.publish(search_ball);

            noball_start_time = 0;
            noball_now_time = 0;
            noball_end_time = false;

            errx0 = point.x;
            erry0 = point.y;

            start_search = 0;

            double ux = ux1 + kpx * errx0;
            double uy = uy1 + kpy * erry0;

            if (ux >= 70) ux = 70;
            else if (ux <= -70) ux = -70;

            if (uy >= 20) uy = 20;
            else if (uy <= -70) uy = -70;

            ux1 = ux;
            errx1 = errx0;

            uy1 = uy;
            erry1 = erry0;

            if ((errx0 < -16 || errx0 > 16) || (erry0 < -16 || erry0 > 16)) {
                ux = (ux * M_PI) / 180;
                uy = (uy * M_PI) / 180;

                geometry_msgs::Point position_point;
                position_point.x = ux;
                position_point.y = uy;

                position_pub.publish(position_point);

                find_ball.data = false;
                ball_pub.publish(find_ball);
            } else {
                find_ball.data = true;
                ball_pub.publish(find_ball);
            }

            center.x = 999;
            center.y = 999;
        }
    }

private:
    ros::Publisher error_pub;
    ros::Publisher position_pub;
    ros::Publisher search_ball_pub;
    ros::Publisher ball_pub;
    ros::Publisher turnNsearch_pub;
    ros::Subscriber sub_center;

    std_msgs::Bool find_ball;
    std_msgs::Bool search_ball;
    std_msgs::Bool turn_search;
    geometry_msgs::Point center;

    double ux0, uy0;
    double ux1, uy1;
    double kpx, kpy;
    double errx0, errx1;
    double erry0, erry1;

    time_t now;
    time_t start_search;
    bool end_search;

    time_t noball_start_time;
    time_t noball_now_time;
    bool noball_end_time;

    int t;
    int head_direction;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "head_control");
    HeadControl headctrl;
    ros::Rate loop_rate(30);  // Adjust loop rate as necessary

    while (ros::ok()) {
        headctrl.control();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
