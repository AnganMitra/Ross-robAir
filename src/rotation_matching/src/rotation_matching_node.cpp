#include "ros/ros.h"
#include "ros/time.h"
// We use the already built messages types
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"

using namespace std;

class rotation_matching {

private:

    ros::NodeHandle n;
    ros::Publisher vis;
    ros::Publisher pub_real_rotation_done;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_estimated_rotation_done;

    int flag_scan;
    float real_rotation_done;
    int *valid1, *valid2;
    float *scan1, *scan2;
    int *associated1;
    int *associated2;
    float range_min, range_max;
    float angle_min, angle_increment;
    int number_beams;
    int allocated;
    float angle_scan;

public:

rotation_matching() {

    // Subscribing to the topic delivered by the hokuyo lidar
    sub_scan = n.subscribe("scan", 1, &rotation_matching::scanCallback, this);

    // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    vis = n.advertise<visualization_msgs::Marker>("rotation_matching", 0);

    // communication with rotation node
    pub_real_rotation_done = n.advertise<std_msgs::Float32>("real_rotation_done", 1);
    sub_estimated_rotation_done = n.subscribe("estimated_rotation_done", 1, &rotation_matching::estimated_rotation_doneCallback, this);

    //flag_scan is used to know the status of the rotation_matching
    //flag_scan = 0 when the roation_matching hasnt started
    //flag scan = 1 the mobile robot will start to rotate, so we will store the scan before rotation in
    flag_scan = 0;

    // to know if scan1 et scan2 ont été alloués
    allocated = 0;

}

float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
// Distance between two points

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

void populateMarkerReference() {
// Draw the field of view and other references

    visualization_msgs::Marker references;
    references.header.frame_id = "base_laser";
    references.header.stamp = ros::Time::now();
    references.ns = "rotation_matching";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136) {
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    vis.publish(references);

}

void populateMarkerTopic(int number_beams, geometry_msgs::Point *current_scan, std_msgs::ColorRGBA *current_colors) {

    visualization_msgs::Marker marker;

    marker.header.frame_id = "base_laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "rotation_matching";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.a = 1.0;

    for (int i = 0; i < number_beams; i++) {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;

        p.x = current_scan[i].x;
        p.y = current_scan[i].y;
        p.z = current_scan[i].z;

        c.r = current_colors[i].r;
        c.g = current_colors[i].g;
        c.b = current_colors[i].b;
        c.a = current_colors[i].a;

        marker.points.push_back(p);
        marker.colors.push_back(c);
    }

    vis.publish(marker);
    populateMarkerReference();

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    if ( flag_scan == 1 ) {
        ROS_INFO("--Listening to scan:");
        ROS_INFO("Timestamp: %d", scan->header.seq);
        ROS_INFO("angle_min: %f", scan->angle_min*180/M_PI);
        ROS_INFO("angle_max: %f", scan->angle_max*180/M_PI);
        ROS_INFO("angle_increment: %f", scan->angle_increment*180/M_PI);
        ROS_INFO("range_min, range_max: %f, %f", scan->range_min, scan->range_max);

        number_beams = ((-1 * scan->angle_min) + scan->angle_max)/scan->angle_increment;

        ROS_INFO("number_of_beams: %d", number_beams);

        // Discard values in ranges that are out of this boundary
        range_min = scan->range_min;
        range_max = scan->range_max;
        angle_min = scan->angle_min;
        angle_increment = scan->angle_increment;

        // the mobile robot will rotate
        // we store the current scan (ie, before rotation) in the table scan1

        if ( !allocated ) {
            allocated = 1;
            ROS_INFO("hi");
            scan1 = new float[number_beams];
            valid1 = new int[number_beams];
            associated1 = new int[number_beams];

            scan2 = new float[number_beams];
            valid2 = new int[number_beams];
            associated2 = new int[number_beams];
        }

        for (int i=0; i< number_beams; i++)
            if ( scan->ranges[i] > range_min && scan->ranges[i] < range_max) {
                scan1[i] = scan->ranges[i];
                valid1[i] = 1;
                //ROS_INFO("scan1[%i] is valid", i);
            }
            else
                valid1[i] = 0;

        ROS_INFO("first scan before rotation");
        display_scan(0, 0);//we display scan1 before rotation
        //getchar();

        flag_scan++;
    }
    if ( flag_scan == 3 ) {

        ROS_INFO("--Listening to scan:");
        ROS_INFO("Timestamp: %d", scan->header.seq);
        ROS_INFO("angle_min: %f", scan->angle_min*180/M_PI);
        ROS_INFO("angle_max: %f", scan->angle_max*180/M_PI);
        ROS_INFO("angle_increment: %f", scan->angle_increment*180/M_PI);
        ROS_INFO("range_min, range_max: %f, %f", scan->range_min, scan->range_max);

        number_beams = ((-1 * scan->angle_min) + scan->angle_max)/scan->angle_increment;

        ROS_INFO("number_of_beams: %d", number_beams);

        // Discard values in ranges that are out of this boundary
        range_min = scan->range_min;
        
        range_max = scan->range_max;
      
        angle_min = scan->angle_min;
        angle_increment = scan->angle_increment;
        flag_scan = 0;
        // the mobile robot has performed a rotation
        // we store the current scan (ie, after rotation) in the table scan2

        for (int i=0; i< number_beams; i++){
            ROS_INFO("hi4");

            if ( scan->ranges[i] > range_min && scan->ranges[i] < range_max ) {
                scan2[i] = scan->ranges[i];
                valid2[i] = 1;
                ROS_INFO("scan2[%i] is valid", i);
            }
            else
                valid2[i] = 0;
        }
        getchar();

        ROS_INFO("first scan before rotation + second scan");
        display_scan(0, 1);//we display scan1 before rotation
        //getchar();

        ROS_INFO("(rotation_matching) 2nd scan stored. Ready to compare the 2 scans");
        float current_rotation = angle_scan;
        int init_score = matching_score(current_rotation);// we compute the score between the current scan (ie, after rotation - scan2) and the previous scan (ie, before rotation - scan1) that has been rotated of 'current_rotation' radians
        int current_score = init_score;// we initialize the current_score with the estimated_rotation_done
        int previous_score;
        display_scan(current_rotation, 1);
        getchar();
        int trigo = 0;

        //we test different rotations (in the trigonometric way) of the previous scan (ie, before rotation - scan1) and compute their score to match to the current scan (ie, after rotation - scan2)
        // we increment "current_action" of about 5 degre (0.08 radian) at each step
        //we continue when the current_score is higher than the previous_score (ie, the current matching of scan1 and scan2 is better than the previous matching)
        ROS_INFO("trigonometric");

        previous_score = current_score;
        do {

        //To complete
            previous_score =  current_score;
			current_rotation += 0.08;
			current_score =  matching_score(current_rotation );
		 
        }
        while ( current_score > previous_score );       
        current_rotation -= 0.08;
        float trigo_score = previous_score;
        float trigo_rotation = current_rotation;

        //ROS_INFO("(scan matching) real_rotation_done = %f", current_rotation*180/M_PI);
        //we test different rotations (in the non trigonometric way) of the previous scan (ie, before rotation - scan1) and compute their score to match to the current scan (ie, after rotation - scan2)
        // we decrement "current_action" of about 5 degre (0.08 radian) at each step
        //we continue when the current_score is higher than the previous_score (ie, the current matching of scan1 and scan2 is better than the previous matching)
        float antitrigo_rotation;
        float antitrigo_score;
        if ( !trigo ) {
            ROS_INFO("anti trigonometric");
            current_score = init_score;
            current_rotation = angle_scan;
            previous_score =  current_score;
            do {

                //to complete
                previous_score =  current_score;
				current_rotation -= 0.08;
				current_score =  matching_score(current_rotation );

            }
            while ( current_score > previous_score );
            current_rotation += 0.08;
            antitrigo_score = previous_score;
			antitrigo_rotation = current_rotation;
        }

        ROS_INFO("(scan matching) real_rotation_done = %f", -current_rotation*180/M_PI);
        //getchar();
        std_msgs::Float32 msg_real_rotation_done;
        //we sent the real_rotation_done to the rotation_action node
        // to complete
		real_rotation_done = trigo_score > antitrigo_score ? trigo_rotation:antitrigo_rotation;
		// publish real rotation done
		msg_real_rotation_done.data = real_rotation_done;
		pub_real_rotation_done.publish(msg_real_rotation_done);

    }

}

int matching_score(float current_rotation) {
//for each hit of scan1 (ie, the scan before rotation), we perform a rotation of this hit of "current_rotation" radians
//and find the closest hit in scan2 (ie, the scan after rotation)
// if the closest hit in scan2 and the current hit in scan2 are located at less than a given threshold (5cms for instance) they are associated
//so we increment the matching_score

    int nb = 0;//to count the number of matching

    ROS_INFO("(rotation_matching) processing matching_score for angle = %f", -current_rotation*180/M_PI);

    for ( int index_scan = 0; index_scan < number_beams; index_scan++ ) {
//        ROS_INFO("%i = %f, %i", index_scan, scan1[index_scan], valid1[index_scan]);

       associated1[index_scan] = 0;
       associated2[index_scan] = 0;
    }

    float beam_angle1 = angle_min;

    // we compute the position of the 1st scan after a rotation of "current_rotation"
    for(int index_scan1 = 0; index_scan1 < number_beams; index_scan1++, beam_angle1 += angle_increment)
//        ROS_INFO("%i = %f, %i", index_scan, scan1[index_scan], valid1[index_scan]);

        if ( valid1[index_scan1] ) {
            geometry_msgs::Point pt1;//coordinate (in the cartesian framework) of the current hit of scan1 after a rotation of "current rotation"
            geometry_msgs::Point best_pt;//coordinate (in the cartesian framework) of the closest hit of scan2 of the current hit of scan1

            //to complete
            pt1.x = scan1[index_scan1]*cos(current_rotation+beam_angle1);
            pt1.y = scan1[index_scan1]*sin(current_rotation+beam_angle1);

            float beam_angle2 = angle_min;
            int index_min;// to store the index of the closest point
            float min_dist = range_max;// to store the distance of the closest point
            float temp_distance;
            //ROS_INFO("scan1 = %i", index_scan);

            // matching in the cartesian space of each point of scan1 with a point of scan2
            for(int index_scan2 = 0; index_scan2 < number_beams; index_scan2++, beam_angle2 += angle_increment)
                if ( valid2[index_scan2] ) {

                    geometry_msgs::Point pt2;//coordinate (in the cartesian framework) of the current hit of scan2

                    //to complete
                    pt2.x = scan2[index_scan2]*cos(beam_angle2);
					pt2.y = scan2[index_scan2]*sin(beam_angle2);
					temp_distance = distancePoints(pt1, pt2);
					if( min_dist > temp_distance)
					{
						min_dist = temp_distance;
						index_min = index_scan2;
                    //we search for the closest point p2 of p1
                    //we store the distance in min_dist and its index in index_min
					}
                }

                    //ROS_INFO("x1 = %f, y1 = %f, x2 = %f, y2 = %f, dist = %f", car_scan1.x, car_scan1.y, x2, y2, current_dist);                    

            if ( min_dist < 0.05 ) {//we found a point p2 that is located at less than 5centimeters of p1
                nb++;
                associated1[index_scan1] = 1;
                associated2[index_min] = 1;
                //ROS_INFO("association %i and %i", index_scan, index_min);
            }

    }

    ROS_INFO("(rotation_matching) score = %i", nb);
    return ( nb );

}

void display_scan(float current_rotation, int display) {

    geometry_msgs::Point current_scan[2*number_beams];
    std_msgs::ColorRGBA current_colors[2*number_beams];

    int nb_beams = 0;
    float beam_angle = angle_min;
    for (int i=0 ; i < number_beams; i++, beam_angle += angle_increment) {
        if ( valid1[i] ) {
            current_scan[nb_beams].x = scan1[i] * cos(beam_angle+current_rotation);
            current_scan[nb_beams].y = scan1[i] * sin(beam_angle+current_rotation);
            current_scan[nb_beams].z = 0.0;

            if ( associated1[i] ) {
                    // blue color when the matching is valid
                    current_colors[nb_beams].r = 0.0;
                    current_colors[nb_beams].g = 0.0;
                    current_colors[nb_beams].b = 1.0;
                    current_colors[nb_beams].a = 1.0;
                }
                else {
                    // green color when the matching is not valid
                    current_colors[nb_beams].r = 0.0;
                    current_colors[nb_beams].g = 1.0;
                    current_colors[nb_beams].b = 0.0;
                    current_colors[nb_beams].a = 1.0;
                }
            nb_beams++;
        }
        if ( valid2[i] )
            if ( display ) {
            current_scan[nb_beams].x = scan2[i] * cos(beam_angle);
            current_scan[nb_beams].y = scan2[i] * sin(beam_angle);
            current_scan[nb_beams].z = 0.0;

            if ( associated2[i] ) {
                // blue color when the matching is valid
                current_colors[nb_beams].r = 0.0;
                current_colors[nb_beams].g = 0.0;
                current_colors[nb_beams].b = 1.0;
                current_colors[nb_beams].a = 1.0;
            }
            else {
                // white color when the matching is not valid
                current_colors[nb_beams].r = 1.0;
                current_colors[nb_beams].g = 1.0;
                current_colors[nb_beams].b = 1.0;
                current_colors[nb_beams].a = 1.0;
            }
            nb_beams++;
        }
    }
    populateMarkerTopic(nb_beams, current_scan, current_colors);

}

void estimated_rotation_doneCallback(const std_msgs::Float32ConstPtr& a) {

    if ( a-> data != 0 ) {
        flag_scan = 3;
        angle_scan = -a->data;// the theoretical angle performed between the 2 scans
    }
    else
        flag_scan = 1;

    ROS_INFO("(rotation_matching) flag_scan = %i, estimated_rotation_done = %f", flag_scan, a->data*180/M_PI);

}

};


int main(int argc, char **argv){

    ros::init(argc, argv, "rotation_matching");

    rotation_matching bsObject;

    ros::spin();

    return 0;
}


