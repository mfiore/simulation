#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread.hpp>
#include "Keyboard.h"
#include "geometry_msgs/Pose.h"
#include "situation_assessment_msgs/PutObjectInHand.h"
#include "situation_assessment_msgs/PlaceObject.h"

using namespace std;

vector<string> tfs;
vector<double> xtfs;
vector<double> ytfs;
vector<double> ztfs;

int selectedTf=0;

Keyboard keyboard;

map<string,int> tf_links_;

#define DEAD_ZONE 0.05
#define DEAD_ZONE_YAW 0.05


// void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
//     int switchTf=msg->buttons[0];
//     int selectAll=msg->buttons[5];

//     if (selectAll==1) {
// 		for (int i=0; i<tfs.size();i++) {
//     			//double newX=xtfs[i]+threshold(msg->axes[1], DEAD_ZONE);
// 				//double newY=ytfs[i]+threshold(msg->axes[0], DEAD_ZONE);
// 			double newX=xtfs[i]+threshold(msg->axes[0], DEAD_ZONE)*0.02*1.5;
// 			double newY=ytfs[i]+threshold(msg->axes[1], DEAD_ZONE)*0.02*1.5;

// 				xtfs[i]=newX;
//     			ytfs[i]=newY;
//     		}
//     }
//     else {
//     	if (switchTf==1) {

//     		selectedTf++;
//     		if (selectedTf==tfs.size()) {
//     			selectedTf=0;
//     		}
//            	cout<<"Switching "<<tfs[selectedTf]<<"\n";

//     	}

// 		double newX=xtfs[selectedTf]+threshold(msg->axes[0], DEAD_ZONE)*0.02*1.5;
// 				double newY=ytfs[selectedTf]+threshold(msg->axes[1], DEAD_ZONE)*0.02*1.5;
//     		xtfs[selectedTf]=newX;
//     		ytfs[selectedTf]=newY;

//     }


// }

double threshold(double x, double x_s) {
    if(x > x_s) {
        return (x - x_s)/(1.0 - x_s);
    }
    else if(x < -x_s) {
        return -(x + x_s)/(-1.0 + x_s);
    }
    else
        return 0.0;
}

void keyboardLoop() {
	bool pressed,new_event;
	uint16_t k, mod;
	bool select_all=false;
	double x,y;
	while (ros::ok) {
		keyboard.get_key(new_event,pressed,k,mod);
		// if (new_event) {
		    switch (k) {
		        case (SDLK_PAGEUP):
		        {
		        	if (pressed) {
		        		selectedTf=selectedTf==tfs.size()-1?0:selectedTf+1;
		        		ROS_INFO("TFKEYBOARD - switching to %s",tfs[selectedTf].c_str());
		        	}
		            break;
		        }
		        case (SDLK_PAGEDOWN):
		        {
		            if (pressed) {
		            	ROS_INFO("TFKEYBOARD - select all status is %d",select_all);
		            	select_all=!select_all;
		            }
		            break;
		        }
		        case (SDLK_w):
		        {
		        	y=pressed?-0.1:0;
		            break;
		        }
		        case (SDLK_s):
		        {
		        	y=pressed?0.1:0;
		        	break;
		        }
		        case (SDLK_a):
		        {
		        	x=pressed?0.1:0;
		        	break;
		        }
		        case (SDLK_d):
		        {
		        	x=pressed?-0.1:0;
		            break;
		        }
		    }
		    	if (select_all) {
		    		for (int i=0; i<tfs.size();i++) {
		    			xtfs[i]=xtfs[i]+x;
		       			ytfs[i]=ytfs[i]+y;
		       		}
		    
		    	}
		    	else {
		    		xtfs[selectedTf]=xtfs[selectedTf]+x;
		    		ytfs[selectedTf]=ytfs[selectedTf]+y;
		    		//check links
		    		if (tf_links_.find(tfs[selectedTf])!=tf_links_.end()) {
		    			int held_object=tf_links_.at(tfs[selectedTf]);
		    			xtfs[held_object]=xtfs[selectedTf]+0.1;
		    			ytfs[held_object]=ytfs[selectedTf]+0.1;
		    		}
		    	}
		    	ros::Duration(0.1).sleep();
		// }


	}
}

void sendPosition() {
    ros::Rate r(10);
    while (ros::ok()) {
        for (int i=0; i<tfs.size(); i++) {
            static tf::TransformBroadcaster br;
	  	    //ROS_INFO("Tf numero %d",i);
	 		 tf::Transform transform;
	  		// if (tf_links_.find(tfs[i])!=tf_links_.end()){
	  		// 	int other_tf=tf_links_[tfs[i]];
	  		// 	transform.setOrigin(tf::Vector3(xtfs[other_tf],ytfs[other_tf],ztfs[other_tf]));	
	  		// }
	  		// else {
	    //  	}
        	transform.setOrigin( tf::Vector3(xtfs[i], ytfs[i], ztfs[i]) );

        	tf::Quaternion q;
		    q.setRPY(0, 0, 0);
	    	transform.setRotation(q);
	        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", tfs[i]));
	    }
        r.sleep();
    }
}

bool putObjectInHand(situation_assessment_msgs::PutObjectInHand::Request& req,
	situation_assessment_msgs::PutObjectInHand::Response& res) {
	if (req.has_object) {
		int object_index=0;
		int agent_index=0;
		bool found_agent=false;
		bool found_object=false;
		while (!found_agent && agent_index<tfs.size()) {
			if (tfs[agent_index]==req.agent) {
				found_agent=true;
			}
			else agent_index++;
		}
		while (!found_object && object_index<tfs.size()) {
			if (tfs[object_index]==req.object) {
				found_object=true;
			}
			else object_index++;
		}
		if (found_object && found_agent) {
			tf_links_[req.agent]=object_index;			
			res.result=true;
			xtfs[object_index]=xtfs[agent_index]+0.1;
			ytfs[object_index]=ytfs[agent_index]+0.1;
			ztfs[object_index]=ztfs[agent_index];
		}
		else {
			res.result=false;
		}

	}
	else {
		tf_links_.erase(req.object);
		res.result=true;
	}
	return true;
}

bool placeObject(situation_assessment_msgs::PlaceObject::Request& req,
	situation_assessment_msgs::PlaceObject::Response& res) {

	int i=0;
	bool found=false;
	while (!found && i<tfs.size()) {
		if (tfs[i]==req.name) {
			found=true;
		}
		else {
			i++;
		}
	}
	if (found) {
		xtfs[i]=req.pose.position.x;
		ytfs[i]=req.pose.position.y;
		ztfs[i]=req.pose.position.z;
		res.result=true;
	}
	else {
		res.result=false;
	}
	return true;
}


int main (int argc, char** argv) {

    ros::init(argc,argv,"tf_keyboard");

    ROS_INFO("Started tf_keyboard");
    ros::NodeHandle n;

    string robot_name;
    n.getParam("simulated_agents",tfs);
    n.getParam("situation_assessment/robot_name",robot_name);
    ROS_INFO("robot name is %s",robot_name.c_str());

    ROS_INFO("Got %ld agents",tfs.size());
    for (int i=0; i<tfs.size();i++) {
        xtfs.push_back(0);
        ytfs.push_back(0);
        ztfs.push_back(0);
        ROS_INFO("- %s",tfs[i].c_str());
    }
    boost::thread kthread(keyboardLoop);
    boost::thread t(sendPosition);
    // ros::Subscriber joy_sub= n.subscribe<sensor_msgs::Joy>("velocity_control", 10, joyCallback);
    ROS_INFO("Subscribed to velocity control");
   
    ros::ServiceServer put_object_service=n.advertiseService("/situation_assessment/put_object_in_hand",putObjectInHand);
    ros::ServiceServer place_object_service=n.advertiseService("/situation_assessment/place_object",placeObject);
    ROS_INFO("Ready");
    ros::spin();
    ros::shutdown();
}



