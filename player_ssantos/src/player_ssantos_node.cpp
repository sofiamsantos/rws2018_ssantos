//System includes
#include <iostream>
#include <vector>

//Boost includes
#include <boost/shared_ptr.hpp>

//ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>

using namespace std;

#define DEFAULT_TIME 0.05

namespace rws_ssantos{

	class Player{
	    public:
	  
	    Player(string name){ this->name = name; }

	    //Set team name, if given a correct team name (accessor)
	    int setTeamName(int idx = 0 /*default value*/){
	    	switch(idx){
	    		case 0:
	    			setTeamName("red");
	    			break;
	    		case 1:
	    			setTeamName("green");
	    			break;
	    		case 2:
	    			setTeamName("blue");
	    			break;
	    		default:
	    			//cout << "wrong team index given. Cannot set team" << endl;
	    			ROS_ERROR("Wrong team index given. Cannot set team");
	    			break;
	    	}
	    }

	    int setTeamName(string team){
	        if (team=="red" || team=="green" || team=="blue")
	        {
	            this->team = team;
	            return 1;
	        }
	        else
	        {
	            //cout << "cannot set team name to " << team << endl;
	            ROS_ERROR("Cannot set team name to %s", team.c_str());
	            return 0;
	        }
	    }

	    //Gets team name (accessor)
	    string getTeamName(void){ return team; }
	    
	    string name; //A public atribute

	    private:
	    string team;
	};

	//
//--Class MYPLAYER extends class Player----------------------------------------------------------------------	
	class MyPlayer : public Player {
		public:

		boost::shared_ptr<Team> red_team;
		boost::shared_ptr<Team>  green_team;
		boost::shared_ptr<Team>  blue_team;

		boost::shared_ptr<Team> my_team;
		boost::shared_ptr<Team>  my_preys;
		boost::shared_ptr<Team>  my_hunters;

		tf::TransformBroadcaster br;				//declare broadcaster
		tf::TransformListener listener;

		ros::NodeHandle n;
		boost::shared_ptr<ros::Subscriber> sub;
		boost::shared_ptr<ros::Publisher> pub;

		tf::Transform transform;					//declare transformation object

	//--Player Constructor----------------------------------------------------------------------------------
		MyPlayer(string name, string team) : Player(name){
			red_team = boost::shared_ptr<Team>(new Team("red"));
			green_team = boost::shared_ptr<Team>(new Team("green"));
			blue_team = boost::shared_ptr<Team>(new Team("blue"));

			// Check Team Rules
			if (red_team->playerBelongsToTeam(name))
			{
				my_team = red_team;
				my_preys = green_team;
				my_hunters = blue_team;
				setTeamName("red");
			}
			else if (green_team->playerBelongsToTeam(name))
			{
				my_team = green_team;
				my_preys = blue_team;
				my_hunters = red_team;
				setTeamName("green");
			}
			else if (blue_team->playerBelongsToTeam(name))
			{
				my_team = blue_team;
				my_preys = red_team;
				my_hunters = green_team;
				setTeamName("blue");
			}

			// Subscribe to Referee Message
			sub = boost::shared_ptr<ros::Subscriber> (new ros::Subscriber());
			*sub = n.subscribe("/make_a_play", 100, &MyPlayer::move,this);

			// Publish visualization marker
			pub = boost::shared_ptr<ros::Publisher> (new ros::Publisher());
			*pub = n.advertise<visualization_msgs::Marker>("/bocas", 0);

			// Spawn at random position
			srand(682*time(NULL)); // set initial seed value to 5323
			double start_x = ((double)rand()/(double)RAND_MAX)*10-5;
			double start_y = ((double)rand()/(double)RAND_MAX)*10-5;
			warp(start_x, start_y, M_PI/2);

			printReport();
		}

	//--Print Player Name and Team---------------------------------------------------------------------------
		void printReport(){
			//cout << "My name is " << name << " and my team is " << getTeamName() << endl;
			//ROS_INFO_STREAM("My name is " << name << " and my team is " << getTeamName());
			ROS_INFO("My name is %s and my team is %s", name.c_str(), getTeamName().c_str());
		}

	//--Rviz Marker Specifications---------------------------------------------------------------------------
		void showMarker(string phrase){
			visualization_msgs::Marker marker;
			marker.header.frame_id = "ssantos";
			marker.header.stamp = ros::Time();
			marker.ns = "ssantos";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.y = 1.05;
			marker.pose.orientation.w = 1.0;
			marker.scale.z = 0.3;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 0.5;
			marker.color.b = 1.0;
			marker.lifetime = ros::Duration(2);
			marker.text = phrase;
			pub->publish( marker );
		}

	//--Calculate Distance to Another Player------------------------------------------------------------------
		double getDistanceToPlayer(string other_player, double time_to_wait = DEFAULT_TIME){
			tf::StampedTransform t;	//The transform object
			ros::Time now = ros::Time(0);	//Get the latest transform received

			try{
				listener.waitForTransform("ssantos", other_player, now, ros::Duration(time_to_wait));
		      	listener.lookupTransform("ssantos", other_player, now, t);
		    }catch (tf::TransformException ex){
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(0.1).sleep();
			}

			return sqrt(pow(t.getOrigin().y(),2) + pow(t.getOrigin().x(),2));
		}

	//--Calculate Angle to Another Player---------------------------------------------------------------------
		double getAngleToPlayer(string other_player, double time_to_wait = DEFAULT_TIME){
			tf::StampedTransform t;	//The transform object
			ros::Time now = ros::Time(0);	//Get the latest transform received

			try{
				listener.waitForTransform("ssantos", other_player, now, ros::Duration(time_to_wait));
		      	listener.lookupTransform("ssantos", other_player, now, t);
		    }catch (tf::TransformException ex){
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(0.1).sleep();
			}

			return atan2(t.getOrigin().y(), t.getOrigin().x());
		}

	//--Get Closest Player: Hunter and Prey---------------------------------------------------------------------
		string closestPrey(const rws2018_msgs::MakeAPlay::ConstPtr& msg){
			double min_dist = 9999999;
			string player = "no_player";

			for(size_t i=0; i < msg->red_alive.size(); i++){
				double dist = getDistanceToPlayer(msg->red_alive[i]);
				if(isnan(dist));
				else if(dist < min_dist){
					min_dist = dist;
					player = msg->red_alive[i];
				}
			}
			return player;
		}

		string closestHunter(const rws2018_msgs::MakeAPlay::ConstPtr& msg){
			double min_dist = 9999999;
			string player = "no_player";

			for(size_t i=0; i < msg->green_alive.size(); i++){
				double dist = getDistanceToPlayer(msg->green_alive[i]);
				if(isnan(dist));
				else if(dist < min_dist){
					min_dist = dist;
					player = msg->green_alive[i];
				}
			}
			return player;
		}

	//--Movement-----------------------------------------------------------------------------------------
		void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg){

			double x = transform.getOrigin().x();
			double y = transform.getOrigin().y();
			double a = 0.0;

			//-----------------------------------------------
			//--- AI PART
			//-----------------------------------------------
			//Find nearest prey -- player_to_hunt is the nearest player
			string prey = closestPrey(msg);
			string hunter = closestHunter(msg);
			double displacement = 6; // velocity
			double delta_alpha = getAngleToPlayer(prey);

			if(getDistanceToPlayer(hunter) < 1 && getDistanceToPlayer(hunter) < getDistanceToPlayer(prey)){
				delta_alpha = -getAngleToPlayer(hunter);
				if(isnan(delta_alpha))
					delta_alpha = 0;
				//create marker
				showMarker("Running "+hunter);
			}
			else{
				delta_alpha = getAngleToPlayer(prey);
				if(isnan(delta_alpha))
					delta_alpha = 0;
				//create marker
				showMarker("Catching "+prey);
			}
			
			//-----------------------------------------------
			//--- CONSTRAINS PART
			//-----------------------------------------------
			//Limiting displacement
			double displacement_max = msg->cat;
			displacement > displacement_max ? displacement = displacement_max: displacement = displacement; 

			//Limiting alpha 
			double delta_alpha_max = M_PI/30;
			fabs(delta_alpha) > fabs(delta_alpha_max) ? delta_alpha = delta_alpha_max * delta_alpha / fabs(delta_alpha): delta_alpha = delta_alpha;

			tf::Transform my_move_T; //declare the transformation object - player's pose wrt world
			my_move_T.setOrigin(tf::Vector3(displacement, 0.0, 0.0));
			tf::Quaternion q1;
			q1.setRPY(0, 0, delta_alpha);
			my_move_T.setRotation(q1);

			transform = transform * my_move_T;

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ssantos"));
			//ROS_INFO("Moving...");
		}

	//--Spawner------------------------------------------------------------------------------------------
		void warp(double x, double y, double alfa){
			transform.setOrigin( tf::Vector3(x, y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, alfa);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ssantos"));
			ROS_INFO("Warping to (%f, %f) with alfa %f", x, y, alfa);
		}

	};

} //end of namespace rws_ssantos

int main(int argc, char **argv){

	ros::init(argc, argv, "ssantos");
    ros::NodeHandle n;

    //Creating an instance of class Player
    rws_ssantos::MyPlayer my_player("ssantos","blue");    

    ros::Rate loop_rate(10);
    /*while (ros::ok()){

		my_player.move();
    
    	ros::spinOnce();
    	loop_rate.sleep();
  	}*/

    ros::spin();

    //string test_param_value;
    //n.getParam("test_param", test_param_value);
    //cout << "read test_param with value " << test_param_value << endl;
}