//System includes
#include <iostream>
#include <vector>

//Boost includes
#include <boost/shared_ptr.hpp>

//ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>

using namespace std;

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

	//Class myPlayer extends class Player
	class MyPlayer : public Player {
		public:

		boost::shared_ptr<Team> red_team;
		boost::shared_ptr<Team>  green_team;
		boost::shared_ptr<Team>  blue_team;

		boost::shared_ptr<Team> my_team;
		boost::shared_ptr<Team>  my_preys;
		boost::shared_ptr<Team>  my_hunters;

		tf::TransformBroadcaster br;				//declare broadcaster

		ros::NodeHandle n;
		boost::shared_ptr<ros::Subscriber> sub;

		tf::Transform transform;					//declare transformation object

		MyPlayer(string name, string team) : Player(name){
			red_team = boost::shared_ptr<Team>(new Team("red"));
			green_team = boost::shared_ptr<Team>(new Team("green"));
			blue_team = boost::shared_ptr<Team>(new Team("blue"));

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

			sub = boost::shared_ptr<ros::Subscriber> (new ros::Subscriber());
			*sub = n.subscribe("/make_a_play", 100, &MyPlayer::move,this);

			// Spawn at random position
			srand(682*time(NULL)); // set initial seed value to 5323
			double start_x = ((double)rand()/(double)RAND_MAX)*10-5;
			double start_y = ((double)rand()/(double)RAND_MAX)*10-5;

			warp(start_x, start_y, M_PI/2);
			printReport();
		}

		void printReport(){
			//cout << "My name is " << name << " and my team is " << getTeamName() << endl;
			//ROS_INFO_STREAM("My name is " << name << " and my team is " << getTeamName());
			ROS_INFO("My name is %s and my team is %s", name.c_str(), getTeamName().c_str());
		}

		void warp(double x, double y, double alfa){
			transform.setOrigin( tf::Vector3(x, y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, alfa);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ssantos"));
			ROS_INFO("Warping to (%f, %f) with alfa %f", x, y, alfa);
		}

		void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg){
			double x = transform.getOrigin().x();
			double y = transform.getOrigin().y();
			double a = 0.0;

			transform.setOrigin( tf::Vector3(x+=0.01, y, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, 0, a);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ssantos"));
			//ROS_INFO("Moving...");
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