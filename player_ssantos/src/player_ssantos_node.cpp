#include <iostream>
    
/*int main()
{
    std::cout << "Hello world" << std::endl;           
    return 1;                                                      
}*/

class Player{
    public:

    Player(std::string name){ this->name = name; }

    std::string name; //A public atribute

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
    			std::cout << "wrong team index given. Cannot set team" << std::endl; break;
    	}
    }

    int setTeamName(std::string team){
        if (team=="red" || team=="green" || team=="blue")
        {
            this->team = team;
            return 1;
        }
        else
        {
            std::cout << "cannot set team name to " << team << std::endl;
            return 0;
        }
    }

    //Gets team name (accessor)
    std::string getTeamName(void){ return team; }
    
    private:
    std::string team;
};

class MyPlayer : public Player{
	public:

	MyPlayer(std::string name, std::string team) : Player(name){
		setTeamName(team);
	}
};

int main(){
    //Creating an instance of class Player
    Player player("ssantos");
    player.setTeamName(1);

    std::cout << "player.name is " << player.name << std::endl;
    std::cout << "team is " << player.getTeamName() << std::endl;

    MyPlayer my_player("ssantos", "blue");
    std::cout << "myplayer team_name" << my_player.getTeamName() << std::endl;
}