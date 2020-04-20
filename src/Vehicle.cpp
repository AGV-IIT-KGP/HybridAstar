#include "../include/Vehicle.hpp"
#include<ros/ros.h>
#include<geometry_msgs/PolygonStamped.h>

vector <State> Vehicle::nextStates(State* n)
{
	State t;
	vector<State> next;
	string temp;char temp_c[100];
	int a=-5,b=-5,c=5,f=5;
	/*ros::param::get("/hybrid_astar_node/carlx",a);
	ros::param::get("/hybrid_astar_node/carly",b);
	ros::param::get("/hybrid_astar_node/carrx",c);
	ros::param::get("/hybrid_astar_node/carry",f);*/
	BOT_W=1.9;
	BOT_L=2.8;
	
	//alpha=steering angle, beta=turning angle, r=turning radius, d=distanced travelled
	float alpha, beta, r, d = 1; 
	/*cout<<BOT_W<<"\n\n\n\n\n\n\n"<<endl;
	cout<<BOT_L<<"\n\n\n\n\n\n\n"<<endl;*/
	for(alpha = -BOT_MAX_ALPHA; alpha <= BOT_MAX_ALPHA+0.01; alpha += BOT_MAX_ALPHA/2)
	{
		beta = abs(d*tan(alpha*M_PI/180)/BOT_L);
		if(abs(beta) > 0.001)
		{
			r = abs(BOT_L/tan(alpha*M_PI/180));
			if(alpha<0)
			{
				t.x = n->x + sin(n->theta)*r - sin(n->theta-beta)*r;
				t.y = n->y - cos(n->theta)*r + cos(n->theta-beta)*r;
				t.theta = fmod(n->theta+beta,2*M_PI);
			}
			else
			{
				t.x = n->x - sin(n->theta)*r + sin(n->theta+beta)*r;
				t.y = n->y + cos(n->theta)*r - cos(n->theta+beta)*r;
				t.theta = fmod(n->theta-beta,2*M_PI);
			}
		
			if(t.theta < 0)
				t.theta += 2*M_PI;
		}
		else
		{
			// if turning radius is very small we assume that the back tire has moved straight
			t.x = n->x + d*cos(n->theta); 
			t.y = n->y + d*sin(n->theta);
			t.theta = n->theta;
			t.g =  n->g + d;
		}

		next.push_back(t);
	}

	return next;		
}
