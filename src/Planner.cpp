#include "../include/Planner.hpp"
#include <boost/heap/fibonacci_heap.hpp>
#include "ros/ros.h"

#define PI 3.14159
/*#define WT 1.1*/

bool operator<(const State& a, const State& b)
{
	return (a.g + a.h  /*10.0-10.0*(1.0*finaluse.at<uchar>((int)a.x*2,(int)a.y*2))/255)*/)> (b.g + b.h  /*10.0-10.0*(1.0*finaluse.at<uchar>((int)b.x*2,(int)b.y*2))/255)*/);
}

Planner::Planner(int map_x, int map_y, float map_grid_resolution, float planner_grid_resolution)
{
	
	this->map_x = map_x;
	this->map_y = map_y;
	this->map_grid_resolution = map_grid_resolution;
	this->planner_grid_resolution = planner_grid_resolution;

	planner_grid_x = toInt(map_x/planner_grid_resolution);
	planner_grid_y = toInt(map_y/planner_grid_resolution);
	planner_grid_theta = 72;
	
	visited_state=new State**[2*planner_grid_x];
	for(int i=0;i<planner_grid_x;i++)
	{
		visited_state[i]=new State*[2*planner_grid_y];
		for(int j=0;j<planner_grid_y;j++)
			visited_state[i][j]=new State[2*72];
	}

	visited=new bool**[planner_grid_x];
	for(int i=0;i<planner_grid_x;i++)
	{
		visited[i]=new bool*[planner_grid_y];
		for(int j=0;j<planner_grid_y;j++)
			visited[i][j]=new bool[72];
	}
	
	return;
}

vector<State> Planner::plan(State start, State end, Vehicle car, int** obstacles, GUI display,Mat final,Mat obs_dist_global)
{
	/*cout << start.x << " " << start.y << endl;
	cout << end.x << " " << end.y << endl;
	
	int b=0;
	int a = 3/b;*/
	//finaluse = final;
	int dist_dubin_shot;
	float WT;
	 ros::param::get("/hybrid_astar_node/dist_dubin_shot", dist_dubin_shot);
	 ros::param::get("/hybrid_astar_node/WT", WT);


	bool DISPLAY_PATH = false;
	bool DISPLAY_SEARCH_TREE = false;

	clock_t map_init_start = clock();
	Map map(obstacles, map_x, map_y, map_grid_resolution, end, car);
	map_init_time = float(clock()-map_init_start)/CLOCKS_PER_SEC;

	if(DISPLAY_SEARCH_TREE)
    	display.draw_obstacles(obstacles, map_grid_resolution);

	clock_t dijkstra_start = clock();
	float dijkstra_grid_resolution = 1;
	Heuristic heuristic(map, dijkstra_grid_resolution, end, car);
	dijkstra_time = float(clock()-dijkstra_start)/CLOCKS_PER_SEC;
	
	for(int i=0; i < planner_grid_x; i++)
		for(int j=0; j < planner_grid_y; j++)
			for(int k=0; k < planner_grid_theta; k++ )
				visited[i][j][k]=false;
				

	// Hybrid Astar Openlist Initiates:
	priority_queue <State, vector<State>> pq;
	start.g = 0;
	start.h = heuristic.get_heuristic(start,final);
	start.parent = NULL;
	pq.push(start);
	
	
	int count=0;
	while(!pq.empty())
	{
		//cout << pq.size() << endl;
		State current=pq.top();
		pq.pop();

		// Angles are taken from 0-360 in steps of 5 so planner_grid_theta is 72 also 
		// theta attribute of State stores angle in radian form.
		int current_grid_x = roundDown(current.x/planner_grid_resolution);
		int current_grid_y = roundDown(current.y/planner_grid_resolution);
		int current_grid_theta = ((int)(current.theta*planner_grid_theta/(2*PI)))%planner_grid_theta;

		if( visited[current_grid_x][current_grid_y][current_grid_theta] )
			continue;

		visited[current_grid_x][current_grid_y][current_grid_theta] = true;
		visited_state[current_grid_x][current_grid_y][current_grid_theta] = current;	
		//cout << "Current node :" << current.x << " " <<current.y << " " << current.theta <<endl;
		// Checks if it has reached the goal
		if(map.isReached(current))
		{	
			State temp=current;
			while( temp.parent != NULL )
			{
				path.push_back(temp);
				temp =*(temp.parent);
			}
			reverse(path.begin(), path.end());	

	        if(DISPLAY_PATH)
	        {
		     	display.clear();

	            display.draw_obstacles(obstacles, map_grid_resolution);
	            for(int i=0;i<=path.size();i++)
	            {
	                display.draw_car(path[i], car);
	                display.show(5);
	            } 
		     	display.show(1000);
		     	display.clear();
	        }		
			return path;
		}

		// This section finds the next states based on trajecory generation.
		vector<State> next=car.nextStates(&current);

		for(vector<State>::iterator it= next.begin(); it!=next.end();it++)
		{
			State nextS = *it;
			if( !map.isValid(nextS))
				continue;

			int next_grid_x = roundDown(nextS.x/planner_grid_resolution);
			int next_grid_y = roundDown(nextS.y/planner_grid_resolution);
			int next_grid_theta = ((int)(nextS.theta*planner_grid_theta/(2*PI)))%planner_grid_theta;
				
			if( visited[next_grid_x][next_grid_y][next_grid_theta] )
				continue;
				
			if( !map.checkCollision(nextS) )   //change
			{
				if(DISPLAY_SEARCH_TREE)
				{
					display.draw_tree(current,nextS);
					display.show(5);					
				}

				it->parent = &(visited_state[current_grid_x][current_grid_y][current_grid_theta]);
				it->g = current.g+1+WT-WT*final.at<uchar>((int)it->x/map_grid_resolution,(int)it->y/map_grid_resolution)/255;;
				it->h = heuristic.get_heuristic(*it,final);
				//cout << "Node added normally : " << it->x << " " << it->y << " " <<it->theta << " " << "Parent : " << it->parent->x << " " << it->parent->y << " " << it->parent->theta <<endl;  

				pq.push(*it);
			}
		}

		// At every few iterations we try to calculate the Dubins Path and add the states 
		// to the openlist. Once it collides we stop further iterations. 
		int prev_grid_x,prev_grid_y,prev_grid_theta,next_grid_x,next_grid_y,next_grid_theta;
		if( count%4==3 )
		{	
			
			vector<State> Path = heuristic.DubinShot(current,end,car.min_radius);

			State prev = current, check=current;
			for(vector<State>::iterator it= Path.begin(); it!=Path.end();it++)
			{
				State nextS = *it;
				
				if( !map.isValid(nextS))
					continue;

				if(map.isReached(nextS))
				{
					State temp=check;
					
					while( temp.parent != NULL )
					{
						path.push_back(temp);
						//cout<<"before seg fault"<<endl;
						//cout << temp.x << " " << temp.y << " " << temp.theta << "Parent : " << (temp.parent)->x << " " << (temp.parent)->y << " " << (temp.parent)->theta;
						temp= *(temp.parent);
						//cout<<"afterrr seg fault"<<endl;
					}
					reverse(path.begin(), path.end());

			        if(DISPLAY_PATH)
					{
				     	display.clear();

			            display.draw_obstacles(obstacles, map_grid_resolution);
			            for(int i=0;i<=path.size();i++)
			            {
			                display.draw_car(path[i], car);
			                display.show(5);
			            } 
				     	display.show(1000);
				     	display.clear();
			        }					
					return path;
				
				}

				// This is to insure that consecutive points are not very close .Because of being very close 
				// consecutive points were assigned same parents and caused problems while storing path.
				if( sqrt(pow(nextS.x-prev.x,2) + pow(nextS.y-prev.y,2)) < 2 )
					continue;

				if( !map.checkCollision(nextS)&&!map.check_min_obs_dis(nextS,obs_dist_global,dist_dubin_shot)) //change
				{
					prev_grid_x = roundDown(prev.x/planner_grid_resolution);;
					prev_grid_y = roundDown(prev.y/planner_grid_resolution);
					prev_grid_theta = ((int)(prev.theta*planner_grid_theta/(2*PI)))%planner_grid_theta;
					
					it->parent = &(visited_state[prev_grid_x][prev_grid_y][prev_grid_theta]);
					it->g = prev.g+sqrt(pow(nextS.x-prev.x,2) + pow(nextS.y-prev.y,2))+WT-WT*final.at<uchar>((int)it->x/map_grid_resolution,(int)it->y/map_grid_resolution)/255;
					it->h = heuristic.get_heuristic(*it,final);

					next_grid_x = roundDown(nextS.x/planner_grid_resolution);;
					next_grid_y = roundDown(nextS.y/planner_grid_resolution);
					next_grid_theta = ((int)(nextS.theta*planner_grid_theta/(2*PI)))%planner_grid_theta;
					//cout << "Visited : " << visited[next_grid_x][next_grid_y][next_grid_theta] <<endl;
					//cout << "Node added from dubins : " << (*it).x << " " << (*it).y << " " <<(*it).theta << " " << " Parent : " << (*(*it).parent).x << " " << (*(*it).parent).y << " " << (*(*it).parent).theta <<endl;  
					visited_state[next_grid_x][next_grid_y][next_grid_theta] = *it;
					check=*it;
					prev=nextS;
					pq.push(*it);
					
				}
				else
				{
					
					break;
				}
			}
			if(DISPLAY_SEARCH_TREE)
			{
				display.draw_dubins(Path);
				display.show(5);				
			}
		}
		count++;
		count = count%4;
	}
	cout<<"Goal cannot be reached"<<endl;
	return path;
}



