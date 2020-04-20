#ifndef FUNCTIONS
#define FUNCTIONS
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include<iostream>
using namespace cv;
using namespace std;
#include<math.h>
#include<queue>

void bfs(Mat obstacle_map, Mat occupancy_grid, int i,int j,int count);
int isvalid(Mat obstacle_map,int i, int j);

Node1 temp_node1(0,0,0,0);
	
void main_bfs(Mat input_obstacle_map)     //Used to identify individual obstacles and find their edges
{
	int l,m;
	Mat obstacle_map(input_obstacle_map.rows,input_obstacle_map.cols,CV_8UC1,Scalar(0));	
	for(l=0;l<obstacle_map.rows;l++)
	{
		for(m=0;m<obstacle_map.cols;m++)
		{
			if(input_obstacle_map.at<uchar>(l,m)<150)
				obstacle_map.at<uchar>(l,m)=0;
			else
				obstacle_map.at<uchar>(l,m)=255;
		}
	}

	Mat occupancy_grid=obstacle_map.clone();
	int i,j,k;
	node1 temp;
	int count=0;

	for(l=0;l<obstacle_map.rows-1;l++)
	{
		for(m=0;m<obstacle_map.cols-1;m++)
		{	
			
			if(occupancy_grid.at<uchar>(l,m)==255)
			{
				temp.x=l;
				temp.y=m;
				temp_node1=Node1(temp.x,temp.y,count+1,0);
				main_q.push(temp_node1);
				myq.push(temp);
				occupancy_grid.at<uchar>(l,m)=0;
				bfs(obstacle_map,occupancy_grid,l,m,count);
				count++; 
			}
		}
	}
	cout << "**************No of obstacles : " << count<<endl;
}

void bfs(Mat obstacle_map,Mat occupancy_grid,int i,int j,int count)   //Used to apply bfs to travers whole of the obstacle edge
{	
	node1 temp;
	int l,m;
	while(!myq.empty())
	{
		for(l=i-1;l<=i+1;l++)
		{
			for(m=j-1;m<=j+1;m++)
			{
				if(isvalid(obstacle_map,l,m)&&occupancy_grid.at<uchar>(l,m)==255)
				{
					occupancy_grid.at<uchar>(l,m)=0;
					common.at<uchar>(l,m)=255;
					temp.x=l;
					temp.y=m;
					temp_node1=Node1(l,m,count+1,0.0);
					main_q.push(temp_node1);
					myq.push(temp);
				}
			}
		}
		myq.pop();
		if(!myq.empty())	
		{
			temp=myq.front();
			i=temp.x;j=temp.y;
		}
		                                
	}
	
}
Mat find_obstacle_dist(Mat input_obstacle_map)    //Used to find min distance from obstacles for each point and find voronoi boundaries from which voronoi fields are generated
{
	Node1 temp_node1=Node1(0,0,0,0),current_node1=Node1(0,0,0,0);
	Mat occupancy_grid=input_obstacle_map.clone();	
	for(int i=0;i<input_obstacle_map.rows;i++){
		for(int j=0;j<input_obstacle_map.cols;j++){
			if(common.at<uchar>(i,j)==255) occupancy_grid.at<uchar>(i,j) =0;
		}
	}
	current_node1=main_q.top();
	int i,j;
	Mat nearest_obs = input_obstacle_map.clone();
	cost_image = input_obstacle_map.clone();
	voronoi_edges=Mat(input_obstacle_map.rows,input_obstacle_map.cols,CV_8UC1,Scalar(0));
	i=current_node1.x;
	j=current_node1.y;
	int l,m;
	while(!main_q.empty())
	{
			if(!main_q.empty())	
			{
				current_node1=main_q.top();
				i=current_node1.x;j=current_node1.y;
				main_q.pop();
				if(occupancy_grid.at<uchar>(i,j)==255) continue;
				occupancy_grid.at<uchar>(i,j)=255;
				if(current_node1.cost<255) cost_image.at<uchar>(i,j)=(int)current_node1.cost;
				else cost_image.at<uchar>(i,j) = 254;
				nearest_obs.at<uchar>(i,j)=current_node1.color_code*10;
						
			}     
			for(l=i-1;l<=i+1;l++)
			{
				for(m=j-1;m<=j+1;m++)
				{
					if(isvalid(input_obstacle_map,l,m))
					{
						if(occupancy_grid.at<uchar>(l,m)==0)
						{
							temp_node1=Node1(l,m,current_node1.color_code,current_node1.cost+1.0+(0.414*((abs(l-i)+abs(m-j)+1)%2)));
							main_q.push(temp_node1);
						}
						else if(nearest_obs.at<uchar>(l,m)!=current_node1.color_code*10&&input_obstacle_map.at<uchar>(l,m)==0)
						{
							voronoi_edges.at<uchar>(l,m)=255;
							Node1 tem=Node1(l,m,0,0.0);
							main_q2.push(tem);
						}
					}
				}
			}
			
	}
	/*cout<<"after while loop"<<endl;*/
	
	return nearest_obs;

}
int isvalid2(Mat input,int i, int j);

Mat find_edge_cost(Mat input)
{	
	Node1 temp_node1=Node1(0,0,0,0),current_node1=Node1(0,0,0,0);
	Mat occupancy_grid=Mat(input.rows,input.cols,CV_8UC1,Scalar(0));
		int i,j;
	voronoi_cost_image = Mat(input.rows,input.cols,CV_8UC1,Scalar(255));
	for(i=0;i<input.rows;i++) for(int j=0;j<input.cols;j++) voronoi_cost_image.at<uchar>(i,j) = 254;
	if(main_q2.empty())
	{
		return input;
	}
	current_node1=main_q2.top();

	i=current_node1.x;
	j=current_node1.y;
	int l,m;
	 
	
	while(!main_q2.empty())
	{
			if(!main_q2.empty())	
			{
				current_node1=main_q2.top();
				i=current_node1.x;j=current_node1.y;
				main_q2.pop();
				if(occupancy_grid.at<uchar>(i,j)==255) continue;
				occupancy_grid.at<uchar>(i,j)=255;
				
				if(current_node1.cost<255) voronoi_cost_image.at<uchar>(i,j)=(int)current_node1.cost;
				else voronoi_cost_image.at<uchar>(i,j) = 254;
			}    
			for(l=i-1;l<=i+1;l++)
			{
				for(m=j-1;m<=j+1;m++)
				{	
					if(isvalid2(input,l,m)&&occupancy_grid.at<uchar>(l,m)==0)
					{
						temp_node1=Node1(l,m,current_node1.color_code,current_node1.cost+1.0+0.414*(abs(l-i+m-j+1)%2));
						main_q2.push(temp_node1);
					}

				}
			}
			
	}
	return voronoi_cost_image;

}



int isvalid2(Mat input,int x, int y)
{
	if(x<0||y<0||x>input.rows-1||y>input.cols-1){
		
	 	return 0;
	}
	else return 1;
}


int isvalid(Mat obstacle_map,int x, int y)
{
	if(x<0||y<0||x>obstacle_map.rows-1||y>obstacle_map.cols-1)
	{
		return 0;
	}
	else return 1;
}

#endif