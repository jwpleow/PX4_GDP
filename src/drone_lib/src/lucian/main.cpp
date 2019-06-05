#include "Dstar.h"
#include <iostream>

int main() {
 Dstar *dstar = new Dstar();
 list<state> mypath;

//Rotate the drone to look at the goal
//Reorient axes to make x in the direction of the goal
int distance=10; //gps distance in m between goal and start
int resolution=5;//Resolution is 1/5 m;

 dstar->init(0,0,distance*resolution,0);         // set start to (0,0) and goal to (10,0)
 for (int i=-distance*resolution;i<=distance*resolution;i++) dstar->updateCell(i,-1,-1); //set the ground to be non-traversable
 
 //Look at sensors, set distance to obstacle xobst in m
 int xobst=4;
 //Rotate to detect height of obstacle hobst in m
 int yobst=3;
 
 for (int i=0;i<=yobst*resolution;i++) dstar->updateCell(xobst*resolution,i,-1);     // set cells (xobst,i) to be non traversable
	int xst=0,yst=0;
	while (!(xst==distance*resolution&&yst==0)){
	dstar->replan();               // plan a path
	mypath = dstar->getPath();     // retrieve path
	
	for(auto &v:mypath) {
		dstar->updateStart(v.x,v.y);      // move start to new path point
		//Move the actual drone
		xst=v.x;yst=v.y;
		std::cout <<v.x <<" "<<v.y<<'\n';
		if (v.x==xobst*resolution && v.y==yobst*resolution+1) {
			//Check for new obstacles
			xobst+=3;
			yobst+=2; //We update the x and y of the new obstacles relative to our current position
			for (int i=0;i<=yobst*resolution;i++) dstar->updateCell(xobst*resolution,i,-1);     // set cells (xobst,i) to be non traversable
			break;
		}
	}
	}
	
 //dstar->updateGoal(0,1);        // move goal to (0,1)
 //dstar->replan();               // plan a path
 //mypath = dstar->getPath();     // retrieve path
/*for (auto &v :mypath)
	{std::cout << v.x<<" "<<v.y<<'\n';}*/
	
/*	for (int i=0;i<=10;i++)
{		for (int j=0;j<=10;j++)
		{ state sth;
		sth.x=j;sth.y=i;
		std::cout<<dstar->getG(sth)<<'\t';
		}
cout<<'\n';}*/
 return 0;
}