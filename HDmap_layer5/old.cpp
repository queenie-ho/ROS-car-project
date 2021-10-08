#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
using namespace std;
std::string path = ".../FYP-data.csv"; // Read the file 
struct Point_3D
    {
        double x; // x coordinate value of a 3D point;
        double y; 
        double z; 
    };

struct Line_3D
    {
        std::vector <Point_3D> start; 
        std::vector <Point_3D> end;
    };

struct Road 
    { 
    int Road_id; //ID
    char Road_name; //Marker
    int Road_lanecount; //max==2lanes
    const float Road_width; // 34
    }; 
struct RoadMap_Features 
    { 
enum type {  Road = 0, Intersection = 1, Roundabout = 2 }; 
    }; 
struct Intersection    
    { 
    int Intersection_id; 
    char Intersection_name; 
    int Intersection_lanecount; //max==2 
    }; 

// storage of the vector elements of smartcity map
class MapData 
{

public: 
    std::vector<Line_3D> solidLineVector; // save the solid line data
    std::vector<Line_3D> dashLineVector; // save the dashLine data
    std::vector<Line_3D> crosswalkLineVector; // save the crosswalkLineVector data
    Point_3D userPosition;
    double x,y;
	FILE *pathFile;
	pathFile = fopen("FYP-data.csv","r");
	if(pathFile == NULL)
    {printf( "open failure" );}
	
public:
   
    void readSolidLineCSV(std::string path)
    {       
        string Type,Marker,ID;
        int PointsNumber;
        double StartPosition_X,StartPosition_Y,EndPosition_X,EndPosition_Y;
    	double value_start[1];double value_end[1];
    	MapData MapData_;
    	map<double, MapData> MapA;
        map<double, MapData>::iterator iterMapA;
        //char line [1000];
        vector<MapData> MapVector;

      {
		
        ifstream fin("FYP-data.csv");  
        string s;  
        while( getline(fin,s) )
        {    
        cout << "Read from file: " << s << endl; 
        }
	    
        vector<string> result;
          while (getline(fin,s)!=EOF)
          {
            string substr;
            getline(s, substr, ',');
            result.push_back(substr);
          }

          MapData_.Type =((result[20]).c_str(), NULL);
          MapData_.Marker = strtoint((result[20]).c_str(), NULL);
          MapData_.ID = strtoint((result[1]).c_str(), NULL);
          MapData_.PointsNumber = strtoint((result[6]).c_str(), NULL);
          MapData_.StartPosition_X = strtod((result[6]).c_str(), NULL);
          MapData_.StartPosition_Y = strtod((result[6]).c_str(), NULL);
          MapData_.EndPosition_X = strtod((result[6]).c_str(), NULL);
          MapData_.EndPosition_Y = strtod((result[6]).c_str(), NULL);
          cout << std::setprecision(7);
		//value_start = std::vector <Point_3D> start = {StartPosition_X,StartPosition_Y} ;  // Adding the values to the map
		//value_end = std::vector <Point_3D> end = {EndPosition_X,EndPosition_Y} ;  // Adding the values to the map			            
		//std::vector<Line_3D> solidLineVector (value_start,value_end); //push them to solidLineVector  
		
		}
    }

    // void readDashLineCSV(std::string path)
    // {
    
    //}
    // void readGivewayLineCSV(std::string path)
    // {
    //     /* read csv data and push them to GivewayLineVector*/     
      
    // }
    // void readCrosswalkLineCSV(std::string path)
    // {
    //     /* read csv data and push them to crosswalkLineVector*/
    // }
    // void readRoundaboutDashLineCSV(std::string path)
    // {
    //      read csv data and push them to RoundaboutDashLineVector
    // }
    // void readRoundaboutSolidLineCSV(std::string path)
    // {
    //     /* read csv data and push them to RoundaboutSolidLineVector*/
    // }
    // void readRoundaboutGivewayLineCSV(std::string path)
    // {
    //     /* read csv data and push them to RoundaboutGivewayLineVector*/
    // }

public:

    // this is a function, which is designed to query current solidlinesroad
    std::vector<Line_3D> getNeighbouringSolidLines(double radius, MapData MapData_)
    {
       double value_start[1];double value_end[1];
       std::vector <Point_3D> start;
       std::vector <Point_3D> end;
        // Given userPosition , then search around to return the nearest solidlines 
        double centerX =  userPosition.x ; 
        double centerY =  userPosition.y ; 
        double rSquared = radius * radius;

        double dX = (x - centerX) * (x - centerX); //delta X
        double dY = (y - centerY) * (y - centerY); //delta Y
        if ( dX + dY <= rSquared ) //Point is within Circle
        value_start =  start = {Position_X,Position_Y} ;  // Adding the values to the map
        value_end = end = {EndPosition_X,EndPosition_Y} ;  // Adding the values to the map                        
        std::vector<Line_3D> solidLineVector (value_start,value_end); //push them to solidLineVector  
        
            return solidLineVector;
    }
    // this is a function, which is designed to query current dashilinesroad
    std::vector<Line_3D> getNeighbouringDashLines(double radius, MapData MapData_)
    {
        double value_start[2];double value_end[2];
        // Given userPosition , then search around to return the nearest solidlines 
        double centerX =  userPosition.x ; 
        double centerY =  userPosition.y ; 
        double rSquared = radius * radius;

        double dX = (x - centerX) * (x - centerX); //delta X
        double dY = (y - centerY) * (y - centerY); //delta Y
        // if ( dX + dY <= rSquared ) //Point is within Circle
        
        // value_start =  start = {Position_X,Position_Y} ;  // Adding the values to the map
        // value_end = end = {EndPosition_X,EndPosition_Y} ;  // Adding the values to the map                        
        // std::vector<Line_3D> dashLineVector (value_start,value_end); //push them to solidLineVector  
        
            return dashLineVector;
    }
};

int main(int argc, char **argv)
{
    MapData MapData_;
    
    Point_3D userPosition;
    userPosition.x = 17;
    userPosition.y = 17;
    userPosition.z = 0;
    MapData_.readSolidLineCSV(path);// read your map 

    std::vector<Line_3D> solidLines = MapData_.getNeighbouringSolidLines(17, MapData_);

    //fclose(pathFile);return0;
}