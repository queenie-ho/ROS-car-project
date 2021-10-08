#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <cmath>
#include "hd_map_msgs/hd_map.h"

using hd_map::Area;
using hd_map::CrossWalk;
using hd_map::Lane;
using hd_map::Line;
using hd_map::Node;
using hd_map::Point;
using hd_map::Pole;
using hd_map::RoadMark;
using hd_map::RoadPole;
using hd_map::RoadSign;
using hd_map::Signal;
using hd_map::StopLine;
using hd_map::Vector;
using hd_map::WhiteLine;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "map");
  ros::NodeHandle n;
  ros::Rate r(30);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100000);
  ros::Publisher hdmap_pub = n.advertise<hdmap_pub::hdmap>("hdmap_marker", 100000);
    

    while (ros::ok())
        {
            
            visualization_msgs:: Marker createMarker ( const  std :: string &ns, int id, int type)  {
                
                visualization_msgs::Marker marker;

                marker.header.frame_id = "map" ;
                marker.ns = ns;
                marker.id = id;
                marker.type = type;
                marker.lifetime = ros::Duration();
                marker.frame_locked = true ;
                disableMarker(marker); //action:delete
                return marker;
                
            }
                
            void  enableMarker (visualization_msgs::Marker &marker)  {
                    marker.action = visualization_msgs::Marker::ADD;   //add marker
                 }
            void  disableMarker (visualization_msgs::Marker &marker)  {
                    marker.action = visualization_msgs::Marker::DELETE; //del marker
                 }
            bool  isValidMarker ( const visualization_msgs::Marker &marker)  {
                return marker.action == visualization_msgs::Marker::ADD;
            }
    
                
            double convertDegreeToRadian ( double degree){
                return degree * M_PI / 180 ;
            }
            double convertRadianToDegree ( double radian){
                return radian * 180 / M_PI;
            }
                    
            geometry_msgs::Point convertPointToGeomPoint ( const Point &point)  {
                geometry_msgs::Point geom_point;
                geom_point.x = point.ly;
                geom_point.y = point.bx ;
                geom_point.z = point.h;
                return geom_point; }
            geometry_msgs::Point convertGeomPointToPoint ( const Point &geom_point)
            {    geometry_msgs:: Point point;
                point.bx = geom_point.y;
                point.ly = geom_point.x;
                point.h = geom_point.z;
                return point; }

            geometry_msgs::Quaternion convertVectorToGeomQuaternion ( const Vector & vector )
            {
                // convert vertical angle to pitch angle
                double pitch = convertDegreeToRadian( vector .vang - 90);
                // convert horizo​​ntal angle to yaw angle
                double yaw = convertDegreeToRadian(- vector .hang + 90 );
              
                return tf::createQuaternionMsgFromRollPitchYaw( 0 , pitch, yaw); }
                
            Vector convertGeomQuaternionToVector ( const geometry_msgs::Quaternion &geom_quaternion)
                {
                tf:: Quaternion quaternion (geom_quaternion.x, geom_quaternion.y, geom_quaternion.z, geom_quaternion.w) ;
                    double roll, pitch, yaw;
                    tf::Matrix3x3(quaternion).getRPY (roll, pitch, yaw);
                    Vector vector ;
                    vector .vang = convertRadianToDegree(pitch) + 90;
                    vector .hang = -convertRadianToDegree(yaw) + 90 ;
                    return vector ; }
            
            //use point to repose sphere
            visualization_msgs:: Marker createPointMarker ( const  std :: string &ns, int id, Color color, const Point &point)
            {
                visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::SPHERE);
                if (point.pid == 0 ) //pid>0 pid=point id
                    return marker;
                //position use GeomPint
                marker.pose.position = convertPointToGeomPoint(point);
                //orientation use GeomQuaternion
                marker.pose.orientation = convertVectorToGeomQuaternion(Vector());
                marker.scale.x = MAKER_SCALE_POINT;
                marker.scale.y = MAKER_SCALE_POINT;
                marker.scale.z = MAKER_SCALE_POINT;
                marker.color = createColorRGBA(color);
                if (marker.color.a == COLOR_VALUE_MIN)
                    return marker;

                enableMarker(marker);
                return marker; }
            
            // set vector
            visualization_msgs:: Marker createVectorMarker ( const  std :: string &ns, int id, Color color, const HDMap &hdmap, const Vector & vector )
            {
                visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::ARROW); //vector
                if ( vector .vid == 0 )   //the id start for no.1
                return marker;
                
                Point point = hdmap.findByKey(Key<Point>( vector .pid)); //find end point
                if (point.pid == 0 )   // the id start for no.1
                return marker;
                
                // set marker
                marker.pose.position = convertPointToGeomPoint(point);
                marker.pose.orientation = convertVectorToGeomQuaternion( vector );
                //hang is orientation of horizontal angle
                hang=90-yaw;
                // vang is orientation of vertical angle
                vang=pitch+90;
                marker.scale.x = MAKER_SCALE_VECTOR_LENGTH;
                marker.scale.y = MAKER_SCALE_VECTOR;
                marker.scale.z = MAKER_SCALE_VECTOR;
                marker.color = createColorRGBA(color);
                if (marker.color.a == COLOR_VALUE_MIN)
                    return marker;
                

          enableMarker(marker);
                return marker; }
             
          //set line
        visualization_msgs:: Marker createLineMarker ( const  std :: string &ns, int id, Color color, const HDMap &hdmap, const Line &line)
            {
                
            visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::LINE_STRIP); //line
            
            if (line.lid == 0 )   // the id start for no.1
                return marker;
            
            Point bp = hdmap.findByKey(Key<Point>(line.bpid)); //find start pt and end pt and link it
            if (bp.pid == 0 ) // start pt id
                return marker;
            Point fp = hdmap.findByKey(Key<Point>(line.fpid)); //end pt
            if (fp.pid == 0 ) //end pt id
                return marker;
                
            marker.points.push_back(convertPointToGeomPoint(bp));
            marker.points.push_back(convertPointToGeomPoint(fp));
            marker.scale.x = MAKER_SCALE_LINE;
            marker.color = createColorRGBA(color);
            if (marker.color.a == COLOR_VALUE_MIN)
                return marker;
                
            enableMarker(marker);
            return marker;
            
        }
            
        //set area
        visualization_msgs:: Marker createAreaMarker ( const  std :: string &ns, int id, Color color, const HDMap &hdmap, const Area &area)
            {
                
            visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::LINE_STRIP);
                if (area.aid == 0 )
                    return marker;
                
                Line line = hdmap.findByKey(Key<Line>(area.slid)); // use line id to find line
                if (line.lid == 0 )
                    return marker;
                if (line.blid != 0 ) // must set beginning line
                    return marker;

                while (line.flid != 0 ) { //start from this line, find all the relate line
                    Point bp = hdmap.findByKey(Key<Point>(line.bpid));
                    if (bp.pid == 0 )
                        return marker;
                    Point fp = hdmap.findByKey(Key<Point>(line.fpid));
                    if (fp.pid == 0 )
                        return marker;
  
                    marker.points.push_back(convertPointToGeomPoint(bp));
                    marker.points.push_back(convertPointToGeomPoint(fp));

                    line = hdmap.findByKey(Key<Line>(line.flid)); //find the next relate line id
                    if (line.lid == 0 )
                        return marker;   }
 
                //find the start pt and end pt of the last line
                Point bp = hdmap.findByKey(Key<Point>(line.bpid));
                if (bp.pid == 0 )
                    return marker;
                Point fp = hdmap.findByKey(Key<Point>(line.fpid));
                if (fp.pid == 0 )
                    return marker;
                //set marker info.
                marker.points.push_back(convertPointToGeomPoint(bp));
                marker.points.push_back(convertPointToGeomPoint(fp));
                marker.scale.x = MAKER_SCALE_AREA;
                marker.color = createColorRGBA(color);
                if (marker.color.a == COLOR_VALUE_MIN)
                    return marker;
                
                enableMarker(marker);
                return marker;
                    
                }
            //set pole
            visualization_msgs:: Marker createPoleMarker ( const  std :: string &ns, int id, Color color, const HDMap &hdmap, const Pole &pole)
            {
                visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::CYLINDER);
                
                if (pole.plid == 0 ) //plid=pole id
                    return marker;
                if (pole.length == 0 || pole.dim == 0 )
                    return marker;

                Vector vector = hdmap.findByKey(Key<Vector>(pole.vid));  if ( vector .vid == 0 )
                    return marker;
                if ( vector .vang != 0 )
                    return marker;
   
                Point point = hdmap.findByKey(Key<Point>(vector .pid)); if (point.pid == 0 )
                    return marker;
                
                geometry_msgs::Point geom_point = convertPointToGeomPoint(point);
                geom_point.z += pole.length / 2 ;
                marker.pose.position = geom_point;
                vector .vang -= 90 ;
                marker.pose.orientation = convertVectorToGeomQuaternion( vector );
                marker. scale.x = pole.dim;
                marker.scale.y = pole.dim;
                marker.scale.z = pole.length;
                marker.color = createColorRGBA(color);
                if (marker.color.a == COLOR_VALUE_MIN)
                    return marker ;
        
                enableMarker(marker);
                return marker;
            }
            
            //set Lane
            bool isBranchingLane(const Lane& lane)
            {
              return lane.jct == Lane::LEFT_BRANCHING || lane.jct == Lane::RIGHT_BRANCHING || lane.jct == Lane::COMPOSITION;
            }

            bool isMergingLane(const Lane& lane)
            {
              return lane.jct == Lane::LEFT_MERGING || lane.jct == Lane::RIGHT_MERGING || lane.jct == Lane::COMPOSITION;
            }
            
            visualization_msgs:: MarkerArray createLaneMarkerArray ( const  std :: string &ns, int id, Color color, const HDMap &hdmap, const Lane &lane)
            {
                visualization_msgs::Marker marker = createMarker(ns, id, visualization_msgs::Marker::LINE_STRIP);
                if (lane.lnid == 0 )//line id
                    return marker;
                if (lane.did == 0 )//distance id
                    return marker;
                
                Point findStartPoint(const HDMap& hdmap, const Lane& lane)
                {
                  Point start_point;
                  Node node = HDmap.findByKey(Key<Node>(lane.bnid));
                  if (node.nid == 0)
                    return start_point;
                  return HDmap.findByKey(Key<Point>(node.pid));
                }

                Point findEndPoint(const HDMap& hdmap, const Lane& lane)
                {
                  Point end_point;
                  Node node = HDmap.findByKey(Key<Node>(lane.fnid));
                  if (node.nid == 0)
                    return end_point;
                  return HDmap.findByKey(Key<Point>(node.pid));
                }

                Point createMedianPoint(const Point& p1, const Point& p2)
                {
                  Point point;
                  point.bx = (p1.bx + p2.bx) / 2;
                  point.ly = (p1.ly + p2.ly) / 2;
                  point.h = (p1.h + p2.h) / 2;
                  return point;
                }

                std::vector<Point> findStartPoints(const HDMap& hdmap)
                {
                  std::vector<Point> start_points;
                  for (const auto& lane : HDmap.findByFilter([](const Lane& lane){return true;}))
                  {
                    Node node = HDmap.findByKey(Key<Node>(lane.bnid));
                    if (node.nid == 0)
                      continue;
                    Point point = HDmap.findByKey(Key<Point>(node.pid));
                    if (point.pid == 0)
                      continue;
                    start_points.push_back(point);
                  }
                  return start_points;
                }

                std::vector<Point> findEndPoints(const HDMap& hdmap)
                {
                  std::vector<Point> end_points;
                  for (const auto& lane : HDmap.findByFilter([](const Lane& lane){return true;}))
                  {
                    Node node = HDmap.findByKey(Key<Node>(lane.fnid));
                    if (node.nid == 0)
                      continue;
                    Point point = HDmap.findByKey(Key<Point>(node.pid));
                    if (point.pid == 0)
                      continue;
                    end_points.push_back(point);
                  }
                  return end_points;
                }

                Point findNearestPoint(const std::vector<Point>& points, const Point& base_point)
                {
                  Point nearest_point;
                  double min_distance = DBL_MAX;
                  for (const auto& point : points)
                  {
                    double distance = computeDistance(base_point, point);
                    if (distance <= min_distance)
                    {
                      nearest_point = point;
                      min_distance = distance;
                    }
                  }
                  return nearest_point;
                }

                std::vector<Point> findNearPoints(const std::vector<Point>& points, const Point& base_point, double radius)
                {
                  std::vector<Point> near_points;
                  for (const auto& point : points)
                  {
                    if (computeDistance(base_point, point) <= radius)
                      near_points.push_back(point);
                  }
                  return near_points;
                }

                std::vector<Lane> findLanesByStartPoint(const HDMap& hdmap, const Point& start_point)
                {
                  std::vector<Lane> lanes;
                  for (const auto& node : HDmap.findByFilter([&start_point](const Node& node){return node.pid == start_point.pid;}))
                  {
                    for (const auto& lane : HDmap.findByFilter([&node](const Lane& lane){return lane.bnid == node.nid;}))
                      lanes.push_back(lane);
                  }
                  return lanes;
                }

                std::vector<Lane> findLanesByEndPoint(const HDMap& hdmap, const Point& end_point)
                {
                  std::vector<Lane> lanes;
                  for (const auto& node : HDmap.findByFilter([&end_point](const Node& node){return node.pid == end_point.pid;}))
                  {
                    for (const auto& lane : HDmap.findByFilter([&node](const Lane& lane){return lane.fnid == node.nid;}))
                      lanes.push_back(lane);
                  }
                  return lanes;
                }

                Lane findStartLane(const HDMap& hdmap, const std::vector<Point>& points, double radius)
                {
                  Lane start_lane;
                  if (points.size() < 2)
                    return start_lane;

                  Point bp1 = points[0];
                  Point bp2 = points[1];
                  double max_score = -DBL_MAX;
                  for (const auto& p1 : findNearPoints(findStartPoints(hdmap), bp1, radius))
                  {
                    for (const auto& lane : findLanesByStartPoint(hdmap, p1))
                    {
                      if (lane.lnid == 0)
                        continue;
                      Point p2 = findEndPoint(hdmap, lane);
                      if (p2.pid == 0)
                        continue;
                      double score = computeScore(bp1, bp2, p1, p2, radius);
                      if (score >= max_score)
                      {
                        start_lane = lane;
                        max_score = score;
                      }
                    }
                  }
                  return start_lane;
                }

                Lane findEndLane(const HDMap& hdmap, const std::vector<Point>& points, double radius)
                {
                  Lane end_lane;
                  if (points.size() < 2)
                    return end_lane;

                  Point bp1 = points[points.size() - 2];
                  Point bp2 = points[points.size() - 1];
                  double max_score = -DBL_MAX;
                  for (const auto& p2 : findNearPoints(findEndPoints(hdmap), bp2, radius))
                  {
                    for (const auto& lane : findLanesByEndPoint(hdmap, p2))
                    {
                      if (lane.lnid == 0)
                        continue;
                      Point p1 = findStartPoint(hdmap, lane);
                      if (p1.pid == 0)
                        continue;
                      double score = computeScore(bp2, bp1, p2, p1, radius);
                      if (score >= max_score)
                      {
                        end_lane = lane;
                        max_score = score;
                      }
                    }
                  }
                  return end_lane;
                }

                Lane findNearestLane(const HDMap& hdmap, const std::vector<Lane>& lanes, const Point& base_point)
                {
                  Lane nearest_lane;
                  double min_distance = DBL_MAX;
                  for (const auto& lane : lanes)
                  {
                    Point start_point = findStartPoint(hdmap, lane);
                    if (start_point.pid == 0)
                      continue;
                    Point end_point = findEndPoint(hdmap, lane);
                    if (end_point.pid == 0)
                      continue;
                    Point median_point = createMedianPoint(start_point, end_point);
                    double distance = computeDistance(base_point, median_point);
                    if (distance <= min_distance)
                    {
                      nearest_lane = lane;
                      min_distance = distance;
                    }
                  }
                  return nearest_lane;
                }

                std::vector<Lane> findNearLanes(const HDMap& hdmap, const std::vector<Lane>& lanes, const Point& base_point,
                                                double radius)
                {
                  std::vector<Lane> near_lanes;
                  for (const auto& lane : lanes)
                  {
                    Point start_point = findStartPoint(hdmap, lane);
                    if (start_point.pid == 0)
                      continue;
                    Point end_point = findEndPoint(hdmap, lane);
                    if (end_point.pid == 0)
                      continue;
                    Point median_point = createMedianPoint(start_point, end_point);
                    if (computeDistance(base_point, median_point) <= radius)
                      near_lanes.push_back(lane);
                  }
                  return near_lanes;
                }

                std::vector<Lane> createFineLanes(const HDMap& hdmap, const autoware_msgs::lane& waypoints, double radius,
                                                  int loops)
                {
                  std::vector<Lane> null_lanes;

                  std::vector<Point> coarse_points;
                  for (const auto& waypoint : waypoints.waypoints)
                    coarse_points.push_back(convertGeomPointToPoint(waypoint.pose.pose.position));//输入的autoware_msgs::lane路径

                  Lane start_lane = findStartLane(hdmap, coarse_points, radius);//find startlane
                  if (start_lane.lnid == 0)
                    return null_lanes;

                  Lane end_lane = findEndLane(hdmap, coarse_points, radius);//find end lane
                  if (end_lane.lnid == 0)
                    return null_lanes;

                  std::vector<Lane> fine_lanes;//the lane is planned
                  Lane current_lane = start_lane;
                  for (int i = 0; i < loops; ++i)
                  {
                    fine_lanes.push_back(current_lane);//add lane to the planned path
                    if (current_lane.lnid == end_lane.lnid)//finish planning
                      return fine_lanes;

                    if (isBranchingLane(current_lane))
                    {
                      Point fine_p1 = findEndPoint(hdmap, current_lane);//find the end pt. of current lane
                      if (fine_p1.pid == 0)
                        return null_lanes;

                      Point coarse_p1 = findNearestPoint(coarse_points, fine_p1); // certainly succeed
                            // Find the point in the input path that is closest to the end of the current lane
                      if (computeDistance(fine_p1, coarse_p1) > radius)//If out of range
                        return null_lanes;

                      Point coarse_p2;
                      double distance = -DBL_MAX;
                      for (const auto& coarse_point : coarse_points)//Find the closest point outside the preview distance
                      {
                        if (distance == -DBL_MAX)
                        { //if it is start pt.
                          if (coarse_point.bx == coarse_p1.bx && coarse_point.ly == coarse_p1.ly)
                            distance = 0;
                          continue;
                        }
                        coarse_p2 = coarse_point;
                        distance = computeDistance(coarse_p2, coarse_p1);
                        if (distance > radius)
                          break;
                      }
                      if (distance <= 0)
                        return null_lanes;

                      double max_score = -DBL_MAX;
                      Filter<Lane> is_next_lane = [&current_lane](const Lane& lane)
                        {
                          return lane.lnid == current_lane.flid || lane.lnid == current_lane.flid2 ||
                                 lane.lnid == current_lane.flid3 || lane.lnid == current_lane.flid4;
                        };
                      for (const auto& lane : hdmap.findByFilter(is_next_lane))
                      {
                        Lane next_lane = lane;
                        Point next_point = findEndPoint(hdmap, next_lane);
                        if (next_point.pid == 0)
                          continue;
                        Point fine_p2 = next_point;
                        while (computeDistance(fine_p2, fine_p1) <= radius && !isBranchingLane(next_lane) && next_lane.flid != 0)
                        {//When fine_p2 is within the range of fine_p1 and the next path is not a BranchingLane
                          next_lane = hdmap.findByKey(Key<Lane>(next_lane.flid));;
                          if (next_lane.lnid == 0)
                            break;
                          next_point = findEndPoint(hdmap, next_lane);
                          if (next_point.pid == 0)
                            break;
                          fine_p2 = next_point;
                        }
                        double score = computeScore(fine_p1, fine_p2, coarse_p1, coarse_p2, radius);
                        if (score >= max_score)
                        {
                          current_lane = lane;
                          max_score = score;
                        }
                      }
                      if (max_score == -DBL_MAX)
                        return null_lanes;
                    }
                    else
                      current_lane = hdmap.findByKey(Key<Lane>(current_lane.flid));;
                    if (current_lane.lnid == 0)
                      return null_lanes;
                  }

                  return null_lanes;
                }
//                Line li = hdmap.findByKey(Key<Line>(lane.blid)); // use line id to find line
//                if (li.lid == 0 )
//                    return marker;
//                if (li.blid != 0 ) // must set beginning line
//                    return marker;
//
//
//                while (line.flid != 0 ) { //start from this line, find all the relate line
//                    Point bp = hdmap.findByKey(Key<Point>(lane.bpid));
//                    if (bp.pid == 0 )
//                        return marker;
//                    Point fp = hdmap.findByKey(Key<Point>(lane.fpid));
//                    if (fp.pid == 0 )
//                        return marker;
//
//                    marker.points.push_back(convertPointToGeomPoint(bp));
//                    marker.points.push_back(convertPointToGeomPoint(fp));
//
//                line = hdmap.findByKey(Key<Line>(lane.flid)); //find the next relate line id
//                    if (line.lid == 0 )
//                        return marker;   }
//                //find the start pt and end pt of the last line
//                Point bp = hdmap.findByKey(Key<Point>(lane.bpid));
//                if (bp.pid == 0 )
//                    return marker;
//                Point fp = hdmap.findByKey(Key<Point>(lane.fpid));
//                if (fp.pid == 0 )
//                    return marker;
//                //set marker info.
//                marker.points.push_back(convertPointToGeomPoint(bp));
//                marker.points.push_back(convertPointToGeomPoint(fp));
//                marker.scale.x = MAKER_SCALE_LANE;
//                marker.color = createColorRGBA(color);
//                if (marker.color.a == COLOR_VALUE_MIN)
//                    return marker;
//
//                enableMarker(marker);
//                return marker;
 
            }
            
                //set white line
            visualization_msgs:: MarkerArray createWhiteLineMarkerArray ( const HDMap &hdmap, Color white_color, Color yellow_color)  {
              visualization_msgs::MarkerArray marker_array;
                int id = 0 ;
                for ( const auto &white_line : hdmap.findByFilter([]( const WhiteLine &white_line) { return true ; } ))
                { if (white_line.lid == 0 ) { //no line
                    ROS_ERROR_STREAM( "[createWhiteLineMarkerArray] invalid white_line: " << white_line);
                    continue ;
                }
              
                  if (white_line.type == WhiteLine::DASHED_LINE_BLANK) // if invisible line空心虛線
                      continue ;
                     
                  Line line = hdmap.findByKey(Key<Line>(white_line.lid)); //find the first start line
                    if (line.lid == 0 )
                    { //find lid>0
                        ROS_ERROR_STREAM( "[createWhiteLineMarkerArray] invalid line: " << line);
                        continue ;
                    }

                  if (line.blid == 0 ) // if beginning line
                  {
                      visualization_msgs::Marker marker;
                      switch (white_line.color) //white_line and Yellow_line
                      {
                          case 'W' :
                              marker = createLinkedLineMarker( "white_line" , id++, white_color, hdmap, line);
                              break ;
                          case 'Y' :
                              marker = createLinkedLineMarker( "white_line" , id++, yellow_color, hdmap, line);
                              break ;
                          default :
                              ROS_ERROR_STREAM( "[createWhiteLineMarkerArray] unknown white_line.color: "<< white_line);
                          continue ;
                              
                      }
                      
                      if (isValidMarker(marker)) //add
                          marker_array.markers.push_back(marker);
                      else
                          ROS_ERROR_STREAM ( "[createWhiteLineMarkerArray] failed createLinkedLineMarker: " << line);                  }
                return marker_array;
                
            }
                 //set stop line
                visualization_msgs:: MarkerArray createStopLineMarkerArray ( const HDMap &hdmap, Color color)
                {
                    visualization_msgs::MarkerArray marker_array;
                    int id = 0 ;
                    for ( const auto &stop_line : hdmap.findByFilter([]( const StopLine &stop_line) { return true ; }))
                    {
                        if (stop_line.lid == 0 )
                        {
                            ROS_ERROR_STREAM( "[createStopLineMarkerArray] invalid stop_line: " << stop_line); continue ;
                            
                        }
        
                        Line line = hdmap.findByKey(Key<Line>(stop_line.lid));
                        if (line.lid == 0 )
                        {
                            ROS_ERROR_STREAM( "[createStopLineMarkerArray] invalid line: " << line);
                            continue ;
                        }

                        if (line.blid == 0 ) // if beginning line is the first line,the continuous line is related
                        {
                            visualization_msgs::Marker marker = createLinkedLineMarker( "stop_line" , id++, color, hdmap, line); //show each line
                            if (isValidMarker(marker))
                                marker_array.markers.push_back(marker);
                            else
                                ROS_ERROR_STREAM( "[createStopLineMarkerArray] failed createLinkedLineMarker: " << line);              }
                        return marker_array;
                        
                    }
                }
                     //set road mark
                    visualization_msgs:: MarkerArray createRoadMarkMarkerArray ( const HDMap &hdmap, Color color)
                    {
                           visualization_msgs::MarkerArray marker_array;
                        int id = 0 ;
                        for ( const auto &road_mark : hdmap.findByFilter([]( const RoadMark &road_mark) { return true ; }))
                        {
                            if (road_mark.aid == 0 ) {
                                ROS_ERROR_STREAM( "[createRoadMarkMarkerArray] invalid road_mark: " << road_mark);
                                continue ;
                            }

                               Area area = hdmap.findByKey(Key<Area>(road_mark.aid));
                            if (area.aid == 0 ) {
                                ROS_ERROR_STREAM( "[createRoadMarkMarkerArray] invalid area: " << area);
                                continue ;
                            }
          
                               visualization_msgs::Marker marker = createAreaMarker( "road_mark" , id++, color, hdmap, area);
                            if (isValidMarker(marker))
                                marker_array.markers.push_back(marker);
                            else
                                ROS_ERROR_STREAM( "[createRoadMarkMarkerArray] failed createAreaMarker: " << area);
                            
                        }
                        return marker_array;
                    }
                               
                    //set cross walk
                    visualization_msgs:: MarkerArray createCrossWalkMarkerArray ( const HDMap &hdmap, Color color)
                    {
                           visualization_msgs::MarkerArray marker_array;
                        int id = 0 ;
                        for ( const auto &cross_walk : hdmap.findByFilter([]( const CrossWalk &cross_walk) { return true ; }))
                        {
                            if (cross_walk.aid == 0 )
                            {
                                ROS_ERROR_STREAM( "[createCrossWalkMarkerArray] invalid cross_walk: " << cross_walk);
                                continue ;
                            }

                               Area area = hdmap.findByKey(Key<Area>(cross_walk.aid)); //use aid find area
                            if (area.aid == 0 ) {
                                ROS_ERROR_STREAM( "[createCrossWalkMarkerArray] invalid area: " << area);
                                continue ;
                            }
                               
                               visualization_msgs::Marker marker = createAreaMarker( "cross_walk" , id++, color, hdmap, area);
                            if (isValidMarker(marker))
                                marker_array.markers.push_back(marker);
                            else
                                ROS_ERROR_STREAM( "[createCrossWalkMarkerArray] failed createAreaMarker: " << area);
                            
                        }
                        return marker_array;
                        
                    }
                               
                    //set road sign
                    visualization_msgs:: MarkerArray   createRoadSignMarkerArray ( const HDMap &hdmap, Color sign_color, Color pole_color)
                    {
                            visualization_msgs::MarkerArray marker_array;
                        int id = 0 ;
                        for ( const auto &road_sign : hdmap.findByFilter([]( const RoadSign &road_sign) { return true ; } ))
                        {
                            if (road_sign.vid == 0 ) {
                                ROS_ERROR_STREAM( "[createRoadSignMarkerArray] invalid road_sign: " << road_sign);
                                continue ;
                            }
                            //find vector
                                 Vector vector = hdmap.findByKey(Key<Vector>(road_sign.vid));
                            if ( vector .vid == 0 ) {
                                ROS_ERROR_STREAM( "[createRoadSignMarkerArray] invalid vector: " << vector );
                                continue ;             }
                            //find pole
                            Pole pole;
                            if (road_sign.plid != 0 ) {
                                pole = hdmap.findByKey(Key<Pole>(road_sign.plid));
                                if (pole.plid == 0 ) {
                                    ROS_ERROR_STREAM( "[createRoadSignMarkerArray] invalid pole: " << pole);
                                    continue ;
                                }
                                
                            }
                            //create vector
                            visualization_msgs::Marker vector_marker = createVectorMarker( "road_sign" , id++, sign_color, hdmap, vector );
                            if (isValidMarker(vector_marker))
                                marker_array.markers.push_back(vector_marker) ;
                            else
                                ROS_ERROR_STREAM( "[createRoadSignMarkerArray] failed createVectorMarker: " << vector ); //create pole
                            if (road_sign.plid != 0 ) {
                                visualization_msgs::Marker pole_marker = createPoleMarker( "road_sign", id++, pole_color, hdmap, pole);
                                if (isValidMarker(pole_marker))
                                    marker_array.markers.push_back(pole_marker);
                                else
                                    ROS_ERROR_STREAM( "[createRoadSignMarkerArray] failed createPoleMarker: " << pole);
                            }
                            
                        }
                        return marker_array;
                        
                    }

                            //set traffic light
                    visualization_msgs:: MarkerArray createSignalMarkerArray ( const HDMap &hdmap, Color red_color, Color green_color, Color yellow_color, Color other_color, Color pole_color)
                {
                        visualization_msgs::MarkerArray marker_array;
                        int id = 0 ;
                        for ( const auto &signal : hdmap.findByFilter([]( const Signal &signal) { return true ; })) {
                            if (signal.vid == 0 ) {
                                ROS_ERROR_STREAM( "[createSignalMarkerArray] invalid signal: " << signal);
                                continue ;
                            }
                        }

                            Vector vector = hdmap.findByKey(Key<Vector>(signal.vid)); //use vector to show traffic light
                            if ( vector .vid == 0 ) {
                                ROS_ERROR_STREAM( "[createSignalMarkerArray] invalid vector: " << vector );
                                continue ;
                            }

                            Pole pole;
                            if (signal.plid != 0 ) {
                                pole = hdmap.findByKey(Key<Pole>(signal.plid)); //find related pole
                                if (pole.plid == 0 ) {
                                    ROS_ERROR_STREAM( " [createSignalMarkerArray] invalid pole: " << pole);
                                    continue ;
                                }
                            }
                        }

                            visualization_msgs::Marker vector_marker;
                            switch (signal.type) { //traffic light color is red yellow green
                                case Signal::RED:
                                case Signal::PEDESTRIAN_RED:
                                vector_marker = createVectorMarker( "signal" , id++, red_color, hdmap, vector );
                                    break ;
                                case Signal::green:
                                case Signal::PEDESTRIAN_GREEN:
                                    vector_marker = createVectorMarker( "signal" , id++, green_color, hdmap, vector );
                                    break ;
                                case Signal::YELLOW:
                                    vector_marker = createVectorMarker( "signal", id++, yellow_color, hdmap, vector );
                                    break ;
                                case Signal::RED_LEFT:
                                    vector_marker = createVectorMarker( "signal" , id++, Color::LIGHT_RED, hdmap, vector );
                                    break ;
                                case Signal::GREEN_LEFT:
                                    vector_marker = createVectorMarker( "signal" , id++, Color::LIGHT_GREEN, hdmap, vector );
                                    break ;
                                case Signal::YELLOW_LEFT:
                                    vector_marker = createVectorMarker( "signal" , id++, Color::LIGHT_YELLOW, hdmap, vector );
                                    break ;
                                case Signal::OTHER:
                                    vector_marker = createVectorMarker( "signal" , id++, other_color, hdmap, vector );
                                break ;
                                default :
                                ROS_WARN_STREAM( "[createSignalMarkerArray] unknown signal.type: " << signal.type << " Creating Marker as OTHER." );
                                vector_marker = createVectorMarker( "signal" , id++, Color::GRAY, hdmap, vector );
                                break ;
                                    
                            }
                            if (isValidMarker(vector_marker)) // action:ADD
                                marker_array.markers.push_back(vector_marker);
                            else
                                ROS_ERROR_STREAM( "[createSignalMarkerArray] failed createVectorMarker: " << vector );

                            if (signal.plid != 0 ) { //create pole
                                 visualization_msgs::Marker pole_marker = createPoleMarker( "signal" , id++, pole_color, hdmap, pole);
                                if (isValidMarker(pole_marker))
                                    marker_array.markers.push_back(pole_marker) ;
                                else
                                    ROS_ERROR_STREAM( "[createSignalMarkerArray] failed createPoleMarker: " << pole);
                            }
                        }
                        return marker_array;
                    }
    //pub publish
    marker_pub.publish(SPHERE);
    marker_pub.publish(ARROW);
    marker_pub.publish(LINE_STRIP);
    marker_pub.publish(CYLINDER);
   
    

    //pub marker information
    ros::Publisher point_pub = nh.advertise<PointArray>( "hd_map_info/point" , 1 , true );
    ros::Publisher vector_pub = nh.advertise<VectorArray>( "hd_map_info/vector" , 1 , true );
    ros::Publisher line_pub = nh.advertise<LineArray>( "hd_map_info/line" , 1 , true );
    ros::Publisher area_pub = nh.advertise<AreaArray>( "hd_map_info/area" , 1 , true );
    ros::Publisher pole_pub = nh.advertise<PoleArray>( "hd_map_info/pole", 1 ,true );
    ros::Publisher node_pub = nh.advertise<NodeArray>( "hd_map_info/node" , 1 , true );
    ros::Publisher lane_pub = nh.advertise<LaneArray>( "hd_map_info/lane" , 1 , true );
    ros::Publisher white_line_pub = nh.advertise<WhiteLineArray>( "hd_map_info/white_line" , 1 ,true );
    ros::Publisher signal_pub = nh.advertise<SignalArray>( "hd_map_info/signal" , 1 , true );
    ros::Publisher stop_line_pub = nh.advertise<StopLineArray>( "hd_map_info/stop_line" , 1 , true );
    ros::Publisher cross_walk_pub = nh.advertise<CrossWalkArray>( "hd_map_info/cross_walk" , 1 , true );
    ros::Publisher road_mark_pub = nh.advertise<RoadMarkArray>( "hd_map_info/road_mark" , 1 , true );
    ros::Publisher road_pole_pub = nh.advertise<RoadPoleArray>( "hd_map_info/road_pole" , 1 , true );
    ros::Publisher road_sign_pub = nh.advertise<RoadSignArray>( "hd_map_info/road_sign" , 1 , true );
    
    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>( "hd_map" , 1 , true );

      //pub csv
    hd_map:: category_t category = Category::NONE;
    for ( const auto &file_path : file_paths) {
        std :: string file_name (basename(file_path.c_str())) ;
        if (file_name == "point.csv" )
        {
            point_pub.publish(createObjectArray<Point, PointArray>(file_path));
            category |= Category::POINT;
        }
        else if (file_name == "vector.csv" ) {
               vector_pub.publish(createObjectArray<Vector, VectorArray>(file_path));
               category |= Category::VECTOR;
           } else  if (file_name == "line.csv" ) {
               line_pub.publish(createObjectArray<Line, LineArray>(file_path) );
               category |= Category::LINE;
           } else  if (file_name == "area.csv" ) {
               area_pub.publish(createObjectArray<Area, AreaArray>(file_path));
               category |= Category::AREA;
           } else  if (file_name == "pole.csv" ) {
               pole_pub.publish(createObjectArray<Pole, PoleArray>(file_path));
               category |= Category::POLE;
           } else  if (file_name == "node.csv" ) {
               node_pub.publish(createObjectArray<Node, NodeArray>(file_path));
               category |= Category::NODE;
           } else  if (file_name == "lane.csv" ) {
               lane_pub.publish(createObjectArray<Lane, LaneArray>(file_path) );
               category |= Category::LANE;
           } else  if (file_name == "whiteline.csv" ) {
               white_line_pub.publish(createObjectArray<WhiteLine, WhiteLineArray>(file_path));
               category |= Category::WHITE_LINE;
           } else  if (file_name == "stopline.csv" ) {
               stop_line_pub.publish(createObjectArray<StopLine, StopLineArray>(file_path) );
               category |= Category::STOP_LINE;
           }  else  if (file_name == "crosswalk.csv") {
               cross_walk_pub.publish(createObjectArray<CrossWalk, CrossWalkArray>(file_path));
               category |= Category::CROSS_WALK;
           } else  if (file_name == "road_surface_mark.csv" ) {
               road_mark_pub.publish(createObjectArray<RoadMark, RoadMarkArray>(file_path) );
               category |= Category::ROAD_MARK;
           } else  if (file_name == "poledata.csv" ) {
               road_pole_pub.publish(createObjectArray<RoadPole, RoadPoleArray>(file_path));
               category |= Category::ROAD_POLE;
           } else  if (file_name == "roadsign.csv") {
               road_sign_pub.publish(createObjectArray<RoadSign, RoadSignArray>(file_path));
               category |= Category::ROAD_SIGN;
           } else  if (file_name == "signaldata.csv" ) {
               signal_pub.publish(createObjectArray<Signal, SignalArray>(file_path) );
               category |= Category::SIGNAL;
           }
        //else  if (file_name == "intersection.csv" ) {
               //cross_road_pub.publish(createObjectArray<CrossRoad, CrossRoadArray>( file_path));
              // category |= Category::CROSS_ROAD;
           //}
           else
                ROS_ERROR_STREAM( "unknown csv file: " << file_path);
       }
    ros::spin();
    return EXIT_SUCCESS;
}
