/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "sspp/astar.h"

namespace SSPP
{

Astar::Astar(ros::NodeHandle & n, Robot *rob, int progressDisplayFrequency):
    nh(n),
    robot(rob),
    root(NULL),
    p(NULL),
    openList(NULL),
    closedList(NULL),
    globalcount(0),
    debug(false),
    progressDisplayFrequency(progressDisplayFrequency),
    debugDelay(0)
{    
    childPosePub         = nh.advertise<geometry_msgs::PoseArray>("child_pose",10);
    childSensorsPub      = nh.advertise<geometry_msgs::PoseArray>("child_sensors",10);
    parentPosePub        = nh.advertise<geometry_msgs::PoseArray>("parent_pose",10);
    parenSensorsPub      = nh.advertise<geometry_msgs::PoseArray>("parent_sensors",10);
    branchPub            = nh.advertise<visualization_msgs::Marker>("branch",10);
    octomapChildPub      = nh.advertise<octomap_msgs::Octomap>("octomap_child", 10);

    nodeToBeVisNum       = 0;//if 0 then it will continue planning till the target otherwise it will stop planning on the specified viewpoint number and will visualize the steps of choosing next viewpoint
    nodesCounter         = 0;
}

Astar::Astar():
    heuristic(NULL),
    root(NULL),
    p(NULL),
    openList(NULL),
    closedList(NULL),
    globalcount(0),
    debug(true)
{
}

Astar::~Astar()
{
    if(openList)
    {
        openList->free();
        delete openList;
    }
    if(closedList)
    {
        closedList->free();
        delete closedList;
    }
}

// find the nearest node to the start
void Astar::findRoot() throw (SSPPException)
{
    SearchSpaceNode * temp;
    if(!this->searchspace)
    {
        throw(SSPPException((char*)"No SearchSpace Defined"));
        return;
    }
    double distance,shortestDist = std::numeric_limits<double>::max();
    // allocate and setup the root node
    root = new Node;
    temp = this->searchspace;
    while(temp!=NULL)
    {
        distance = Dist(temp->location,start.p);
        // Initialize the root node information and put it in the open list
        if (distance < shortestDist)
        {
            shortestDist = distance;

            root->pose.p.position.x = temp->location.position.x;
            root->pose.p.position.y = temp->location.position.y;
            root->pose.p.position.z = temp->location.position.z;
            root->pose.p.orientation.x = temp->location.orientation.x;
            root->pose.p.orientation.y = temp->location.orientation.y;
            root->pose.p.orientation.z = temp->location.orientation.z;
            root->pose.p.orientation.w = temp->location.orientation.w;
            root->senPoses.erase(root->senPoses.begin(),root->senPoses.end());
            for(int i=0; i<temp->sensorLocation.poses.size();i++)
            {
                Pose tempPose;
                tempPose.p.position.x = temp->sensorLocation.poses[i].position.x;
                tempPose.p.position.y = temp->sensorLocation.poses[i].position.y;
                tempPose.p.position.z = temp->sensorLocation.poses[i].position.z;
                tempPose.p.orientation.x = temp->sensorLocation.poses[i].orientation.x;
                tempPose.p.orientation.y = temp->sensorLocation.poses[i].orientation.y;
                tempPose.p.orientation.z = temp->sensorLocation.poses[i].orientation.z;
                tempPose.p.orientation.w = temp->sensorLocation.poses[i].orientation.w;

                root->senPoses.push_back(tempPose);

            }
            root->id = temp->id;
        }
        temp = temp->next;
    }

    root->id = 0;
    root->parent = NULL;
    root->next = NULL;
    root->prev = NULL;
    root->g_value = 0;
    root->distance = 0;
    root->coverage = 0;
    root->h_value = 0;
    heuristic->calculateHeuristic(root);
    root->depth = 0;
    //Translate(root->pose,start.phi);
    std::cout<<"\n"<<"	---->>>Root is Set to be X="<<root->pose.p.position.x<<" Y="<<root->pose.p.position.y<<" Z="<<root->pose.p.position.z;
}


void Astar::setRobot(Robot*rob)
{
    this->robot = rob;
}

void Astar::setHeuristicFucntion(Heuristic* heuristicFun)
{
    heuristic = heuristicFun;
}

void Astar::displayOctree()
{
  //display the child and the octree
  geometry_msgs::PoseArray childPose,parentPose, childSensors, parentSensors;
  std::vector<geometry_msgs::Point> lineSegments;
  if(nodesCounter == nodeToBeVisNum && nodeToBeVisNum != 0)
  {
    geometry_msgs::Pose parent;

    geometry_msgs::Pose child;
    geometry_msgs::Point linePoint;

    std::cout<<"I entered tree childrens of the desired node "<<std::endl;

    child = curChild->pose.p;
    parent = current->pose.p;
    childPose.poses.push_back(child);
    parentPose.poses.push_back(parent);
    for(int i=0; i<curChild->senPoses.size();i++)
    {
      childSensors.poses.push_back(curChild->senPoses[i].p);
      parentSensors.poses.push_back(current->senPoses[i].p);
    }

    linePoint.x = current->pose.p.position.x;
    linePoint.y = current->pose.p.position.y;
    linePoint.z = current->pose.p.position.z;
    lineSegments.push_back(linePoint);
    linePoint.x = child.position.x;
    linePoint.y = child.position.y;
    linePoint.z = child.position.z;
    lineSegments.push_back(linePoint);

    //visualization
    childPose.header.frame_id= "map";
    childPose.header.stamp = ros::Time::now();
    childPosePub.publish(childPose);
    childSensors.header.frame_id= "map";
    childSensors.header.stamp = ros::Time::now();
    childSensorsPub.publish(childSensors);
    parentPose.header.frame_id= "map";
    parentPose.header.stamp = ros::Time::now();
    parentPosePub.publish(parentPose);
    parentSensors.header.frame_id= "map";
    parentSensors.header.stamp = ros::Time::now();
    parenSensorsPub.publish(parentSensors);
    visualization_msgs::Marker linesList1 = drawLines(lineSegments,1,6,100000,0.1);
    branchPub.publish(linesList1);


    octomap_msgs::Octomap octomap ;
    octomap.binary = 1 ;
    octomap.id = 1 ;
    octomap.resolution =0.25;
    octomap.header.frame_id = "map";
    octomap.header.stamp = ros::Time::now();
    bool res = octomap_msgs::fullMapToMsg(*curChild->octree, octomap);
    if(res)
    {
      octomapChildPub.publish(octomap);
    }
    else
    {
      ROS_WARN("OCT Map serialization failed!");
    }
    childSensors.poses.erase(childSensors.poses.begin(),childSensors.poses.end() );
    childPose.poses.erase(childPose.poses.begin(),childPose.poses.end() );
    lineSegments.erase(lineSegments.begin(), lineSegments.end());
    ros::Duration(2).sleep();
  }
}

Node *Astar::astarSearch(Pose start, bool continuous)
{
    Node *path = NULL;
    int NodesExpanded = 0;
    globalID = 0;
    bool condition;
    if(this->tree.size() > 0)
        this->tree.clear();
    if(!openList)
    {
        openList   = new LList;
    }
    if(!closedList)
    {
        closedList = new LList;
    }
    // Be sure that open and closed lists are empty
    openList->free();
    closedList->free();
    if(!this->searchspace)
    {
        std::cout<<"\nGenerate SearchSpace before Searching !!!";
        return NULL;
    }

    this->start.p.position.x = start.p.position.x;
    this->start.p.position.y = start.p.position.y;
    this->start.p.position.z = start.p.position.z;
    this->start.p.orientation.x = start.p.orientation.x;
    this->start.p.orientation.y = start.p.orientation.y;
    this->start.p.orientation.z = start.p.orientation.z;
    this->start.p.orientation.w = start.p.orientation.w;

    // std::vector<Eigen::Vector3i> ListofNodes; // Added Line
    // Eigen::Vector3i startcoords(start.p.position.x,start.p.position.y,start.p.position.z);// Added Line
    // ListofNodes.push_back(startcoords); // Added Line
    // // Prints out the list of nodes: 
    // std::cout<<" the List of Nodes is = "<<ListofNodes.size()<<" nodes Long\n"; // Added line 
    // for (auto i = ListofNodes.begin(); i != ListofNodes.end(); ++i){ // Added line 
    //     std::cout << *i << ' '; // Added line 
    //     }
    // std::cout<<"\n-------------------------------------------"; // Added line 
    int DeclineCounter = 0; 
    int GainCounter = 0;
    double maxCov = 0;  

    std::cout<<"\n	--->>> Search Started <<<---"<<std::endl;
    findRoot();

    std::vector<Eigen::Vector3i> ListofNodes; // Added Line
    Eigen::Vector3i rootcoords(root->pose.p.position.x,root->pose.p.position.y,root->pose.p.position.z);// Added Line
    ListofNodes.push_back(rootcoords); // Added Line
     // Prints out the list of nodes: 
    std::cout<<" the List of Nodes is = "<<ListofNodes.size()<<" nodes Long\n"; // Added line 
    for (auto i = ListofNodes.begin(); i != ListofNodes.end(); ++i){ // Added line 
        std::cout << *i << ' '; // Added line 
        }

    openList->add(root,heuristic->isCost());
    // while openList is not empty
    int count = 0;
    while(openList->Start != NULL)
    {
        if(progressDisplayFrequency > 0 && (count++%progressDisplayFrequency) == 0)
        {
            heuristic->displayProgress(tree);
            //seconds to usec
            if(debugDelay!=0)
                usleep(debugDelay*1000000);
        }
        // Get the node with the highest cost (first node) (it was the cheapest one before since we were taking the lower cost but now it is converted to a reward function)
        current = new Node(openList->getHead());
        openList->remove(openList->getHead());

        
        // std::cout<<" the current node f_value is = "<<current->f_value<<" \n"; // Added line 
        Eigen::Vector3i nodecoords(current->pose.p.position.x, current->pose.p.position.y, current->pose.p.position.z); // get current nodes position
        bool PointMatch = CheckRepeatPoint(ListofNodes,nodecoords);  // check to see if path has visited this point before  
        std::cout<<" PointMatch = "<<PointMatch<<" \n"; // Added line
        tf::Quaternion curNodeQT(current->pose.p.orientation.x,current->pose.p.orientation.y,current->pose.p.orientation.z,current->pose.p.orientation.w);
        pt = new Node(current); // define a separate node "pt" that's the same as the current 

        // if (closedList->Start)
        //     ClosedPt = new Node(closedList->Start);
        // else 
        //     ClosedPt = new Node(root);

        // ****Check to See if the current node is a repeat node *** 
       
        // bool PointMatch = CheckRepeatPoint(current,ClosedPt);
        // bool PointMatch = CheckForRepeatPoints(current, ClosedPt, continuous);

        double parentCoverage;
        if (!current->parent) // if the node doesn't have a parent (it's the root), give initial values for parent coverage
            parentCoverage = 0.0;
        else
            parentCoverage = current->parent->coverage;
        // std::cout<<" GainCounter = "<<GainCounter<<" \n"; // Added line
        // std::cout<<" DeclineCounter = "<<DeclineCounter<<" \n"; // Added line


        
        // double DesiredCoverageGain;
        // if (GainCounter < 8)
        //     DesiredCoverageGain = 0.25; // Make sure to change in coverage_path_planning_heuristic.cpp line 298 as well
        // else 
        //     DesiredCoverageGain = 0.05; // Make sure to change in coverage_path_planning_heuristic.cpp line 298 as well
        // bool CovGain = CheckCoverageGain(current,DesiredCoverageGain);
 
        // std::cout<<" the Original Pointmatch is "<<PointMatch<<" and the original CovGain is "<<CovGain<<" The DesiredCoverageGain is: "<<DesiredCoverageGain<<"\n"; //Want 0 and 1 
        // int j = 0; 
        // while (PointMatch || (!CovGain)) // while there is a repeated point (pointmatch = true) and the desired coverage gain is not met (covgain = false)
        // {
        //     if (PointMatch)
        //     {
        //        Node *ptmatch =  GetNextNonRepeatedPoint(current,continuous);
        //        current = new Node(ptmatch);
        //     }
        //     if (!CovGain)
        //     {
        //         Node *Cgain = GetNextPointwithDesiredCoverageGain(current, DesiredCoverageGain, parentCoverage);
        //         current = new Node(Cgain);
        //     }
        //     ClosedPt = new Node(closedList->Start); 
        //     PointMatch = CheckForRepeatPoints(current, ClosedPt, continuous);
        //     CovGain = CheckCoverageGain(current,DesiredCoverageGain);
        //     if (!CovGain) // Allows it to try 3 times before accepting a point that doesn't meet the desired coverage gain
        //         j++;
        //     if (j>3)
        //         break; 
        //     // std::cout<<" the Pointmatch is "<<PointMatch<<" and the CovGain is "<<CovGain<<" \n";
        // }


        // pt = new Node(current); // define a separate node "pt" that's the same as the current 
        // if (continuous) // If continuous=true, then the path will not have any repeat points
        // {
        //     if (PointMatch) // If there is a matcehd point, find the next highest point in the OpenList that doesn't match any point in the ClosedList
        //     {
        //         int j = 0;
        //         while(j<1) 
        //         {
        //             pt = pt->next; //get the next point from the OpenList
        //             ClosedPt = new Node(closedList->Start); 
        //             PointMatch = CheckForRepeatPoints(pt,ClosedPt,true); // checks for JUST coordinates to match
        //             openList->remove((openList->getHead())->next); //Takes away the point from the OpenList
        //             if(!PointMatch) // if the points do not match, add 1 to break it out of the while loop
        //                 j++;
        //             std::cout<<"Break Point 1... \n";
        //         }
        //         current = new Node(pt); //set the current node equal to the new "pt" node           
        //     }
        // }
        // else // if continuous=false, the path can have repeated points with different orientatations, but it cannot go back to a point already visited
        // {
        //     if (PointMatch)
        //     {
        //                         double dist;
        //         dist = Dist(current->pose.p,current->parent->pose.p); // distance between last point and current point 
        //         // std::cout<<"Distance is "<<dist<<"m \n"; 
        //         // Check to see if the exact position/orientation has been visited before
        //         while (dist ==0) // This loop finds the next point with a unique position and orientation
        //         {
        //             int k = 0; 
        //             ClosedPt = new Node(closedList->Start);
        //             PointMatch = CheckForRepeatPoints(pt,ClosedPt,false);
        //             if (PointMatch)
        //                 k++;
        //             if (k>0)
        //             {
        //                 current = new Node(current->next);
        //                 dist = Dist(current->pose.p,current->parent->pose.p); // distance between last point and current point 
        //             }
        //             else
        //                 break; 
        //         }
        //         if (!(dist == 0)) // if it's going back to a point the path has already visited (distance not 0), find the the next highest point in the openList that the path hasn't visited
        //         {
        //             // std::cout<<" the old coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n"; // Added line
        //             int i = 0;
        //             while(i<1) // for some reason didn't work when it was while(PointMatch)
        //             {
        //                 pt = pt->next;
        //                 ClosedPt = new Node(closedList->Start);
        //                 PointMatch = CheckForRepeatPoints(pt,ClosedPt,false);
        //                 Eigen::Vector3i nodecoords(pt->pose.p.position.x, pt->pose.p.position.y, pt->pose.p.position.z);
        //                 PointMatch = CheckRepeatPoint(ListofNodes,nodecoords);
        //                 if(!PointMatch) // add one if the points do not match to break it out of the while loop
        //                     i++;
        //             }
        //             current = new Node(pt);
        //             // std::cout<<" the NEW node f_value is = "<<current->f_value<<" \n"; // Added line
        //             // std::cout<<" the new coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n";
        //             openList->remove((openList->getHead())->next);
        //         }

                // double dist;
                // dist = Dist(current->pose.p,current->parent->pose.p); // distance between last point and current point
                // while (!(dist == 0) && (PointMatch==1)) // Check to see if if it's going back to a point the path has already visited (distance not 0), if so find the the next highest point in the openList that the path hasn't visited
                // {
                //     // std::cout<<" the old coord was  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n"; // Added line
                //     int i = 0;
                //     while(i<1) 
                //     {
                //         pt = pt->next;
                //         ClosedPt = new Node(closedList->Start); 
                //         PointMatch = CheckForRepeatPoints(pt,ClosedPt,false); // checks for the coordinates AND orientation to match (it can rotate at the same point)
                //         openList->remove((openList->getHead())->next); //Takes away the point from the OpenList
                //         if(!PointMatch) // add one if the points do not match to break it out of the while loop
                //             i++;
                //         // std::cout<<"Break Point 2... \n";
                //     }
                //     dist = Dist(pt->pose.p,current->parent->pose.p); // distance between pt and original parent point
                //     std::cout<<" the new coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n";
                // }
                // if (dist==0) // While dist is zero (UAV hasn't moved, just check it's not the same orientation )
                // {
                //     int j = 0;
                //     while(j<1) 
                //     {
                //         ClosedPt = new Node(closedList->Start); 
                //         PointMatch = CheckForRepeatPoints(pt,ClosedPt,false); // checks for just coordinates AND orientation to match
                //         if(!PointMatch) // if the points do not match, add 1 to break it out of the while loop
                //             j++;
                //         else
                //         {
                //             pt = pt->next; // if it was a match, try the next point in the open list
                //             openList->remove((openList->getHead())->next); //Takes away the point from the OpenList
                //         }
                //         std::cout<<"Break Point 3... \n";
                //     }
                //     // dist = Dist(pt->pose.p,current->parent->pose.p); // check the distance between the new point and the original parent point
                // }
                // current = new Node(pt);
                // std::cout<<"Break Point 4... \n";
        //     }
        // }




        // *******************************************MY ORIGINAL***********************************************
        if (continuous) // If continuous=True, path will not contain any points with multiple orientations. 
        { 
            if (PointMatch) // if the point was visited, keep checking the next point in the openList
            {
                std::cout<<"Break 1 \n"; 
                double dist;
                dist = Dist(current->pose.p,current->parent->pose.p); // distance between last point and current point 
                // std::cout<<"Distance is "<<dist<<"m \n"; 
                // Check to see if the exact position/orientation has been visited before
                while (dist ==0) // This loop finds the next point with a unique position and orientation
                {
                    int k = 0; 
                    std::cout<<"Break 2 \n";
                    ClosedPt = new Node(closedList->Start);
                    while(ClosedPt)
                    {
                        bool QTMatch = CheckRepeatPointwithOrientation(current,ClosedPt); 
                        if(QTMatch)
                            k++; // Increase the count of how many matches there are
                        ClosedPt = ClosedPt->next;
                    }
                    if (k>0) // if the position/orientation is repeated
                    {
                        current = new Node(current->next);
                        std::cout<<"Break 3 \n";
                        dist = Dist(current->pose.p,current->parent->pose.p); // distance between last point and current point 
                    }
                    else
                        break;  // not a repeated point/orientation, break out of the loop. 
                }
                // std::cout<<" the old coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n"; // Added line
                int i = 0;
                while(i<1) // This loop find the next Point (not point/orientation) that is unique/not visited
                {
                    pt = pt->next;
                    std::cout<<"Break 4 \n";
                    Eigen::Vector3i nodecoords(pt->pose.p.position.x, pt->pose.p.position.y, pt->pose.p.position.z);
                    PointMatch = CheckRepeatPoint(ListofNodes,nodecoords);
                    if(!PointMatch) // add one if the points do not match to break it out of the while loop
                        i++;
                }
                current = new Node(pt);
                std::cout<<"Break 5 \n";
                // std::cout<<" the NEW node f_value is = "<<current->f_value<<" \n"; // Added line
                // std::cout<<" the new coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n";
                openList->remove((openList->getHead())->next);

            }
        }
        if (!continuous) // If continuous=false, multiple orientations at the same point are allowed. 
        { 
            if (PointMatch) // if the point was visited, keep checking the next point in the openList
            {
                // std::cout<<"Got a repeated point here... \n"; 
                double dist;
                std::cout<<"Break 6 \n";
                dist = Dist(current->pose.p,current->parent->pose.p); // distance between last point and current point 
                std::cout<<"Break 6.1 \n";
                std::cout<<"dist = "<<dist<<" \n";
                // std::cout<<"Distance is "<<dist<<"m \n"; 
                // Check to see if the exact position/orientation has been visited before
                while (dist ==0) // This loop finds the next point with a unique position and orientation
                {
                    int k = 0; 
                    std::cout<<"Break 7 \n";
                    ClosedPt = new Node(closedList->Start);
                    while(ClosedPt)
                    {
                        bool QTMatch = CheckRepeatPointwithOrientation(current,ClosedPt); 
                        std::cout<<"Break 8 \n";
                        if(QTMatch)
                            k++; // Increase the count of how many matches there are
                        ClosedPt = ClosedPt->next;
                    }
                    if (k>0) // if the position/orientation is repeated
                    {
                        current = new Node(current->next);
                        std::cout<<"Break 9 \n";
                        dist = Dist(current->pose.p,current->parent->pose.p); // distance between last point and current point 
                    }
                    else
                        break;  // not a repeated point, break out of the loop. 
                }
                // ********* if you DO NOT want to have multiple pointing angles at the same point, pull the contents of this second if loop into the first if loop
                if (!(dist == 0)) // if it's going back to a point the path has already visited (distance not 0), find the the next highest point in the openList that the path hasn't visited
                {
                    // std::cout<<" the old coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n"; // Added line
                    int i = 0;
                    while(i<1) // for some reason didn't work when it was while(PointMatch)
                    {
                        pt = pt->next;
                        std::cout<<"Break 10 \n";
                        Eigen::Vector3i nodecoords(pt->pose.p.position.x, pt->pose.p.position.y, pt->pose.p.position.z);
                        PointMatch = CheckRepeatPoint(ListofNodes,nodecoords);
                        if(!PointMatch) // add one if the points do not match to break it out of the while loop
                            i++;
                    }
                    current = new Node(pt);
                    std::cout<<"Break 11 \n";
                    // std::cout<<" the NEW node f_value is = "<<current->f_value<<" \n"; // Added line
                    // std::cout<<" the new coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n";
                    openList->remove((openList->getHead())->next);
                }
            }
        }
        std::cout<<"Break 12 \n";
        Eigen::Vector3i updatednodecoords(current->pose.p.position.x, current->pose.p.position.y, current->pose.p.position.z); // get updated current nodes position (in case the original current node was a repeat)         
        ListofNodes.push_back(updatednodecoords); // Add the current node to the list of point to cross check against future "current" nodes
        
        maxCov = GetMaxCoverage(current, parentCoverage, maxCov);
        bool decline = VerifyCoverageDecline(current,parentCoverage,maxCov);
        if (decline)
            DeclineCounter++; 
        // bool gainDecline = VerifyCoverageGains(current);
        // if (gainDecline)
        //     GainCounter++; 
        // std::cout<<"\n"<<"  decline :"<<decline<< " gainDecline: "<<gainDecline<<" \n";
        bool DeclineCovMax, DeclineGainMax;
        if ((DeclineCounter > 2))
            DeclineCovMax = true;
        else 
            DeclineCovMax = false; 
        // if (GainCounter > 10)
        //     DeclineGainMax = true;
        // else
        //     DeclineGainMax = false; 
        // DeclineCovMax = false;
        DeclineGainMax = false; 
        std::cout<<"Break 18 \n";
        // std::cout<<"\n"<<"  DeclineCovMax :"<<DeclineCovMax<< " DeclineGainMax: "<<DeclineGainMax<<" \n";
            
        // put the current node onto the closed list, ==>> already visited List
        closedList->add(current,heuristic->isCost());

        NodesExpanded++;
        if ((heuristic->terminateConditionReached(current,DeclineCovMax,DeclineGainMax, maxCov) && current!= root) || (nodesCounter==nodeToBeVisNum && nodeToBeVisNum!=0))
        {
            //the last node in the path
            current->next = NULL;
            std::cout<<"*************Cumulative distance : "<<current->distance<<"************ \n";
            std::cout<<"\n"<<"	--->>> Goal state reached with :"<<globalID<< "nodes created and :"<<NodesExpanded<<" nodes expanded <<<---";
            fflush(stdout);
            p = current;
            path = NULL;
            while (p != NULL)
            {
                if(p->prev != NULL)
                    (p->prev)->next = p->next;
                if(p->next != NULL)
                    (p->next)->prev = p->prev;
                // check if we're removing the top of the list
                if(p == closedList->Start)
                    closedList->next();
                // set it up in the path
                p->next = path;
                path = p;
                p = p->parent;
            }
            // now delete all nodes on OPEN and Closed Lists
            openList->free();
            closedList->free();
            return path;
        }
        debug = false;
        // Create List of Children for the current NODE
        if(!(childList = makeChildrenNodes(current)))
        {
            std::cout<<"\n	--->>> Search Ended On this Branch / We Reached a DEAD END <<<---";
        }
        // insert the children into the OPEN list according to their f values
        nodesCounter++;
        while (childList != NULL)
        {
            curChild  = childList;
            childList = childList->next;
            curChild->next = NULL;
            curChild->prev = NULL;
            // calculate f_value
            heuristic->calculateHeuristic(curChild);
            //TODO:: pass the correct parameters to the display function
            displayOctree();
            globalcount++;
            Node * p= NULL;
            if(debug)
            {
                std::cout<<"\n ID= "<<curChild->id<<" global count:"<<globalcount<<" nodeExpanded:"<<NodesExpanded<<" mem address:"<<curChild;
                std::cout<<"\ncurChildren f value= "<<curChild->f_value;
                std::cout<<"\nFinding the curchild : "<<curChild->pose.p.position.x<<" "<< curChild->pose.p.position.y<<" "<<curChild->pose.p.position.z<<" "<<curChild->pose.p.orientation.x<<" "<<curChild->pose.p.orientation.y<<" "<<curChild->pose.p.orientation.z<<" "<<curChild->pose.p.orientation.w;
            }

            if(heuristic->isCost())
                condition = (current->f_value <=curChild->f_value);
            else
                condition = (current->f_value >=curChild->f_value);

            if(openList!=NULL)
            {
              if(openList->find(curChild) && condition)
              {
                freeNode(curChild);
                curChild = NULL;
                continue;
              }
            }
            if(closedList!=NULL)
            {
              if(closedList->find(curChild) && condition)
              {
                freeNode(curChild);
                curChild = NULL;
                continue;
              }
            }
            // remove any similar node in open list as it's higher cost
            openList->remove(curChild);
            // add the new one, it's lower cost
            openList->add(curChild,heuristic->isCost()); // adds them in ascending order if INFOGAIN, descending otherwise
        }
        // Test to see if we have expanded too many nodes without a solution
        if (current->id > this->MAXNODES)
        {
            std::cout<<"\nExpanded:"<<current->id<<" Nodes which is more than the maximum allowed MAXNODE:"<<MAXNODES;
            //Delete Nodes in Open and Closed Lists
            closedList->free();
            openList->free();
            std::cout<<"\nThe closed list and open list have been cleared"<<"\n";fflush(stdout);
            path = NULL;
            return path; // Expanded more than the maximium nodes state
        }
    }
    closedList->free();
    std::cout<<"\n	--->>>No Path Found<<<---";
    return NULL;
}

Node *Astar::makeChildrenNodes(Node *parent)
{
    geometry_msgs::Pose P;
    Node  *p, *q, *r;
    SearchSpaceNode *temp;

    P.position.x     = parent->pose.p.position.x;
    P.position.y     = parent->pose.p.position.y;
    P.position.z     = parent->pose.p.position.z;
    P.orientation.x  = parent->pose.p.orientation.x;
    P.orientation.y  = parent->pose.p.orientation.y;
    P.orientation.z  = parent->pose.p.orientation.z;
    P.orientation.w  = parent->pose.p.orientation.w;

    Tree t;
    t.location = P;
    if(!searchspace)
        return NULL;
    temp = searchspace;

    // Locate the node in the Search Space, necessary to determine the neighbours
    while(temp!=NULL)
    {
        //added the orientation since we have different
        if(isPositionEqual(temp->location.position,P.position) && isOrientationEqual(temp->location.orientation,P.orientation))
            break;
        temp = temp->next;
    }

    if(!temp)
    {
        return NULL;
    }
    q = NULL;

    if(debug)
    {
        std::cout<<"\n\n\n#############children Size: "<< temp->children.size() <<" ##################\n\n\n";fflush(stdout);
    }

    // Check Each neighbour
    for(int i=0;i<temp->children.size();i++)
    {
        //TODO: check for collision before adding the node, use previous implementation for reference
        p = new Node;
        p->pose.p.position.x = temp->children[i]->location.position.x;
        p->pose.p.position.y = temp->children[i]->location.position.y;
        p->pose.p.position.z = temp->children[i]->location.position.z;
        p->pose.p.orientation.x = temp->children[i]->location.orientation.x;
        p->pose.p.orientation.y = temp->children[i]->location.orientation.y;
        p->pose.p.orientation.z = temp->children[i]->location.orientation.z;
        p->pose.p.orientation.w = temp->children[i]->location.orientation.w;

        for(int j=0; j<temp->children[i]->sensorLocation.poses.size();j++)
        {
            Pose tempPose;
            tempPose.p.position.x = temp->children[i]->sensorLocation.poses[j].position.x;
            tempPose.p.position.y = temp->children[i]->sensorLocation.poses[j].position.y;
            tempPose.p.position.z = temp->children[i]->sensorLocation.poses[j].position.z;
            tempPose.p.orientation.x = temp->children[i]->sensorLocation.poses[j].orientation.x;
            tempPose.p.orientation.y = temp->children[i]->sensorLocation.poses[j].orientation.y;
            tempPose.p.orientation.z = temp->children[i]->sensorLocation.poses[j].orientation.z;
            tempPose.p.orientation.w = temp->children[i]->sensorLocation.poses[j].orientation.w;

            p->senPoses.push_back(tempPose);
        }
        // move the " in closed list detection" to the astar
        p->id = temp->children[i]->id;
        p->parent = parent;
        p->next   = q;
        p->prev   = NULL;
        p->depth  = parent->depth + 1;
        p->id     = globalID++;
        q = p;
        t.children.push_back(p->pose.p);
    }
    // Save the search tree so that it can be displayed later
    if (t.children.size() > 0)
        tree.push_back(t);
    if(debug)
    {
        std::cout<<" Making children nodes: "<<t.children.size()<<"\n"; fflush(stdout);
    }
    return q;
}

// Free node function
void Astar::freeNode(Node *n)
{
    delete n;
}

void Astar::setProgressDisplayFrequency(int progressDisplayFrequency)
{
    this->progressDisplayFrequency = progressDisplayFrequency;
}

void Astar::setDebugDelay(double delay)
{
    this->debugDelay = delay;
}

// // Added Function
bool Astar::CheckRepeatPoint(std::vector<Eigen::Vector3i>& ListofNodes, Eigen::Vector3i nodecoords)
{   
    int ListSize = ListofNodes.size();
    std::cout<<" the list size is  = "<<ListSize<<" \n"; // Added line
    int x = nodecoords[0];
    int y = nodecoords[1];
    int z = nodecoords[2];
    std::cout<<" base coord is  = "<<x<<" " << y <<" " << z <<" \n"; // Added line
    bool match; 
    int i = 0; 
    while (i <=ListSize)
    {   
        int currX = ListofNodes[i][0];
        int currY = ListofNodes[i][1];
        int currZ = ListofNodes[i][2];
        std::cout<<"Break 13 \n";
        std::cout<<" the current coord is  = "<<currX<<" " << currY <<" " << currZ <<" \n"; // Added line
        std::cout<<" Inside the loop, the List of Nodes is = "<<ListofNodes.size()<<" nodes Long\n"; // Added line 
                for (auto i = ListofNodes.begin(); i != ListofNodes.end(); ++i){ // Added line 
                    std::cout << *i << ' '; // Added line 
                }

        if((x==currX) && (y==currY) && (z==currZ))
        {   //std::cout<<"Breakpoint 1.6.5. \n"; 
            match = true;
            break;
        } 
        else
        {   
            match = false;
            i++;
        }

    }

    // if(match)
    //      std::cout<<" This node is repeated\n"; // Added line
    // else 
    //     std::cout<<" This node is unique\n"; // Added line

    return match; 
} 

// bool Astar::CheckRepeatPoint(Node * current, Node * ClosedPt)
// {   
//     int x = current->pose.p.position.x;
//     int y = current->pose.p.position.y;
//     int z = current->pose.p.position.z;
//     // std::cout<<" base coord is  = "<<x<<" " << y <<" " << z <<" \n"; // Added line
//     bool match; 
//     while (ClosedPt)
//     {
//         int CL_x = ClosedPt->pose.p.position.x;
//         int CL_y = ClosedPt->pose.p.position.y;
//         int CL_z = ClosedPt->pose.p.position.z;
//         // std::cout<<" CLosed Point coord is  = "<<CL_x<<" " << CL_y <<" " << CL_z <<" \n"; // Added line
//         if((x==CL_x) && (y==CL_y) && (z==CL_z))
//         {    
//             match = true;
//             // std::cout<<" This node is repeated\n"; // Added line
//             break; 
//         } 
//         else
//         {
//             match = false;
//             // std::cout<<" This node is unique\n"; // Added line
//         }    
//         ClosedPt = ClosedPt->next;
//     }
//     return match; 
// }    

// Added Function
bool Astar::CheckForRepeatPoints(Node * current, Node * ClosedPt, bool continuous)
{   
    int x = current->pose.p.position.x;
    int y = current->pose.p.position.y;
    int z = current->pose.p.position.z;
    double qtX = current->pose.p.orientation.x;
    double qtY = current->pose.p.orientation.y;
    double qtZ = current->pose.p.orientation.z;
    double qtW = current->pose.p.orientation.w;
    std::cout<<" base coord is  = "<<x<<" " << y <<" " << z <<qtX<<qtY<<qtZ<<qtW<<" \n"; // Added line
    bool match; 
    while (ClosedPt) // Loop through list of Closed Points, redefining each time 
    {
        int CL_x = ClosedPt->pose.p.position.x;
        int CL_y = ClosedPt->pose.p.position.y;
        int CL_z = ClosedPt->pose.p.position.z;
        double CL_qtX = ClosedPt->pose.p.orientation.x;
        double CL_qtY = ClosedPt->pose.p.orientation.y;
        double CL_qtZ = ClosedPt->pose.p.orientation.z;
        double CL_qtW = ClosedPt->pose.p.orientation.w;
        // std::cout<<"Break Point 7... \n";
        std::cout<<" CLosed Point coord is  = "<<CL_x<<" " << CL_y <<" " << CL_z <<CL_qtX<<CL_qtY<<CL_qtZ<<CL_qtW<<" \n"; // Added line

        if (continuous==1) // if you don't want any repeat points continuous = true
        {
            std::cout<<"Break Point 6... \n";
            if((x==CL_x) && (y==CL_y) && (z==CL_z)) //looking for just the coordinates to match 
            {    
                match = true;
                // std::cout<<" This node is repeated\n"; // Added line
                break; 
            } 
            else
            {
                match = false;
                // std::cout<<" This node is unique\n"; // Added line
            }    
        }
        else // continuous = false, you just don't want repeated points with the same orientation
        {
            std::cout<<"Break Point 7... \n";
            if((x==CL_x) && (y==CL_y) && (z==CL_z) && (qtX==CL_qtX) && (qtY==CL_qtY) && (qtZ==CL_qtZ) && (qtW == CL_qtW))
            {    
                match = true;
                break; 
            } 
            else
            {   
                match = false;
            }
        }
        ClosedPt = ClosedPt->next; // move to the next point on the closed list
    } 
    return match; 
}  

// Added Function
bool Astar::CheckRepeatPointwithOrientation(Node * current, Node * ClosedPt)
{   
    int x = current->pose.p.position.x;
    int y = current->pose.p.position.y;
    int z = current->pose.p.position.z;
    double qtX = current->pose.p.orientation.x;
    double qtY = current->pose.p.orientation.y;
    double qtZ = current->pose.p.orientation.z;
    double qtW = current->pose.p.orientation.w;
    // std::cout<<" base coord is  = "<<x<<" " << y <<" " << z <<" \n"; // Added line
    bool match; 

    int CL_x = ClosedPt->pose.p.position.x;
    int CL_y = ClosedPt->pose.p.position.y;
    int CL_z = ClosedPt->pose.p.position.z;
    double CL_qtX = ClosedPt->pose.p.orientation.x;
    double CL_qtY = ClosedPt->pose.p.orientation.y;
    double CL_qtZ = ClosedPt->pose.p.orientation.z;
    double CL_qtW = ClosedPt->pose.p.orientation.w;
    std::cout<<"Break 14 \n";
    // std::cout<<" CLosed Point coord is  = "<<CL_x<<" " << CL_y <<" " << CL_z <<" \n"; // Added line
    if((x==CL_x) && (y==CL_y) && (z==CL_z) && (qtX==CL_qtX) && (qtY==CL_qtY) && (qtZ==CL_qtZ) && (qtW == CL_qtW))
    {    
        match = true;
    } 
    else
    {   
        match = false;
    }
    return match; 
}    

Node *Astar::GetNextNonRepeatedPoint(Node * current, bool continuous)
{
    // ClosedPt = new Node(closedList->Start); 
    // bool PointMatch = CheckForRepeatPoints(current, ClosedPt, continuous);
    pt = new Node(current); 
    int i = 0; 
    while (i < 1)
    {
        ClosedPt = new Node(closedList->Start);
        pt= new Node(pt->next);
        bool PointMatch = CheckForRepeatPoints(pt, ClosedPt, continuous);
        std::cout<<"Pointmatch (1=true): "<<PointMatch<<" \n"; 
        if(!PointMatch) // add one if the points do not match to break it out of the while loop
            i++; 
        openList->remove((openList->getHead())->next); // Takes away the point from the OpenList
        if (!current->next)
        {
            std::cout<<"Unable to Find a path that doesn't repeat points. This point will be repeated \n"; 
            break; 
        }
    }
    return pt; 
}

Node *Astar::GetNextPointwithDesiredCoverageGain(Node * current, double DesiredCoverageGain, double parentCoverage)
{
        // double parentCoverage;
        // if (!current->parent) // if the node doesn't have a parent (it's the root), give initial values for parent coverage
        //     parentCoverage = 0.0;
        // else
        //     parentCoverage = current->parent->coverage;
        double coverageIncrease = current->coverage - parentCoverage;
        double prevCovIncrease;
        // std::cout<<" the parent coverage is = "<<parentCoverage<<" \n"; // Added line
        if (coverageIncrease <= DesiredCoverageGain)
        {
            int i = 0; 
            while ((coverageIncrease <= DesiredCoverageGain) && (current->next))
            {
                // std::cout<<" the original coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n";
                current = new Node(current->next);
                // std::cout<<" the NEW coord is  = "<<current->pose.p.position.x <<" " << current->pose.p.position.y  <<" " << current->pose.p.position.z <<" \n";
                coverageIncrease = current->coverage - parentCoverage;
                // std::cout<<" the NEW coverage gain is = "<<coverageIncrease<<" \n"; // Added line
                if (prevCovIncrease || (coverageIncrease <= 0))
                {
                    if (coverageIncrease == prevCovIncrease  || (coverageIncrease == 0))
                    {
                        i++;
                        DesiredCoverageGain = DesiredCoverageGain/2; 
                    }
                    if (coverageIncrease < 0)
                    {
                        i++;
                        i++;
                        i++;
                    }
                }
                if (i>2)
                    break; 
                prevCovIncrease = coverageIncrease;
            } 
        }
        return current; 
}

bool Astar::CheckCoverageGain(Node *current, double DesiredCoverageGain)
{
    double parentCoverage;
    if (!current->parent) // if the node doesn't have a parent (it's the root), give initial values for parent coverage
        parentCoverage = 0.0;
    else
        parentCoverage = current->parent->coverage;
    double coverageIncrease = current->coverage - parentCoverage;
    if (coverageIncrease <= DesiredCoverageGain)
        return false;
    else
        return true;  
}

double Astar::GetMaxCoverage(Node *node, double parentCoverage, double maxCov)
{
    // std::cout<<" the old MAX coverage is = "<<maxCov<<" \n"; // Added line
    // std::cout<<" the Parent coverage is = "<<parCov<<" \n"; // Added line
    // std::cout<<" the Current coverage is = "<<node->coverage<<" \n"; // Added line
    std::cout<<"Break 16 \n";
    if ((node->coverage > parentCoverage) && (node->coverage > maxCov)) 
        maxCov = node->coverage;
    else 
        maxCov = maxCov;
    // std::cout<<" the new MAX coverage is = "<<maxCov<<" \n"; // Added line
    return maxCov; 
}

bool Astar::VerifyCoverageDecline(Node *node, double parentCoverage, double maxCov)
{
    double parCov = parentCoverage;
    bool decline; 
    if (node->coverage < maxCov) // return TRUE if there is a decline
    {
        decline = true;  
    }
    else
        decline = false; 
    return decline; 
}

bool Astar::VerifyCoverageGains(Node *node)
{
    double parCov;
    bool gainDecline; 
    if (!node->parent) // if the node doesn't have a parent (it's the root), give initial values for parent coverage and max coverage
    {
        parCov = 0.0;
    }
    else
        parCov = node->parent->coverage;
    // Define max coverage as the current node's coverage if it's more than it's parent's coverage and more than the current max coverage
    if ((node->coverage-parCov) <= 0.25)
    {
        gainDecline = true; 
    }
    else
        gainDecline = false;
    return gainDecline; 
}

}
