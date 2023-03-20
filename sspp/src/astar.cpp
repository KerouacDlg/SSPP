
#include "sspp/astar.h"

namespace SSPP
{

    Astar::Astar(ros::NodeHandle &n, Robot *rob, int progressDisplayFrequency) : nh(n),
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
        childPosePub = nh.advertise<geometry_msgs::PoseArray>("child_pose", 10);
        childSensorsPub = nh.advertise<geometry_msgs::PoseArray>("child_sensors", 10);
        parentPosePub = nh.advertise<geometry_msgs::PoseArray>("parent_pose", 10);
        parenSensorsPub = nh.advertise<geometry_msgs::PoseArray>("parent_sensors", 10);
        branchPub = nh.advertise<visualization_msgs::Marker>("branch", 10);
        octomapChildPub = nh.advertise<octomap_msgs::Octomap>("octomap_child", 10);

        nodeToBeVisNum = 0; // if 0 then it will continue planning till the target otherwise it will stop planning on the specified viewpoint number and will visualize the steps of choosing next viewpoint
        nodesCounter = 0;
    }

    Astar::Astar() : heuristic(NULL),
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
        if (openList)
        {
            openList->free();
            delete openList;
        }
        if (closedList)
        {
            closedList->free();
            delete closedList;
        }
    }

    // find the nearest node to the start
    void Astar::findRoot() throw(SSPPException)
    {
        SearchSpaceNode *temp;
        if (!this->searchspace)
        {
            throw(SSPPException((char *)"No SearchSpace Defined"));
            return;
        }
        double distance, shortestDist = std::numeric_limits<double>::max();
        // allocate and setup the root node
        root = new Node;
        temp = this->searchspace;
        while (temp != NULL)
        {
            distance = Dist(temp->location, start.p);
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
                root->senPoses.erase(root->senPoses.begin(), root->senPoses.end());
                for (int i = 0; i < temp->sensorLocation.poses.size(); i++)
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
        // Translate(root->pose,start.phi);
        std::cout << "\n"
                  << "	---->>>Root is Set to be X=" << root->pose.p.position.x << " Y=" << root->pose.p.position.y << " Z=" << root->pose.p.position.z;
    }

    void Astar::setRobot(Robot *rob)
    {
        this->robot = rob;
    }

    void Astar::setHeuristicFucntion(Heuristic *heuristicFun)
    {
        heuristic = heuristicFun;
    }

    void Astar::displayOctree()
    {
        // display the child and the octree
        geometry_msgs::PoseArray childPose, parentPose, childSensors, parentSensors;
        std::vector<geometry_msgs::Point> lineSegments;
        if (nodesCounter == nodeToBeVisNum && nodeToBeVisNum != 0)
        {
            geometry_msgs::Pose parent;

            geometry_msgs::Pose child;
            geometry_msgs::Point linePoint;

            std::cout << "I entered tree childrens of the desired node " << std::endl;

            child = curChild->pose.p;
            parent = current->pose.p;
            childPose.poses.push_back(child);
            parentPose.poses.push_back(parent);
            for (int i = 0; i < curChild->senPoses.size(); i++)
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

            // visualization
            childPose.header.frame_id = "map";
            childPose.header.stamp = ros::Time::now();
            childPosePub.publish(childPose);
            childSensors.header.frame_id = "map";
            childSensors.header.stamp = ros::Time::now();
            childSensorsPub.publish(childSensors);
            parentPose.header.frame_id = "map";
            parentPose.header.stamp = ros::Time::now();
            parentPosePub.publish(parentPose);
            parentSensors.header.frame_id = "map";
            parentSensors.header.stamp = ros::Time::now();
            parenSensorsPub.publish(parentSensors);
            visualization_msgs::Marker linesList1 = drawLines(lineSegments, 1, 6, 100000, 0.1);
            branchPub.publish(linesList1);

            octomap_msgs::Octomap octomap;
            octomap.binary = 1;
            octomap.id = 1;
            octomap.resolution = 0.25;
            octomap.header.frame_id = "map";
            octomap.header.stamp = ros::Time::now();
            bool res = octomap_msgs::fullMapToMsg(*curChild->octree, octomap);
            if (res)
            {
                octomapChildPub.publish(octomap);
            }
            else
            {
                ROS_WARN("OCT Map serialization failed!");
            }
            childSensors.poses.erase(childSensors.poses.begin(), childSensors.poses.end());
            childPose.poses.erase(childPose.poses.begin(), childPose.poses.end());
            lineSegments.erase(lineSegments.begin(), lineSegments.end());
            ros::Duration(2).sleep();
        }
    }

    Node *Astar::MCTSSearch(Pose start)
    {

        std::cout << "\n	--->>>Start MCTS Search<<<---";
        Node *path = NULL;
        globalID = 0;

        if (this->tree.size() > 0)
            this->tree.clear();
        if (!openList)
        {
            openList = new LList;
        }
        // Be sure that open and closed lists are empty
        openList->free();
        if (!this->searchspace)
        {
            std::cout << "\nGenerate SearchSpace before Searching !!!";
            return NULL;
        }
        SearchSpaceNode *temp = searchspace;
        nodevalueinit(temp); // 初始化每个searchspace节点的visited和reward

        this->start.p.position.x = start.p.position.x;
        this->start.p.position.y = start.p.position.y;
        this->start.p.position.z = start.p.position.z;
        this->start.p.orientation.x = start.p.orientation.x;
        this->start.p.orientation.y = start.p.orientation.y;
        this->start.p.orientation.z = start.p.orientation.z;
        this->start.p.orientation.w = start.p.orientation.w;

        std::vector<Eigen::Vector3f> List1ofNodes;
        ListofNodes = List1ofNodes;                                                              // 初始化ListofNodes
        Eigen::Vector3f startcoords(start.p.position.x, start.p.position.y, start.p.position.z); // Added Line
        ListofNodes.push_back(startcoords);                                                      // Added Line

        std::cout << "\n	--->>> Search Started <<<---" << std::endl;
        findRoot();
        openList->add(root, 1);
        int simtime = 0;
        std::cout << "\n	--->>> While(simtime<=4000) Started <<<---" << std::endl;
        Eigen::Vector3f nodecoords;
        bool PointMatch;
        double curcoverage;
        while (simtime <= 4000)
        {

            current = new Node(openList->getHead());

            nodecoords[0] = current->pose.p.position.x;
            nodecoords[1] = current->pose.p.position.y;
            nodecoords[2] = current->pose.p.position.z;
            ListofNodes.push_back(nodecoords);
            curcoverage = current->coverage;
            if ((heuristic->terminateConditionReached(current) && current != root) || (simtime >= 2000))
            {
                // the last node in the path
                current->next = NULL;
                std::cout << "*************Cumulative distance : " << current->distance << "************ \n";
                fflush(stdout);
                p = current;
                path = NULL;
                while (p != NULL)
                {
                    if (p->prev != NULL)
                        (p->prev)->next = p->next;
                    if (p->next != NULL)
                        (p->next)->prev = p->prev;
                    p->next = path;
                    path = p;
                    p = p->parent;
                }
                openList->free();
                return path;
            }
            openList->free();

            if (!(childList = makeChildrenNodes(current)))
            {
                std::cout << "\n	--->>> Search Ended On this Branch / We Reached a DEAD END <<<---";
            }

            while (childList != NULL)
            {

                curChild = childList;
                childList = childList->next;
                curChild->next = NULL;
                curChild->prev = NULL;
                nodecoords[0] = curChild->pose.p.position.x;
                nodecoords[1] = curChild->pose.p.position.y;
                nodecoords[2] = curChild->pose.p.position.z;
                PointMatch = CheckRepeatPoint(ListofNodes, nodecoords, 0);
                if (PointMatch)
                {

                    if (childList != NULL)
                    {
                        continue;
                    }
                    std::cout << "\n  if(curChild == NULL)  ";
                    break;
                }
                heuristic->calculateHeuristic(curChild);
                simtime = simtime + SimulationandBackup(curChild);
            }

            if (!(childList = makeChildrenNodes(current)))
            {
                std::cout << "\n	--->>> Search Ended On this Branch / We Reached a DEAD END <<<---";
            }

            while (childList != NULL)
            {
                // std::cout << "\n in while (childList != NULL) 2222";
                curChild = childList;
                childList = childList->next;
                curChild->next = NULL;
                curChild->prev = NULL;

                nodecoords[0] = curChild->pose.p.position.x;
                nodecoords[1] = curChild->pose.p.position.y;
                nodecoords[2] = curChild->pose.p.position.z;
                PointMatch = CheckRepeatPoint(ListofNodes, nodecoords, 0);
                if (PointMatch)
                {

                    if (childList != NULL)
                    {
                        continue;
                    }
                    std::cout << "\n  if(curChild == NULL) 222222 ";
                    break;
                }
                heuristic->calculateHeuristic(curChild);
                curChild->f_value = (curChild->reward + curcoverage / 100) / curChild->visited + 2 * sqrt(log10(simtime) / (curChild->reward + curcoverage / 100));

                // std::cout << "\n curChild->f_value = " << curChild->f_value;
                openList->add(curChild, 1);
                if (childList == NULL)
                {
                    break;
                }

                // freeNode(curChild);
            }
        }

        std::cout << "\n	--->>>No Path Found<<<---";
        return NULL;
    }

    Node *Astar::makeChildrenNodes(Node *parent)
    {
        geometry_msgs::Pose P;
        Node *p, *q;
        SearchSpaceNode *temp;

        P.position.x = parent->pose.p.position.x;
        P.position.y = parent->pose.p.position.y;
        P.position.z = parent->pose.p.position.z;
        P.orientation.x = parent->pose.p.orientation.x;
        P.orientation.y = parent->pose.p.orientation.y;
        P.orientation.z = parent->pose.p.orientation.z;
        P.orientation.w = parent->pose.p.orientation.w;

        Tree t;
        t.location = P;
        if (!searchspace)
            return NULL;
        temp = searchspace;

        // Locate the node in the Search Space, necessary to determine the neighbours
        while (temp != NULL)
        {
            // added the orientation since we have different
            if (isPositionEqual(temp->location.position, P.position) && isOrientationEqual(temp->location.orientation, P.orientation))
                break;
            temp = temp->next;
        }

        if (!temp)
        {
            return NULL;
        }
        q = NULL;

        // Check Each neighbour
        for (int i = 0; i < temp->children.size(); i++)
        {
            // TODO: check for collision before adding the node, use previous implementation for reference
            p = new Node;
            p->pose.p.position.x = temp->children[i]->location.position.x;
            p->pose.p.position.y = temp->children[i]->location.position.y;
            p->pose.p.position.z = temp->children[i]->location.position.z;
            p->pose.p.orientation.x = temp->children[i]->location.orientation.x;
            p->pose.p.orientation.y = temp->children[i]->location.orientation.y;
            p->pose.p.orientation.z = temp->children[i]->location.orientation.z;
            p->pose.p.orientation.w = temp->children[i]->location.orientation.w;

            for (int j = 0; j < temp->children[i]->sensorLocation.poses.size(); j++)
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
            p->next = q;
            p->prev = NULL;
            p->depth = parent->depth + 1;
            // p->id = globalID++;
            p->visited = temp->children[i]->visited;
            p->reward = temp->children[i]->reward;
            q = p;
            t.children.push_back(p->pose.p);
        }

        if (t.children.size() > 0)
            tree.push_back(t);

        return q;
    }

    int Astar::SimulationandBackup(Node *node)
    {
        bool condition;
        LList *openListMCT = new LList; // 重新定义的OPenlist
        Node *childListSimularion = new Node;
        Node *curChildSimularion = new Node;
        Node *currentSimularion = new Node;
        currentSimularion = node;
        openListMCT->free();
        
        if (!this->searchspace)
        {
            std::cout << "\nGenerate SearchSpace before Searching !!!";
            return 0;
        }
        std::vector<Eigen::Vector3f> ListofNodesSimulation = ListofNodes; // Added Line
        openListMCT->add(currentSimularion, 1);
        int countSimulation = 1;
        double begingcoverage = node->coverage;
        heuristic->calculateHeuristic(currentSimularion);
        bool PointMatch;
        Eigen::Vector3f nodecoords(currentSimularion->pose.p.position.x, currentSimularion->pose.p.position.y, currentSimularion->pose.p.position.z); // get current nodes position
        ListofNodesSimulation.push_back(nodecoords);
        while (openListMCT->Start != NULL)
        {
            currentSimularion = new Node(openListMCT->getHead());
            nodecoords[0] = currentSimularion->pose.p.position.x;
            nodecoords[1] = currentSimularion->pose.p.position.y;
            nodecoords[2] = currentSimularion->pose.p.position.z;
            openListMCT->remove(openListMCT->getHead());
            ListofNodesSimulation.push_back(nodecoords); // Add the current node to the list of point to cross check against future "current" nodes

            if (((currentSimularion->coverage >= 80) && currentSimularion != node) || (nodesCounter == nodeToBeVisNum && nodeToBeVisNum != 0) || countSimulation >= 4)
            {
                // the last node in the path
                currentSimularion->next = NULL;
                // fflush(stdout);
                p = currentSimularion;
                
                double reward = 1;
                if ((currentSimularion->coverage - begingcoverage) >= 6)
                {
                    reward = 0.4 + currentSimularion->coverage / 100;
                }
                else
                {

                    reward = 0;
                }
                // std::cout << "\n reward=" << reward;
                while (p != NULL)
                {

                    if (p->prev != NULL)
                        (p->prev)->next = p->next;
                    if (p->next != NULL)
                        (p->next)->prev = p->prev;
                    // check if we're removing the top of the list
                    
                    geometry_msgs::Pose P;
                    P.position.x = p->pose.p.position.x;
                    P.position.y = p->pose.p.position.y;
                    P.position.z = p->pose.p.position.z;
                    P.orientation.x = p->pose.p.orientation.x;
                    P.orientation.y = p->pose.p.orientation.y;
                    P.orientation.z = p->pose.p.orientation.z;
                    P.orientation.w = p->pose.p.orientation.w;
                    if (!searchspace)
                        return 0;

                    SearchSpaceNode *temp = searchspace;

                    backvalue(temp, P, reward); // 将reward回溯到每一个node
                    p = p->parent;
                }

                return countSimulation;
            }

            if (!(childListSimularion = makeChildrenNodes(currentSimularion)))
            {
                std::cout << "\n	--->>> Search Ended On this Branch / We Reached a DEAD END <<<---";
            }

            while (childListSimularion != NULL)
            {

                curChildSimularion = childListSimularion;
                childListSimularion = childListSimularion->next;
                curChildSimularion->next = NULL;
                curChildSimularion->prev = NULL;
                nodecoords[0] = curChildSimularion->pose.p.position.x;
                nodecoords[1] = curChildSimularion->pose.p.position.y;
                nodecoords[2] = curChildSimularion->pose.p.position.z;
                PointMatch = CheckRepeatPoint(ListofNodesSimulation, nodecoords, 0); // check to see if path has visited this point before
                if (PointMatch)
                {
                    if (childListSimularion != NULL)
                    {
                        continue;
                    }
                    break;
                }
                heuristic->calculateHeuristic(curChildSimularion);
                globalcount++;
                condition = (currentSimularion->f_value <= curChildSimularion->f_value);
                if (openListMCT != NULL)
                {
                    if (openListMCT->find(curChildSimularion) && condition)
                    {
                        freeNode(curChildSimularion);
                        curChildSimularion = NULL;
                        continue;
                    }
                }
                // remove any similar node in open list as it's higher cost
                openListMCT->remove(curChildSimularion);
                // add the new one, it's lower cost
                openListMCT->add(curChildSimularion, 1);
            }
            countSimulation++;
            // std::cout << "\n	countSimulation="<<countSimulation;
        }
        
        std::cout << "\n	--->>>No Path Found<<<---";
        return 0;
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

    // Added Function
    bool Astar::CheckRepeatPoint(std::vector<Eigen::Vector3f> &ListofNodes, Eigen::Vector3f nodecoords, bool debug)
    {
        int ListSize = ListofNodes.size();
        double x = nodecoords[0];
        double y = nodecoords[1];
        double z = nodecoords[2];
        if (debug)
        {
            std::cout << "Break 11 \n";
            std::cout << " the list size is  = " << ListSize << " \n";               // Added line
            std::cout << " base coord is  = " << x << " " << y << " " << z << " \n"; // Added line
        }

        bool match;
        int i = 0;
        while (i <= ListSize)
        {
            double currX = ListofNodes[i][0];
            double currY = ListofNodes[i][1];
            double currZ = ListofNodes[i][2];
            if (debug)
            {
                std::cout << "Break 12 \n";
                std::cout << " the current coord is  = " << currX << " " << currY << " " << currZ << " \n";
                std::cout << " Inside the loop, the List of Nodes is = " << ListofNodes.size() << " nodes Long\n";
                for (auto i = ListofNodes.begin(); i != ListofNodes.end(); ++i)
                {
                    std::cout << *i << ' '; // Added line
                }
            }

            if ((x == currX) && (y == currY) && (z == currZ))
            {
                match = true;
                if (debug)
                    std::cout << "true \n";
                break;
            }
            else
            {
                match = false;
                if (debug)
                    std::cout << "false \n";
                i++;
            }
        }

        return match;
    }

    // Added Function
    bool Astar::CheckRepeatPointwithOrientation(Node *current, Node *ClosedPt, bool debug)
    {
        if (debug)
            std::cout << "Break 13 \n";
        double x = current->pose.p.position.x;
        double y = current->pose.p.position.y;
        double z = current->pose.p.position.z;
        double qtX = current->pose.p.orientation.x;
        double qtY = current->pose.p.orientation.y;
        double qtZ = current->pose.p.orientation.z;
        double qtW = current->pose.p.orientation.w;
        // std::cout<<" base coord is  = "<<x<<" " << y <<" " << z <<" \n"; // Added line
        bool match;

        double CL_x = ClosedPt->pose.p.position.x;
        double CL_y = ClosedPt->pose.p.position.y;
        double CL_z = ClosedPt->pose.p.position.z;
        double CL_qtX = ClosedPt->pose.p.orientation.x;
        double CL_qtY = ClosedPt->pose.p.orientation.y;
        double CL_qtZ = ClosedPt->pose.p.orientation.z;
        double CL_qtW = ClosedPt->pose.p.orientation.w;

        if ((x == CL_x) && (y == CL_y) && (z == CL_z) && (qtX == CL_qtX) && (qtY == CL_qtY) && (qtZ == CL_qtZ) && (qtW == CL_qtW))
        {
            match = true;
        }
        else
        {
            match = false;
        }

        // if(match)
        //      std::cout<<" This node is repeated\n"; // Added line
        // else
        //     std::cout<<" This node is unique\n"; // Added line

        return match;
    }

}
