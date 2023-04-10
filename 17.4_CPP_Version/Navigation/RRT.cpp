#include "RRT.h"
#include <fstream>

RRT::RRT(){
    // Node start(myStart);
    // Node end(myEnd);
    // this->tree = Tree(myStart,myEnd);
    // this->obstacles = obstacles;
}

void RRT::setObstacles(vector<Obstacle*> newObstacles){
    this->obstacles=newObstacles;
}


vector<Obstacle*> RRT::getObstacles(){
    return this->obstacles;
}

//cartesian distance between 2 points
double distance(vector<double> start, vector<double> end){
    double x_t = (end.at(0) - start.at(0));
    double y_t = (end.at(1) - start.at(1));
    double d = sqrt(x_t*x_t + y_t*y_t);
    // printf("Distance: %f\n",d);
    return d;
}

vector<double> generatePoint(unsigned int seed){
    vector<double> p(2);
    // vector<int> f;
    
    //random_device rd; // obtain a random number from hardware
    // mt19937 gen(rd()); // seed the generator
    // uniform_int_distribution<float> distr(-20, 20); // define the range CHANGE when the field is known and decide if cm or mm
    
    srand(seed);
    double randomX = rand() % 1001 + (-500);
    double randomY = rand() % 1001 + (-500);
    // seed

    
    double x, y;
    //generate until point we havent used (not necessarily a good or bad thing)
    x=randomX/100;
    y=randomY/100;
    p.at(0) = x;
    p.at(1) = y;
    
    //handle if point is within radius of a robot here as obstacle to bump back up to do part of loop
    // while(considered.find(p) != considered.end());
    // considered.insert(p);
    
    // f.push_back(x);
    // f.push_back(y);
    
    return p;
}

void RRT::setAngle(vector<double> start, vector<double> end){
    double deltaY=end[1]-start[1];
    double deltaX=end[0]-start[0];
    if(deltaX != 0){
        this->angle=atan2(deltaY,deltaX);
    }
    else{
        this->angle=atan2(deltaY,0.000001);
    }
    
}

//Getter dAng
double RRT::getAngle(){
    return this->angle;
}

// Node newNode(vector<int> point){
//     return Node(point);
// }

bool RRT::isInObstacle(vector<double> newPoint){
    bool inObstacle=false;
    if(this->getObstacles().empty()){
        cout<<"empty"<<endl;
    }
    // printf("point: %f\t, %f\n",newPoint.at(0),newPoint.at(1));
    for(Obstacle *obs : this->getObstacles()){
        // printf("ID,X,Y: %i,%f,%f\n",obs->ID,obs->myRK.getX0(),obs->myRK.getY0());
        inObstacle=obs->isInObstacle(newPoint);
        // printf("X,Y: %f,%f\n",obs.myRK.getX0(),obs.myRK.getY0());
        // printf("InObstacle: %d\n",inObstacle);
        if(inObstacle){
            this->numObstruction++;
            return inObstacle;
            break;
        }
    }
    return inObstacle;
}

//shifts vector between considered point and RNG to origin, finds angle, calcs distance, adds along vector
vector<double> calcNewNode(double distance,vector<double> start, vector<double> end){
    //get start and end points
    double x_0 = start.at(0); 
    double y_0 = start.at(1);
    double x_1 = end.at(0);
    double y_1 = end.at(1);

    //shift vector to origin
    if((x_0 != 0) || (y_0 != 0)){ //Confused on why taking -1
        double x_p = x_0*(-1);
        double y_p = y_0*(-1);
        
        x_1 += x_p;
        y_1 += y_p;
    }
    
    double theta = atan2(y_1, x_1);    //calc shifted values to get angle 
    //change in x and y to get new point along ray
    double dx = distance*cos(theta); 
    double dy = distance*sin(theta);
    double xNew=round((x_0 + dx)*100.0)/100.0;
    double yNew=round((y_0 + dy)*100)/100;
    
    vector<double> n = {xNew, yNew}; //x and y values moved in the correct amount
    
    return n;
}

void RRT::setTransform(double newAngle){
    this->transform={{cos(newAngle), -sin(newAngle), 0},
                     {sin(newAngle), cos(newAngle), 0},
                     {0,0,1.0}};//cos -sin sin cos

}

vector<vector<double>> RRT::getTransform(){
    return this->transform;
}

vector<double> RRT::calcNewPoint(vector<double> newPoint){
    
    vector<double> point={this->getTransform()[0][0]*newPoint[0]+this->getTransform()[0][1]*newPoint[1],
                          this->getTransform()[1][0]*newPoint[0]+this->getTransform()[1][1]*newPoint[1]};
    // cout<<this->getTransform()[0][0]<<endl;
    // cout<<this->getTransform()[0][1]<<endl;
    return point;
}



list<vector<double>> RRT::calculatePath(vector<double> myStart, vector<double> myEnd){
    cout << "Starting new path"<< endl;
    // this->counterRRT=0;
    this->setAngle(myStart,myEnd);
    this->setTransform(-1*this->getAngle());
    this->tree = Tree(myStart,myEnd);
    this->D_ADD = 0.03;
    this->numObstruction = 0;
    printf("Starting: %f, %f\n",myStart.at(0),myStart.at(1));
    this->finished=false;
    //Creates path to be returned
    list<vector<double>> path;
    
    srand(2);
    unsigned int seed = rand();
    //Saves start point and Node
    vector<double> startPoint=this->tree.start.point;
    Node lastNode=this->tree.start;
    
    //Saves end point
    vector<double> goalPoint=this->tree.goal.point;
    Node goalNode = Node(goalPoint);

    //Initializes cost as the distance moving 2cm left or right from starting point then to goal point
    double cost=distance(startPoint, goalPoint)+0.02;//+0.02

    //Initializes if we have reached the goal
    bool goalReached=false;

    if(distance(startPoint,goalPoint)<=this->D_ADD){
        goalReached=true;
    }

    //while we are still looking for the goal...
    cout << "Calculating" << endl;
    while(!goalReached){
        
        //Generates a new point (check params in method for generation area)
        vector<double> randPoint=generatePoint(seed);
        seed=rand();
        
        // if(this->hasCollided){
        //     this->D_ADD=0.21;
        //     this->hasCollided=false;
        // }
        
        vector<double> newPoint=calcNewNode(D_ADD,lastNode.point,randPoint);

        //If the newPoint is closer to the goal AND the point produced by moving in the direction of the newPoint isn't in an obstacle 
        //then create the node and move on
        if(distance(newPoint,goalPoint)<cost && !(this->isInObstacle(newPoint))){
            // counterRRT++;
            cout << "point added" << endl;
            //Create new Node
            Node newNode = Node(newPoint);
            
            //Set new cost as the distance from new node to goal 
            cost=distance(newPoint,goalPoint)+0.005;//-0.0001
            printf("Last Point: %f,%f\n",newPoint.at(0),newPoint.at(1));
            printf("New cost: %f\n",cost);
            

            //Link new node to the past node
            Link newLink= Link(lastNode,newNode);

            //Add Link to the Tree
            this->tree.addLink(newLink);

            //For next iteration make lastNode the newNode we just used
            lastNode=newNode;

            //checks if last node of last link was within radius of goal AKA goalReached
            goalReached=this->tree.goalReached();

        }

    }
    
    if(goalReached && lastNode.point != goalNode.point){
        if(lastNode.point != goalNode.point){
            //Link new node to the past node
            Link newLink= Link(lastNode,goalNode);

            //Add Link to the Tree
            this->tree.addLink(newLink);
            cout << "Calculations complete" << endl;

        }
        else{
            cout << "Calculations complete" << endl;
        }
        
    }

    list<vector<double>> beforeTransform;

    for (Link link : this->tree.links)
    {
        beforeTransform.push_back(link.child.point);
        path.push_back(this->calcNewPoint(link.child.point));
    }
    // cout << "Before Tranform" << endl;
    // for(vector<double> point : beforeTransform){
    //     printf("%f,%f\n",point.at(0),point.at(1));
    // }
    cout << "Before best fit" << endl;
    for(vector<double> point : path){
        printf("%f,%f\n",point.at(0),point.at(1));
    }
    if(!path.empty()){
        N=int(path.size());
        if(this->numObstruction > 0){
            this->n=5;
        }
        else{
            cout << "linear" << endl;
            this->n=1;
        }
        
    }
    else{
        N=0;
    }
    
    
    double x[N],y[N];
    list<vector<double>> tempList;

    for (int i=0;i<N;i++){
        vector<double> temp=path.front();
        x[i]= temp.at(0);
        y[i]= temp.at(1);
        tempList.push_back(temp);
        path.pop_front();
    }
    cout << path.size() << endl;

    double X[2*n+1];

    for (int i=0;i<2*n+1;i++)
    {
        X[i]=0;
        for (int j=0;j<N;j++)
            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }

    double B[n+1][n+2],a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    
    for (int i=0;i<=n;i++){
        for (int j=0;j<=n;j++)
            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    }
    double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    
    for (int i=0;i<n+1;i++)
    {    
        Y[i]=0;
        for (int j=0;j<N;j++)
        Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    
    for (int i=0;i<=n;i++)
        B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    
    this->n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    

    for (int i=0;i<n;i++){                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (int k=i+1;k<n;k++)
            if (B[i][i]<B[k][i])
                for (int j=0;j<=n;j++)
                {
                    double temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }
    }

    for (int i=0;i<n-1;i++){          //loop to perform the gauss elimination
        for (int k=i+1;k<n;k++)
        {
            double t=B[k][i]/B[i][i];
            for (int j=0;j<=n;j++)
                B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
    }
    for (int i=n-1;i>=0;i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        for (int j=0;j<n;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i]=a[i]-B[i][j]*a[j];
        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }

    if(!finished){
        double y=0;
        if(n<=2){
            a[1]=(tempList.back()[1]-tempList.front()[1])/(tempList.back()[0]-tempList.front()[0]);
            a[0]=tempList.front()[1]-a[1]*tempList.front()[0];
        }
        for (int i=0;i<N;i++){
            vector<double> temp=tempList.front();
            double x=temp.at(0);
            printf("a: %f,%f\n",a[0],a[1]);
            if(n>2){
                y=a[0]+(a[1]*pow(x,1))+(a[2]*pow(x,2))+(a[3]*pow(x,3))+(a[4]*pow(x,4))+(a[5]*pow(x,5));
            }
            if(n<=2){
                y=a[0]+(a[1]*pow(x,1));
            }
            // printf("%f,%f\n",x,y);
            if(abs(y-myEnd.at(1)) <=5.0){// && ((x>=0 && lastX>=0)||(x<=0 && lastX<=0))
                path.push_back({x,y});
                tempList.pop_front();
            }
            else{
                tempList.pop_front();
                continue;
            }
        }
        finished = true;
        
    }
    this->setTransform(this->getAngle());
    for(int i=0; i<N;i++){
        path.push_back(this->calcNewPoint(path.front()));
        path.pop_front();
    }
    cout << "After Best Fit" << endl;
    for(vector<double> point : path){
        printf("%f,%f\n",point.at(0),point.at(1));
    }
    return path;   
    
    
}

void Obstacle::setPath(vector<double> newDestination){
    if(this->finalDestination!=newDestination){
        this->overriding=true;
        this->finalDestination=newDestination;
        this->myRK.setWheelSpeeds({0,0,0,0});
        this->hasPath=false;
        cout<<"Updating Path"<<endl;
    }
    
}

void Obstacle::setPath(){
    this->lastCollision=this->whoCollision;
    this->myRK.setWheelSpeeds({0,0,0,0});
    this->hasPath=false;
    cout<<"Updating Path"<<endl;
}

bool Obstacle::isCollision(){
    bool isCollision=false;
    for(Obstacle *obs : this->myRRT.getObstacles()){
        // printf("%i,%d,%f,%f\n",obs->ID,obs->color,obs->myRK.getX0(),obs->myRK.getY0());
        if(sqrt(pow(obs->myRK.getX0()-this->myRK.getX0(),2)+pow(obs->myRK.getY0()-this->myRK.getY0(),2))<(this->radius-0.06)){
            printf("%i,%d,%f\n",obs->ID,obs->color,sqrt(pow(obs->myRK.getX0()-this->myRK.getX0(),2)+pow(obs->myRK.getY0()-this->myRK.getY0(),2)));
            isCollision=true;
            this->whoCollision=obs->ID;
            this->myRRT.hasCollided=isCollision;
            return isCollision;
            break;
        }
    }
    if(isCollision){
        return isCollision;
    }
    else{
        return false;
    }
    
}

list<vector<double>> Obstacle::getPath(){
    return this->myPath;
}

Obstacle::Obstacle(int newID, bool newColor){
    this->ID=newID;
    this->color=newColor;
}
Obstacle::Obstacle(){}

Obstacle::~Obstacle(){
    
}

bool Obstacle::isInObstacle(vector<double> newPoint){
    bool inObstacle;
    double distance=sqrt(pow(this->myRK.getX0()-newPoint.at(0),2)+pow(this->myRK.getY0()-newPoint.at(1),2));
    // cout<<distance<<endl;
    // printf("ID:%i\t Color:%d\t X:%f\t Y:%f\n",(*this).ID,(*this).color,(*this).myRK.getX0(),(*this).myRK.getY0());
    // printf("Coordinates:%f\t Radius:%f\n",pow(newPoint.at(0)-this->myRK.getX0(),2)+pow(newPoint.at(1)-this->myRK.getY0(),2),pow(this->radius,2));
    if(distance<=this->radius){
        inObstacle=true;
    }
    else{
        inObstacle=false;
    }
    return inObstacle;
}

void Obstacle::followPath(){
    if(this->overriding){
        this->overriding=false;
    }
    else if(!this->hasPath&&!this->finalDestination.empty()){
        this->myPath=this->myRRT.calculatePath({this->myRK.getX0(),this->myRK.getY0()},{this->finalDestination.at(0),this->finalDestination.at(1)});
        printf("Empty Path: %d\n",this->myPath.empty());
        if(!this->myPath.empty()){
            // this->nextPoint={0.0,0.03,this->finalDestination.at(2)};
            // this->myRK.pid_x.reset();
            // this->myRK.pid_y.reset();
            this->nextPoint={this->myPath.front().at(0),this->myPath.front().at(1),this->finalDestination.at(2)};
        }
        this->hasPath=true;
    }
    else if(!this->myPath.empty()){
        cout << this->ID << " " << this->color << " is following the path" << endl;
        // this->nextPoint={0.03,0,this->finalDestination.at(2)};
        this->nextPoint={this->getPath().front().at(0),this->getPath().front().at(1),this->nextPoint.at(2)};
        this->myRK.MoveToPoint(this->nextPoint.at(0),this->nextPoint.at(1),this->nextPoint.at(2));
        // printf("Wheelspeeds: %f\n",abs(this->myRK.getDeltaX()));
        if(abs(this->myRK.getDeltaDistance()[0]) < 0.05){ 
            // printf("Going To: %f\t %f\n",this->getPath().front().at(0),this->getPath().front().at(1));
            // list<vector<double>> empty;
            // this->myPath=empty;
            this->myPath.pop_front();
            // this->hasPath=false;
        }
    }
    else if(!this->finalDestination.empty()&&!this->nextPoint.empty()){
        this->myRK.setUsePID(true);
        this->myRK.MoveToPoint(this->nextPoint.at(0),this->nextPoint.at(1),this->nextPoint.at(2));
        
        if((abs(this->myRK.getDeltaTheta())<0.0001 && abs(this->myRK.getDeltaX())<0.001 && abs(this->myRK.getDeltaY())<0.001)&&(abs((this->myRK.getWheelSpeeds()[0])<5 && abs(this->myRK.getWheelSpeeds()[1])<5 && abs(this->myRK.getWheelSpeeds()[2])<5&&abs(this->myRK.getWheelSpeeds()[3])<5))){//abs(this->myRK.getDeltaY())>=0.001||
            // cout<<abs(this->myRK.getDeltaX())<<endl;
            this->myRK.setWheelSpeeds({0,0,0,0});
            // this->hasPath=false;
            // this->finalDestination.clear();
            vector<double> empty;
            this->nextPoint=empty;
            // this->nextPoint=empty;
            cout << this->ID << " " << this->color <<" has finished their path"<<endl;
            // this->myRK.MoveToPoint(this->lastPoint.at(0),this->lastPoint.at(1),this->finalDestination.at(2));
        }
        else{
            cout<<"Path Ending" <<endl;
        }
    }
    else{
        this->myRK.setWheelSpeeds({0,0,0,0});
        cout << this->ID << " "<< this->color <<" has no Path" << endl;
    }
    
}


Tree::Tree(vector<double> startPoint, vector<double> goalPoint){
    // Goal aGoal= 
    this->start=Node(startPoint);
    this->goal= Goal(goalPoint);
}

Tree::Tree(){}

void Tree::addLink(Link newLink){
    this->links.push_back(newLink);
}

bool Tree::goalReached(){
    bool inGoal=false;
    Node* n = &(this->links.back().child);
    if(pow(n->point.at(0)-this->goal.point.at(0),2)+pow(n->point.at(1)-this->goal.point.at(1),2)<=pow(this->goal.radius,2))
    {
        inGoal=true;
    }
    else{
        inGoal=false;
    }
    return inGoal;
}

Goal::Goal(vector<double> newPoint){
    this->point=newPoint;
}

Goal::Goal(){}

Link::Link(Node p,Node c){
    this->parent=p;
    this->child=c;
}

Node::Node(vector<double> newPoint){
    this->point=newPoint;
}

Node::Node(){}







