/*
CPT212: Assignment 2
This code is written by:
(1) Siti Adibah binti Zaini(152978)- Function 1
(2) Nur Alya binti Mazlan(153436)- Function 2
(3) Nur Irdina binti Mohamad Irwan(153256) - Function 3
(4) Nur Farhana binti Alam Shah(153769)- Function 4
*/

#include <bits/stdc++.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

enum Color {WHITE, GRAY, BLACK};

//check if pathExist from s to d
bool pathExist;

// keep track if the node has been visited or not. All are initialized as false as no node is visited yet
vector<bool> visited(5,false);  
// track the final connections that the MST has
vector<int> connection(5,-1);  
// store the minimum weight for a node
vector<int> value(5, INT_MAX); 
// priority queue to extract minimum weights
priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> MSTque;


//--------------------------------------------BASIC FUNCTION-----------------------------------------------
// To add an edge
void addEdge(vector <pair<int, int> > adj[], int u, int v, int wt)
{
	adj[u].push_back(make_pair(v, wt));
}
 
// Print adjacency list representation of graph
void printGraph(vector<pair<int,int> > adj[], int V)
{
    int v, w;
    string stateName[5] = {"Miami", "Istanbul", "Toronto", "Pyongyang", "Yokohama"};
    
    for (int u = 0; u < V; u++)
    {
        cout << "\t\t ";
		cout << stateName[u] << "(" << u << ") --> ";
        for (auto it = adj[u].begin(); it!=adj[u].end(); it++)
        {
            v = it->first;
            w = it->second;
            cout << "(" << v << ")" << w << "km --> ";
        }
        cout << "\n";
    }
}
// Delete edge
void delEdge(vector<pair<int,int> > adj[], int fromVertex, int toVertex, int V)
{
   	for (int u = 0; u < V ; u++)
	{
     	if (u==fromVertex)
     	{
     		for(vector<pair<int, int>> :: iterator it = adj[fromVertex].begin(); it != adj[fromVertex].end(); it++)
     		{
     			if (it->first == toVertex)
        		{
            	adj[u].erase(it);
            	break;
            	}
          }
        }
	}
}
// generate random edge
void generateRandomEdgeSP(vector <pair<int, int> > adj[])
{
    srand(time(0));
    int randEdge = rand() % 14;

    switch(randEdge) {
    case 0:
      addEdge(adj, 2, 0, 1989);
      break;

    case 1:
      addEdge(adj, 0, 3, 12928);
      break;

    case 2:
      addEdge(adj, 3, 0, 12928);
      break;

    case 3:
      addEdge(adj, 0, 4, 6566);
      break;

    case 4:
      addEdge(adj, 4, 0, 6566);
      break;
    
    case 5:
      addEdge(adj, 2, 1, 8193);
      break;

    case 6:
      addEdge(adj, 1, 2, 8193);
      break;

    case 7:
      addEdge(adj, 2, 3, 13317);
      break;

    case 8:
      addEdge(adj, 3, 2, 13317);
      break;

    case 9:
      addEdge(adj, 4, 2, 8183);
      break;

    case 10:
      addEdge(adj, 1, 0, 9611);
      break;

    case 11:
      addEdge(adj, 3, 1, 7456);
      break;
    
    case 12:
      addEdge(adj, 1, 4, 10575);
      break;
    
    case 13:
      addEdge(adj, 4, 1, 10575);
      break;

    case 14:
      addEdge(adj, 4, 3, 7419);
      break;

    default:
      break;
    }
}
	 
int checkDistance(int id1, int id2)
{
  int distance;

  if ((id1 == 2 && id2 == 0) || (id1 == 0 && id2 == 2))// if id1 = Toronto and id2 = Miami
  {distance = 1989;}
  
  else if ((id1 == 0 && id2 == 3) || (id1 == 3 && id2 == 0))  // if (id1 = Miami and id2 = Pyongyang) or (id1 = Pyongyang or id2 = Miami)
  {distance = 12928;}
  
  else if ((id1 == 0 && id2 == 4) || (id1 == 4 && id2 == 0)) // if (id1 = Miami and id2 = Yokohama) or (id1 = Yokohama or id2 = Miami)
  {distance = 6566;}

  else if ((id1 == 2 && id2 == 1) || (id1 == 1 && id2 == 2)) // if (id1 = Toronto and id2 = Istanbul) or (id1 = Istanbul or id2 = Toronto)
  {distance = 8193;}

  else if ((id1 == 2 && id2 == 3) || (id1 == 3 && id2 == 2)) // if (id1 = Toronto and id2 = Pyongyang) or (id1 = Pyongyang or id2 = Toronto)
  {distance = 13317;}

  else if ((id1 == 4 && id2 == 2) || (id1 == 2 && id2 == 4))// if id1 = Yokohama and id2 = Toronto
  {distance = 8183;}

  else if ((id1 == 1 && id2 == 0) || (id1 == 0 && id2 == 1))// if id1 = Istanbul and id2 = Miami
  {distance = 9611;}

  else if ((id1 == 3 && id2 == 1) || (id1 == 1 && id2 == 3))// if id1 = Pyongyang and id2 = Istanbul
  {distance = 7456;}

  else if ((id1 == 1 && id2 == 4) || (id1 == 4 && id2 == 1)) // if (id1 = Istanbul and id2 = Yokohama) or (id1 = Yokohama or id2 = Istanbul)
  {distance = 10575;}

  else if ((id1 == 4 && id2 == 3) || (id1 == 3 && id2 == 4))// if id1 = Yokohama and id2 = Pyongyang
  {distance = 7419;}
  return distance;
}
//--------------------------------FUNCTION 1: STRONGLY CONNECTED -----------------------------------------------------
int dfs(int v, bool visited[]){
	visited [v] = true;
	vector<pair<int, int> > adj[v];
	vector<pair<int, int>> :: iterator i;
	for (i = adj[v].begin(); i != adj[v].end(); ++i)
	if (!visited[v])
	return false;
}

void reverseArc(vector<pair<int,int> > adj[], int V) {
   for (int v = 0; v < V; v++) {
    	vector<pair<int, int> > adj[V] ;
		vector<pair<int, int>> :: iterator i;
      for(i = adj[v].begin(); i != adj[v].end(); ++i)
        	generateRandomEdgeSP(adj);
   }
}

int isStronglyConnected(vector<pair<int,int> > adj[],int v) {
	bool visited[v];
   		for (int i = 0; i < v; i++)
   		visited[i] = false;
   	dfs(0, visited);
   		for (int i = 0; i < v; i++)
    	if (visited[i] == false)
    	return false;
	reverseArc(adj, v);
	for (int i = 0; i < v; i++)
    	if (visited[i] == false)
    	return false;
	dfs(0, visited);
	for (int i = 0; i < v; i++)
    	if (visited[i] == false)
    	return false;
	return true;
}
//--------------------------------FUNCTION 2: CYCLE DETECTION --------------------------------------------------------
bool DFSUtil(int u, int color[], vector<pair<int,int> > adj[], string stateName [])
{
    // assign current vertex as gray color to indicate that it is currently being processed 
    color[u] = GRAY;
 
    // Iterate through all adjacent vertices
    vector<pair<int, int>> :: iterator it;
    for (it = adj[u].begin(); it != adj[u].end(); ++it)
    {
        int v = it->first;  // An adjacent of u

        cout << "\t\t" << stateName[u] << "(" << u << ")" << " -> " << stateName[v] << "(" << v << ")" << endl;
 
        // If the adjacent of u is in gray color (which means it has been processed)
        if (color[v] == GRAY)
        {   
            return true;  // there is a cycle 
        }
 
        // If v is not processed and there is a back
        // edge in subtree rooted with v
        if (color[v] == WHITE && DFSUtil(v, color, adj, stateName))
        {
            return true;
        }
    }
 
    // Mark this vertex as processed
    color[u] = BLACK;
 
    return false;
}

// Returns true if there is a cycle in graph
bool isCyclic(int V, vector<pair<int,int> > adj[], string stateName [])
{
    // Initialize color of all vertices as WHITE
    int *color = new int[V];
    for (int i = 0; i < V; i++)
        color[i] = WHITE;

    cout << "\t\tFollowing are the paths traversed in the graph by DFS algorithm: " << endl << endl;
 
    // Perform DFS traversal for all vertices
    for (int i = 0; i < V; i++)
        if (color[i] == WHITE)
           if (DFSUtil(i, color, adj, stateName) == true)
              return true;
 
    return false;
}

void generateRandomEdge(vector <pair<int, int> > adj[], int *v1, int *v2)
{
    srand(time(0));
    int randEdge = rand() % 14;

    switch(randEdge) {
    case 0:
      addEdge(adj, 2, 0, 1989);
      *v1 = 2, *v2 = 0;
      break;

    case 1:
      addEdge(adj, 0, 3, 12928);
      *v1 = 0, *v2 = 3;
      break;

    case 2:
      addEdge(adj, 3, 0, 12928);
      *v1 = 3, *v2 = 0;
      break;

    case 3:
      addEdge(adj, 0, 4, 6566);
      *v1 = 0, *v2 = 4;
      break;

    case 4:
      addEdge(adj, 4, 0, 6566);
      *v1 = 4, *v2 = 0;
      break;
    
    case 5:
      addEdge(adj, 2, 1, 8193);
      *v1 = 2, *v2 = 1;
      break;

    case 6:
      addEdge(adj, 1, 2, 8193);
      *v1 = 1, *v2 = 2;
      break;

    case 7:
      addEdge(adj, 2, 3, 13317);
      *v1 = 2, *v2 = 3;
      break;

    case 8:
      addEdge(adj, 3, 2, 13317);
      *v1 = 3, *v2 = 2;
      break;

    case 9:
      addEdge(adj, 4, 2, 8183);
      *v1 = 4, *v2 = 2;
      break;

    case 10:
      addEdge(adj, 1, 0, 9611);
      *v1 = 1, *v2 = 0;
      break;

    case 11:
      addEdge(adj, 3, 1, 7456);
      *v1 = 3, *v2 = 1;
      break;
    
    case 12:
      addEdge(adj, 1, 4, 10575);
      *v1 = 1, *v2 = 4;
      break;
    
    case 13:
      addEdge(adj, 4, 1, 10575);
      *v1 = 4, *v2 = 1;
      break;

    case 14:
      addEdge(adj, 4, 3, 7419);
      *v1 = 4, *v2 = 3;
      break;

    default:
      break;
    }
}
//----------------------------- FUNCTION 3: SHORTEST PATH -----------------------------------------------
//function to find minimum distance
int min_distance(int dist[], bool visited[]){
	int Min = INT_MAX, index = 0;
  	
	for(int i = 0; i < 5; i++)
	//if distance that is less than or equal to Min and unvisited, update Min = dist[i] and index i
   		if(dist[i] <= Min && visited[i] == false){
    		Min = dist[i];
    		index = i;
}
    return index;
}

// Function to print shortest path from source to j using
// parent array
void printPath(int parent[], int j,string stateName[])
{
    // Base Case : If j is source
    if (parent[j] == -1)
        return;
    printPath(parent, parent[j],stateName);
    cout << stateName[j] << "(" <<j << ") -> ";
 
}

//function to display either path exist and the shortest path and distance if path exist
void printShortPath(int path[], int dist[], int origin, int destination,int parent[],int V,string stateName[])
{   
    //nopath from source to dest
	if (dist[destination]==INT_MAX)
	{
	cout << "\n\t\tNo path detected from " << stateName[origin] << "("<< origin << ") to " << stateName[destination] << "(" << destination << ") ... \n\n\t\tGenerating new edge for you!" ;
	cout << "\n\n\t\tEnter 7 again to find shortest path.";
	}

    else
    {
    //print shortest path	
    int src = origin;
    cout << "\n\n\t\tShortest Path From " << stateName[origin] << "("<< origin << ") to " << stateName[destination] << "(" << destination << ") is: " ; //print origin and destination
    cout << endl << "\t\t";
	if(origin==0)
    {
    	cout << "\n\t\t\tMiami(0) -> ";
	}
	else if(origin!=0){
		cout << "\n\t\t\t";
	}
       for (int i = 0; i < V; i++) {
    if(src==origin&& i==destination)
        printPath(parent, i,stateName);
    }
    
    //print shortest distance
    cout << "\n\n\n\t\tShortest Distance From " << stateName[origin] << "("<< origin << ") to " << stateName[destination] << "(" << destination << ") : " << dist[destination] << "km" << endl;//print total distance form destination
	}
}

// Function that implements Dijkstra's from source to destination vertex
// shortest path algorithm for a graph represented using
// adjacency matrix representation
int* dijkstra(vector<pair<int,int> > adj[], int s, int d, int path[], int index,int V,string stateName[]){
    
    // The output array. dist[i] will hold the shortest
    // distance from src to i
	int dist[V];
	// visited[i] will true if vertex i is included / in
    // shortest path tree or shortest distance from src to i
    // is finalized
    bool visited[V] = {false};
    
    // Parent array to store shortest path tree
  	int parent[V] = { -1 };
  	
  	// Initialize all distances as INFINITE
    for(int i = 0; i < 6; i++)
    	dist[i] = INT_MAX;
    	
     // Distance of source vertex from itself is always 0	
    	dist[s] = 0;
    	
     // Find shortest path for all vertices
    for(int i = 0; i < V-1; i++){
    	// Pick the minimum distance vertex from the set of
        // vertices not yet processed. u is always equal to
        // src in first iteration.
    	int u = min_distance(dist, visited);
    	// Mark the picked vertex as processed
    	visited[u] = true;    
		// Update dist value of the adjacent vertices of the
        // picked vertex
    for(vector<pair<int, int>> :: iterator it = adj[u].begin(); it != adj[u].end(); it++)
    		// Update dist[it->first] only if is not in visited,
            // there is an edge from u to it->first, and total
            // weight of path from src to it->first through u is
            // smaller than current value of dist[it->first]
    	if(visited[it->first] == false && dist[u] != INT_MAX && dist[it->first] > dist[u] + it->second)
    	{    
    		parent[it->first] = u;
			dist[it->first] = dist[u] + it->second;
		}
  }

    // print the constructed distance array
    printShortPath(path, dist, s, d,parent,V,stateName);
	
	//return distance
	return dist;
}
   
 //function to print all possible paths from source to dest
void printAllPathsUtil(vector<pair<int,int> > adj[], int s, int d, bool visited[], int path[], int index,int V,string stateName[]) 
{ 
	//mark source as visited
	visited[s] = true;
	//store source in path array
    path[index]= s;
    //index counter 
    index++;
	
	//if s is equals to dest 	
    if(s==d)
	{
        int i;
        if(!pathExist)        
        cout<<"\n\t\tFollowing are the paths between " << path[0]<<" and "<< path[index-1] <<endl;
        
 		//mark as path exist
		pathExist=true;
		
		//print all possible paths from source to dest
		cout << "\n\t\t\t";
        for(i=0;i<index-1;i++)
            cout<< path[i] << " -> ";
        	cout<< path[i] << endl;
	}
    
    else{ //if s is not equals to dest 
    //traverse through all adjacent nodes from s 
    for(vector<pair<int, int>> :: iterator it = adj[s].begin(); it != adj[s].end(); it++)
	 	{   //if dest is not visited
            if (!visited[it->first]) 
                //call recursive function from node to other adjacent node
                printAllPathsUtil(adj,it->first, d, visited, path, index, V,stateName); 
    	}
		
		}
		
    // Remove current vertex from path[] and mark it as unvisited 
    index--;
    visited[s]=false;
}


// DFS traversal of the vertices reachable from source vertex 
void printAllPaths(vector<pair<int,int> > adj[], int s, int d,int V,string stateName[]) 
{ 
	//create array to store visited node
    bool *visited = new bool[V]; 
    //initialize all nodes as not visited
    for (int i = 0; i < V; i++) 
        visited[i] = false;
         
    // Create an array to store paths 
    int *path = new int[V];
    
    //declare index 0
    int index = 0;
    
    //declare no path exist first
    pathExist=false;   
    
    system ("cls");
    cout << "\n\n\t\t==============================================================\n";
    cout << "\t\t\t\t  Function 3 : Shortest Path \n";
    cout << "\t\t==============================================================\n";
    
    //call function to print all possible paths from source to dest
    printAllPathsUtil(adj,s,d,visited,path,index,V,stateName);
    
    //call function to find shortest path and shortest distance 
    dijkstra(adj,s,d,path,index,V,stateName); 
    
} 
//----------------------------- FUNCTION 4: MINUMUM SPANNING TREE -----------------------------------------------
void generateRandomEdgeMST(vector <pair<int, int> > adj[])
{
    int randEdge = rand() % 14;

    switch(randEdge) {
    case 0:
      addEdge(adj, 2, 0, 1989);
      break;

    case 1:
      addEdge(adj, 0, 3, 12928);
      break;

    case 2:
      addEdge(adj, 3, 0, 12928);
      break;

    case 3:
      addEdge(adj, 0, 4, 6566);
      break;

    case 4:
      addEdge(adj, 4, 0, 6566);
      break;
    
    case 5:
      addEdge(adj, 2, 1, 8193);
      break;

    case 6:
      addEdge(adj, 1, 2, 8193);
      break;

    case 7:
      addEdge(adj, 2, 3, 13317);
      break;

    case 8:
      addEdge(adj, 3, 2, 13317);
      break;

    case 9:
      addEdge(adj, 4, 2, 8183);
      break;

    case 10:
      addEdge(adj, 1, 0, 9611);
      break;

    case 11:
      addEdge(adj, 3, 1, 7456);
      break;
    
    case 12:
      addEdge(adj, 1, 4, 10575);
      break;
    
    case 13:
      addEdge(adj, 4, 1, 10575);
      break;

    case 14:
      addEdge(adj, 4, 3, 7419);
      break;

    default:
      break;
    }
}

// function to check if each node has incoming edges
// to make every node reachable to each other
void checkIncEdges(vector<pair<int,int> > adj[],int V){
	
	int* checkIncEdge;
	int f=0, y=0, a=0,b=0,c=0,d=0,e=0;
	int present=0, present1=0, present2=0, present3=0, present4=0;
	
	// manually create edge from Miami(0) to Istanbul(1)
	// if Miami has no outgoing edge
	// to avoid error while computing MST
	if(adj[0].size()==0){
		addEdge(adj,0,1,9611);
	}
	// get the number of outgoing edge for each node
	for (int u = 0; u < V ; u++){
     	for(vector<pair<int, int>> :: iterator it = adj[u].begin(); it != adj[u].end(); it++){
     		if(u==0) a++;
     		else if(u==1) b++;
     		else if(u==2) c++;
     		else if(u==3) d++;
     		else if(u==4) e++;		 
		}
	} 
	// create array to store the vertex that has incoming edges
	f=a+b+c+d+e;
	checkIncEdge = new int[f]();
	
	// store the vertex of the incoming edge in array
	for (int u = 0; u < V ; u++){
    	for(vector<pair<int, int>> :: iterator it = adj[u].begin(); it != adj[u].end(); it++){
    		checkIncEdge[y] = it->first;
    		y++;
    	}
	}
	
	// check if there is node that is not included in the array
	for(int i=0; i<f; i++){
		if(checkIncEdge[i]==1) present1=1;
		else if(checkIncEdge[i]==2) present2=1;
		else if(checkIncEdge[i]==3) present3=1;
		else if(checkIncEdge[i]==4) present4=1;
	}
	present = present1*present2*present3*present4;
	
	// if true, then generate random edge until every node has incoming edges
	// else, continue to compute MST
	if(present == 0){
		generateRandomEdgeMST(adj);
		checkIncEdges(adj,V);
	}
}

// function to compute Minimum Spanning Tree (MST)
void MinSpanningTree(vector<pair<int,int> > adj[],int V,int noEdges, int id3[],int id4[], int& sumWeight){
	// Push source node = 0 into the priority queue
	MSTque.push(make_pair(0, 0)); 
	// initialize source node weight to 0
    value[0]=0;                 
    while (!MSTque.empty()){ 
    	// get the node
        int node = MSTque.top().second;  
        // if the node is visited, change the bool value to true
        visited[node] = true;        
        MSTque.pop();               
		// check all its neighbours     
        for (auto neighbor : adj[node]) {   
            int weight = neighbor.second;       
            int vertex = neighbor.first;        
            // if the node is not visited and has less weight
            // update the values and push to MSTque to check its neighbours
            if (!visited[vertex] && value[vertex] > weight) {   
                value[vertex] = weight;                         
                connection[vertex] = node;
                MSTque.push(make_pair(value[vertex], vertex));    
            }
        }
    }
    // compute MST (calculate the total weight for MST)
    for(int i=0;i<V;i++){
    	sumWeight+=value[i];
	}
}

// function to print MST graph
void print_graphMST(vector<pair<int,int> > adj[],int V, int& sumWeight){
	
	string stateName[5] = {"Miami", "Istanbul", "Toronto", "Pyongyang", "Yokohama"};
	
	printGraph(adj,V);
	cout << "\n\t\t========================================\n";
    cout << "\t\t	   MINIMUM SPANNING TREE\n";
    cout << "\t\t========================================\n";
	// print out MST graph
    for (int i = 1; i < 5; ++i){
    	if(connection[i]==0)
			cout << "\t\tMiamu(";
		else if(connection[i]==1)
			cout << "\t\tIstanbul(";
		else if(connection[i]==2)
			cout << "\t\tToronto(";
		else if(connection[i]==3)
			cout << "\t\tPyongyang(";
		else if(connection[i]==4)
			cout << "\t\tYokohama(";
		cout << connection[i] << ") --> " << stateName[i] << "(" << i << ")"<<endl;
	}
	// print out the weight
    cout << "\n\t\tTotal Weight = " << sumWeight << " km\n\n";
}
//----------------------------------MAIN FUNCTION -------------------------------------
int main()
{
    int V = 5;

	string stateName[5] = {"Miami", "Istanbul", "Toronto", "Pyongyang", "Yokohama"};
	
    int id1, id2, w;
    int option, counter=0;
    int noEdges=0,sumWeight,valid1=0,valid2=0;
    int *id3, *id4;
    
    vector<pair<int, int> > adj[V] ;

    addEdge(adj, 0, 1, 9611);
    addEdge(adj, 0, 2, 1989);
    addEdge(adj, 2, 4, 8183);
    addEdge(adj, 1, 3, 7456);
    addEdge(adj, 3, 4, 7419);

    cout << "\t\t========================================\n";
    cout << "\t\t\t    DIRECTED GRAPH\n";
    cout << "\t\t========================================\n";
    printGraph(adj, V);
    cout << "\t\t========================================\n";

    do
  {
    cout << "\n\n\tWhat operation do you want to perform? "
         << " Choose a number or enter 0 to exit.\n"
         << endl;
    cout << "\t\t\t<1> Add Edge" << endl;
    cout << "\t\t\t<2> Delete Edge" << endl;
    cout << "\t\t\t<3> Print Graph" << endl;
    cout << "\t\t\t<4> Reset Graph" << endl;
    cout << "\t\t\t<5> Strongly Connected" << endl;
    cout << "\t\t\t<6> Cycle Detection" << endl;
    cout << "\t\t\t<7> Shortest Path" << endl;
    cout << "\t\t\t<8> Minimum Spanning Tree" << endl;
    cout << "\t\t\t<0> Exit Program" << endl;

    cout << "\n\tOption: ";
    cin >> option;

    switch (option)
    {
    case 0:
      break;

    case 1:
      cout << "\n\tAdd Edge Operation" << endl << endl;
      cout << "\tEnter ID of Source Vertex(State): ";
      cin >> id1;
      cout << "\tEnter ID of Destination Vertex(State): ";
      cin >> id2;
      w = checkDistance(id1, id2);
      addEdge(adj, id1, id2, w);
 	 
      break;

    case 2:
      cout << "\n\tDelete Edge Operation -" << endl<< endl;
      cout << "\tEnter ID of Source Vertex(State): ";
      cin >> id1;
      cout << "\tEnter ID of Destination Vertex(State): ";
      cin >> id2;
      delEdge(adj, id1, id2, V);
      break;

    case 3:
      cout << "\t\t========================================\n";
      cout << "\t\t\t    NEW DIRECTED GRAPH\n";
      cout << "\t\t========================================\n";
      printGraph(adj, V);
      cout << "\t\t========================================\n";
      break;

    case 4:
      cout << "\n\tGraph has been reset successfully!" << endl << endl;
      main();
      break;
    
    case 5: 
    	cout << "\t\t========================================\n";
      	cout << "\t\t         STRONGLY CONNECTED GRAPH\n";
      	cout << "\t\t========================================\n";
      	isStronglyConnected(adj,V)? cout << "\n\t\tThis graph is strongly connected" <<endl : cout << "\n\tThis graph is not strongly connected" << endl << endl;
		cout << "\t\t========================================\n";
      	cout << "\t\t      NEW STRONGLY CONNECTED GRAPH\n";
      	cout << "\t\t========================================\n";
    	printGraph(adj, V);
		break;
	
    case 6: 
      cout << "\t\t========================================\n";
      cout << "\t\t     FUNCTION 2: CYCLE DETECTION\n";
      cout << "\t\t========================================\n";
      cout << endl;

      int v1, v2;
      
      if (isCyclic(V, adj, stateName))
      {
        cout << "\n\n\t\tThe graph contains a cycle!";
        cout << "\n\n\t\tThere is a back edge detected in the graph which is the last path displayed above.";
		cout << "\n\n\t\tThe back edge indicates a cycle in the graph!";
      }
      else
      {
        cout << "\n\n\t\tThe graph doesn't contain a cycle!\n\n\t\tGenerating a random edge...";
        generateRandomEdge(adj, &v1, &v2);
        cout << "\n\n\t\tAn edge between " << stateName[v1] << "(" << v1 << ")" << " and " << stateName[v2] << "(" << v2 << ")" << " has been generated!" << endl;
        cout << "\n\t\tEnter 6 again if you wish to check if the graph has a cycle." << endl;
      }
		break;
    
	case 7: 
	  //Function to print shortest path between two cities
      //user input source and destination vertex
      //int v1, v2;
      cout << "\tEnter ID of Source Vertex(State): ";
      cin >> id1;
      cout << "\tEnter ID of Destination Vertex(State): ";
      cin >> id2;
      //find all paths and dijkstra algo starts
      printAllPaths(adj,id1,id2,V,stateName);
  	  //if path does not exist
      if(pathExist==false)
      {
      	generateRandomEdgeSP(adj);
	  }				
      break;
      
    case 8:
    	// allow user to select few edges with input validations
    	srand(time(0));
    	do{
    		cout << "\n\tEnter number of edges: ";
    		cin >> noEdges;
    		if(noEdges<1||noEdges>4)
    			cout << "\n\tNumber of edges should be between 1 to 4\n\n";
		}while(noEdges<1||noEdges>4);
    	id3 = new int [noEdges];
    	id4 = new int [noEdges];
    	for(int i=0; i<noEdges; i++){
    		do{
    			cout << "\tEnter 2 ID of Vertex (with space) for edge #" << i+1 << ": ";
    			cin >> valid1;
    			cin >> valid2;
    			if(valid1==valid2)
    				cout << "\n\tError: Edges should be different from each other\n\n";
    			else{
    				id3[i]=valid1;
    				id4[i]=valid2;
				}	
			}while(valid1==valid2);
		}
		system("CLS");
		cout << "\t\t========================================\n";
     	cout << "\t\t\t  NEW DIRECTED GRAPH\n";
   	 	cout << "\t\t========================================\n";
		checkIncEdges(adj,V);
		MinSpanningTree(adj,V,noEdges,id3,id4,sumWeight);
		print_graphMST(adj,V,sumWeight);
    	break;
    	
    default:
    	cout << "\n\tInvalid option!! Enter your choice again." << endl;
    }
    cout << endl;

  	}while (option != 0);
	return 0;
}
