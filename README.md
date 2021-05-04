# Reasons for Choosing the Data Structures used in the Assignment
# Phase 1   
## 1) More Often Called Functions:  
Unordered_map<id, struct> is used because it requires looking up value with id. Since it is less likely to get the worst case, finding id is at the last key value of the unordered_map, using unordered_map complexity can be considered as constant. Struct contains the information for places or areas.  
e.g. Struct for place includes PlaceID, Name, PlaceType, Coord  
     Struct for area includes AreaID, Name, vector of Coord, parent shared_ptr, vector of shared_ptr for children
   * std::pair<Name, PlaceType> get_place_name_type(PlaceID id)
   * Coord get_place_coord(PlaceID id)
   * Name get_area_name(AreaID id)

## 2) Function with Additional Data Structure:   
   * Multi_map  
   Since it requires finding value with another key than id in order and there might be several places with the same key, multi_map with needed key for searching is used.
      * std::vector<PlaceID> places_alphabetically():  
      Key is name.
      * std::vector<PlaceID> places_coord_order():  
      Key is pair of distance to origin (0, 0) and y axis. I am using the key pair because first, it needs to be in order by distance, and with the same distance, a place with the smaller y needs to come first.
      * std::vector<PlaceID> find_places_name(Name const& name):  
      Key is name. It does not require to be in order, but there is multi_map from the alphabet order, so it is using the same one.
      * std::vector<PlaceID> places_closest_to(Coord xy, PlaceType type):  
      key is pair of distance to given xy and square of y axis difference to given y of xy. I am using the key pair because first, it needs to be in order by distance, and with the same distance, a place with the smaller y needs to come first.
   * Unordered_multi_map    
      * std::vector<PlaceID> find_places_type(PlaceType type)  
      It requires to find value with another key than id, and there might be several places with the same type, but it does not require the places in order by type, unordered_multi_map is proper for this function.
   * Unordered_map
      * AreaID common_area_of_subareas(AreaID id1, AreaID id2)  
As a local variable, unordered_map is used for saving all ancestors of id1. Then finding the lowest common area can be done by unordered_map.find with the lowest height (parent of id2) to the higher ancestor of id2.  
   * Queue
       * std::vector<AreaID> all_subareas_in_area(AreaID id)  
       Since first in first out structure is needed for saving each children, queue is used as local variable.
# Phase 2  
## 1) Way and Crossroad 
* Way   
Struct for each way is used to include way information, e.g. WayID, vector of coords, and distance of the way. It is combined with unordered_map with WayID as the key because finding coords by id is often called.   
* Crossroad   
Unordered_map with nested unordered_map is used to save which node is connected to which node, what is the way id between these two nodes, and what is the distance of the way. In other words, the first key is the Coord of the node, and the second key, the key of nested unordered_map, is WayID with values of neighbor coord and the distance of the way.   
Since some of the information, e.g. the color of the crossroad to check it is visited or not, what was the parent node of each node, the distance from beginning, and so on, depends on each function, I decide to use nested unordered_map with function-independent information instead of the struct for crossroad. The benefit of nested unordered_map is when you add the crossroad, no need to check what kind of crossroad (node) is already there because the combination of node Coord and wayID will be unique. Also, no need to reset the function-dependent information of crossroad every time to the default value. Lastly, nested unordered_map makes it easy and efficient to remove the way because erase element by key at unordered_map is constant on average, and no need to traverse the value of neighbor coords to erase specific neighbor.   
## 2) Algorithms   
* Breadth-first search (BFS)   
It is used for the route_any function. Depth-first search (DFS) also can be used for finding any route. Both have the same complexity, O(V+E), where V is the number of nodes, and E is the number of edges. However, in the worst case, BFS is better than DFS. BFS worst case, O(b^d), is smaller than DFS worst case, O(b^m), where b is the breath of the graph, d is goal node's depth, and m is the max depth of the graph.   
* Depth-first search (DFS)   
It is used for the route_with_cycle function because DFS can easily find the cycle.   
* Dijkstra's algorithm   
It is used for the route_least_crossroads function and route_shortest_distance function because Dijkstra's algorithm is made for these kinds of purposes.   
* Prim's algorithm   
It is used for the trim_ways function because Prim's algorithm can find a minimum spanning tree. For this case, it could be the  minimum spanning forest, so Prim's algorithm is combined with checking what is the unvisited crossroads.   
Kruskal's algorithm is also one of the well-known minimum spanning forest finding algorithms. I decide to use Prim's algorithm because it can be started from a random node, so no need to find the shortest distance way in the beginning. Also, it is expanding from neighbor nodes, so the implementation is less complicated in the project case.   
