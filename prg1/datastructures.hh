// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>
#include <functional>
#include <unordered_map>
#include<map>
#include <QSharedPointer>

// Types for IDs
using PlaceID = long int;
using AreaID = long int;
using Name = std::string;
using WayID = std::string;

// Return values for cases where required thing was not found
PlaceID const NO_PLACE = -1;
AreaID const NO_AREA = -1;
WayID const NO_WAY = "!!No way!!";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

// Enumeration for different place types
// !!Note since this is a C++11 "scoped enumeration", you'll have to refer to
// individual values as PlaceType::SHELTER etc.
enum class PlaceType { OTHER=0, FIREPIT, SHELTER, PARKING, PEAK, BAY, AREA, NO_TYPE };

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for a distance (in metres)
using Distance = int;

// Return value for cases where Duration is unknown
Distance const NO_DISTANCE = NO_VALUE;



// This is the class you are supposed to implement

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance: O(1)
    // Short rationale for estimate: Using hashtable index of unordered map
    int place_count();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Destroying each element at the data structure
    void clear_all();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Printing by iterating the unordered_map
    std::vector<PlaceID> all_places();

    // Estimate of performance: O(n) for the worst case, O(log n) for the average case
    // Short rationale for estimate: Searching element by key and adding element anywhere at unordered_map and multi_map
    bool add_place(PlaceID id, Name const& name, PlaceType type, Coord xy);

    // Estimate of performance: O(n) for the worst case, ϴ(1) for the average case
    // Short rationale for estimate: Searching value from unordered_map by key
    std::pair<Name, PlaceType> get_place_name_type(PlaceID id);

    // Estimate of performance: O(n) for the worst case, ϴ(1) for the average case
    // Short rationale for estimate: Searching value from unordered_map by key
    Coord get_place_coord(PlaceID id);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: O(n)
    // Short rationale for estimate: Printing by iterating the multi_map
    std::vector<PlaceID> places_alphabetically();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Printing by iterating the multi_map
    std::vector<PlaceID> places_coord_order();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Finding element by equal_range function and looping with the finding range to get all multimap values
    std::vector<PlaceID> find_places_name(Name const& name);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Finding element by equal_range function and looping with the finding range to get all multimap value
    std::vector<PlaceID> find_places_type(PlaceType type);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Using std::find_if for searching the right value multimap key
    // and updating with multi_map.erase(position), multi_map.insert and unordered_map.at
    bool change_place_name(PlaceID id, Name const& newname);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Using std::find_if for searching the right value multimap key
    // and updating with multi_map.erase(position), multi_map.insert and unordered_map.at
    bool change_place_coord(PlaceID id, Coord newcoord);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: O(n) for the worst case, ϴ(1) for the average case
    // Short rationale for estimate: Checking existance of id by unordered_map.find() and adding element anywhere at unordered_map
    bool add_area(AreaID id, Name const& name, std::vector<Coord> coords);

    // Estimate of performance: O(n) for the worst case, ϴ(1) for the average case
    // Short rationale for estimate: Searching name by unordered_map.find(id)
    Name get_area_name(AreaID id);

    // Estimate of performance: O(n) for the worst case, ϴ(1) for the average case
    // Short rationale for estimate: Searching coords by unordered_map.find(id)
    std::vector<Coord> get_area_coords(AreaID id);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Push back id by iterating unordered_map
    std::vector<AreaID> all_areas();

    // Estimate of performance: O(n) for the worst case, ϴ(1) for the average case
    // Short rationale for estimate: Using unordered_map.find(id) and edit value of element
    bool add_subarea_to_area(AreaID id, AreaID parentid);

    // Estimate of performance:O(n^2) for the worst case, ϴ(n) for the average case
    // Short rationale for estimate: Using unordered_map.find(id) and while loop to pushback all ancester areas
    std::vector<AreaID> subarea_in_areas(AreaID id);

    // Non-compulsory operations

    // Estimate of performance: O(1)
    // Short rationale for estimate: No implementation at this function
    void creation_finished();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Using std::queue to go through each children area and pushback child to the return vector
    std::vector<AreaID> all_subareas_in_area(AreaID id);

    // Estimate of performance: O(n log n)
    // Short rationale for estimate: Adding element at multimap with key of new distance and new y by iterating sub range by range_equal
    std::vector<PlaceID> places_closest_to(Coord xy, PlaceType type);

    // Estimate of performance: O(n)
    // Short rationale for estimate: Using std::find_if for searching the right value multimap / unordered_multimap key and erase by using iterater
    bool remove_place(PlaceID id);

    // Estimate of performance: O(n^2) for the worst case, ϴ(n) for the average case
    // Short rationale for estimate: Using std::unordered_map to add ancestor with id1 by while loop through the height of area tree,
    //  and checking common area using unordered_map.find with id2, parent of id2, and so on until it finds common ancestor area.
    AreaID common_area_of_subareas(AreaID id1, AreaID id2);

private:
    // Add stuff needed for your class implementation here
    struct Place_
    {
        PlaceID place_id_;
        Name place_name_;
        PlaceType place_type_;
        Coord coord_;
    };

    struct Area_
    {
        AreaID area_id_;
        Name area_name_;
        std::vector<Coord> coords_;
        std::shared_ptr<Area_> parent_;
        std::vector<std::shared_ptr<Area_>> children_;
    };

   std::unordered_map<PlaceID, std::shared_ptr<Place_>> places_;
   std::unordered_map<AreaID, std::shared_ptr<Area_>> areas_;

   std::multimap<Name, std::shared_ptr<Place_>> name_places_;
   std::multimap<std::pair<float, int>, std::shared_ptr<Place_>> dist_places_;

   std::unordered_multimap<PlaceType, std::shared_ptr<Place_>> type_places_;
};

#endif // DATASTRUCTURES_HH
