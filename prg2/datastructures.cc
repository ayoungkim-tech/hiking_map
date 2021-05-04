// Datastructures.cc

#include "datastructures.hh"

#include <random>
#include <cmath>
#include <QDebug>
#include <QString>
#include <queue>

using Step = int;

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures()
{

}

Datastructures::~Datastructures()
{

}

int Datastructures::place_count()
{
    return places_.size();
}

void Datastructures::clear_all()
{
    places_.clear();
    areas_.clear();
    name_places_.clear();
    dist_places_.clear();
    type_places_.clear();
}

std::vector<PlaceID> Datastructures::all_places()
{
    std::vector<PlaceID> place_ids;
    for (auto& place: places_)
        place_ids.push_back(place.first);
    return place_ids;
}

bool Datastructures::add_place(PlaceID id, const Name& name, PlaceType type, Coord xy)
{
    auto itr = places_.find(id);
    if (itr == places_.end())
    {
        std::shared_ptr<Place_> place(new Place_ {id, name, type, xy});
        places_.insert({id, place});
        name_places_.insert({name, place});
        float distance = sqrt(pow(xy.x, 2) + pow(xy.y, 2));
        dist_places_.insert({{distance, xy.y}, place});
        type_places_.insert({type, place});
        return true;
    }
    return false;
}

std::pair<Name, PlaceType> Datastructures::get_place_name_type(PlaceID id)
{
    auto itr = places_.find(id);
    if (itr != places_.end())
    {
        Place_ place = (*itr->second);
        return {place.place_name_, place.place_type_};
    }
    return {NO_NAME, PlaceType::NO_TYPE};
}

Coord Datastructures::get_place_coord(PlaceID id)
{
    auto itr = places_.find(id);
    if (itr != places_.end())
    {
        return (*itr->second).coord_;
    }
    return NO_COORD;
}

bool Datastructures::add_area(AreaID id, const Name &name, std::vector<Coord> coords)
{
    auto itr = areas_.find(id);
    if (itr == areas_.end())
    {
        std::shared_ptr<Area_> area(new Area_ {id, name, coords, nullptr, {}});
        areas_.insert({id, area});
        return true;
    }
    return false;
}

Name Datastructures::get_area_name(AreaID id)
{
    auto itr = areas_.find(id);
    if (itr != areas_.end())
    {
        return {(*itr->second).area_name_};
    }
    return NO_NAME;
}

std::vector<Coord> Datastructures::get_area_coords(AreaID id)
{
    std::vector<Coord> coords;
    auto itr = areas_.find(id);
    if (itr != areas_.end())
    {
        coords = (*itr->second).coords_;
        return coords;
    }
    return {NO_COORD};
}

void Datastructures::creation_finished()
{

}


std::vector<PlaceID> Datastructures::places_alphabetically()
{
    std::vector<PlaceID> place_ids;
    for (auto& place: name_places_)
        place_ids.push_back((*place.second).place_id_);
    return place_ids;
}

std::vector<PlaceID> Datastructures::places_coord_order()
{
    std::vector<PlaceID> place_ids;
    for (auto& place: dist_places_)
        place_ids.push_back((*place.second).place_id_);
    return place_ids;
}

std::vector<PlaceID> Datastructures::find_places_name(Name const& name)
{
    std::vector<PlaceID> place_ids;
    std::multimap<Name, std::shared_ptr<Place_>>::iterator it;
    std::pair <std::multimap<Name, std::shared_ptr<Place_>>::iterator,
            std::multimap<Name, std::shared_ptr<Place_>>::iterator> ret;
    ret = name_places_.equal_range(name);
    for (it = ret.first; it != ret.second; ++it)
        place_ids.push_back((*it->second).place_id_);
    return place_ids;
}

std::vector<PlaceID> Datastructures::find_places_type(PlaceType type)
{
    std::vector<PlaceID> place_ids;
    std::unordered_multimap<PlaceType, std::shared_ptr<Place_>>::iterator it;
    std::pair <std::unordered_multimap<PlaceType, std::shared_ptr<Place_>>::iterator,
            std::unordered_multimap<PlaceType, std::shared_ptr<Place_>>::iterator> ret;
    ret = type_places_.equal_range(type);
    for (it = ret.first; it != ret.second; ++it)
        place_ids.push_back((*it->second).place_id_);
    return place_ids;
}

bool Datastructures::change_place_name(PlaceID id, const Name& newname)
{
    auto itr = std::find_if(name_places_.begin(), name_places_.end(), [=](auto find_id)
    {return find_id.second->place_id_ == id;});
    if (itr != name_places_.end())
    {
        std::shared_ptr<Place_> place_ptr = places_.at(id);
        (*place_ptr).place_name_ = newname;
        name_places_.erase(itr);
        name_places_.insert({newname, place_ptr});
        return true;
    }
    return false;
}

bool Datastructures::change_place_coord(PlaceID id, Coord newcoord)
{
    auto itr = std::find_if(dist_places_.begin(), dist_places_.end(), [=](auto find_id)
    {return find_id.second->place_id_ == id;});
    if (itr != dist_places_.end())
    {
        std::shared_ptr<Place_> place_ptr = places_.at(id);
        (*place_ptr).coord_ = newcoord;
        dist_places_.erase(itr);
        float new_dist = sqrt(pow(newcoord.x, 2) + pow(newcoord.y, 2));
        dist_places_.insert({{new_dist, newcoord.y}, place_ptr});
        return true;
    }
    return false;
}

std::vector<AreaID> Datastructures::all_areas()
{
    std::vector<AreaID> area_ids;
    for (auto& area: areas_)
        area_ids.push_back(area.first);
    return area_ids;
}

bool Datastructures::add_subarea_to_area(AreaID id, AreaID parentid)
{
    auto itr = areas_.find(id);
    auto parent_itr = areas_.find(parentid);
    if (itr != areas_.end() and parent_itr != areas_.end())
    {
        std::shared_ptr<Area_> area_ptr = itr->second;
        if (area_ptr->parent_ == nullptr)
        {
            std::shared_ptr<Area_> parent_ptr = parent_itr->second;
            area_ptr->parent_ = parent_ptr;
            parent_ptr->children_.push_back(area_ptr);
            return true;
        }
    }
    return false;
}

std::vector<AreaID> Datastructures::subarea_in_areas(AreaID id)
{
    std::vector<AreaID> parent_areas;
    auto itr = areas_.find(id);
    if (itr != areas_.end())
    {
        auto parent_ptr = itr->second->parent_;
        while (parent_ptr != nullptr)
        {
            parent_areas.push_back(parent_ptr->area_id_);
            parent_ptr = parent_ptr -> parent_;
        }
        return parent_areas;
    }
    return {NO_AREA};
}

std::vector<PlaceID> Datastructures::places_closest_to(Coord xy, PlaceType type)
{
    std::vector<PlaceID> place_ids;
    std::multimap<std::pair<float, int>, std::shared_ptr<Place_>> dist_places;
    if (type != PlaceType::NO_TYPE)
    {
        std::unordered_multimap<PlaceType, std::shared_ptr<Place_>>::iterator it;
        std::pair <std::unordered_multimap<PlaceType, std::shared_ptr<Place_>>::iterator,
                std::unordered_multimap<PlaceType, std::shared_ptr<Place_>>::iterator> ret;
        ret = type_places_.equal_range(type);
        for (it = ret.first; it != ret.second; ++it)
        {
            std::shared_ptr<Place_> place_ptr = (it->second);
            float distance = sqrt(pow((xy.x - (*place_ptr).coord_.x), 2) + pow((xy.y - (*place_ptr).coord_.y), 2));
            int place_y = abs(xy.y - (*place_ptr).coord_.y);
            dist_places.insert({{distance, place_y}, place_ptr});
        }
    }
    else
    {
        for (auto& itr: dist_places_)
        {
            std::shared_ptr<Place_> place_ptr = (itr.second);
            float distance = sqrt(pow((xy.x - (*place_ptr).coord_.x), 2) + pow((xy.y - (*place_ptr).coord_.y), 2));
            int place_y = abs(xy.y - (*place_ptr).coord_.y);
            dist_places.insert({{distance, place_y}, place_ptr});
        }
    }
    int vec_size = 0;
    for (auto& place: dist_places) // Take first three elements
    {
        place_ids.push_back((*place.second).place_id_);
        vec_size += 1;
        if (vec_size == 3)
            break;
    }
    return place_ids;
}

bool Datastructures::remove_place(PlaceID id)
{
    auto itr_name = std::find_if(name_places_.begin(), name_places_.end(), [=](auto find_id)
    {return find_id.second->place_id_ == id;});
    if (itr_name != name_places_.end())
    {
        name_places_.erase(itr_name);
    }
    auto itr_dist = std::find_if(dist_places_.begin(), dist_places_.end(), [=](auto find_id)
    {return find_id.second->place_id_ == id;});
    if (itr_dist != dist_places_.end())
    {
        dist_places_.erase(itr_dist);
    }
    auto itr_type = std::find_if(type_places_.begin(), type_places_.end(), [=](auto find_id)
    {return find_id.second->place_id_ == id;});
    if (itr_type != type_places_.end())
    {
        type_places_.erase(itr_type);
        places_.erase(id);
        return true;
    }
    return false;
}

std::vector<AreaID> Datastructures::all_subareas_in_area(AreaID id)
{
    auto itr = areas_.find(id);
    if (itr == areas_.end())
        return {NO_AREA};
    std::vector<AreaID> area_ids;
    std::queue <std::shared_ptr<Area_>> q;
    std::shared_ptr<Area_> root = areas_.at(id);
    q.push(root);
    while (!q.empty())
    {
        int n = q.size();
        while (n > 0)
        {
            std::shared_ptr<Area_> node = q.front();
            q.pop();
            if ((*node).area_id_ != id)
                area_ids.push_back((*node).area_id_);
            std::vector <std::shared_ptr<Area_>> children = node->children_;
            for (unsigned int i = 0; i < children.size(); i++)
                q.push(children[i]);
            n--;
        }
    }
    return area_ids;
}

AreaID Datastructures::common_area_of_subareas(AreaID id1, AreaID id2)
{
    auto itr_id1 = areas_.find(id1);
    auto itr_id2 = areas_.find(id2);
    if (itr_id1 != areas_.end() and itr_id2 != areas_.end())
    {
        std::unordered_map <std::shared_ptr<Area_>, bool> ancestors;
        std::shared_ptr<Area_> area1 = itr_id1->second->parent_;
        std::shared_ptr<Area_> area2 = itr_id2->second->parent_;
        while (area1 != nullptr)
        {
            ancestors[area1] = true;
            area1 = area1->parent_;
        }
        while (area2 != nullptr)
        {
            if (ancestors.find(area2) != ancestors.end())
                return area2->area_id_;
            area2 = area2->parent_;
        }
    }
    return NO_AREA;
}

std::vector<WayID> Datastructures::all_ways()
{
    std::vector<WayID> way_ids;
    for (auto& way: ways_)
        way_ids.push_back(way.first);
    return way_ids;
}

bool Datastructures::add_way(WayID id, std::vector<Coord> coords)
{   
    auto itr_id = ways_.find(id);
    if (itr_id == ways_.end())
    {
        // Calculate distance
        Distance tot_dist = 0;
        unsigned int coords_size = coords.size();
        for (unsigned int idx = 1; idx < coords_size; idx++)
        {
            Coord from_xy = coords.at(idx - 1);
            Coord to_xy = coords.at(idx);
            Distance dist = sqrt(pow((from_xy.x - to_xy.x), 2) + pow((from_xy.y - to_xy.y), 2));;
            tot_dist += round(dist);
        }
        // Add to ways_
        std::shared_ptr<Way_> way(new Way_ {id, coords, tot_dist});
        ways_.insert({id, way});
        // Add to crossroads_
        Coord start_xy = coords.at(0);
        Coord end_xy = coords.at(coords_size - 1);
        crossroads_[start_xy][id] = {tot_dist,end_xy};
        crossroads_[end_xy][id] = {tot_dist, start_xy};
        return true;
    }
    return false;
}

std::vector<std::pair<WayID, Coord>> Datastructures::ways_from(Coord xy)
{    
    auto itr_from_xy = crossroads_.find(xy);
    if (itr_from_xy != crossroads_.end())
    {
        std::vector<std::pair<WayID, Coord>> destinations;
        std::unordered_map<WayID, std::pair<Distance, Coord>>::iterator itr_to_xy;
        std::unordered_map<WayID, std::pair<Distance, Coord>> nested_unordered_map = itr_from_xy->second;
        for (itr_to_xy = nested_unordered_map.begin(); itr_to_xy != nested_unordered_map.end(); itr_to_xy++)
        {
            destinations.push_back({itr_to_xy->first, itr_to_xy->second.second});
        }
        return destinations;
    }
    return {};
}

std::vector<Coord> Datastructures::get_way_coords(WayID id)
{
    auto itr = ways_.find(id);
    if (itr != ways_.end())
    {
        return (*itr->second).coords_;
    }
    return {NO_COORD};
}

void Datastructures::clear_ways()
{
    ways_.clear();
    crossroads_.clear();
}

std::vector<std::tuple<Coord, WayID, Distance> > Datastructures::route_any(Coord fromxy, Coord toxy)
{
    auto itr_from_xy = crossroads_.find(fromxy);
    auto itr_to_xy = crossroads_.find(toxy);
    if (itr_from_xy != crossroads_.end() && itr_to_xy != crossroads_.end())
    {
        // BFS
        std::unordered_map<Coord, std::tuple<Coord, WayID, Distance>, CoordHash, std::equal_to<Coord>> visited;
        std::queue<std::tuple<Coord, Coord, WayID, Distance>> queue;
        Coord parent_node = NO_COORD;
        WayID from_way_id = NO_WAY;
        route_dist_ = 0;
        queue.push({fromxy, parent_node, from_way_id, route_dist_});
        while (!queue.empty())
        {
            Coord current_node = std::get<0>(queue.front());
            parent_node = std::get<1>(queue.front());
            from_way_id = std::get<2>(queue.front());
            route_dist_ = std::get<3>(queue.front());
            visited.insert({current_node, {parent_node, from_way_id, route_dist_}});
            queue.pop();
            if (current_node == toxy)
            {
                break;
            }
            std::unordered_map<WayID, std::pair<Distance, Coord>>::iterator itr_neighbor;
            std::unordered_map<WayID, std::pair<Distance, Coord>> nested_unordered_map = crossroads_.at(current_node);
            for (itr_neighbor = nested_unordered_map.begin(); itr_neighbor != nested_unordered_map.end(); itr_neighbor++)
            {
                WayID from_way_id = itr_neighbor->first;
                Distance from_way_dist = itr_neighbor->second.first;
                Coord neighbor_node = itr_neighbor->second.second;
                auto itr_visited = visited.find(neighbor_node);
                if (itr_visited == visited.end())
                {
                    queue.push({neighbor_node, current_node, from_way_id, route_dist_ + from_way_dist});
                }
            }
        }
        // Print path
        auto itr_current_xy = visited.find(toxy);
        if (itr_current_xy == visited.end())
        {
            return {{NO_COORD, NO_WAY, NO_DISTANCE}};
        }
        std::vector<std::tuple<Coord, WayID, Distance>> route;
        Coord current_node = toxy;
        WayID to_way_id = NO_WAY;
        Distance dist_so_far;
        while (current_node != NO_COORD)
        {
            std::tuple<Coord, WayID, Distance> node_info = visited.at(current_node);
            dist_so_far = std::get<2>(node_info);
            route.push_back({current_node, to_way_id, dist_so_far});
            to_way_id = std::get<1>(node_info);
            current_node = std::get<0>(node_info);
        }
        std::reverse(route.begin(), route.end());
        return route;
    }
    return {{NO_COORD, NO_WAY, NO_DISTANCE}};
}

bool Datastructures::remove_way(WayID id)
{
    auto itr_id = ways_.find(id);
    if (itr_id != ways_.end())
    {
        std::vector <Coord> coords = itr_id->second->coords_;
        unsigned int coords_size = coords.size();
        Coord start_xy = coords.at(0);
        Coord end_xy = coords.at(coords_size - 1);
        crossroads_.at(start_xy).erase(id);
        crossroads_.at(end_xy).erase(id);
        ways_.erase(itr_id);
        return true;
    }
    return false;
}

std::vector<std::tuple<Coord, WayID, Distance> > Datastructures::route_least_crossroads(Coord fromxy, Coord toxy)
{
    auto itr_from_xy = crossroads_.find(fromxy);
    auto itr_to_xy = crossroads_.find(toxy);
    if (itr_from_xy != crossroads_.end() && itr_to_xy != crossroads_.end())
    {
        std::unordered_map <Coord, Step, CoordHash, std::equal_to<Coord>> step_to_start;
        for (auto itr = crossroads_.begin(); itr != crossroads_.end(); ++itr)
        {
            step_to_start.insert({itr->first, INT_MAX});
        }
        // Dijkstra
        std::unordered_map<Coord, std::tuple<Coord, WayID, Distance, Step>, CoordHash, std::equal_to<Coord>> visited;
        auto comparison = [] (std::tuple<Coord, Distance, Step, Coord, WayID> a, std::tuple<Coord, Distance, Step, Coord, WayID> b)
        {
            return std::get<2>(a) > std::get<2>(b);
        };
        std::priority_queue<std::tuple<Coord, Distance, Step, Coord, WayID>, std::vector<std::tuple<Coord, Distance, Step, Coord, WayID>>,
                decltype(comparison)> priority_queue(comparison);
        route_dist_ = 0;
        Step tot_step = 0;
        WayID from_way_id = NO_WAY;
        Coord parent_node = NO_COORD;
        priority_queue.push({fromxy, route_dist_, tot_step, parent_node, from_way_id});
        step_to_start.at(fromxy) = tot_step;
        while (!priority_queue.empty())
        {
            Coord current_node = std::get<0>(priority_queue.top());
            route_dist_ = std::get<1>(priority_queue.top());
            tot_step = std::get<2>(priority_queue.top());
            parent_node = std::get<3>(priority_queue.top());
            from_way_id = std::get<4>(priority_queue.top());
            visited.insert({current_node, {parent_node, from_way_id, route_dist_, tot_step}});
            priority_queue.pop();
            if (current_node == toxy)
            {
                break;
            }
            std::unordered_map<WayID, std::pair<Distance, Coord>>::iterator itr_neighbor;
            std::unordered_map<WayID, std::pair<Distance, Coord>> nested_unordered_map = crossroads_.at(current_node);
            for (itr_neighbor = nested_unordered_map.begin(); itr_neighbor != nested_unordered_map.end(); itr_neighbor++)
            {
                WayID from_way_id = itr_neighbor->first;
                Distance from_way_dist = itr_neighbor->second.first;
                Coord neighbor_node = itr_neighbor->second.second;
                auto itr_visited = visited.find(neighbor_node);
                if (itr_visited == visited.end() && step_to_start.at(neighbor_node) > tot_step + 1)
                {
                    priority_queue.push({neighbor_node, route_dist_ + from_way_dist, tot_step + 1, current_node, from_way_id});
                    step_to_start.at(neighbor_node) = tot_step + 1;
                }
            }
        }
        // Print path
        std::vector<std::tuple<Coord, WayID, Distance>> route;
        auto itr_current_xy = visited.find(toxy);
        if (itr_current_xy != visited.end())
        {
            Coord current_node = toxy;
            WayID to_way_id = NO_WAY;
            Distance dist_so_far;
            while (current_node != NO_COORD)
            {
                std::tuple<Coord, WayID, Distance, Step> node_info = visited.at(current_node);
                dist_so_far = std::get<2>(node_info);
                route.push_back({current_node, to_way_id, dist_so_far});
                to_way_id = std::get<1>(node_info);
                current_node = std::get<0>(node_info);
            }
            std::reverse(route.begin(), route.end());
        }
        return route;
    }
    return {{NO_COORD, NO_WAY, NO_DISTANCE}};
}

std::vector<std::tuple<Coord, WayID> > Datastructures::route_with_cycle(Coord fromxy)
{
    auto itr_from_xy = crossroads_.find(fromxy);
    if (itr_from_xy != crossroads_.end())
    {
        // DFS
        cycle_route_.clear();
        visited_.clear();
        way_id_ = NO_WAY;
        start_node_ = NO_COORD;
        end_node_ = NO_COORD;
        if (DFS(fromxy, NO_COORD))
        {
            std::reverse(cycle_route_.begin(), cycle_route_.end());
            cycle_route_.push_back({end_node_, way_id_});
            cycle_route_.push_back({start_node_, NO_WAY});
            return cycle_route_;
        }
        else
            return cycle_route_;
    }
    return {{NO_COORD, NO_WAY}};
}

std::vector<std::tuple<Coord, WayID, Distance> > Datastructures::route_shortest_distance(Coord fromxy, Coord toxy)
{
    auto itr_from_xy = crossroads_.find(fromxy);
    auto itr_to_xy = crossroads_.find(toxy);
    if (itr_from_xy != crossroads_.end() && itr_to_xy != crossroads_.end())
    {
        std::unordered_map <Coord, Distance, CoordHash, std::equal_to<Coord>> dist_to_start;
        for (auto itr = crossroads_.begin(); itr != crossroads_.end(); ++itr)
        {
            dist_to_start.insert({itr->first, INT_MAX});
        }
        // Dijkstra
        std::unordered_map<Coord, std::tuple<Coord, WayID, Distance>, CoordHash, std::equal_to<Coord>> visited;
        auto comparison = [] (std::tuple<Coord, Distance, Coord, WayID> a, std::tuple<Coord, Distance, Coord, WayID> b)
        {
            return std::get<1>(a) > std::get<1>(b);
        };
        std::priority_queue<std::tuple<Coord, Distance, Coord, WayID>, std::vector<std::tuple<Coord, Distance, Coord, WayID>>,
                decltype(comparison)> priority_queue(comparison);
        route_dist_ = 0;
        WayID from_way_id = NO_WAY;
        Coord parent_node = NO_COORD;
        priority_queue.push({fromxy, route_dist_, parent_node, from_way_id});
        dist_to_start.at(fromxy) = route_dist_;
        while (!priority_queue.empty())
        {
            Coord current_node = std::get<0>(priority_queue.top());
            route_dist_ = std::get<1>(priority_queue.top());
            parent_node = std::get<2>(priority_queue.top());
            from_way_id = std::get<3>(priority_queue.top());
            visited.insert({current_node, {parent_node, from_way_id, route_dist_}});
            priority_queue.pop();
            if (current_node == toxy)
            {
                break;
            }
            std::unordered_map<WayID, std::pair<Distance, Coord>>::iterator itr_neighbor;
            std::unordered_map<WayID, std::pair<Distance, Coord>> nested_unordered_map = crossroads_.at(current_node);
            for (itr_neighbor = nested_unordered_map.begin(); itr_neighbor != nested_unordered_map.end(); itr_neighbor++)
            {
                WayID from_way_id = itr_neighbor->first;
                Distance from_way_dist = itr_neighbor->second.first;
                Coord neighbor_node = itr_neighbor->second.second;
                auto itr_visited = visited.find(neighbor_node);
                if (itr_visited == visited.end() && dist_to_start.at(neighbor_node) > route_dist_ + from_way_dist)
                {
                    priority_queue.push({neighbor_node, route_dist_ + from_way_dist, current_node, from_way_id});
                    dist_to_start.at(neighbor_node) = route_dist_ + from_way_dist;
                }
            }
        }
        // Print path
        std::vector<std::tuple<Coord, WayID, Distance>> route;
        auto itr_current_xy = visited.find(toxy);
        if (itr_current_xy != visited.end())
        {
            Coord current_node = toxy;
            WayID to_way_id = NO_WAY;
            Distance dist_so_far;
            while (current_node != NO_COORD)
            {
                std::tuple<Coord, WayID, Distance> node_info = visited.at(current_node);
                dist_so_far = std::get<2>(node_info);
                route.push_back({current_node, to_way_id, dist_so_far});
                to_way_id = std::get<1>(node_info);
                current_node = std::get<0>(node_info);
            }
            std::reverse(route.begin(), route.end());
        }
        return route;
    }
    return {{NO_COORD, NO_WAY, NO_DISTANCE}};
}

Distance Datastructures::trim_ways()
{
    std::unordered_map <Coord, std::pair<WayID, Distance>, CoordHash, std::equal_to<Coord>> not_visited;
    for (auto itr = crossroads_.begin(); itr != crossroads_.end(); ++itr)
    {
        not_visited.insert({itr->first, {NO_WAY, INT_MAX}});
    }
    // Prim's Algorithm
    Distance msf_dist = 0;
    while (!not_visited.empty())
    {
        auto comparison = [] (std::tuple<Coord, Distance> a, std::tuple<Coord, Distance> b)
        {
            return std::get<1>(a) > std::get<1>(b);
        };
        std::priority_queue<std::tuple<Coord, Distance>, std::vector<std::tuple<Coord, Distance>>,
                decltype(comparison)> priority_queue(comparison);
        Coord start_node = not_visited.begin()->first;
        priority_queue.push({start_node, 0});
        WayID from_way_id = NO_WAY;
        not_visited.at(start_node) = {from_way_id, 0};
        while (!priority_queue.empty())
        {
            Coord current_node = std::get<0>(priority_queue.top());
            Distance current_dist = std::get<1>(priority_queue.top());
            priority_queue.pop();
            auto itr_not_visited = not_visited.find(current_node);
            if (itr_not_visited != not_visited.end())
            {
                msf_dist += current_dist;
                from_way_id = itr_not_visited->second.first;
                not_visited.erase(itr_not_visited);
                std::unordered_map<WayID, std::pair<Distance, Coord>>::iterator itr_neighbor;
                std::unordered_map<WayID, std::pair<Distance, Coord>> nested_unordered_map = crossroads_.at(current_node);
                for (itr_neighbor = nested_unordered_map.begin(); itr_neighbor != nested_unordered_map.end(); itr_neighbor++)
                {
                    Coord neighbor_node = itr_neighbor->second.second;
                    Distance neighbor_dist = itr_neighbor->second.first;
                    WayID to_way_id = itr_neighbor->first;
                    auto itr_not_visited = not_visited.find(neighbor_node);
                    if (itr_not_visited != not_visited.end())
                    {
                        priority_queue.push({neighbor_node, neighbor_dist});
                        if (itr_not_visited->second.second > neighbor_dist)
                        {
                            itr_not_visited->second = {to_way_id, neighbor_dist};
                        }
                    }
                    else if (itr_not_visited == not_visited.end() && to_way_id != from_way_id)
                    {
                        remove_way(to_way_id);
                    }
                }
            }
        }
    }
    return msf_dist;
}

bool Datastructures::DFS(Coord current_node, Coord parent_node)
{
    visited_.insert({current_node, {parent_node, way_id_}});
    std::unordered_map<WayID, std::pair<Distance, Coord>> nested_unordered_map = crossroads_.at(current_node);
    for (auto itr_neighbor = nested_unordered_map.begin(); itr_neighbor != nested_unordered_map.end(); itr_neighbor++)
    {
        Coord neighbor_node = itr_neighbor->second.second;
        way_id_ = itr_neighbor->first;
        if (neighbor_node == parent_node)
            continue;
        auto itr_visited = visited_.find(neighbor_node);
        if (itr_visited != visited_.end())
        {
            end_node_ = current_node;
            start_node_ = neighbor_node;                                  
            return true;
        }
        else if (DFS(neighbor_node, current_node))
        {
            cycle_route_.push_back({current_node, itr_neighbor->first});
            return true;
        }
    }
    return false;
}
