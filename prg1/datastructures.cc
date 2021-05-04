// Datastructures.cc

#include "datastructures.hh"

#include <random>

#include <cmath>

#include <QDebug>

#include <queue>

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
