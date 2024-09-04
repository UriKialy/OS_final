#include "MST.hpp"







int MST::getWieght(){
    int weight = 0;
    for(int i = 0; i < mst.size(); i++){
        for(int j = 0; j < mst.size(); j++){
            weight += mst.at(i).at(j);
        }
    }
    return weight;
}