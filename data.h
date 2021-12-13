#include <vector>
#include <fstream>
#include <sstream>
#include <regex>
#include "LinearMath/btTransform.h"
#include "LinearMath/btQuaternion.h"

bool extract_trans(const std::string& line, btTransform& trans) {
    float t_x, t_y, t_z, r_x, r_y, r_z, r_a, r_d;
    int result = sscanf(line.c_str(), "\ttrans_model = [translation = [%f, %f, %f], rotation = [axis = [%f, %f, %f], angle = %f, degree = %f]]",
           &t_x, &t_y, &t_z, &r_x, &r_y, &r_z, &r_a, &r_d);
    
    if (result == 0) {
        return false;
    }
    
    trans.setIdentity();
    trans.setOrigin(btVector3(t_x, t_y, t_z));
    trans.setRotation(btQuaternion(btVector3(r_x, r_y, r_z), r_a));
    
    return true;
}

bool get_transforms_from_file(const std::string& filename, std::vector<btTransform>& transforms) {
    std::ifstream fs(filename);
    if (!fs.is_open()) {
        return false;
    }
    
    transforms.clear();
    std::string line;
    while(std::getline(fs, line)) {
        std::istringstream iss(line);
        std::string head;
        iss >> head;
        
        btTransform trans;
        if (head == "trans_model" && extract_trans(line, trans)) {
            transforms.emplace_back(std::move(trans));
        }
    }
    
    if (transforms.empty()) {
        return false;
    }
    
    return true;
}
