#include <Lidar_MCduo_2023/Lidar_declare.h>

using namespace std;

void Filter::TC_filter(vector<objectInfo>& objs, bool flag){
    if(!flag) return;
    
    for (vector<objectInfo>::iterator it = objs.begin(); it != objs.end();) {   
        if (it->zMax > 0.2 || (abs(it->zMax-it->zMin) > 0.75 || abs(it->zMax-it->zMin) < 0.1) || (abs(it->xMax-it->xMin) > 0.45 || abs(it->xMax-it->xMin) < 0.00) || (abs(it->yMax-it->yMin) > 0.45 || abs(it->yMax-it->yMin) < 0.00)){
            it = objs.erase(it); //edit object vector
        }
        else it++;
    }
}

void Filter::DY_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag){
    if(!flag) return;
    vector<pair<PXYZI,string>>::iterator it = sorted_OBJ.begin();
    for (int i = 0; i < sorted_OBJ.size(); i++, it++) {
        if(sorted_OBJ[i].first.z < Ransac_Z_ROI){
            it = sorted_OBJ.erase(it);
            it--;
            i--;
        }
    }
}

void Filter::DY_filter(vector<objectInfo>& objs, bool flag){
    if(!flag) return;

    for (vector<objectInfo>::iterator it = objs.begin(); it != objs.end();){

        if (it->zMax < Ransac_Z_ROI){
            //cout << "\033[1;7mDY_Filter erase obj info\033[0m ";
            //cout << "  z max = " << it->zMax;
            //cout << "  z center = " << it->z << endl;
            it = objs.erase(it); //edit object vector
        }
        else it++;
    }
}


void Filter::jiwon_filter(vector<pair<PXYZI,string>>& sorted_OBJ, bool flag) {
    if(!flag) return;
    vector<pair<PXYZI,string>>::iterator it = sorted_OBJ.begin();
    for (int i = 0; i < sorted_OBJ.size(); i++) {
        it = sorted_OBJ.begin();
        for (int k = 0; k < i + 1; k++) {
            it++;
        }
        for (int j = i + 1; j < sorted_OBJ.size(); j++) {
            if (check_in(sorted_OBJ[i].first, sorted_OBJ[j].first)) {
                it = sorted_OBJ.erase(it);
                it--;
                j--;
            }
            it++;
        }
    }
}

void Filter::jiwon_filter(vector<objectInfo>& objs, bool flag) {
    if(!flag) return;

    for (vector<objectInfo>::iterator iit = objs.begin(), jit; iit != objs.end(); iit++){

        jit = iit + 1;

        for(jit; jit != objs.end();){

            if(((abs(iit->x - jit->x) <= REMOVE_FACTOR) && (abs(iit->y - jit->y) <= REMOVE_FACTOR))){
                //cout << "\033[1;46mjiwon_Filter merged\033[0m" << endl;

                //edit obj info
                iit->xMin = (iit->xMin < jit->xMin) ? iit->xMin : jit->xMin;
                iit->yMin = (iit->yMin < jit->yMin) ? iit->yMin : jit->yMin;
                iit->zMin = (iit->zMin < jit->zMin) ? iit->zMin : jit->zMin;
                iit->xMax = (iit->xMax > jit->xMax) ? iit->xMax : jit->xMax;
                iit->yMax = (iit->yMax > jit->yMax) ? iit->yMax : jit->yMax;
                iit->zMax = (iit->zMax > jit->zMax) ? iit->zMax : jit->zMax;
                iit->x = (iit->xMin + iit->xMax) / 2;
                iit->y = (iit->yMin + iit->yMax) / 2;
                iit->z = (iit->zMin + iit->zMax) / 2;

                //merge point indices
                iit->objPoints->indices.insert(iit->objPoints->indices.end(), jit->objPoints->indices.begin(), jit->objPoints->indices.end());

                jit = objs.erase(jit); //edit object vector
            }
            else jit++;
        }

    }
}

void Filter::generate_return_PointCloud(PCXYZI::Ptr inputCloud, PCXYZI& returnCloud, vector<objectInfo>& objs){
    short intensity = 0; //color PointCloud to seperate each cluster
    returnCloud.clear();
    for (const objectInfo& obj : objs){
        for(const int& idx : obj.objPoints->indices){
            PXYZI tmpInput = inputCloud->points[idx];
            tmpInput.intensity = (intensity) % 10;
            returnCloud.push_back(tmpInput);
        }
        intensity++;
    }
}