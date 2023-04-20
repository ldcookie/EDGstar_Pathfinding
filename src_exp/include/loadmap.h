#include <iostream>
#include <fstream>
#include<vector>
#include <string> 
using namespace std;

vector<int> Get_map(const char* path) {
    string tmpStr;
    vector<int> map_1_source;

    fstream myfile;
    myfile.open(path);

    myfile >> tmpStr;
    while (tmpStr != "map") {
        myfile >> tmpStr;
        //std::cout<< tmpStr<<endl;
    }

    int obi = 0;
    while (!myfile.eof()) {
        myfile >> tmpStr;//

        //切片
        for (auto i : tmpStr) {
            if (i == '.') {
                map_1_source.push_back(0);
                //std::cout << "该点空白" << endl;
            }
            else if (i == '@') {
                map_1_source.push_back(1);
                //std::cout << "该点占用" << endl;
                obi++;
            }
        }

    }
    //std::cout << "占用栅格数量为" << obi <<endl;

    myfile.close();
    return map_1_source;
}