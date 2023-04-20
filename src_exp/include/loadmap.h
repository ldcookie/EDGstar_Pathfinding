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

        //��Ƭ
        for (auto i : tmpStr) {
            if (i == '.') {
                map_1_source.push_back(0);
                //std::cout << "�õ�հ�" << endl;
            }
            else if (i == '@') {
                map_1_source.push_back(1);
                //std::cout << "�õ�ռ��" << endl;
                obi++;
            }
        }

    }
    //std::cout << "ռ��դ������Ϊ" << obi <<endl;

    myfile.close();
    return map_1_source;
}