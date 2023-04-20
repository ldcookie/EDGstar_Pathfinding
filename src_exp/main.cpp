#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "jump_point_map.h"
#include "astar_search.h"
#include "load_matrix.h"
#include "ScenarioLoader.h"
#include "loadmap.h"
#include "coutpng.h"
#include <chrono>


#include "DGR_Star.h"
using namespace std;

using namespace cv;


//ȫ�ֱ���
// ��ͼ�Ŀ�
int k_map_width_expan;
// ��ͼ�ĸ�
int k_map_height_expan;

//����ȫ�ֱ���
int k_ite_num;
int k_son_node_num;


void Line_bresenham(pair<int, int> point1, pair<int, int> point2, int map_width, vector<int>& line_bresenham_vector) {
    int x1 = point1.second;    //�߶�
    int y1 = point1.first;  //���
    int x2 = point2.second;
    int y2 = point2.first;


    int dx = abs(x2 - x1);

    int dy = abs(y2 - y1);

    bool is_great_than_45_degree = false;

    if (dx <= dy)
    {
        // ����45��
        // Y �����ϱ仯���ʿ��� X �����ϱ仯���ʣ�ѡ���� Y �����ϵ�������ÿ�ε����м��� X ���ϱ仯��
        is_great_than_45_degree = true;
    }

    int fx = (x2 - x1) > 0 ? 1 : -1;
    int fy = (y2 - y1) > 0 ? 1 : -1;

    if (dy == 0)//0��
        fy = 0;
    if (dx == 0)//90��
        fx = 0;

    int ix = x1;
    int iy = y1;
    int p1 = 2 * dy - dx; //С�ڵ���45��
    int p2 = 2 * dx - dy; //����45��

    if (is_great_than_45_degree) {


        while (iy != y2) {

            line_bresenham_vector.push_back(ix * map_width + iy);

            if (p2 > 0) {
                p2 += 2 * (dx - dy);
                ix += fx;
            }
            else {
                p2 += 2 * dx;
            }
            iy += fy;

            line_bresenham_vector.push_back(ix * map_width + iy);

        }
    }

    else {
        while (ix != x2) {

            line_bresenham_vector.push_back(ix * map_width + iy);

            if (p1 > 0) {
                p1 += 2 * (dy - dx);
                iy += fy;
            }
            else {
                p1 += 2 * dy;
            }
            ix += fx;

            line_bresenham_vector.push_back(ix * map_width + iy);

        }
    }
    return;

}





int main() {

    bool exp1 = 0;
    bool exp2 = 0;
    //bool exp3 = false;
    bool exp3 = 1;





    int map_height;
    int map_width;

    string temp_str1;
    string temp_str2;
    string temp_str3;
    string temp_str4;


    //��� (x,y) ���Ͻ�Ϊ(0,0) x ����   y ����
    pair<int, int> point_start_A;
    //�յ�
    pair<int, int> point_target_A;


    pair<int, int> point_start_DGR;
    //�յ�
    pair<int, int> point_target_DGR;



    int corner_num_A;
    int corner_num_D;
    int corner_num_jump;




    int ite_num_A;
    int ite_num_D;
    int ite_num_J;


    int son_node_num_A;
    int son_node_num_D;
    int son_node_num_J;



    vector<pair<int, int>> path_A_1;
    vector<pair<int, int>> path_D_1;
    vector<pair<int, int>> path_jump_1;

    vector<pair<int, int>> path_zero;

    vector<double> map_1_source_A;

    vector<double> map_1_source_A_empty;


    int ite_threshold = 2000000000;

    DGR_Star dgr_star(ite_threshold);
    Edge_Node* initEdgeNode;
    vector<int> map_1_source_DGR;
    vector<int> map_1_source_DGR_empty;

    //���й���������ʹ��ʱ��Ҫ�ÿգ����ر��ǰ���  push  ������

    //��ȡ�û����� ���ж�Ӧʵ��


    if (exp1) {

        bool is_cout_png = 0;

        vector<pair<int, int>> path_A_all;
        vector<pair<int, int>> path_D_all;
        vector<pair<int, int>> path_jump_all;




        // ��txt�ĵ���д��·���ؼ�������
        const char* dataPath_out_path = "C:\\Users\\duanlian\\Desktop\\DGRʵ��\\Experment_1\\Path\\Path_Key_Point.txt";


        ofstream dataFile;
        //dataFile.open(dataPath_out, ofstream::app);  ׷��ģʽ

        dataFile.open(dataPath_out_path);
        fstream file("dataPath_out", ios::out);
        dataFile << "ע�������ʽΪ(x,y), ԭ��λ�ڵ�ͼ���Ϸ���һ�����ص㣬x������Ϊ����y������Ϊ��" << endl;
        dataFile << "���У�" << endl;
        dataFile << "\tA*�㷨 ������ϢΪ ��ǰ�㵽Ŀ���������پ���" << endl;
        dataFile << "\tDijkstra�㷨 ��������Ϣ" << endl;
        dataFile << "\tJPS�㷨 ������ϢΪ ��ǰ�㵽Ŀ����ŷ������" << endl;
        dataFile << "\tDGR*�㷨�� ���о��� ������ ŷ������" << endl;
        dataFile << endl;
        dataFile << endl;




        for (int i = 32; i < 1025; i *= 2) {
            //std::cout << endl;
            //std::cout << endl;

            //���ص�ͼ
            string temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/DataSet/IRM/map_";
            string temp_str2 = std::to_string(i);
            string temp_str3 = ".png";
            string temp_str = temp_str1 + temp_str2 + temp_str3;

            const cv::String dataPath_in = temp_str.c_str();

            Mat img = imread(dataPath_in, IMREAD_GRAYSCALE);

            //std::cout << "��" << i  << "����ͼ��ȡ�ɹ����������" << endl;

            // ��ͼ��ʵ�ʿ�
            map_width = img.cols;
            // ��ͼ��ʵ�ʸ�
            map_height = img.rows;
            // mat תvector
            //map_1_sourceΪ��ͼ��һά������ʽ
            vector<int> map_1_source = img.reshape(1, 1);




            //��� (y,x) x ���� y����
            point_start_DGR = pair<int, int>(map_height + 1, 2);
            //�յ�
            point_target_DGR = pair<int, int>(2, map_width + 1);



            //��� (x,y) x ���� y����
            point_start_A = pair<int, int>(0, map_height - 1);
            //�յ�
            point_target_A = pair<int, int>(map_width - 1, 0);



            //��ͼ����
            vector<double> map_1_binary_A;

            for (int i = 0; i < map_1_source.size(); i++) {
                if (map_1_source[i] <= 100) {
                    map_1_binary_A.push_back(-1);
                }
                else {
                    map_1_binary_A.push_back(1);

                }

                //����Լ�ָ�� ��ֹ�㣬���£�
                //if (map_1_source[i ] == 142) {
                //    point_target.first = i / map_width;
                //    point_target.second = i % map_width;
                //}
                //if (map_1_source[i] == 138) {
                //    point_start.first = i / map_width;
                //    point_start.second = i % map_width;
                //}

            }




            //std::cout << "�ɹ���ȡ�����:" << "x=" << point_start_A.first << ",y=" << point_start_A.second << endl;
            //std::cout << "�ɹ���ȡ���յ�:" << "x=" << point_target_A.first << ",y=" << point_target_A.second << endl;


            //ÿ����ͼ ����ѭ������20��
            for (int j = 1; j < 21; j++) {
                //const char* dataPath_out_EI = "C:\\Users\\duanlian\\Desktop\\DGRʵ��\\Experment_1\\Evaluation_Indicator.txt";

                std::cout << "��" << i << "����ͼ����" << j << "�����п�ʼ" << endl;

                string temp_str11 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_1/Evaluation_Indicator/Evaluation_Indicator_";
                string temp_str22 = std::to_string(j);
                string temp_str33 = ".txt";
                string temp_str44 = temp_str11 + temp_str22 + temp_str33;

                string dataPath_out_EI = temp_str44.c_str();



                ofstream dataFile_EI;

                dataFile_EI.open(dataPath_out_EI, ofstream::app);
                fstream file_EI("dataPath_out_EI", ios::out);

                if (i == 32) {

                    dataFile_EI << "ע�������ʽΪ(x,y), ԭ��λ�ڵ�ͼ���Ϸ���һ�����ص㣬x������Ϊ����y������Ϊ��" << endl;
                    dataFile_EI << "���У�" << endl;
                    dataFile_EI << "\tA*�㷨 ������ϢΪ ��ǰ�㵽Ŀ���������پ���" << endl;
                    dataFile_EI << "\tDijkstra�㷨 ��������Ϣ" << endl;
                    dataFile_EI << "\tJPS�㷨 ������ϢΪ ��ǰ�㵽Ŀ����ŷ������" << endl;
                    dataFile_EI << "\tDGR*�㷨�� ���о��� ������ ŷ������" << endl;
                    dataFile_EI << endl;
                    dataFile_EI << endl;

                    dataFile_EI << "��" << j << "�����е�ʵ����" << endl;
                }



                // A* ---------------------------------------------------------------------------------------------------------------
                //std::cout << "A*�滮��ʼ" << endl;

                int A_run_time;
                std::chrono::system_clock::time_point A_start_time = std::chrono::system_clock::now();

                //һ���ȸ���ȣ��ٸ��߶Ȳ�����ȷʶ��һά��ͼ
                fudge::GridMap<double> map_A(map_width, map_height, map_1_binary_A);

                //A*���ĺ���   û��д��   manhattan_distance 
                const std::vector<std::pair<int, int>> path_A = fudge::astar_search(
                    map_A, fudge::Coord(point_start_A.first, point_start_A.second), fudge::Coord(point_target_A.first, point_target_A.second),
                    fudge::GridMap<double>::manhattan_distance);

                std::chrono::system_clock::time_point A_end_time = std::chrono::system_clock::now();
                //std::cout << "A*�滮����" << endl;


                //std::cout << "A*��ʼͳ��ʵ������" << endl;

                A_run_time = std::chrono::duration_cast<std::chrono::microseconds>(A_end_time - A_start_time).count();


                if (j == 1) {
                    //��������
                    ite_num_A = k_ite_num;
                    //�ӽڵ���
                    son_node_num_A = k_son_node_num;

                    //����ת�����----------------

                    path_A_all = path_A;
                    path_A_all.push_back(point_start_A);
                    path_A_1 = path_zero;
                    path_A_1.push_back(*path_A_all.begin());
                    for (auto it = (path_A_all.begin() + 1); it != path_A_all.end() - 1; it++) {

                        auto it_front = it - 1;
                        auto it_back = it + 1;

                        if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                            != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                            path_A_1.push_back(*it);
                        }
                    }
                    path_A_1.push_back(*(path_A_all.end() - 1));


                    //path_A_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ�� Ϊ���
                    //ת�����Ϊ path_A_1.size() - 2


                                    //������ײ����
                    corner_num_A = 0;
                    pair<int, int> point1_A_path;
                    pair<int, int> point2_A_path;
                    pair<int, int> point3_map;
                    for (int i = 0; i < map_1_binary_A.size(); i++) {
                        if (map_1_binary_A[i] == -1) {
                            point3_map.first = i % map_width;
                            point3_map.second = i / map_width;

                            for (auto it = path_A_1.begin(); it != path_A_1.end() - 1; it++) {
                                point1_A_path = *it;
                                point2_A_path = *(it + 1);

                                if (CalculatePointToLineDistance(point1_A_path, point2_A_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                    //std::cout << "���µ���·���ཻ" << endl;
                                    //std::cout << point1_A_path.first << "," << point1_A_path.second << endl;
                                    //std::cout << point2_A_path.first << "," << point2_A_path.second << endl;
                                    //std::cout << point3_map.first <<"," << point3_map.second << endl;
                                    //std::cout << "=============" << endl;
                                    corner_num_A++;
                                    //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                }

                            }


                        }
                    }
                }








                //Dijkstra ----------------

                int D_run_time;
                //std::cout << "Dijkstra�滮��ʼ" << endl;
                std::chrono::system_clock::time_point D_start_time = std::chrono::system_clock::now();

                fudge::GridMap<double> map_D(map_width, map_height, map_1_binary_A);



                //���ĺ���   û��д��   Dijkstra_distance = 0 
                const std::vector<std::pair<int, int>> path_D = fudge::astar_search(
                    map_D, fudge::Coord(point_start_A.first, point_start_A.second), fudge::Coord(point_target_A.first, point_target_A.second),
                    fudge::GridMap<double>::Dijkstra_distance);

                std::chrono::system_clock::time_point D_end_time = std::chrono::system_clock::now();
                //std::cout << "Dijkstra�滮����" << endl;

                //std::cout << "  Dijkstra��ʼͳ��ʵ������" << endl;
                D_run_time = std::chrono::duration_cast<std::chrono::microseconds>(D_end_time - D_start_time).count();



                if (j == 1) {
                    //��������
                    ite_num_D = k_ite_num;
                    //�ӽڵ���
                    son_node_num_D = k_son_node_num;


                    //���е�ԭʼ path ��ȱ�����

                    //����ת�����----------------
                    path_D_all = path_D;
                    path_D_all.push_back(point_start_A);
                    path_D_1 = path_zero;
                    path_D_1.push_back(*path_D.begin());

                    for (auto it = (path_D_all.begin() + 1); it != path_D_all.end() - 1; it++) {

                        auto it_front = it - 1;
                        auto it_back = it + 1;

                        if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                            != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                            path_D_1.push_back(*it);
                        }
                    }
                    //path_D_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ��
                    path_D_1.push_back(*(path_D_all.end() - 1));

                    //ת�����Ϊ path_jump_1.size() - 2


                                    //������ײ����----------------

                    corner_num_D = 0;
                    pair<int, int> point1_D_path;
                    pair<int, int> point2_D_path;
                    pair<int, int> point3_map;

                    for (int i = 0; i < map_1_binary_A.size(); i++) {
                        if (map_1_binary_A[i] == -1) {
                            point3_map.first = i % map_width;
                            point3_map.second = i / map_width;

                            for (auto it = path_D_1.begin(); it != path_D_1.end() - 1; it++) {
                                point1_D_path = *it;
                                point2_D_path = *(it + 1);

                                if (CalculatePointToLineDistance(point1_D_path, point2_D_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                    //std::cout << "���µ���·���ཻ" << endl;
                                    //std::cout << point1_A_path.first << "," << point1_A_path.second << endl;
                                    //std::cout << point2_A_path.first << "," << point2_A_path.second << endl;
                                    //std::cout << point3_map.first <<"," << point3_map.second << endl;
                                    //std::cout << "=============" << endl;
                                    corner_num_D++;
                                    //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                }

                            }


                        }
                    }

                }







                // jumpA* ----------------
                fudge::JumpPointMap<double> map_jump(map_width, map_height, map_1_binary_A);
                fudge::Coord start(point_start_A.first, point_start_A.second);
                fudge::Coord goal(point_target_A.first, point_target_A.second);
                map_jump.goal_ = goal; // This is needed for checking Jump Point.
                int jump_run_time;

                //std::cout << "jump_A*�滮��ʼ" << endl;

                std::chrono::system_clock::time_point jump_start_time = std::chrono::system_clock::now();
                //jump A* ���ĺ���
                const std::vector<fudge::Coord> path_jump = fudge::astar_search(
                    map_jump, start, goal,
                    fudge::GridMap<double>::euclidean_distance);

                std::chrono::system_clock::time_point jump_end_time = std::chrono::system_clock::now();

                //std::cout << "jump_A*�滮����" << endl;





                //std::cout << "jump_A*��ʼͳ��ʵ������" << endl;
                jump_run_time = std::chrono::duration_cast<std::chrono::microseconds>(jump_end_time - jump_start_time).count();


                if (j == 1) {
                    //��������
                    ite_num_J = k_ite_num;
                    //�ӽڵ���
                    son_node_num_J = k_son_node_num;

                    //ͳ��ת�����

                    path_jump_all = path_jump;
                    path_jump_all.push_back(point_start_A);
                    path_jump_1 = path_zero;
                    path_jump_1.push_back(*path_jump.begin());

                    for (auto it = (path_jump_all.begin() + 1); it != path_jump_all.end() - 1; it++) {

                        auto it_front = it - 1;
                        auto it_back = it + 1;

                        if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                            != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                            path_jump_1.push_back(*it);

                        }
                    }
                    //path_jump_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ�� Ϊ�����һ����  ȱ���
                    path_jump_1.push_back(*(path_jump_all.end() - 1));

                    //ת�����Ϊ path_jump_1.size() - 2
                                    //ͳ����ײ����
                    corner_num_jump = 0;
                    pair<int, int> point1_jump_path;
                    pair<int, int> point2_jump_path;
                    pair<int, int> point3_map;

                    for (int i = 0; i < map_1_binary_A.size(); i++) {
                        if (map_1_binary_A[i] == -1) {
                            point3_map.first = i % map_width;
                            point3_map.second = i / map_width;

                            for (auto it = path_jump_1.begin(); it != path_jump_1.end() - 1; it++) {
                                point1_jump_path = *it;
                                point2_jump_path = *(it + 1);

                                if (CalculatePointToLineDistance(point1_jump_path, point2_jump_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                    //std::cout << "���µ���·���ཻ" << endl;
                                    //std::cout << point1_jump_path.first << "," << point1_jump_path.second << endl;
                                    //std::cout << point2_jump_path.first << "," << point2_jump_path.second << endl;
                                    //std::cout << point3_map.first << "," << point3_map.second << endl;
                                    //std::cout << "=============" << endl;
                                    corner_num_jump++;
                                    //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                }

                            }


                        }
                    }

                }






                //ֻ�� j = 1 ʱ������
                if (j == 1) {

                    //std::cout << "���ڵ�����" << i << "����ͼ��·���滮���" << endl;
                    dataFile << endl;
                    dataFile << "-----------------------------  map_" << i << " ·���滮���------------------------------" << endl;
                    dataFile << endl;
                    dataFile << "---A*�㷨·���滮���:" << endl;

                    dataFile << "��ʼ��Ϊ��(" << point_start_A.first << "," << point_start_A.second << ")" << endl;
                    dataFile << "Ŀ���Ϊ��(" << point_target_A.first << "," << point_target_A.second << ")" << endl;
                    dataFile << "һ����" << path_A_1.size() << "�����Ƶ㣬�����λ���£�" << endl;
                    for (auto it = path_A_1.end() - 1; it != path_A_1.begin(); it--) {

                        dataFile << "(" << it->first << "," << it->second << ")->";

                    }

                    //�յ������� 
                    dataFile << "(" << path_A_1.begin()->first << "," << path_A_1.begin()->second << ")" << endl;;
                    dataFile << endl;
                    dataFile << endl;


                    dataFile << "---Dijkstra�㷨·���滮���:" << endl;

                    dataFile << "��ʼ��Ϊ��(" << point_start_A.first << "," << point_start_A.second << ")" << endl;
                    dataFile << "Ŀ���Ϊ��(" << point_target_A.first << "," << point_target_A.second << ")" << endl;
                    dataFile << "һ����" << path_D_1.size() << "�����Ƶ㣬�����λ���£�" << endl;

                    for (auto it = path_D_1.end() - 1; it != path_D_1.begin(); it--) {

                        dataFile << "(" << it->first << "," << it->second << ")->";

                    }
                    dataFile << "(" << path_D_1.begin()->first << "," << path_D_1.begin()->second << ")" << endl;;
                    dataFile << endl;
                    dataFile << endl;


                    dataFile << "---JPS�㷨·���滮���:" << endl;
                    dataFile << "��ʼ��Ϊ��(" << point_start_A.first << "," << point_start_A.second << ")" << endl;
                    dataFile << "Ŀ���Ϊ��(" << point_target_A.first << "," << point_target_A.second << ")" << endl;
                    dataFile << "һ����" << path_jump_1.size() << "�����Ƶ㣬�����λ���£�" << endl;
                    for (auto it = path_jump_1.end() - 1; it != path_jump_1.begin(); it--) {

                        dataFile << "(" << it->first << "," << it->second << ")->";

                    }
                    dataFile << "(" << path_jump_1.begin()->first << "," << path_jump_1.begin()->second << ")" << endl;












                }

                //DGR*------------------------------------------------------------------------------------------------------

                  //��ͼ�������ܸ���չ2
                k_map_width_expan = map_width + 4;
                // ��ͼ�ĸ�
                k_map_height_expan = map_height + 4;


                //�����Ƶ�ͼ�ӿ�2 map_1_binary_expan ��СΪ (map_width+4)*(map_height+4) ��ʼֵ��Ϊ0
                vector<int> map_1_binary_expan(k_map_width_expan * k_map_height_expan, 0);

                //��һ���ǵ�Ļ���
                const int g_base_num = 2 * k_map_width_expan + 2;
                vector<int> map_1_binary_path;

                float one_num = 0;
                for (int i = 0; i < map_1_source.size(); i++) {



                    //�ϰ�����Ϣ��ȡ
                    if (map_1_source[i] <= 100) {
                        map_1_binary_expan[g_base_num + i + (i / map_width) * 4] = 1;
                        one_num++;
                        map_1_binary_path.push_back(1);

                    }
                    else {
                        map_1_binary_expan[g_base_num + i + (i / map_width) * 4] = 0;
                        map_1_binary_path.push_back(0);
                    }
                }

                //std::cout << "�ɹ���ȡ�����:" << "x=" << point_start_A.first << ",y=" << point_target_A.second << endl;
                //std::cout << "�ɹ���ȡ���յ�:" << "x=" << point_start_A.first << ",y=" << point_target_A.second << endl;



                //Ϊ��ַ ָ��
                Edge_Node* initEdgeNode = new Edge_Node(point_start_DGR, point_target_DGR);

                int ite_threshold = 100000;

                //------------------------���ý���-----------------------------------


                //------------------------��ʼ���-----------------------------------

                DGR_Star DGR_Star(ite_threshold);

                DGR_Star.Init_map(map_1_binary_expan, k_map_width_expan, k_map_height_expan);
                //DGR_Star.Cout_map(map_1_binary_expan);



                //std::cout << "DGR_Star�㷨�������" << endl;


                //std::cout << "-----------" << endl;

                DGR_Star.Core_function(initEdgeNode);

                //------------------------������-----------------------------------

                vector<Edge_Node*>& path_vector = DGR_Star.path_vector;
                //std::cout << "DGR_Star�滮����" << endl;

                //std::cout << "DGR_Star�㷨����ͳ�Ƽ�����" << endl;

                if (path_vector.empty()) {
                    std::cout << "DGR_Star�㷨���ʧ��" << endl;
                }
                else {

                    if (i == 1) {
                        dataFile << endl;
                        dataFile << endl;

                        dataFile << "---DGR*�㷨·���滮���:" << endl;

                        dataFile << "��ʼ��Ϊ��(" << point_start_DGR.second - 2 << "," << point_start_DGR.first - 2 << ")" << endl;
                        dataFile << "Ŀ���Ϊ��(" << point_target_DGR.second - 2 << "," << point_target_DGR.first - 2 << ")" << endl;

                        dataFile << "һ����" << path_vector.size() + 1 << "�����Ƶ㣬�����λ���£�" << endl;

                        vector<Edge_Node*> path_for_order = path_vector;
                        pair<int, int> lsat_point = point_start_DGR;

                        while (!path_for_order.empty()) {
                            for (std::vector<Edge_Node*>::iterator it = path_for_order.begin(); it != path_for_order.end(); it++) {
                                if ((*it)->point1.first == lsat_point.first and (*it)->point1.second == lsat_point.second) {

                                    dataFile << "(" << lsat_point.second - 2 << "," << lsat_point.first - 2 << ")->";
                                    lsat_point = (*it)->point2;
                                    path_for_order.erase(it);
                                    break;
                                }
                                else if ((*it)->point2.first == lsat_point.first and (*it)->point2.second == lsat_point.second) {
                                    dataFile << "(" << lsat_point.second - 2 << "," << lsat_point.first - 2 << ")->";
                                    lsat_point = (*it)->point1;
                                    path_for_order.erase(it);
                                    break;
                                }
                            }
                        }
                        // point2 Ϊ�յ�
                        dataFile << "(" << point_target_DGR.second - 2 << "," << point_target_DGR.first - 2 << ")" << endl;

                        dataFile << endl;

                    }







                }


                //�����������

                int corner_num_DGR = DGR_Star.Cal_corner_num();


                //std::cout << "���ڵ�����" << i << "����ͼ��·���滮���" << endl;




                {


                    dataFile_EI << endl;
                    dataFile_EI << j << endl;


                    dataFile_EI << ite_num_A << endl;
                    dataFile_EI << ite_num_D << endl;
                    dataFile_EI << ite_num_J << endl;
                    dataFile_EI << DGR_Star.ite_num << endl;


                    dataFile_EI << son_node_num_A << endl;
                    dataFile_EI << son_node_num_D << endl;
                    dataFile_EI << son_node_num_J << endl;
                    dataFile_EI << DGR_Star.son_node_num << endl;

                    dataFile_EI << A_run_time << endl;
                    dataFile_EI << D_run_time << endl;
                    dataFile_EI << jump_run_time << endl;
                    dataFile_EI << DGR_Star.DGR_run_time << endl;

                    dataFile_EI << endl;
                    dataFile_EI << endl;

                    dataFile_EI << path_A_1.size() - 2 << std::endl;
                    dataFile_EI << path_D_1.size() - 2 << std::endl;
                    dataFile_EI << path_jump_1.size() - 2 << std::endl;
                    dataFile_EI << path_vector.size() - 1 << std::endl;

                    dataFile_EI << corner_num_A << endl;
                    dataFile_EI << corner_num_D << endl;
                    dataFile_EI << corner_num_jump << endl;
                    dataFile_EI << corner_num_DGR << endl;



                    dataFile_EI << map_A.node(path_A[0])->f_ << endl;
                    dataFile_EI << map_D.node(path_D[0])->f_ << endl;
                    dataFile_EI << map_jump.node(path_jump[0])->f_ << endl;
                    dataFile_EI << path_vector[0]->sum_cost_f << endl;
                    dataFile_EI << endl;

                    if (i == 1024) {
                        dataFile_EI << endl;
                        dataFile_EI << endl;
                        dataFile_EI << "��������������" << endl;

                        dataFile_EI.close();                           // �ر��ĵ�
                    }


                }

                //�����ͼ
                if (j == 1) {
                    vector<int> path_cout_A;
                    vector<int> path_cout_D;
                    vector<int> path_cout_J;
                    path_cout_A = map_1_binary_path;
                    path_cout_D = map_1_binary_path;
                    path_cout_J = map_1_binary_path;

                    for (auto it = path_A_all.begin(); it != path_A_all.end(); it++) {

                        path_cout_A[it->first + it->second * map_width] = -1;
                    }

                    if (is_cout_png) {
                        temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_1/Path/Astar/map_";
                        temp_str2 = std::to_string(i);
                        temp_str3 = "_Astar.png";
                        temp_str4 = temp_str1 + temp_str2 + temp_str3;
                        const cv::String dataPath_out_png = temp_str4.c_str();

                        map2png(path_cout_A, i, i, dataPath_out_png);

                    }

                    for (auto it = path_D_all.begin(); it != path_D_all.end(); it++) {

                        path_cout_D[it->first + it->second * map_width] = -1;
                    }

                    if (is_cout_png) {
                        temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_1/Path/Dijkstra/map_";
                        temp_str2 = std::to_string(i);
                        temp_str3 = "_Dijkstra.png";
                        temp_str4 = temp_str1 + temp_str2 + temp_str3;
                        const cv::String dataPath_out_png = temp_str4.c_str();

                        map2png(path_cout_D, i, i, dataPath_out_png);

                    }

                    //��Ҫ��դ�� �ٸ�ֵ

                    vector<int> Line_bresenham_JPS;
                    for (auto it = path_jump_all.begin(); it != path_jump_all.end() - 1; it++) {

                        Line_bresenham(*it, *(it + 1), map_width, Line_bresenham_JPS);

                    }
                    for (auto i : Line_bresenham_JPS) {
                        path_cout_J[i] = -1;

                    }


                    if (is_cout_png) {
                        temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_1/Path/JPS/map_";
                        temp_str2 = std::to_string(i);
                        temp_str3 = "_JPS.png";
                        temp_str4 = temp_str1 + temp_str2 + temp_str3;
                        const cv::String dataPath_out_png = temp_str4.c_str();

                        map2png(path_cout_J, i, i, dataPath_out_png);

                    }



                    if (is_cout_png) {
                        temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_1/Path/DGRstar/map_";
                        temp_str2 = std::to_string(i);
                        temp_str3 = "_DGRstar.png";
                        temp_str4 = temp_str1 + temp_str2 + temp_str3;
                        const cv::String dataPath_out_png = temp_str4.c_str();

                        map2png(DGR_Star.path_DGR, i + 4, i + 4, dataPath_out_png);

                    }


                }

            }


        }





        dataFile << endl;
        dataFile << endl;
        dataFile << "��������������" << endl;
        dataFile.close();                           // �ر��ĵ�
        //std::cout << "���е�ͼ·���滮����" << endl;


    }

    if (exp2) {

        bool is_cout_png = 0;

        vector<pair<int, int>> path_A_all;
        vector<pair<int, int>> path_D_all;
        vector<pair<int, int>> path_jump_all;
        vector<int> map_1_binary_path;



        // ��txt�ĵ���д��·���ؼ�������
        const char* dataPath_out_path = "C:\\Users\\duanlian\\Desktop\\DGRʵ��\\Experment_2\\Path\\Path_Key_Point.txt";


        ofstream dataFile;
        //dataFile.open(dataPath_out, ofstream::app);  ׷��ģʽ

        dataFile.open(dataPath_out_path);
        fstream file("dataPath_out", ios::out);
        dataFile << "ע�������ʽΪ(x,y), ԭ��λ�ڵ�ͼ���Ϸ���һ�����ص㣬x������Ϊ����y������Ϊ��" << endl;
        dataFile << "���У�" << endl;
        dataFile << "\tA*�㷨 ������ϢΪ ��ǰ�㵽Ŀ���������پ���" << endl;
        dataFile << "\tDijkstra�㷨 ��������Ϣ" << endl;
        dataFile << "\tJPS�㷨 ������ϢΪ ��ǰ�㵽Ŀ����ŷ������" << endl;
        dataFile << "\tDGR*�㷨�� ���о��� ������ ŷ������" << endl;
        dataFile << endl;
        dataFile << endl;



        for (int i = 1; i < 31; i++) {
            temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/DataSet/CSMScan/scen_";
            temp_str2 = std::to_string(i);
            temp_str3 = ".scen";
            temp_str4 = temp_str1 + temp_str2 + temp_str3;

            const char* dataPath_in = temp_str4.c_str();

            ScenarioLoader my_scen_load(dataPath_in);

            //std::cout << "��ǰΪ��" << i << "����ͼ" << endl;
            Experiment cur_scen = my_scen_load.GetNthExperiment(1);

            bool load_map = true;
            if (load_map) {

                map_height = cur_scen.GetYScale();
                map_width = cur_scen.GetXScale();

                //��� (y,x) x ���� y����
                point_start_DGR = pair<int, int>(map_height + 1, 2);
                //�յ�
                point_target_DGR = pair<int, int>(2, map_width + 1);



                //��� (x,y) x ���� y����
                point_start_A = pair<int, int>(0, map_height - 1);
                //�յ�
                point_target_A = pair<int, int>(map_width - 1, 0);


                temp_str2 = cur_scen.GetMapName();

                //Berlin_1_1024.map  ��Ҫƾ��
                temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/DataSet/CSM1/";

                temp_str4 = temp_str1 + temp_str2;

                const char* mapPath_in_map = temp_str4.c_str();
                // 1 �ϰ���  0 ����

                //std::cout << mapPath_in << endl;
                //std::cout << "�ѻ�ȡԭʼ��ͼ·��  " << mapPath_in << endl;

                map_1_source_DGR = Get_map(mapPath_in_map); //��ȡ��ά����
                //std::cout << "�ѻ�ȡԭʼ��ͼ,��ͼ��СΪ" << map_1_source_DGR.size() << endl;
                //std::cout << "-------------------------------" << endl;
                //std::cout << "�ѻ�ȡ��ͼ����" << temp_str2 << endl;

                // 1 �ϰ���  0 ����
                //�ÿ�
                map_1_binary_path = map_1_source_DGR;
                map_1_source_A = map_1_source_A_empty;
                for (auto obi : map_1_source_DGR) {
                    if (obi == 0) {
                        map_1_source_A.push_back(1);
                    }
                    else {
                        map_1_source_A.push_back(-1);

                    }
                }



                // ��һ�� ��ͼԤ����
                k_map_width_expan = map_width + 4;
                // ��ͼ�ĸ�
                k_map_height_expan = map_height + 4;

                vector<int> map_1_binary_expan(k_map_width_expan * k_map_height_expan, 0);

                const int g_base_num = 2 * k_map_width_expan + 2;

                for (int i = 0; i < map_1_source_DGR.size(); i++) {

                    map_1_binary_expan[g_base_num + i + (i / map_width) * 4] = map_1_source_DGR[i];
                }

                //int obi = 0;
                //for (int i = 0; i < map_1_binary_expan.size(); i++) {
                //    if (map_1_binary_expan[i] == 1) {
                //        obi++;
                //    }
                //}

                //std::cout << "��ͼ����չ��������ô��ռ��դ�� " << obi << endl;

                dgr_star.Init_map(map_1_binary_expan, k_map_width_expan, k_map_height_expan);

            }
            load_map = false;





            //std::cout << "�ɹ���ȡ�����:" << "x=" << point_start_A.first << ",y=" << point_start_A.second << endl;
            //std::cout << "�ɹ���ȡ���յ�:" << "x=" << point_target_A.first << ",y=" << point_target_A.second << endl;




            //ÿ����ͼ ����ѭ������20��
            for (int j = 1; j < 21; j++) {
                //const char* dataPath_out_EI = "C:\\Users\\duanlian\\Desktop\\DGRʵ��\\Experment_1\\Evaluation_Indicator.txt";

                cout << "��" << i << "����ͼ����" << j << "�����п�ʼ" << endl;

                string temp_str11 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_2/Evaluation_Indicator/Evaluation_Indicator_";
                string temp_str22 = std::to_string(j);
                string temp_str33 = ".txt";
                string temp_str44 = temp_str11 + temp_str22 + temp_str33;

                string dataPath_out_EI = temp_str44.c_str();



                ofstream dataFile_EI;

                dataFile_EI.open(dataPath_out_EI, ofstream::app);
                fstream file_EI("dataPath_out_EI", ios::out);

                if (i == 1) {

                    dataFile_EI << "ע�������ʽΪ(x,y), ԭ��λ�ڵ�ͼ���Ϸ���һ�����ص㣬x������Ϊ����y������Ϊ��" << endl;
                    dataFile_EI << "���У�" << endl;
                    dataFile_EI << "\tA*�㷨 ������ϢΪ ��ǰ�㵽Ŀ���������پ���" << endl;
                    dataFile_EI << "\tDijkstra�㷨 ��������Ϣ" << endl;
                    dataFile_EI << "\tJPS�㷨 ������ϢΪ ��ǰ�㵽Ŀ����ŷ������" << endl;
                    dataFile_EI << "\tDGR*�㷨�� ���о��� ������ ŷ������" << endl;
                    dataFile_EI << endl;
                    dataFile_EI << endl;
                    dataFile_EI << "��" << j << "�����е�ʵ����" << endl;
                }




                // A* ---------------------------------------------------------------------------------------------------------------
                //std::cout << "A*�滮��ʼ" << endl;

                int A_run_time;
                std::chrono::system_clock::time_point A_start_time = std::chrono::system_clock::now();

                //һ���ȸ���ȣ��ٸ��߶Ȳ�����ȷʶ��һά��ͼ
                fudge::GridMap<double> map_A(map_width, map_height, map_1_source_A);

                //std::cout << "A*��ͼ��С" << map_1_source_A.size() << endl;

                //A*���ĺ���   û��д��   manhattan_distance 
                const std::vector<std::pair<int, int>> path_A = fudge::astar_search(
                    map_A, fudge::Coord(point_start_A.first, point_start_A.second), fudge::Coord(point_target_A.first, point_target_A.second),
                    fudge::GridMap<double>::manhattan_distance);

                std::chrono::system_clock::time_point A_end_time = std::chrono::system_clock::now();
                //std::cout << "A*�滮����" << endl;


                //std::cout << "A*��ʼͳ��ʵ������" << endl;

                A_run_time = std::chrono::duration_cast<std::chrono::microseconds>(A_end_time - A_start_time).count();


                if (j == 1) {
                    //��������
                    ite_num_A = k_ite_num;
                    //�ӽڵ���
                    son_node_num_A = k_son_node_num;

                    //����ת�����----------------

                    path_A_all = path_A;
                    path_A_all.push_back(point_start_A);

                    path_A_1 = path_zero;
                    path_A_1.push_back(*path_A_all.begin());


                    for (auto it = (path_A_all.begin() + 1); it != path_A_all.end() - 1; it++) {

                        auto it_front = it - 1;
                        auto it_back = it + 1;

                        if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                            != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                            path_A_1.push_back(*it);
                        }
                    }
                    path_A_1.push_back(*(path_A_all.end() - 1));


                    //path_A_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ�� Ϊ���
                    //ת�����Ϊ path_A_1.size() - 2


                                    //������ײ����
                    corner_num_A = 0;
                    pair<int, int> point1_A_path;
                    pair<int, int> point2_A_path;
                    pair<int, int> point3_map;
                    for (int i = 0; i < map_1_source_A.size(); i++) {
                        if (map_1_source_A[i] == -1) {
                            point3_map.first = i % map_width;
                            point3_map.second = i / map_width;

                            for (auto it = path_A_1.begin(); it != path_A_1.end() - 1; it++) {
                                point1_A_path = *it;
                                point2_A_path = *(it + 1);

                                if (CalculatePointToLineDistance(point1_A_path, point2_A_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                    //std::cout << "���µ���·���ཻ" << endl;
                                    //std::cout << point1_A_path.first << "," << point1_A_path.second << endl;
                                    //std::cout << point2_A_path.first << "," << point2_A_path.second << endl;
                                    //std::cout << point3_map.first <<"," << point3_map.second << endl;
                                    //std::cout << "=============" << endl;
                                    corner_num_A++;
                                    //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                }

                            }


                        }
                    }
                }








                //Dijkstra ----------------

                int D_run_time;
                //std::cout << "Dijkstra�滮��ʼ" << endl;
                std::chrono::system_clock::time_point D_start_time = std::chrono::system_clock::now();

                fudge::GridMap<double> map_D(map_width, map_height, map_1_source_A);



                //���ĺ���   û��д��   Dijkstra_distance = 0 
                const std::vector<std::pair<int, int>> path_D = fudge::astar_search(
                    map_D, fudge::Coord(point_start_A.first, point_start_A.second), fudge::Coord(point_target_A.first, point_target_A.second),
                    fudge::GridMap<double>::Dijkstra_distance);

                std::chrono::system_clock::time_point D_end_time = std::chrono::system_clock::now();
                //std::cout << "Dijkstra�滮����" << endl;

                //std::cout << "  Dijkstra��ʼͳ��ʵ������" << endl;
                D_run_time = std::chrono::duration_cast<std::chrono::microseconds>(D_end_time - D_start_time).count();



                if (j == 1) {
                    //��������
                    ite_num_D = k_ite_num;
                    //�ӽڵ���
                    son_node_num_D = k_son_node_num;


                    //���е�ԭʼ path ��ȱ�����

                    //����ת�����----------------
                    path_D_all = path_D;
                    path_D_all.push_back(point_start_A);
                    path_D_1 = path_zero;
                    path_D_1.push_back(*path_D.begin());

                    for (auto it = (path_D_all.begin() + 1); it != path_D_all.end() - 1; it++) {

                        auto it_front = it - 1;
                        auto it_back = it + 1;

                        if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                            != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                            path_D_1.push_back(*it);
                        }
                    }
                    //path_D_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ��
                    path_D_1.push_back(*(path_D_all.end() - 1));

                    //ת�����Ϊ path_jump_1.size() - 2


                                    //������ײ����----------------

                    corner_num_D = 0;
                    pair<int, int> point1_D_path;
                    pair<int, int> point2_D_path;
                    pair<int, int> point3_map;

                    for (int i = 0; i < map_1_source_A.size(); i++) {
                        if (map_1_source_A[i] == -1) {
                            point3_map.first = i % map_width;
                            point3_map.second = i / map_width;

                            for (auto it = path_D_1.begin(); it != path_D_1.end() - 1; it++) {
                                point1_D_path = *it;
                                point2_D_path = *(it + 1);

                                if (CalculatePointToLineDistance(point1_D_path, point2_D_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                    //std::cout << "���µ���·���ཻ" << endl;
                                    //std::cout << point1_A_path.first << "," << point1_A_path.second << endl;
                                    //std::cout << point2_A_path.first << "," << point2_A_path.second << endl;
                                    //std::cout << point3_map.first <<"," << point3_map.second << endl;
                                    //std::cout << "=============" << endl;
                                    corner_num_D++;
                                    //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                }

                            }


                        }
                    }

                }







                // jumpA* ----------------
                fudge::JumpPointMap<double> map_jump(map_width, map_height, map_1_source_A);
                fudge::Coord start(point_start_A.first, point_start_A.second);
                fudge::Coord goal(point_target_A.first, point_target_A.second);
                map_jump.goal_ = goal; // This is needed for checking Jump Point.
                int jump_run_time;

                //std::cout << "jump_A*�滮��ʼ" << endl;

                std::chrono::system_clock::time_point jump_start_time = std::chrono::system_clock::now();
                //jump A* ���ĺ���
                const std::vector<fudge::Coord> path_jump = fudge::astar_search(
                    map_jump, start, goal,
                    fudge::GridMap<double>::euclidean_distance);

                std::chrono::system_clock::time_point jump_end_time = std::chrono::system_clock::now();

                //std::cout << "jump_A*�滮����" << endl;





                //std::cout << "jump_A*��ʼͳ��ʵ������" << endl;
                jump_run_time = std::chrono::duration_cast<std::chrono::microseconds>(jump_end_time - jump_start_time).count();


                if (j == 1) {
                    //��������
                    ite_num_J = k_ite_num;
                    //�ӽڵ���
                    son_node_num_J = k_son_node_num;

                    //ͳ��ת�����

                    path_jump_all = path_jump;
                    path_jump_all.push_back(point_start_A);
                    path_jump_1 = path_zero;
                    path_jump_1.push_back(*path_jump.begin());

                    for (auto it = (path_jump_all.begin() + 1); it != path_jump_all.end() - 1; it++) {

                        auto it_front = it - 1;
                        auto it_back = it + 1;

                        if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                            != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                            path_jump_1.push_back(*it);

                        }
                    }
                    //path_jump_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ�� Ϊ�����һ����  ȱ���
                    path_jump_1.push_back(*(path_jump_all.end() - 1));

                    //ת�����Ϊ path_jump_1.size() - 2
                                    //ͳ����ײ����
                    corner_num_jump = 0;
                    pair<int, int> point1_jump_path;
                    pair<int, int> point2_jump_path;
                    pair<int, int> point3_map;

                    for (int i = 0; i < map_1_source_A.size(); i++) {
                        if (map_1_source_A[i] == -1) {
                            point3_map.first = i % map_width;
                            point3_map.second = i / map_width;

                            for (auto it = path_jump_1.begin(); it != path_jump_1.end() - 1; it++) {
                                point1_jump_path = *it;
                                point2_jump_path = *(it + 1);

                                if (CalculatePointToLineDistance(point1_jump_path, point2_jump_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                    //std::cout << "���µ���·���ཻ" << endl;
                                    //std::cout << point1_jump_path.first << "," << point1_jump_path.second << endl;
                                    //std::cout << point2_jump_path.first << "," << point2_jump_path.second << endl;
                                    //std::cout << point3_map.first << "," << point3_map.second << endl;
                                    //std::cout << "=============" << endl;
                                    corner_num_jump++;
                                    //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                }

                            }


                        }
                    }

                }






                //ֻ�� j = 1 ʱ������
                if (j == 1) {

                    //std::cout << "���ڵ�����" << i << "����ͼ��·���滮���" << endl;
                    dataFile << endl;
                    dataFile << "--------------------------" << temp_str2 << " ·���滮���------------------------------" << endl;
                    dataFile << endl;
                    dataFile << "---A*�㷨·���滮���:" << endl;

                    dataFile << "��ʼ��Ϊ��(" << point_start_A.first << "," << point_start_A.second << ")" << endl;
                    dataFile << "Ŀ���Ϊ��(" << point_target_A.first << "," << point_target_A.second << ")" << endl;
                    dataFile << "һ����" << path_A_1.size() << "�����Ƶ㣬�����λ���£�" << endl;
                    for (auto it = path_A_1.end() - 1; it != path_A_1.begin(); it--) {

                        dataFile << "(" << it->first << "," << it->second << ")->";

                    }

                    //�յ������� 
                    dataFile << "(" << path_A_1.begin()->first << "," << path_A_1.begin()->second << ")" << endl;;
                    dataFile << endl;
                    dataFile << endl;


                    dataFile << "---Dijkstra�㷨·���滮���:" << endl;

                    dataFile << "��ʼ��Ϊ��(" << point_start_A.first << "," << point_start_A.second << ")" << endl;
                    dataFile << "Ŀ���Ϊ��(" << point_target_A.first << "," << point_target_A.second << ")" << endl;
                    dataFile << "һ����" << path_D_1.size() << "�����Ƶ㣬�����λ���£�" << endl;

                    for (auto it = path_D_1.end() - 1; it != path_D_1.begin(); it--) {

                        dataFile << "(" << it->first << "," << it->second << ")->";

                    }
                    dataFile << "(" << path_D_1.begin()->first << "," << path_D_1.begin()->second << ")" << endl;;
                    dataFile << endl;
                    dataFile << endl;


                    dataFile << "---JPS�㷨·���滮���:" << endl;
                    dataFile << "��ʼ��Ϊ��(" << point_start_A.first << "," << point_start_A.second << ")" << endl;
                    dataFile << "Ŀ���Ϊ��(" << point_target_A.first << "," << point_target_A.second << ")" << endl;
                    dataFile << "һ����" << path_jump_1.size() << "�����Ƶ㣬�����λ���£�" << endl;
                    for (auto it = path_jump_1.end() - 1; it != path_jump_1.begin(); it--) {

                        dataFile << "(" << it->first << "," << it->second << ")->";

                    }
                    dataFile << "(" << path_jump_1.begin()->first << "," << path_jump_1.begin()->second << ")" << endl;












                }

                //DGR*------------------------------------------------------------------------------------------------------


                //Ϊ��ַ ָ��
                initEdgeNode = new Edge_Node(point_start_DGR, point_target_DGR);


                //------------------------���ý���-----------------------------------


                //------------------------��ʼ���-----------------------------------

                //DGR_Star.Cout_map(map_1_binary_expan);



                //std::cout << "DGR_Star�㷨�������" << endl;


                //std::cout << "-----------" << endl;

                dgr_star.Core_function(initEdgeNode);

                //------------------------������-----------------------------------

                vector<Edge_Node*>& path_vector = dgr_star.path_vector;
                //std::cout << "DGR_Star�滮����" << endl;

                //std::cout << "DGR_Star�㷨����ͳ�Ƽ�����" << endl;

                if (path_vector.empty()) {
                    std::cout << "DGR_Star�㷨���ʧ��" << endl;
                }
                else {

                    if (i == 1) {
                        dataFile << endl;
                        dataFile << endl;

                        dataFile << "---DGR*�㷨·���滮���:" << endl;

                        dataFile << "��ʼ��Ϊ��(" << point_start_DGR.second - 2 << "," << point_start_DGR.first - 2 << ")" << endl;
                        dataFile << "Ŀ���Ϊ��(" << point_target_DGR.second - 2 << "," << point_target_DGR.first - 2 << ")" << endl;

                        dataFile << "һ����" << path_vector.size() + 1 << "�����Ƶ㣬�����λ���£�" << endl;

                        vector<Edge_Node*> path_for_order = path_vector;
                        pair<int, int> lsat_point = point_start_DGR;

                        while (!path_for_order.empty()) {
                            for (std::vector<Edge_Node*>::iterator it = path_for_order.begin(); it != path_for_order.end(); it++) {
                                if ((*it)->point1.first == lsat_point.first and (*it)->point1.second == lsat_point.second) {

                                    dataFile << "(" << lsat_point.second - 2 << "," << lsat_point.first - 2 << ")->";
                                    lsat_point = (*it)->point2;
                                    path_for_order.erase(it);
                                    break;
                                }
                                else if ((*it)->point2.first == lsat_point.first and (*it)->point2.second == lsat_point.second) {
                                    dataFile << "(" << lsat_point.second - 2 << "," << lsat_point.first - 2 << ")->";
                                    lsat_point = (*it)->point1;
                                    path_for_order.erase(it);
                                    break;
                                }
                            }
                        }
                        // point2 Ϊ�յ�
                        dataFile << "(" << point_target_DGR.second - 2 << "," << point_target_DGR.first - 2 << ")" << endl;

                        dataFile << endl;

                    }







                }


                //�����������

                int corner_num_DGR = dgr_star.Cal_corner_num();


                //std::cout << "���ڵ�����" << i << "����ͼ��·���滮���" << endl;




                {


                    dataFile_EI << endl;
                    dataFile_EI << j << endl;


                    dataFile_EI << ite_num_A << endl;
                    dataFile_EI << ite_num_D << endl;
                    dataFile_EI << ite_num_J << endl;
                    dataFile_EI << dgr_star.ite_num << endl;


                    dataFile_EI << son_node_num_A << endl;
                    dataFile_EI << son_node_num_D << endl;
                    dataFile_EI << son_node_num_J << endl;
                    dataFile_EI << dgr_star.son_node_num << endl;

                    dataFile_EI << A_run_time << endl;
                    dataFile_EI << D_run_time << endl;
                    dataFile_EI << jump_run_time << endl;
                    dataFile_EI << dgr_star.DGR_run_time << endl;

                    dataFile_EI << endl;
                    dataFile_EI << endl;

                    dataFile_EI << path_A_1.size() - 2 << std::endl;
                    dataFile_EI << path_D_1.size() - 2 << std::endl;
                    dataFile_EI << path_jump_1.size() - 2 << std::endl;
                    dataFile_EI << path_vector.size() - 1 << std::endl;

                    dataFile_EI << corner_num_A << endl;
                    dataFile_EI << corner_num_D << endl;
                    dataFile_EI << corner_num_jump << endl;
                    dataFile_EI << corner_num_DGR << endl;



                    dataFile_EI << map_A.node(path_A[0])->f_ << endl;
                    dataFile_EI << map_D.node(path_D[0])->f_ << endl;
                    dataFile_EI << map_jump.node(path_jump[0])->f_ << endl;
                    dataFile_EI << path_vector[0]->sum_cost_f << endl;
                    dataFile_EI << endl;

                    if (i == 30) {
                        dataFile_EI << endl;
                        dataFile_EI << endl;
                        dataFile_EI << "��������������" << endl;

                        dataFile_EI.close();                           // �ر��ĵ�
                    }


                }

                //�����ͼ
                if (j == 1) {
                    vector<int> path_cout_A;
                    vector<int> path_cout_D;
                    vector<int> path_cout_J;

                    path_cout_A = map_1_binary_path;
                    path_cout_D = map_1_binary_path;
                    path_cout_J = map_1_binary_path;

                    for (auto it = path_A_all.begin(); it != path_A_all.end(); it++) {

                        path_cout_A[it->first + it->second * map_width] = -1;
                    }

                    if (is_cout_png) {
                        temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_2/Path/Astar/";
                        temp_str2 = cur_scen.GetMapName();
                        temp_str3 = "_Astar.png";
                        temp_str4 = temp_str1 + temp_str2 + temp_str3;
                        const cv::String dataPath_out_png = temp_str4.c_str();

                        map2png(path_cout_A, 1024, 1024, dataPath_out_png);

                    }

                    for (auto it = path_D_all.begin(); it != path_D_all.end(); it++) {

                        path_cout_D[it->first + it->second * map_width] = -1;
                    }

                    if (is_cout_png) {
                        temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_2/Path/Dijkstra/";
                        temp_str2 = cur_scen.GetMapName();
                        temp_str3 = "_Dijkstra.png";
                        temp_str4 = temp_str1 + temp_str2 + temp_str3;
                        const cv::String dataPath_out_png = temp_str4.c_str();

                        map2png(path_cout_D, 1024, 1024, dataPath_out_png);

                    }

                    //��Ҫ��դ�� �ٸ�ֵ

                    vector<int> Line_bresenham_JPS;
                    for (auto it = path_jump_all.begin(); it != path_jump_all.end() - 1; it++) {

                        Line_bresenham(*it, *(it + 1), map_width, Line_bresenham_JPS);

                    }
                    for (auto i : Line_bresenham_JPS) {
                        path_cout_J[i] = -1;

                    }


                    if (is_cout_png) {
                        temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_2/Path/JPS/";
                        temp_str2 = cur_scen.GetMapName();
                        temp_str3 = "_JPS.png";
                        temp_str4 = temp_str1 + temp_str2 + temp_str3;
                        const cv::String dataPath_out_png = temp_str4.c_str();

                        map2png(path_cout_J, 1024, 1024, dataPath_out_png);

                    }



                    if (is_cout_png) {
                        temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_2/Path/DGRstar/";
                        temp_str2 = cur_scen.GetMapName();
                        temp_str3 = "_DGRstar.png";
                        temp_str4 = temp_str1 + temp_str2 + temp_str3;
                        const cv::String dataPath_out_png = temp_str4.c_str();

                        map2png(dgr_star.path_DGR, 1024 + 4, 1024 + 4, dataPath_out_png);

                    }


                }

            }


        }





        dataFile << endl;
        dataFile << endl;
        dataFile << "��������������" << endl;
        dataFile.close();                           // �ر��ĵ�
        //std::cout << "���е�ͼ·���滮����" << endl;


    }

    //�ѵ�  10��������   ÿ������ֻ�� һ��
    if (exp3) {

        //1����������
        //2����ʼ����ͼ
        //3���㷨���
        //4�����
        //5��������

        //1����������
        //�ļ����ϳ�
        bool Arun = 1;
        bool Drun = 1;
        bool Jrun = 1;
        bool DGRrun = 1;

        int A_run_time = 0;
        int D_run_time = 0;
        int J_run_time = 0;
        int DGR_run_time = 0;
        int DGR_map_run_time = 0;


        long long int total_runtime_A = 0;
        long long int total_runtime_D = 0;
        long long int total_runtime_J = 0;
        long long int total_runtime_DGR = 0;

        long float total_len_A = 0.f;
        long float total_len_D = 0.f;
        long float total_len_J = 0.f;
        long float total_len_DGR = 0.f;
        long float total_len_Base = 0.f;

        int total_Base = 0;


        //int avg_runtime_A = 0;
        //int avg_runtime_D = 0;
        //int avg_runtime_J = 0;
        //int avg_runtime_DGR = 0;

        long int total_runtime_DGR_map = 0;
        //int avg_runtime_DGR_map = 0;




        //float avg_len_A = 0;
        //float avg_len_D = 0;
        //float avg_len_J = 0;
        //float avg_len_DGR = 0;

        float len_A = 0.f;
        float len_D = 0.f;
        float len_J = 0.f;
        float len_DGR = 0.f;
        float len_Base = 0.f;


        //float avg_len_Base = 0;

        int opt_A = 0;
        int opt_D = 0;
        int opt_J = 0;
        int opt_DGR = 0;

        bool is_opt_A = 0;
        bool is_opt_D = 0;
        bool is_opt_J = 0;
        bool is_opt_DGR = 0;

        int solved_A = 0;
        int solved_D = 0;
        int solved_J = 0;
        int solved_DGR = 0;


        int pre_map_DGR = 0;



        int falseEx = 0;
        bool load_map;
        bool is_cal_for_A = 0;

        string cur_map_name;
        for (int map_num = 1; map_num < 31; map_num++) {

            total_runtime_A = 0;
            total_runtime_D = 0;
            total_runtime_J = 0;
            total_runtime_DGR = 0;

            total_len_A = 0;
            total_len_D = 0;
            total_len_J = 0;
            total_len_DGR = 0;

            total_len_Base = 0;

            total_Base = 0;

            total_runtime_DGR_map = 0;


            solved_A = 0;
            solved_D = 0;
            solved_J = 0;
            solved_DGR = 0;

            opt_A = 0;
            opt_D = 0;
            opt_J = 0;
            opt_DGR = 0;

            temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/DataSet/CSMScan/scen_";
            temp_str2 = std::to_string(map_num);
            temp_str3 = ".scen";

            temp_str4 = temp_str1 + temp_str2 + temp_str3;

            const char* dataPath_in = temp_str4.c_str();

            //char scenName[1024];

            ScenarioLoader my_scen_load(dataPath_in);

            //std::cout << "��������Ϊ" << my_scen_load.GetNumExperiments() << endl;
            total_Base += my_scen_load.GetNumExperiments();
            //ѭ��ÿ������
            load_map = true;






            int NumExperiments = my_scen_load.GetNumExperiments();
            NumExperiments = 20;
            for (int scen_iter = 0; scen_iter < NumExperiments; scen_iter++) {

                std::cout << "��ǰΪ��" << map_num << "����ͼ����" << scen_iter + 1 << "������" << endl;





                Experiment cur_scen = my_scen_load.GetNthExperiment(scen_iter);
                cur_map_name = cur_scen.GetMapName();

                //�������   �����ԭʼ����
                string temp_str11 = "C:/Users/duanlian/Desktop/DGRʵ��/Experment_3/";
                string temp_str22 = cur_scen.GetMapName();
                string temp_str33 = "_EI.txt";
                string temp_str44 = temp_str11 + temp_str22 + temp_str33;
                string dataPath_out_EI = temp_str44.c_str();
                ofstream dataFile_EI;
                dataFile_EI.open(dataPath_out_EI, ofstream::app);
                fstream file_EI("dataPath_out_EI", ios::out);







                //��һ�Σ����ص�ͼ
                if (load_map) {

                    map_height = cur_scen.GetYScale();
                    map_width = cur_scen.GetXScale();

                    temp_str2 = cur_scen.GetMapName();

                    //Berlin_1_1024.map  ��Ҫƾ��
                    temp_str1 = "C:/Users/duanlian/Desktop/DGRʵ��/DataSet/CSM/";

                    temp_str4 = temp_str1 + temp_str2;

                    const char* mapPath_in = temp_str4.c_str();
                    // 1 �ϰ���  0 ����

                    //std::cout << mapPath_in << endl;
                    //std::cout << "�ѻ�ȡԭʼ��ͼ·��  " << mapPath_in << endl;

                    map_1_source_DGR = Get_map(mapPath_in); //��ȡ��ά����
                    //std::cout << "�ѻ�ȡԭʼ��ͼ,��ͼ��СΪ" << map_1_source_DGR.size() << endl;
                    //std::cout << "-------------------------------" << endl;

                    // 1 �ϰ���  0 ����
                    //�ÿ�
                    map_1_source_A = map_1_source_A_empty;
                    for (auto obi : map_1_source_DGR) {
                        if (obi == 0) {
                            map_1_source_A.push_back(1);
                        }
                        else {
                            map_1_source_A.push_back(-1);

                        }
                    }



                    // ��һ�� ��ͼԤ����
                    k_map_width_expan = map_width + 4;
                    // ��ͼ�ĸ�
                    k_map_height_expan = map_height + 4;

                    vector<int> map_1_binary_expan(k_map_width_expan * k_map_height_expan, 0);

                    const int g_base_num = 2 * k_map_width_expan + 2;

                    for (int i = 0; i < map_1_source_DGR.size(); i++) {

                        map_1_binary_expan[g_base_num + i + (i / map_width) * 4] = map_1_source_DGR[i];
                    }

                    //int obi = 0;
                    //for (int i = 0; i < map_1_binary_expan.size(); i++) {
                    //    if (map_1_binary_expan[i] == 1) {
                    //        obi++;
                    //    }
                    //}

                    //std::cout << "��ͼ����չ��������ô��ռ��դ�� " << obi << endl;

                    dgr_star.Init_map(map_1_binary_expan, k_map_width_expan, k_map_height_expan);





                }

                load_map = false;


                //��ʼ��  (x,y) ���Ͻ�Ϊ(0,0) x ����   y ����
                //point_start = pair<int, int>(cur_scen.GetStartY(), cur_scen.GetStartX());
                //point_target = pair<int, int>(cur_scen.GetGoalY(), cur_scen.GetGoalX());




                point_start_DGR = pair<int, int>(cur_scen.GetStartY() + 2, cur_scen.GetStartX() + 2);
                point_target_DGR = pair<int, int>(cur_scen.GetGoalY() + 2, cur_scen.GetGoalX() + 2);

                //������Ч���ж�
                //if (dgr_star.map_1_obatacle[point_start_DGR.first * k_map_width_expan + point_start_DGR.second]
                //    or dgr_star.map_1_obatacle[point_target_DGR.first * k_map_width_expan + point_target_DGR.second]
                //    ) {
                //    length_win_Base_1++;
                //    continue;
                //}

                point_start_A = pair<int, int>(cur_scen.GetStartX(), cur_scen.GetStartY());
                point_target_A = pair<int, int>(cur_scen.GetGoalX(), cur_scen.GetGoalY());

                //std::cout << "��ǰ���Ϊ" << point_start_A.first << "," << point_start_A.second << endl;
                //std::cout << "��ǰ�յ�Ϊ" << point_target_A.first << "," << point_target_A.second << endl;



                //�������㷨���бȽ�



                //A*��ʼ��
                if (Arun) {
                    std::chrono::system_clock::time_point A_start_time = std::chrono::system_clock::now();

                    //һ���ȸ����x���ٸ��߶�y������ȷʶ��һά��ͼ  


                    fudge::GridMap<double> map_A(map_width, map_height, map_1_source_A);
                    //A*���ĺ���   û��д��   manhattan_distance 
                    const std::vector<std::pair<int, int>> path_A = fudge::astar_search(
                        map_A, fudge::Coord(point_start_A.first, point_start_A.second), fudge::Coord(point_target_A.first, point_target_A.second),
                        fudge::GridMap<double>::manhattan_distance);

                    std::chrono::system_clock::time_point A_end_time = std::chrono::system_clock::now();

                    A_run_time = std::chrono::duration_cast<std::chrono::microseconds>(A_end_time - A_start_time).count();

                    total_runtime_A += A_run_time;
                    len_Base = cur_scen.GetDistance();
                    total_len_Base += len_Base;

                    if (path_A.size() == 0) {
                        len_A = len_Base;
                        is_opt_A = 0;

                        total_len_A += len_Base;

                    }
                    else {

                        len_A = map_A.node(path_A[0])->f_;
                        total_len_A += len_A;

                        solved_A++;
                        if ((len_A - len_Base) < 1e-01) {
                            opt_A++;
                            is_opt_A = 1;
                        }
                        else {
                            is_opt_A = 0;

                        }
                    }






                    if (is_cal_for_A) {

                        //std::cout << "A*��ʼͳ��ʵ������" << endl;
                        //����ʱ��

                        //��������
                        ite_num_A = k_ite_num;
                        //�ӽڵ���
                        son_node_num_A = k_son_node_num;



                        // MIT �Դ����·����
                        //std::cout << "��ʼ���A*·����������" << endl;
                        //for (const auto& i : path_A) {
                        //    std::cout << map_A.node(i)->to_string() << std::endl;
                        //}





                      //����ת�����----------------

                        vector<pair<int, int>> path_A_all = path_A;

                        //���е��
                        path_A_all.push_back(point_start_A);

                        path_A_1 = path_zero;
                        path_A_1.push_back(*path_A_all.begin());
                        for (auto it = (path_A_all.begin() + 1); it != path_A_all.end() - 1; it++) {

                            auto it_front = it - 1;
                            auto it_back = it + 1;

                            if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                                != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                                path_A_1.push_back(*it);
                            }
                        }
                        //�ǹؼ����
                        path_A_1.push_back(*(path_A_all.end() - 1));


                        //path_A_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ�� Ϊ���
                        //ת�����Ϊ path_A_1.size() - 2







                        //������ײ����
                        corner_num_A = 0;
                        pair<int, int> point1_A_path;
                        pair<int, int> point2_A_path;
                        pair<int, int> point3_map;
                        for (int i = 0; i < map_1_source_A.size(); i++) {
                            if (map_1_source_A[i] == -1) {
                                point3_map.first = i % map_width;
                                point3_map.second = i / map_width;

                                for (auto it = path_A_1.begin(); it != path_A_1.end() - 1; it++) {
                                    point1_A_path = *it;
                                    point2_A_path = *(it + 1);

                                    if (CalculatePointToLineDistance(point1_A_path, point2_A_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                        //std::cout << "���µ���·���ཻ" << endl;
                                        //std::cout << point1_A_path.first << "," << point1_A_path.second << endl;
                                        //std::cout << point2_A_path.first << "," << point2_A_path.second << endl;
                                        //std::cout << point3_map.first <<"," << point3_map.second << endl;
                                        //std::cout << "=============" << endl;
                                        corner_num_A++;
                                        //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                    }

                                }


                            }
                        }

                    }

                }

                if (Drun) {
                    //Dijkstra ----------------

                    //std::cout << "Dijkstra�滮��ʼ" << endl;
                    std::chrono::system_clock::time_point D_start_time = std::chrono::system_clock::now();

                    fudge::GridMap<double> map_D(map_width, map_height, map_1_source_A);



                    //���ĺ���   û��д��   Dijkstra_distance = 0 
                    const std::vector<std::pair<int, int>> path_D = fudge::astar_search(
                        map_D, fudge::Coord(point_start_A.first, point_start_A.second), fudge::Coord(point_target_A.first, point_target_A.second),
                        fudge::GridMap<double>::Dijkstra_distance);
                    std::chrono::system_clock::time_point D_end_time = std::chrono::system_clock::now();
                    //std::cout << "Dijkstra�滮����" << endl;

                    //std::cout << "Dijkstra��ʼͳ��ʵ������" << endl;

                    D_run_time = std::chrono::duration_cast<std::chrono::microseconds>(D_end_time - D_start_time).count();

                    total_runtime_D += D_run_time;

                    if (path_D.size() == 0) {
                        len_D = len_Base;
                        is_opt_D = 0;

                        total_len_D += len_Base;

                    }
                    else {
                        len_D = map_D.node(path_D[0])->f_;
                        total_len_D += len_D;

                        solved_D++;
                        if ((len_D - len_Base) < 1e-01) {
                            opt_D++;
                            is_opt_D = 1;
                        }
                        else {
                            is_opt_D = 0;

                        }
                    }







                    if (is_cal_for_A) {
                        ite_num_D = k_ite_num;
                        son_node_num_D = k_son_node_num;

                        //���е�ԭʼ path ��ȱ�����

                        //����ת�����----------------
                        vector<pair<int, int>> path_D_all = path_D;
                        path_D_all.push_back(point_start_A);
                        path_D_1 = path_zero;

                        path_D_1.push_back(*path_D.begin());
                        for (auto it = (path_D_all.begin() + 1); it != path_D_all.end() - 1; it++) {

                            auto it_front = it - 1;
                            auto it_back = it + 1;

                            if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                                != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                                path_D_1.push_back(*it);
                            }
                        }
                        //path_D_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ��
                        path_D_1.push_back(*(path_D_all.end() - 1));

                        //ת�����Ϊ path_jump_1.size() - 2



                        //������ײ����----------------

                        corner_num_D = 0;
                        pair<int, int> point1_D_path;
                        pair<int, int> point2_D_path;
                        pair<int, int> point3_map;
                        for (int i = 0; i < map_1_source_A.size(); i++) {
                            if (map_1_source_A[i] == -1) {
                                point3_map.first = i % map_width;
                                point3_map.second = i / map_width;

                                for (auto it = path_D_1.begin(); it != path_D_1.end() - 1; it++) {
                                    point1_D_path = *it;
                                    point2_D_path = *(it + 1);

                                    if (CalculatePointToLineDistance(point1_D_path, point2_D_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                        //std::cout << "���µ���·���ཻ" << endl;
                                        //std::cout << point1_A_path.first << "," << point1_A_path.second << endl;
                                        //std::cout << point2_A_path.first << "," << point2_A_path.second << endl;
                                        //std::cout << point3_map.first <<"," << point3_map.second << endl;
                                        //std::cout << "=============" << endl;
                                        corner_num_D++;
                                        //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                    }

                                }


                            }
                        }

                    }

                }




                if (Jrun) {
                    // jumpA* ----------------
                    fudge::JumpPointMap<double> map_jump(map_width, map_height, map_1_source_A);
                    fudge::Coord start(point_start_A.first, point_start_A.second);
                    fudge::Coord goal(point_target_A.first, point_target_A.second);
                    map_jump.goal_ = goal; // This is needed for checking Jump Point.

                    //std::cout << "jump_A*�滮��ʼ" << endl;

                    std::chrono::system_clock::time_point jump_start_time = std::chrono::system_clock::now();
                    //jump A* ���ĺ���
                    const std::vector<fudge::Coord> path_jump = fudge::astar_search(
                        map_jump, start, goal,
                        fudge::GridMap<double>::euclidean_distance);

                    std::chrono::system_clock::time_point jump_end_time = std::chrono::system_clock::now();
                    J_run_time = std::chrono::duration_cast<std::chrono::microseconds>(jump_end_time - jump_start_time).count();

                    total_runtime_J += J_run_time;

                    if (path_jump.size() == 0) {
                        len_J = len_Base;
                        is_opt_J = 0;

                        total_len_J += len_Base;

                    }
                    else {
                        len_J = map_jump.node(path_jump[0])->f_;
                        total_len_J += len_J;

                        solved_J++;
                        if ((len_J - len_Base) < 1e-01) {
                            opt_J++;
                            is_opt_J = 1;
                        }
                        else {
                            is_opt_J = 0;

                        }
                    }





                    if (is_cal_for_A) {

                        ite_num_J = k_ite_num;
                        son_node_num_J = k_son_node_num;



                        //std::cout << "jump_A*��ʼͳ��ʵ������" << endl;

                        //ͳ��ת�����
                        path_jump_1 = path_zero;

                        vector<pair<int, int>> path_jump_all = path_jump;
                        path_jump_all.push_back(point_start_A);
                        path_jump_1.push_back(*path_jump.begin());

                        for (auto it = (path_jump_all.begin() + 1); it != path_jump_all.end() - 1; it++) {

                            auto it_front = it - 1;
                            auto it_back = it + 1;

                            if (((*it_front).first - (*it_back).first) * ((*it).second - (*it_back).second)
                                != ((*it_front).second - (*it_back).second) * ((*it).first - (*it_back).first)) {
                                path_jump_1.push_back(*it);

                            }
                        }


                        //path_jump_1  ��һ��Ԫ��Ϊ�յ�   ���һ��Ԫ�� Ϊ�����һ����  ȱ���
                        path_jump_1.push_back(*(path_jump_all.end() - 1));

                        //ת�����Ϊ path_jump_1.size() - 2




                      //  std::cout << "��ʼ���·��" << endl;
                      //for (const auto &e : path_jump)
                      //  std::cout << map_jump.node(e)->to_string() << std::endl;



                        //ͳ����ײ����
                        corner_num_jump = 0;
                        pair<int, int> point1_jump_path;
                        pair<int, int> point2_jump_path;
                        pair<int, int> point3_map;
                        //std::cout << "��ʼͳ����ײ����" << endl;


                        for (int i = 0; i < map_1_source_A.size(); i++) {

                            if (map_1_source_A[i] == -1) {
                                point3_map.first = i % map_width;
                                point3_map.second = i / map_width;

                                for (auto it = path_jump_1.begin(); it != path_jump_1.end() - 1; it++) {
                                    point1_jump_path = *it;
                                    point2_jump_path = *(it + 1);

                                    if (CalculatePointToLineDistance(point1_jump_path, point2_jump_path, point3_map) <= 0.51) {  //����ֵʱ�����ƽ��
                                        //std::cout << "���µ���·���ཻ" << endl;
                                        //std::cout << point1_jump_path.first << "," << point1_jump_path.second << endl;
                                        //std::cout << point2_jump_path.first << "," << point2_jump_path.second << endl;
                                        //std::cout << point3_map.first << "," << point3_map.second << endl;
                                        //std::cout << "=============" << endl;
                                        corner_num_jump++;
                                        //CalculatePointToLineDistance((*it)->point_start), (*it)->point_target), point3))

                                    }

                                }


                            }
                        }

                    }
                    //is_cal_for_A = false;

                }






                if (DGRrun) {
                    //std::cout << "DGR*  ��ʼ" << endl;

                    //DGR*  


                    initEdgeNode = new Edge_Node(point_start_DGR, point_target_DGR);



                    //��core�У���Ҫ��ʼ���������ͼ�޹صĲ���  �ر� ����  �;����ۼ����ʵ�
                    dgr_star.Core_function(initEdgeNode);


                    vector<Edge_Node*>& path_vector = dgr_star.path_vector;

                    if (path_vector.empty()) {
                        std::cout << "��ǰΪ��" << map_num << "����ͼ" << endl;
                        std::cout << "�ѳɹ����ص� " << scen_iter + 1 << " ������" << endl;
                        std::cout << "DGR_Star�㷨���ʧ��" << endl;
                    }
                    else {


                        len_DGR = path_vector[0]->sum_cost_f;

                        DGR_run_time = dgr_star.DGR_run_time;

                        if (scen_iter == 0) {

                            DGR_map_run_time = dgr_star.map_run_time;
                            total_runtime_DGR_map += DGR_map_run_time;

                        }
                        else {
                            DGR_map_run_time = 0;

                        }


                        total_runtime_DGR += dgr_star.DGR_run_time;


                        total_len_DGR += len_DGR;
                        solved_DGR++;


                        if ((len_DGR - len_Base) < 1e-01) {
                            opt_DGR++;
                            is_opt_DGR = 1;

                        }
                        else {
                            is_opt_DGR = 0;

                        }









                    }




                }




                //��ʼ�������
                if (scen_iter == 0) {
                    dataFile_EI << "��ͼ���ƣ�" << cur_scen.GetMapName() << endl;
                    dataFile_EI << "����������" << my_scen_load.GetNumExperiments() << endl;
                    dataFile_EI << endl;
                    dataFile_EI << endl;

                    dataFile_EI << "���" << "\t\t";
                    dataFile_EI << "����" << "\t\t";
                    dataFile_EI << "��ʱ(΢��)" << "\t\t";
                    dataFile_EI << "��׼����" << "\t\t";
                    dataFile_EI << "����" << "\t\t";
                    dataFile_EI << "�ҵ���" << "\t\t";
                    dataFile_EI << "��Ч��" << "\t\t";
                    dataFile_EI << "���Ž�" << "\t\t";
                    dataFile_EI << "Ԥ�����ʱ(΢��)" << "\t\t" << endl;

                    dataFile_EI << "Seq." << "\t\t";
                    dataFile_EI << "Method" << "\t\t";
                    dataFile_EI << "Time(us)" << "\t\t";
                    dataFile_EI << "Ben.Len." << "\t\t";
                    dataFile_EI << "Len." << "\t\t";
                    dataFile_EI << "Solved" << "\t\t";
                    dataFile_EI << "Invalid" << "\t\t";
                    dataFile_EI << "Opt " << "\t\t";
                    dataFile_EI << "Pre.Map(us)" << "\t\t" << endl;



                }

                dataFile_EI << endl;
                dataFile_EI << scen_iter + 1 << "\t\t";
                dataFile_EI << "A*" << "\t\t";
                dataFile_EI << A_run_time << "\t\t";
                dataFile_EI << len_Base << "\t\t";
                dataFile_EI << len_A << "\t\t";
                dataFile_EI << 1 << "\t\t";
                dataFile_EI << 0 << "\t\t";
                dataFile_EI << is_opt_A << "\t\t";
                dataFile_EI << 0 << "\t\t" << endl;

                dataFile_EI << scen_iter + 1 << "\t\t";
                dataFile_EI << "Dijkstra" << "\t\t";
                dataFile_EI << D_run_time << "\t\t";
                dataFile_EI << len_Base << "\t\t";
                dataFile_EI << len_D << "\t\t";
                dataFile_EI << 1 << "\t\t";
                dataFile_EI << 0 << "\t\t";
                dataFile_EI << is_opt_D << "\t\t";
                dataFile_EI << 0 << "\t\t" << endl;

                dataFile_EI << scen_iter + 1 << "\t\t";
                dataFile_EI << "JPS" << "\t\t";
                dataFile_EI << J_run_time << "\t\t";
                dataFile_EI << len_Base << "\t\t";
                dataFile_EI << len_J << "\t\t";
                dataFile_EI << 1 << "\t\t";
                dataFile_EI << 0 << "\t\t";
                dataFile_EI << is_opt_J << "\t\t";
                dataFile_EI << 0 << "\t\t" << endl;

                dataFile_EI << scen_iter + 1 << "\t\t";
                dataFile_EI << "DGR*" << "\t\t";
                dataFile_EI << DGR_run_time << "\t\t";
                dataFile_EI << len_Base << "\t\t";
                dataFile_EI << len_DGR << "\t\t";
                dataFile_EI << 1 << "\t\t";
                dataFile_EI << 0 << "\t\t";
                dataFile_EI << is_opt_DGR << "\t\t";
                dataFile_EI << DGR_map_run_time << "\t\t" << endl;
                dataFile_EI << endl;


                //���һ������ ���ͳ�ƽ��
                if (scen_iter == (NumExperiments - 1)) {
                    dataFile_EI << endl;
                    dataFile_EI << endl;

                    dataFile_EI << endl;
                    dataFile_EI << endl;
                    dataFile_EI << endl;
                    dataFile_EI << endl;
                    dataFile_EI << "��ͼ���ƣ�" << cur_scen.GetMapName() << endl;
                    dataFile_EI << "����������" << my_scen_load.GetNumExperiments() << endl;
                    dataFile_EI << endl;
                    dataFile_EI << endl;

                    dataFile_EI << "��������" << "\t\t";
                    dataFile_EI << "���� " << "\t\t";
                    dataFile_EI << "�ܺ�ʱ(΢��)" << "\t\t";
                    dataFile_EI << "��׼�ܳ���" << "\t\t";
                    dataFile_EI << "�ܳ���" << "\t\t";
                    dataFile_EI << "�н���" << "\t\t";
                    dataFile_EI << "��Ч����" << "\t\t";
                    dataFile_EI << "���Ž���" << "\t\t";
                    dataFile_EI << "Ԥ�����ʱ(΢��)" << "\t\t" << endl;

                    dataFile_EI << "Num.Total" << "\t\t";
                    dataFile_EI << "Method" << "\t\t";
                    dataFile_EI << "ToTal(us)" << "\t\t";
                    dataFile_EI << "Total_Ben.Len." << "\t\t";
                    dataFile_EI << "Total_Len." << "\t\t";
                    dataFile_EI << "Num.Solved" << "\t\t";
                    dataFile_EI << "Num.Invalid" << "\t\t";
                    dataFile_EI << "Num.Opt." << "\t\t";
                    dataFile_EI << "Pre.Map(us)" << "\t\t" << endl;
                    dataFile_EI << endl;



                    dataFile_EI << total_Base << "\t\t";
                    dataFile_EI << "A*" << "\t\t";
                    dataFile_EI << total_runtime_A << "\t\t";
                    dataFile_EI << total_len_Base << "\t\t";
                    dataFile_EI << total_len_A << "\t\t";
                    dataFile_EI << solved_A << "\t\t";
                    dataFile_EI << total_Base - solved_A << "\t\t";
                    dataFile_EI << opt_A << "\t\t";
                    dataFile_EI << 0 << "\t\t" << endl;


                    dataFile_EI << total_Base << "\t\t";
                    dataFile_EI << "Dijkstra" << "\t\t";
                    dataFile_EI << total_runtime_D << "\t\t";
                    dataFile_EI << total_len_Base << "\t\t";
                    dataFile_EI << total_len_D << "\t\t";
                    dataFile_EI << solved_D << "\t\t";
                    dataFile_EI << total_Base - solved_D << "\t\t";
                    dataFile_EI << opt_D << "\t\t";
                    dataFile_EI << 0 << "\t\t" << endl;

                    dataFile_EI << total_Base << "\t\t";
                    dataFile_EI << "JPS" << "\t\t";
                    dataFile_EI << total_runtime_J << "\t\t";
                    dataFile_EI << total_len_Base << "\t\t";
                    dataFile_EI << total_len_J << "\t\t";
                    dataFile_EI << solved_J << "\t\t";
                    dataFile_EI << total_Base - solved_J << "\t\t";
                    dataFile_EI << opt_J << "\t\t";
                    dataFile_EI << 0 << "\t\t" << endl;


                    dataFile_EI << total_Base << "\t\t";
                    dataFile_EI << "DGR*" << "\t\t";
                    dataFile_EI << total_runtime_DGR << "\t\t";
                    dataFile_EI << total_len_Base << "\t\t";
                    dataFile_EI << total_len_DGR << "\t\t";
                    dataFile_EI << solved_DGR << "\t\t";
                    dataFile_EI << total_Base - solved_DGR << "\t\t";
                    dataFile_EI << opt_DGR << "\t\t";
                    dataFile_EI << total_runtime_DGR_map << "\t\t" << endl;
                    dataFile_EI << endl;
                    dataFile_EI << endl;
                    dataFile_EI << "��������������";


                }

            }



















        }



    }






    return 0;
}







