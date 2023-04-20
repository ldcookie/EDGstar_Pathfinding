#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

void map2png(vector<int> mapdata, int width, int height, const cv::String dataPath_in, bool Is_bold) {
	//白色 (255, 255, 255)
	//黑色 (0, 0, 0)
	//红色 (0, 0, 255)

	cv::Mat my_image(width, height, CV_8UC3, cv::Scalar(255, 255, 255));

	//先黑白  and 单线
	for (int i = 0; i < my_image.rows; i++)
	{
		for (int j = 0; j < my_image.cols; j++)
		{
			if (mapdata[i * width + j] == 0) {
				my_image.at<Vec3b>(i, j)[0] = 255;
				my_image.at<Vec3b>(i, j)[1] = 255;
				my_image.at<Vec3b>(i, j)[2] = 255;

			}
			else if (mapdata[i * width + j] == 1) {
				my_image.at<Vec3b>(i, j)[0] = 41;
				my_image.at<Vec3b>(i, j)[1] = 29;
				my_image.at<Vec3b>(i, j)[2] = 21;

			}

			else {

				if (!Is_bold) {
					//当前点
					my_image.at<Vec3b>(i, j)[0] = 32;
					my_image.at<Vec3b>(i, j)[1] = 41;
					my_image.at<Vec3b>(i, j)[2] = 209;
				}







			}

		}
	}

	if (Is_bold) {

		for (int i = 0; i < my_image.rows; i++)
		{
			for (int j = 0; j < my_image.cols; j++)
			{

				if (mapdata[i * width + j] == -1) {
					//先找到需要赋值的区域
					int ii;
					int jj;

					if (i < 4) {
						ii = 4;
					}
					else if (i > (height - 5)) {
						ii = (height - 5);
					}
					else {
						ii = i;
					}


					if (j < 4) {
						jj = 4;
					}
					else if (j > (width - 5)) {
						jj = (width - 5);
					}
					else {
						jj = j;
					}

					for (int iii = ii - 4; iii < ii + 5; iii++) {
						for (int jjj = jj - 4; jjj < jj + 5; jjj++) {

							if (iii == ii - 4 and (jjj == jj - 4 or jjj == jj + 4 or jjj == jj - 3 or jjj == jj + 3 or jjj == jj - 2 or jjj == jj + 2)
								or iii == ii + 4 and (jjj == jj - 4 or jjj == jj + 4 or jjj == jj - 3 or jjj == jj + 3 or jjj == jj - 2 or jjj == jj + 2)

								or iii == ii - 3 and (jjj == jj - 4 or jjj == jj + 4)
								or iii == ii + 3 and (jjj == jj - 4 or jjj == jj + 4)

								or iii == ii - 2 and (jjj == jj - 4 or jjj == jj + 4)
								or iii == ii + 2 and (jjj == jj - 4 or jjj == jj + 4)

								) {
							}
							else {
								//一个for循环 赋值
								my_image.at<Vec3b>(iii, jjj)[0] = 32;
								my_image.at<Vec3b>(iii, jjj)[1] = 41;
								my_image.at<Vec3b>(iii, jjj)[2] = 209;
							}


						}

					}

				}




			}
		}


	}


	imwrite(dataPath_in, my_image);

}
