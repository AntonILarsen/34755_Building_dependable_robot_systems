#include <iostream>
#include <vector>
#include <future>
#include <cmath>
#include "utime.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>


cv::Mat source;

const int MHL = 1080;
float hist[MHL] = {0};
int histCnt[MHL] = {0};

int makeHistogramsForAngle_part(cv::Mat gray, int r0, int r1, float drdc)
{
  int h = gray.rows;
  int w = gray.cols;
  int result = 0;
  for (int r = r0; r < r1; r++)
  { // number of histograms are equal to the number of rows
    hist[r] = 0;
    for (int c = 0; c < w; c++)
    { // make histogram starting with this row
      // changing row with this dr/dc (drdc)
      int row = r + cvRound(float(c) * drdc);
      if (row >= 0 and row < h)
      { // this point is valid
        uchar * px = (uchar*) gray.ptr(row); // gray
        hist[r] += px[c];
        histCnt[r]++;
        result++;
      }
    }
  }
  return result;
}


bool makeHistogramsForAngle(int starty)
{
//   bool wireFound = false;
  cv::Mat gray;
  cv::cvtColor(source, gray, cv::COLOR_BGR2GRAY);
  int h = gray.rows;
  int w = gray.cols;
  //
  const int n = 5;
  std::future<int> his[n];
//   cv::Mat debugImg;
  // reserve a histogram for every height line in ROI
//   hists.reserve(h);
  float drdc = -float(starty)/float(w/2);
  //   printf("# got img=%d, %dx%d image, starty = %d, drdc=%f\n",imgNum, h, w, starty, drdc);
  int stride = h/n;
  int m = 0;
  for (int r = 0; r < h; r += stride)
  { // number of histograms are equal to the number of rows
    printf("# starting thread %d\n", m);
    int end = r + stride;
    if (end >= h)
      end = h;
    his[m] = std::async(makeHistogramsForAngle_part, gray, r, end, drdc);
    m++;
  }
  printf("waiting\n");
  for (int i = 0; i < n; i++)
    his[i].wait();
  return true;
}


double taylor_part(double x, size_t start, size_t end)
{
  double result = 0;
  for (size_t i = start; i < end; i++)
  {
    result += std::pow(-1, i+1)/i * std::pow(x-1,i);
  }
  return result;
}

int main(int argc, char **argv)
{
  int startx = -100;
  source = cv::imread("/home/chr/svnjca/wire_vision/img/img_39.jpg");
  UTime t("now");
  int n = makeHistogramsForAngle(startx);
  printf("# Make took %g sec\n", t.getTimePassed());
//   printf("# cnt:");
//   for (int i = 0; i < MHL; i++)
//     printf(" %d", histCnt[i]);
//   printf("\n");
  printf("# cnt:");
  for (int i = 0; i < MHL; i++)
    printf(" %.0f", hist[i]/histCnt[i]);
  printf("\n");
  return EXIT_SUCCESS;
}

// int main(int argc, char **argv)
// {
//   double x = 0.5;
//   size_t n = 400000;
//   UTime t("now");
//   std::future<double> f1 = std::async(taylor_part,x, 1, n/2);
//   std::future<double> f2 = std::async(taylor_part,x, n/2, n);
// //   std::future<double> f3 = std::async(taylor_part,x, n/2, (n*3)/4);
// //   std::future<double> f4 = std::async(taylor_part,x, (n*3)/4, n);
//   double fs = f1.get() + f2.get(); // +f3.get() + f4.get();
//   double dt = t.getTimePassed();
//   //
//   t.now();
//   double a3 = taylor_part(x, 1, n);
//   double da3 = t.getTimePassed();
//   //
//   printf("# ln(%g) = %g (took %g sec using future)\n", x, fs, dt);
//   printf("# ln(%g) = %g (took %g sec using one)\n", x, a3, da3);
//   //
//   return EXIT_SUCCESS;
// }
