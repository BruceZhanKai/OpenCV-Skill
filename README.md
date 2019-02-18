# OpenCV-Skill

## Rect

### cv::Mat(cv::Mat img, cv::Rect rect) - Get ROI from Mat by Rect

- [How do I convert a cv::Mat to a cv::Rect](https://stackoverflow.com/questions/20055741/how-do-i-convert-a-cvmat-to-a-cvrect)

```
cv::Mat origin;
cv::Rect rect;
cv::Mat roi = cv::Mat(origin,rect);
```

### roi.copyTo(image(roi_rect)); - Get Mat with new ROI from Rect

- [【OpenCV3】将图像指定区域使用另一图像（或ROI）覆盖](https://blog.csdn.net/guduruyu/article/details/72843368)

```
cv::Mat image = cv::Mat::zeros(512, 512, CV_8UC3);
image.setTo(cv::Scalar(100, 0, 0));
cv::imshow("origin", image);
//读取待复制图片
cv::Mat roi = cv::imread("E:\\Images\\Hepburn.png", cv::IMREAD_COLOR);
//设置画布绘制区域并复制
cv::Rect roi_rect = cv::Rect(128, 128, roi.cols, roi.rows);
roi.copyTo(image(roi_rect));
cv::imshow("result", image);
```

### cv::boundingRect - Rect from contours

- [简单opencv 外轮廓检测以加矩形框](https://blog.csdn.net/daixiangzi/article/details/78994990)
- [Creating Bounding boxes and circles for contours](https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html)

```
cv::Rect rect = cv::boundingRect(contours[0]);
cv::rectangle(image,rect,color);	
```

### cv::minAreaRect - Rotated Rect

- [minAreaRect函数](http://www.voidcn.com/article/p-emcpncjs-bek.html)

```
std::vector<cv::RotatedRect> minrect(1);
minrect[0] = cv::minAreaRect( cv::Mat(hull[0]) );
cv::Point2f rect_points[4]; 
minrect[0].points( rect_points );
```

```
class CV_EXPORTS RotatedRect
{
public:
    //! various constructors
    RotatedRect();
    RotatedRect(const Point2f& center, const Size2f& size, float angle);
    RotatedRect(const CvBox2D& box);

    //! returns 4 vertices of the rectangle
    void points(Point2f pts[]) const;
    //! returns the minimal up-right rectangle containing the rotated rectangle
    Rect boundingRect() const;
    //! conversion to the old-style CvBox2D structure
    operator CvBox2D() const;

    Point2f center; //< the rectangle mass center
    Size2f size;    //< width and height of the rectangle
    float angle;    //< the rotation angle. When the angle is 0, 90, 180, 270 etc., the rectangle becomes an up-right rectangle.
};
```


### Rect class

- [Opencv3中Rect和rectangle函数](https://blog.csdn.net/wwwlyj123321/article/details/80563114)

```
//如果创建一个Rect对象rect(100, 50, 50, 100)，那么rect会有以下几个功能：  
rect.area();     //返回rect的面积 5000  
rect.size();     //返回rect的尺寸 [50 × 100]  
rect.tl();       //返回rect的左上顶点的坐标 [100, 50]  
rect.br();       //返回rect的右下顶点的坐标 [150, 150]  
rect.width();    //返回rect的宽度 50  
rect.height();   //返回rect的高度 100  
rect.contains(cv::Point(x, y));  //返回布尔变量，判断rect是否包含Point(x, y)点  
  
//还可以求两个矩形的交集和并集  
rect = rect1 & rect2;  
rect = rect1 | rect2;  
  
//还可以对矩形进行平移和缩放    
rect = rect + cv::Point(-100, 100); //平移，也就是左上顶点的x坐标-100，y坐标+100  
rect = rect + cv::Size(-100, 100);  //缩放，左上顶点不变，宽度-100，高度+100  
  
//还可以对矩形进行对比，返回布尔变量  
rect1 == rect2;  
rect1 != rect2;  
```

```
void rectangle(CV_IN_OUT Mat& img, Rect rec,
                          const Scalar& color, int thickness = 1,
                          int lineType = LINE_8, int shift = 0);
						  
cv::rectangle(matImage,cv::Point(20,200),cv::Point(200,300),cv::Scalar(255,0,0),1,1,0);  
//Rect(int a,int b,int c,int d)a,b为矩形的左上角坐标,c,d为矩形的长和宽  
cv::rectangle(matImage,cv::Rect(100,300,20,200),cv::Scalar(0,0,255),1,1,0);  
```

## Image Proccessing

### cv::subtract - Background clipping

- [背景剪除和OpenCV中的实现](https://www.cnblogs.com/zhchoutai/p/7092521.html)

```
cv::Mat background, foreground, diff;
cv::subtract(foreground, background, diff);
```

### cv::convexHull - 「凸包」或「凸殼」

- [Convex Hull](http://www.csie.ntnu.edu.tw/~u91029/ConvexHull.html)
- [基于深度摄像头的障碍物检测（realsense+opencv）](https://blog.csdn.net/u012423865/article/details/72910516)

```
std::vector<std::vector<cv::Point> > hull(contours.size());   
for (int i = 0; i < contours.size(); i++)  {  
cv::convexHull(cv::Mat(contours[i]), hull[i], false);  
}  

```

### cv::morphologyEx - 形態學

- [OpenCV学习十二：morphologyEx，图像的开运算、闭运算、顶帽运算、黑帽运算以及形态学梯度](https://blog.csdn.net/kakiebu/article/details/79324301)

```
//CV_MOP_OPEN CV_MOP_CLOSE CV_MOP_GRADIENT CV_MOP_TOPHAT CV_MOP_BLACKHAT
cv::morphologyEx(origin_gray, result_gray, 1, kernel);
```

### cv::getStructuringElement - 形態學結構元素

- [opencv getStructuringElement函数](https://blog.csdn.net/kksc1099054857/article/details/76569718)
- [侵蝕、膨脹(erode、dilate)](http://monkeycoding.com/?p=577)

```
int g_nStructElementSize = 3; //结构元素(内核矩阵)的尺寸
cv::Mat element = cv::getStructuringElement(MORPH_RECT,
		Size(g_nStructElementSize,g_nStructElementSize));

//侵蝕
cv::erode(const Mat &src, Mat &dst, Mat kernel, Point anchor=Point(-1,-1), int iterations=1)
cv::erode( src, dst, element );
//膨脹
cv::dilate(const Mat &src, Mat &dst, Mat kernel, Point anchor=Point(-1,-1), int iterations=1)
```


## Time

### cv::getTickCount() - cost time

- [opencv中测量运行时间的函数](https://blog.csdn.net/u013488563/article/details/19921073)

```
double t = (double)cv::getTickCount();  
// do something ...  
t = ((double)cv::getTickCount() - t)/cv::getTickFrequency(); // t=100(sec)
````

## String

### Convert between String & Int

- [convert between std::string and int](http://www.cplusplus.com/forum/general/13135/)

```
std::string myString = "45";
int value = std::atoi(myString.c_str()); //value = 45
```

```
int value = 45;
std::string myString = std::to_string(value);
```

```
#include <sstream>
#include <string>
std::string myStream = "45";
std::istringstream buffer(myString);
int value;
buffer >> value;   // value = 45 
```


## Contours

### cv::cornerHarris - Corner Detection

- [OpenCV探索之路（十五）：角点检测](https://www.cnblogs.com/skyfsm/p/6899627.html)

```
cv::cornerHarris(g_grayImage, dstImage, 2, 3, 0.04, cv::BORDER_DEFAULT);
```

### Shi-Tomsi=Harris加强版

- [OpenCV探索之路（十五）：角点检测](https://www.cnblogs.com/skyfsm/p/6899627.html)
- [goodFeaturesToTrack函数详细注释](https://blog.csdn.net/xdfyoga1/article/details/44175637)

```
std::vector<cv::Point2f> corners;
double qualityLevel = 0.01;
double minDistance = 10;
int blockSize = 3;
bool useHarrisDetector = false;
double k = 0.04;
cv::goodFeaturesToTrack(src_gray,corners,maxCorners,qualityLevel,minDistance,Mat(),blockSize,useHarrisDetector,k);
```


















