#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter                                                                                                         //implementazione della classe ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()                                                                                                           //costruttore con argomenti             
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,                                                                 //questo nodo si iscrive al topic che possiede le immagini rilevate dalla camera del kuka iiwa
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);                                                          //topic su cui questo nodo pubblica

    cv::namedWindow(OPENCV_WINDOW);                                                                                          //creo finestra di visualizzazione di nome "Image Window" (vedi riga 10)
  }

  ~ImageConverter()                                                                                                          //distruttore   
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)                                                                        //callback function. Viene chiamata nel costruttore ogni volta che ricevo un messaggio di tipo sensor_msgs/Image dal topic "/iiwa/camera1/image_raw"
  {
    cv_bridge::CvImagePtr cv_ptr;                                                                                            //dichiaro puntatore a oggetto che contiene un'immagine di tipo OpenCv
    try                                                                                                                      //se nel blocco successivo { ... } va tutto bene, non succede nulla
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);                                                 //copio l'immagine nel messaggio ros in un oggetto cv::Mat (formato per le immagini usato da OpenCV). Il formato è quello BGR (NON RGB!) a 8 bit
    }
    catch (cv_bridge::Exception& e)                                                                                          //in caso di fallimento, lancio un'eccezione
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Detect the circular object
  /*    
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
  */


////////////////////////////////////////////////////////////////////
//                        HOMEWORK3                               //
////////////////////////////////////////////////////////////////////

    using namespace cv;
    
    // Read image
    Mat im_gray;                                                                                                             //cv::Mat è la classe di OpenCV per gestire le immagini
    cvtColor(cv_ptr->image, im_gray, cv::COLOR_BGR2GRAY);                                                                    //cv::cvtColor: legge l'immagine "cv_ptr->image" in formato BGR e la converte in scala di grigi "im_gray".

    // Setup SimpleBlobDetector parameters
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;
 
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 1500;
 
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.1;
 
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.87;
 
    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;

    // Set up the detector with parameters (default ones).
    Ptr<SimpleBlobDetector> detector=SimpleBlobDetector::create(params);                                                     //Ptr<T> è un puntatore smart utilizzato in OpenCV. In questo caso, è un puntatore a un oggetto del tipo SimpleBlobDetector. SimpleBlobDetector è una classe di OpenCV che rileva blob nelle immagini. SimpleBlobDetector::create() è una funzione statica che crea un'istanza di SimpleBlobDetector con i parametri "params"
 
    // Detect blobs.
    std::vector<KeyPoint> keypoints;                                                                                         //creo un vettore di keypoints (oggetti di classe cv::Keypoint)
    detector->detect(im_gray, keypoints);                                                                                    //cv::SimpleBlobDetector::detect: metodo per rilevare i blob presenti in im_gray, e memorizzarli nel vettore keypoints
 
    // Draw detected blobs as circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob             //creo oggetto "immagine + keypoints rilevati"
    drawKeypoints( cv_ptr->image, keypoints, cv_ptr->image, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );        //convenzione non RGB, ma BGR! Quindi dato che il mio oggetto è rosso, converto i cerchietti in blu
    
    //debug
    if(keypoints.size()) std::cout << "Circle detected" << std::endl;                                                        //se il vettore di keypoints viene minimamente riempito, stampo

///////////////////////////////////////////////////////////////////


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW,cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}