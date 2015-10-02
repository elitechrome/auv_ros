#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include "dirent.h"
#include <ios>
#include <fstream>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include "opencv2/gpu/gpu.hpp"
#include <iostream>

#define SVMLIGHT 1
#define LIBSVM 2

#define TRAINHOG_USEDSVM SVMLIGHT

#if TRAINHOG_USEDSVM == SVMLIGHT
#include "svmlight/svmlight.h"
#define TRAINHOG_SVM_TO_TRAIN SVMlight
#elif TRAINHOG_USEDSVM == LIBSVM
#include "libsvm/libsvm.h"
#define TRAINHOG_SVM_TO_TRAIN libSVM
#endif
#define PLAY

using namespace std;
using namespace cv;

int one = 1;
class PedestrianNodeClass{
private:
    cv::Mat img;
    // <editor-fold defaultstate="collapsed" desc="Init">
    HOGDescriptor hog; // Use standard parameters here
    double hitThreshold;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub ;
public:
    PedestrianNodeClass(ros::NodeHandle &nh):it(nh)
    {
        sub = it.subscribe("stereo/left/image_raw", 1, &PedestrianNodeClass::imageCallback, this);


        //hog.winSize = Size(32, 64); // Default training images size as used in paper
        hog.winSize = Size(64,128);
        TRAINHOG_SVM_TO_TRAIN::getInstance()->loadModelFromFile("svmlightmodel.dat");
        vector<float> descriptorVector;
        TRAINHOG_SVM_TO_TRAIN::getInstance()->getSingleDetectingVector(descriptorVector, std::vector<unsigned int>());
        //hog.setSVMDetector(descriptorVector);
        hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        //hog.setSVMDetector(cv::gpu::HOGDescriptor::getDefaultPeopleDetector());

    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);


};
void PedestrianNodeClass::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    static unsigned int count = 0;
    try
    {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    vector<Rect> found, found_filtered;
    hitThreshold = TRAINHOG_SVM_TO_TRAIN::getInstance()->getThreshold();
    //hog.detectMultiScale(img, found, hitThreshold, Size(8,8), Size(32,32), 1.05, 2);
    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);

    size_t i, j;
    for (i=0; i<found.size(); i++)
    {
        Rect r = found[i];
        for (j=0; j<found.size(); j++)
            if (j!=i && (r & found[j])==r)
                break;
        if (j==found.size())
            found_filtered.push_back(r);
    }
    for (i=0; i<found_filtered.size(); i++)
    {
        Rect r = found_filtered[i];

        std::ostringstream stringStream;
        stringStream << "tmp/"<< count++ << ".jpg";
        std::string copyOfStr = stringStream.str();
        if (((r.x + r.width) > img.cols) || ((r.x) < 0))
        {
            continue;
        }
        if (((r.y + r.height) > img.rows) || ((r.y) < 0))
        {
            continue;
        }

        imwrite(copyOfStr, img(r).clone());
        //rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
    }
    imshow("view", img);


    //waitKey(1);



    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main (int argc, char * argv[])
{
    ros::init(argc, argv, "pedestrian_detection");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::namedWindow("roi");

    cv::startWindowThread();
    PedestrianNodeClass ped(nh);
    ros::spin();
    cv::destroyWindow("view");
    cv::destroyWindow("roi");


    return 0;
}


/*


















#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include "dirent.h"
#include <ios>
#include <fstream>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>

#define SVMLIGHT 1
#define LIBSVM 2

#define TRAINHOG_USEDSVM SVMLIGHT

#if TRAINHOG_USEDSVM == SVMLIGHT
#include "svmlight/svmlight.h"
#define TRAINHOG_SVM_TO_TRAIN SVMlight
#elif TRAINHOG_USEDSVM == LIBSVM
#include "libsvm/libsvm.h"
#define TRAINHOG_SVM_TO_TRAIN libSVM
#endif
#define PLAY

using namespace std;
using namespace cv;


static string posSamplesDir = "pos/";

static string negSamplesDir = "neg/";

static string featuresFile = "genfiles/features.dat";

static string svmModelFile = "genfiles/svmlightmodel.dat";

static string descriptorVectorFile = "genfiles/descriptorvector.dat";


static const Size trainingPadding = Size(0, 0);
static const Size winStride = Size(8, 8);
// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="Helper functions">

static string toLowerCase(const string& in) {
    string t;
    for (string::const_iterator i = in.begin(); i != in.end(); ++i) {
        t += tolower(*i);
    }
    return t;
}

static void storeCursor(void) {
    printf("\033[s");
}

static void resetCursor(void) {
    printf("\033[u");
}



static void saveDescriptorVectorToFile(vector<float>& descriptorVector, vector<unsigned int>& _vectorIndices, string fileName) {
    printf("Saving descriptor vector to file '%s'\n", fileName.c_str());
    string separator = " "; // Use blank as default separator between single features
    fstream File;
    float percent;
    File.open(fileName.c_str(), ios::out);
    if (File.good() && File.is_open()) {
        printf("Saving %lu descriptor vector features:\t", descriptorVector.size());
        storeCursor();
        for (int feature = 0; feature < descriptorVector.size(); ++feature) {
            if ((feature % 10 == 0) || (feature == (descriptorVector.size() - 1))) {
                percent = ((1 + feature) * 100 / descriptorVector.size());
                printf("%4u (%3.0f%%)", feature, percent);
                fflush(stdout);
                resetCursor();
            }
            File << descriptorVector.at(feature) << separator;
        }
        printf("\n");
        File << endl;
        File.flush();
        File.close();
    }
}


static void getFilesInDirectory(const string& dirName, vector<string>& fileNames, const vector<string>& validExtensions) {
    printf("Opening directory %s\n", dirName.c_str());
    struct dirent* ep;
    size_t extensionLocation;
    DIR* dp = opendir(dirName.c_str());
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            // Ignore (sub-)directories like . , .. , .svn, etc.
            if (ep->d_type & DT_DIR) {
                continue;
            }
            extensionLocation = string(ep->d_name).find_last_of("."); // Assume the last point marks beginning of extension like file.ext
            // Check if extension is matching the wanted ones
            string tempExt = toLowerCase(string(ep->d_name).substr(extensionLocation + 1));
            if (find(validExtensions.begin(), validExtensions.end(), tempExt) != validExtensions.end()) {
                printf("Found matching data file '%s'\n", ep->d_name);
                fileNames.push_back((string)dirName + ep->d_name);
            }
            else {
                printf("Found file does not match required file type, skipping: '%s'\n", ep->d_name);
            }
        }
        (void)closedir(dp);
    }
    else {
        printf("Error opening directory '%s'!\n", dirName.c_str());
    }
    return;
}


static void calculateFeaturesFromInput(const string& imageFilename, vector<float>& featureVector, HOGDescriptor& hog) {

    Mat imageData = imread(imageFilename, 0);
    if (imageData.empty()) {
        featureVector.clear();
        printf("Error: HOG image '%s' is empty, features calculation skipped!\n", imageFilename.c_str());
        return;
    }
    resize(imageData, imageData, Size(32, 64));
    // Check for mismatching dimensions
    if (imageData.cols != hog.winSize.width || imageData.rows != hog.winSize.height) {
        featureVector.clear();
        printf("Error: Image '%s' dimensions (%u x %u) do not match HOG window size (%u x %u)!\n", imageFilename.c_str(), imageData.cols, imageData.rows, hog.winSize.width, hog.winSize.height);
        return;
    }
    vector<Point> locations;
    hog.compute(imageData, featureVector, winStride, trainingPadding, locations);
    imageData.release(); // Release the image again after features are extracted
}


static void showDetections(const vector<Point>& found, Mat& imageData) {
    size_t i, j;
    for (i = 0; i < found.size(); ++i) {
        Point r = found[i];
        // Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
        rectangle(imageData, Rect(r.x - 16, r.y - 32, 32, 64), Scalar(64, 255, 64), 3);
    }
}


static void showDetections(const vector<Rect>& found, Mat& imageData) {
    static unsigned int count = 0;
    vector<Rect> found_filtered;
    size_t i, j;
    for (i = 0; i < found.size(); ++i) {
        Rect r = found[i];
        for (j = 0; j < found.size(); ++j)
            if (j != i && (r & found[j]) == r)
                break;
        if (j == found.size())
            found_filtered.push_back(r);
    }
    for (i = 0; i < found_filtered.size(); i++) {
        Rect r = found_filtered[i];
        std::ostringstream stringStream;
        stringStream << "tmp/"<< count++ << ".jpg";
        std::string copyOfStr = stringStream.str();
        if (((r.x + r.width) > imageData.cols) || ((r.x) < 0))
        {
            continue;
        }
        if (((r.y + r.height) > imageData.rows) || ((r.y) < 0))
        {
            continue;
        }

        //imwrite(copyOfStr, imageData(r).clone());
        imshow("roi", imageData(r).clone());
        waitKey(1);
        rectangle(imageData, r.tl(), r.br(), Scalar(64, 255, 64), 3);
    }
}


static void detectTrainingSetTest(const HOGDescriptor& hog, const double hitThreshold, const vector<string>& posFileNames, const vector<string>& negFileNames) {
    unsigned int truePositives = 0;
    unsigned int trueNegatives = 0;
    unsigned int falsePositives = 0;
    unsigned int falseNegatives = 0;
    vector<Point> foundDetection;
    // Walk over positive training samples, generate images and detect
    for (vector<string>::const_iterator posTrainingIterator = posFileNames.begin(); posTrainingIterator != posFileNames.end(); ++posTrainingIterator) {
        Mat imageData = imread(*posTrainingIterator, 0);
        resize(imageData, imageData, Size(32, 64));
        hog.detect(imageData, foundDetection, hitThreshold, winStride, trainingPadding);
        if (foundDetection.size() > 0) {
            ++truePositives;
            falseNegatives += foundDetection.size() - 1;
        }
        else {
            ++falseNegatives;
        }
    }
    // Walk over negative training samples, generate images and detect
    for (vector<string>::const_iterator negTrainingIterator = negFileNames.begin(); negTrainingIterator != negFileNames.end(); ++negTrainingIterator) {
        const Mat imageData = imread(*negTrainingIterator, 0);
        hog.detect(imageData, foundDetection, hitThreshold, winStride, trainingPadding);
        if (foundDetection.size() > 0) {
            falsePositives += foundDetection.size();
        }
        else {
            ++trueNegatives;
        }
    }

    printf("Results:\n\tTrue Positives: %u\n\tTrue Negatives: %u\n\tFalse Positives: %u\n\tFalse Negatives: %u\n", truePositives, trueNegatives, falsePositives, falseNegatives);
}


static void detectTest(const HOGDescriptor& hog, const double hitThreshold, Mat& imageData) {
    vector<Rect> found;
    Size padding(Size(32, 32));
    Size winStride(Size(8, 8));
    hog.detectMultiScale(imageData, found, hitThreshold, winStride, padding);
    //hog.detectMultiScale(imageData, found, 0, Size(8,8), Size(32,32));
    showDetections(found, imageData);

}
// </editor-fold>
int one = 1;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    // <editor-fold defaultstate="collapsed" desc="Init">
    HOGDescriptor hog; // Use standard parameters here
    double hitThreshold;
    try
    {
        if( one == 1 )
        {
            hog.winSize = Size(32, 64); // Default training images size as used in paper
            //hog.winSize = Size(64,128);
            //printf("Testing training phase using training set as test set (just to check if training is ok - no detection quality conclusion with this!)\n");
            //detectTrainingSetTest(hog, hitThreshold, positiveTrainingImages, negativeTrainingImages);
            TRAINHOG_SVM_TO_TRAIN::getInstance()->loadModelFromFile("svmlightmodel.dat");
            vector<float> descriptorVector;
            TRAINHOG_SVM_TO_TRAIN::getInstance()->getSingleDetectingVector(descriptorVector, std::vector<unsigned int>());
            hog.setSVMDetector(descriptorVector);
            //hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
            hitThreshold = TRAINHOG_SVM_TO_TRAIN::getInstance()->getThreshold();

            one = 2;
        }
        else
        {
            //printf("Testing custom detection using camera\n");
            hitThreshold = TRAINHOG_SVM_TO_TRAIN::getInstance()->getThreshold();
            detectTest(hog, hitThreshold, img);
            imshow("view", img);
            //waitKey(1);
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_sample");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::namedWindow("roi");

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("stereo/left/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
    cv::destroyWindow("roi");

}
*/

