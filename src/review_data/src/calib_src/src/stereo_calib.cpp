#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "circle_board.h"


#define USING_BLOB_DETECTOR

using namespace cv;
using namespace std;

// use cv::calibrateCameraRO() function to calibrate


class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{"
                  << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Write_outputFileName"  << outputFileName
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["VISUALIZE_KEYPTS"] >> VISUALIZE_KEYPTS;
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Write_outputFileName"] >> outputFileName;
        node["Input"] >> input;
        node["InputIrParAddr"] >> ir_calib_addr;
        node["InputRgbParAddr"] >> rgb_calib_addr;
        nrFrames = 100;
        validate();
    }

    void readCameraMat(const std::string &yaml_addr, cv::Mat &cameraMat, cv::Mat &distCoeff)
    {
        std::cout << "yaml_addr = " << yaml_addr << std::endl;
        FileStorage fs(yaml_addr, FileStorage::READ);
        fs["CameraMat"] >> cameraMat;
        fs["DistCoeff"] >> distCoeff;

    }

    void validate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (isListOfImages(input) && readStringList(input, irImageList, rgbImageList))
        {
            nrFrames = (nrFrames < (int)irImageList.size()) ? nrFrames : (int)irImageList.size();
        }
        else{
            goodInput = false;
        }
       
        
        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
        {
            cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
            goodInput = false;
        }
    }
    


    static bool readStringList( const string& filename, vector<string>& irImageList, vector<string>& rgbImageList)
    {
        irImageList.clear();
        rgbImageList.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        int index = 0;
        for( ; it != it_end; ++it )
        {
            std::string ir_address, rgb_address;

                
            if (index %2 == 0){
                ir_address = (string)*it; 
                std::cout << "ir_address = " << ir_address << std::endl;
                irImageList.push_back(ir_address);

            }
            else{
                rgb_address = (string)*it;
                std::cout << "rgb_address = " << rgb_address << std::endl;
                rgbImageList.push_back(rgb_address);
            }
            index += 1;
        }
        return true;
    }

    static bool isListOfImages( const string& filename)
    {
        string s(filename);
        // Look for file extension
        if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
            return false;
        else
            return true;
    }
public:
    Size boardSize;              // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                // The number of frames to use from the input for calibration
    string outputFileName;       // The name of the file where to write
    string input;                // The input ->
   
    bool VISUALIZE_KEYPTS;       // visualize key pts or not
    std::vector<std::string> valid_imgs_lst; // 
    std::string ir_calib_addr, rgb_calib_addr;

    vector<string> irImageList;
    vector<string> rgbImageList;
    bool goodInput;

private:
    string patternToUse;


};

static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}


bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object);


static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
        break;
    default:
        break;
    }
}

int main(int argc, char* argv[])
{
    const String keys
        = "{help h usage ? |           | print this message            }"
          "{@settings      |/home/why/ROS_self/publish_collect_review_data/src/review_data/src/calib_src/config/stereo_setting.xml| input setting file            }"
          "{d              |           | actual distance between top-left and top-right corners of "
          "the calibration grid }"
          "{winSize        | 11        | Half of search window for cornerSubPix }";
    CommandLineParser parser(argc, argv, keys);
    parser.about("This is a camera calibration sample.\n"
                 "Usage: camera_calibration [configuration_file -- default ./default.xml]\n"
                 "Near the sample file you'll find the configuration file, which has detailed help of "
                 "how to edit it. It may be any OpenCV supported file format XML/YAML.");
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    //! [file_read]
    Settings s;
    const string inputSettingsFile = parser.get<string>(0);
    std::cout << "inputSettingsFile = " << inputSettingsFile << std::endl;
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        parser.printMessage();
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    int winSize = parser.get<int>("winSize");

    float grid_width = s.squareSize * (s.boardSize.width - 1);
    bool release_object = false;
	if (parser.has("d")) {
		grid_width = parser.get<float>("d");
		release_object = true;
	}

    vector<vector<Point2f> > imagePoints;
	// CameraMatrix => intrinsic matrix
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    
	clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    //! [get_input]
	Ptr<FeatureDetector> irBlobDetector, rgbBlobDetector;
	get_blob_detectors(&irBlobDetector, true);
	get_blob_detectors(&rgbBlobDetector, false);
	vector< vector< Point2f > > ir_image_pts, rgb_image_pts;
	for(int i=0; i < s.irImageList.size();++i)
    {
        std::string ir_img_address = s.irImageList[i];
        cv::Mat ir_img = imread(ir_img_address, IMREAD_COLOR);
        std::string rgb_img_address = s.rgbImageList[i];
        cv::Mat rgb_img = imread(rgb_img_address, IMREAD_COLOR);
        
        //-----  If no more image, or got enough, then stop calibration and show result -------------
		std::cout << "img_count = " << imagePoints.size() << ", frame_num = " << (size_t)s.nrFrames << std::endl;
		
        // if(ir_rgb_pair[0].empty())          // If there are no more images stop the loop
        // {
        //     if ( imagePoints.size() < 1)
        //     {
        //         std::cout << "detected board num is " << imagePoints.size() << "(less than 1)" << std::endl;
        //         return -1;
        //     }
        //     // if calibration threshold was not reached yet, calibrate now
        //         // start calibration!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// 		// runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width,
        //         //                       release_object);
        //     break;
        // }

        // visualize keypts detected by blob detector
        if(s.VISUALIZE_KEYPTS){
            std::vector<KeyPoint> ir_centers, rgb_centers;

            irBlobDetector->detect(ir_img, ir_centers);
            rgbBlobDetector->detect(rgb_img, rgb_centers);
            
            namedWindow("circle pattern", 0);
            cv::Mat ir_with_pts, rgb_with_pts;
            drawKeypoints(ir_img, ir_centers, ir_with_pts, (255, 255, 0));
            drawKeypoints(rgb_img, rgb_centers, rgb_with_pts, (255, 255, 0));
            // resizeWindow("circle pattern", view.cols * 4, view.rows * 4);
            imshow("ir circle pattern", ir_with_pts);
            imshow("rgb circle pattern", rgb_with_pts);
                // cv::waitKey(0);
        }

        //! [find_pattern]
        vector<Point2f> irPointBuf, rgbPointBuf;
        bool ir_found, rgb_found;

        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            // use HE(histogram equalization) and adaptive threshold segmentation
            ir_found = findChessboardCorners( ir_img, s.boardSize, irPointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
            rgb_found = findChessboardCorners( rgb_img, s.boardSize, rgbPointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
			ir_found = findCirclesGrid(ir_img, s.boardSize, irPointBuf, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, irBlobDetector);
			rgb_found = findCirclesGrid(rgb_img, s.boardSize, rgbPointBuf, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, rgbBlobDetector);
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            ir_found = findCirclesGrid( ir_img, s.boardSize, irPointBuf, CALIB_CB_ASYMMETRIC_GRID );
            rgb_found = findCirclesGrid( rgb_img, s.boardSize, rgbPointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            ir_found = false;
            rgb_found = false;
            break;
        }
        if ( ir_found && rgb_found)                // If done with success,
        {
                ir_image_pts.push_back(irPointBuf);
                rgb_image_pts.push_back(rgbPointBuf);
                s.valid_imgs_lst.push_back(ir_img_address + "    " + rgb_img_address);
                // Draw the corners.
                drawChessboardCorners( ir_img, s.boardSize, Mat(irPointBuf), ir_found );
                drawChessboardCorners( rgb_img, s.boardSize, Mat(rgbPointBuf), rgb_found );
        
        }
        
        //! [output_undistorted]
        //------------------------------ Show image and check for input commands -------------------
        //! [await_input]
        imshow("ir Image View", ir_img);
        imshow("rgb Image View", rgb_img);
        std::cout << "finished imshow" << std::endl;

        char key;
        if (s.VISUALIZE_KEYPTS){key = (char)waitKey(0);}
        else {key = (char)waitKey(100);}
    }
    cv::Mat rgb_camera_mat, rgb_dist_mat, ir_camera_mat, ir_dist_mat;
    s.readCameraMat(s.ir_calib_addr, ir_camera_mat, ir_dist_mat);
    s.readCameraMat(s.rgb_calib_addr, rgb_camera_mat, rgb_dist_mat);
    std::cout << rgb_camera_mat << ir_camera_mat << std::endl;
    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    // runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width,
    //                                   release_object);
    cv::Mat R, T, E, F;
    stereoCalibrate(objectPoints, ir_image_pts, rgb_image_pts, 
    ir_camera_mat,ir_dist_mat, rgb_camera_mat, rgb_dist_mat, imageSize, 
    R, T, E, F);

};

//! [compute_errors]
static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}


// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr, const vector<Point3f>& newObjPoints )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "CalibrationTime" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "ValidFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    
    fs << "BoardWidth" << s.boardSize.width;
    fs << "BoardHeight" << s.boardSize.height;
    fs << "SquareSize" << s.squareSize;




    // fs << "fisheye_model" << s.useFisheye;

    fs << "CameraMat" << cameraMatrix;
    fs << "DistCoeff" << distCoeffs;
    std::vector <int>linshi_size(2);
    linshi_size[0] = imageSize.width;
    linshi_size[1] = imageSize.height;

    fs << "ImageSize" << linshi_size;
    fs << "ReprojectionError" << totalAvgErr;
    fs << "ValidImageFiles" << s.valid_imgs_lst;
    fs << "EachReprojectionError" << Mat(reprojErrs);
    
}

//! [run_and_save]
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object)
{
    // vector<Mat> rvecs, tvecs;
    // vector<float> reprojErrs;
    // double totalAvgErr = 0;
    // vector<Point3f> newObjPoints;

    // bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
    //                          totalAvgErr, newObjPoints, grid_width, release_object);
    // cout << (ok ? "Calibration succeeded" : "Calibration failed")
    //      << ". avg re projection error = " << totalAvgErr << endl;

    // if (ok)
    //     saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
    //                      totalAvgErr, newObjPoints);
    // return ok;
}
//! [run_and_save]

