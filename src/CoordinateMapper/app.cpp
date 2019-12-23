#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>

#include <ppl.h>
#include <direct.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>


// Choose Resolution
//#define COLOR
#define DEPTH

// Constructor
Kinect::Kinect()
{
    // Initialize
    initialize();
}

// Destructor
Kinect::~Kinect()
{
    // Finalize
    finalize();
}

// Processing
void Kinect::run()
{
	bool isCapturing = false;
	int numPhotos = 0;
	int increment = 0;
	std::string timeDate = this->getDate();

	// Start OpenCV timer variable here
	double recordSeconds = 0;
	cv::TickMeter tm;
	tm.start();

    // Main Loop
    while( true ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Key Check
        const int key = cv::waitKey( 10 );
		increment++; if (increment >= 10) increment = 0;

		// Add frame filter. increment = 10 means 10 * 10 ms. Captures in 10 fps.
		// But this is not always the case. In reality, only 6 frames are captured. So it is important to measure number of photos taken and recording time.
		if (isCapturing && increment == 0) {
			this->capture(timeDate, numPhotos);
			numPhotos++;
		}

		char c = (char)key;
        if( key == VK_ESCAPE ){
            break;
        }
		else if (c == 'c') {
			isCapturing = !isCapturing;
			if (isCapturing) {
				// Capture On
				numPhotos = 0;
				timeDate = this->getDate();
				tm.reset();
				tm.start();
				std::cout << "Recording... " << std::endl;
				recordSeconds = 0;
			}
			else {
				// Capture Off
				std::cout << "Recording finished." << std::endl;
				tm.stop();
				recordSeconds = tm.getTimeSec();

				// Write file. 
				// Recording time, number of pictures taken. Movement Range: empty
				std::cout << "Record time: " << recordSeconds << "\n";
				std::cout << "Frames taken: " << numPhotos << "\n";

				std::ostringstream ss;
				ss << "Recorder/" << timeDate << "/recordLog.yml";
				std::string fileName = ss.str();
				cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
				fs << "RecordSeconds" << recordSeconds;
				fs << "FramesTaken" << numPhotos;
				fs << "Range" << "";

				fs.release();
			}
		}
    }
}

// Initialize
void Kinect::initialize()
{
    cv::setUseOptimized( true );

    // Initialize Sensor
    initializeSensor();

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();

    // Wait a Few Seconds until begins to Retrieve Data from Sensor ( about 2000-[ms] )
    std::this_thread::sleep_for( std::chrono::seconds( 2 ) );
}

// Initialize Sensor
inline void Kinect::initializeSensor()
{
    // Open Sensor
    ERROR_CHECK( GetDefaultKinectSensor( &kinect ) );

    ERROR_CHECK( kinect->Open() );

    // Check Open
    BOOLEAN isOpen = FALSE;
    ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
    if( !isOpen ){
        throw std::runtime_error( "failed IKinectSensor::get_IsOpen( &isOpen )" );
    }

    // Retrieve Coordinate Mapper
    ERROR_CHECK( kinect->get_CoordinateMapper( &coordinateMapper ) );
}

// Initialize Color
inline void Kinect::initializeColor()
{
    // Open Color Reader
    ComPtr<IColorFrameSource> colorFrameSource;
    ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
    ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

    // Retrieve Color Description
    ComPtr<IFrameDescription> colorFrameDescription;
    ERROR_CHECK( colorFrameSource->CreateFrameDescription( ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
    ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) ); // 1920
    ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) ); // 1080
    ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) ); // 4

    // Allocation Color Buffer
    colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
}

// Initialize Depth
inline void Kinect::initializeDepth()
{
    // Open Depth Reader
    ComPtr<IDepthFrameSource> depthFrameSource;
    ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
    ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

    // Retrieve Depth Description
    ComPtr<IFrameDescription> depthFrameDescription;
    ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
    ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) ); // 512
    ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) ); // 424
    ERROR_CHECK( depthFrameDescription->get_BytesPerPixel( &depthBytesPerPixel ) ); // 2

    // Allocation Depth Buffer
    depthBuffer.resize( depthWidth * depthHeight );
}

// Finalize
void Kinect::finalize()
{
    cv::destroyAllWindows();

    // Close Sensor
    if( kinect != nullptr ){
        kinect->Close();
    }
}

// Update Data
void Kinect::update()
{
    // Update Color
    updateColor();

    // Update Depth
    updateDepth();
}

// Update Color
inline void Kinect::updateColor()
{
    // Retrieve Color Frame
    ComPtr<IColorFrame> colorFrame;
    const HRESULT ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Convert Format ( YUY2 -> BGRA )
    ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );
}

// Update Depth
inline void Kinect::updateDepth()
{
    // Retrieve Depth Frame
    ComPtr<IDepthFrame> depthFrame;
    const HRESULT ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Depth Data
    ERROR_CHECK( depthFrame->CopyFrameDataToArray( static_cast<UINT>( depthBuffer.size() ), &depthBuffer[0] ) );
}

// Draw Data
void Kinect::draw()
{
    // Draw Color
    drawColor();

    // Draw Depth
    drawDepth();
}

// Draw Color
inline void Kinect::drawColor()
{
#ifdef DEPTH
    // Retrieve Mapped Coordinates
    std::vector<ColorSpacePoint> colorSpacePoints( depthWidth * depthHeight );
    ERROR_CHECK( coordinateMapper->MapDepthFrameToColorSpace( depthBuffer.size(), &depthBuffer[0], colorSpacePoints.size(), &colorSpacePoints[0] ) );

    // Mapping Color to Depth Resolution
    std::vector<BYTE> buffer( depthWidth * depthHeight * colorBytesPerPixel );

    Concurrency::parallel_for( 0, depthHeight, [&]( const int depthY ){
        const unsigned int depthOffset = depthY * depthWidth;
        for( int depthX = 0; depthX < depthWidth; depthX++ ){
            unsigned int depthIndex = depthOffset + depthX;
            const int colorX = static_cast<int>( colorSpacePoints[depthIndex].X + 0.5f );
            const int colorY = static_cast<int>( colorSpacePoints[depthIndex].Y + 0.5f );
            if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                const unsigned int colorIndex = ( colorY * colorWidth + colorX ) * colorBytesPerPixel;
                depthIndex = depthIndex * colorBytesPerPixel;
                buffer[depthIndex + 0] = colorBuffer[colorIndex + 0];
                buffer[depthIndex + 1] = colorBuffer[colorIndex + 1];
                buffer[depthIndex + 2] = colorBuffer[colorIndex + 2];
                buffer[depthIndex + 3] = colorBuffer[colorIndex + 3];
            }
        }
    } );

    // Create cv::Mat from Coordinate Buffer
    colorMat = cv::Mat( depthHeight, depthWidth, CV_8UC4, &buffer[0] ).clone();

	// Create cv::Mat from Color Buffer
	colorMat_u = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
#else
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
#endif

}

// Draw Depth
inline void Kinect::drawDepth()
{
#ifdef COLOR
    // Retrieve Mapped Coordinates
    std::vector<DepthSpacePoint> depthSpacePoints( colorWidth * colorHeight );
    ERROR_CHECK( coordinateMapper->MapColorFrameToDepthSpace( depthBuffer.size(), &depthBuffer[0], depthSpacePoints.size(), &depthSpacePoints[0] ) );

    // Mapping Depth to Color Resolution
    std::vector<UINT16> buffer( colorWidth * colorHeight );

    Concurrency::parallel_for( 0, colorHeight, [&]( const int colorY ){
        const unsigned int colorOffset = colorY * colorWidth;
        for( int colorX = 0; colorX < colorWidth; colorX++ ){
            const unsigned int colorIndex = colorOffset + colorX;
            const int depthX = static_cast<int>( depthSpacePoints[colorIndex].X + 0.5f );
            const int depthY = static_cast<int>( depthSpacePoints[colorIndex].Y + 0.5f );
            if( ( 0 <= depthX ) && ( depthX < depthWidth ) && ( 0 <= depthY ) && ( depthY < depthHeight ) ){
                const unsigned int depthIndex = depthY * depthWidth + depthX;
                buffer[colorIndex] = depthBuffer[depthIndex];
            }
        }
    } );

    // Create cv::Mat from Coordinate Buffer
    depthMat = cv::Mat( colorHeight, colorWidth, CV_16UC1, &buffer[0] ).clone();
#else
    // Create cv::Mat from Depth Buffer
    depthMat = cv::Mat( depthHeight, depthWidth, CV_16UC1, &depthBuffer[0]);
#endif
}

// Show Data
void Kinect::show()
{
    // Show Color
    showColor();

    // Show Depth
    showDepth();
}

inline std::string Kinect::getDate() {
	std::time_t t = std::time(nullptr);
	std::ostringstream oss;
	oss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H%M%S"); // Cannot use colon as a file name
	std::string timeDate = oss.str();
	return timeDate;
}


void Kinect::capture(std::string timeDate, int numPhotos)
{
	// 2019/05/21 Problems: 
	// What is saved in the depth image is just the mask, wrong parameter to be saved.
	// Freezes at time_t
	// http://schima.hatenablog.com/entry/2014/03/06/190454
	std::cout << "Recording started.";
	float interval = 0.05;
	// Do recording command here.
	//cv::TickMeter meter;
	//int durationSec = 10000;
	//meter.start();
	int photos = 0;

	std::ostringstream ssFolder;
	ssFolder << "Recorder" << "/" << timeDate;
	std::string folderName = ssFolder.str();
	
	mkdir(folderName.c_str());
	
	std::string fileType = ".png";
	std::string number = "000";
	

	std::ostringstream ssCol;
	std::ostringstream ssCol_u;
	std::ostringstream ssDep;
	ssCol << folderName <<  "/" << "color" << "_" << threedigits(numPhotos) << fileType;
	ssCol_u << folderName << "/" << "color_unmapped" << "_" << threedigits(numPhotos) << fileType;
	ssDep << folderName <<  "/" << "depth" << "_" << threedigits(numPhotos) << fileType;

	std::string colorName = ssCol.str();
	std::string coloruName = ssCol_u.str();
	std::string depthName = ssDep.str();
	//std::cout << colorName << " and " << depthName << std::endl;

	// Save RGB image with naming colorMat
	cv::imwrite(colorName, colorMat);
	cv::imwrite(coloruName, colorMat_u);

	// Save Depth image after the data composition is converted to scaleMat
	// 0-8000 -> 8000-0
	cv::Mat scaleMat;
	depthMat.convertTo(scaleMat, CV_16U, 1, 0);
	//depthMat.convertTo(scaleMat, CV_8U, -255.0 / 8000.0, 255.0);
	cv::imwrite(depthName, scaleMat);
	photos++;
	
	// ==== Check value of depth image
	std::cout << "Size of matrix: " << scaleMat.rows << " and " << scaleMat.cols << " ";
	cv::Mat imgMat = cv::imread(depthName, CV_16U);
	/*for (int i = 0; i < 100; i++) {
		std::cout << imgMat.at<UINT16>(i, 0) << " ";
	}*/
	std::cout << "\n";

	// This is in series with streaming code. Need to do this in parallel to make it work.
	//while (true)
	//{
	//	if (colorMat.empty()) continue;

	//	std::ostringstream ssCol;
	//	std::ostringstream ssDep;
	//	ssCol << folderName << "/" << "color" << std::put_time(std::localtime(&t), "%Y-%m-%d-%H%M%S") << "_" << threedigits(photos) << fileType;
	//	ssDep << folderName << "/" << "depth" << std::put_time(std::localtime(&t), "%Y-%m-%d-%H%M%S") << "_" << threedigits(photos) << fileType;
	//	
	//	std::string colorName = ssCol.str();
	//	std::string depthName = ssDep.str();
	//	std::cout << colorName << " and " << depthName << std::endl;

	//	// Save RGB image with naming colorMat
	//	cv::imwrite(colorName, colorMat);

	//	// Save Depth image after the data composition is converted to scaleMat
	//	cv::Mat scaleMat;
	//	depthMat.convertTo(scaleMat, CV_8U, -255.0 / 8000.0, 255.0);
	//	cv::imwrite(depthName, scaleMat);
	//	photos++;

	//	char stop = (char)cv::waitKey(100); // Saves jpg in 1000/100 fps
	//	if (stop == 'c') {
	//		std::cout << "Recording stopped." << std::endl;
	//		break;
	//	}
	//}

	
}

inline std::string Kinect::threedigits(int n) {
	std::string str = "";
	if (n < 10) return str.append("00").append(std::to_string(n));
	else if (10 <= n < 100) return str.append("0").append(std::to_string(n));
	else return str.append(std::to_string(n));
}

// Show Color
inline void Kinect::showColor()
{
    if( colorMat.empty() ){
        return;
    }

#ifdef COLOR
    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Color", resizeMat );
#else
    // Show Image
    cv::imshow( "Color", colorMat );
#endif
}

// Show Depth
inline void Kinect::showDepth()
{
    if( depthMat.empty() ){
        return;
    }

    // Scaling ( 0-8000 -> 255-0 )
    cv::Mat scaleMat;
    depthMat.convertTo( scaleMat, CV_8U, -255.0 / 8000.0, 255.0 );
    //cv::applyColorMap( scaleMat, scaleMat, cv::COLORMAP_BONE );

#ifdef COLOR
    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( scaleMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Depth", resizeMat );
#else
    // Show Image
    cv::imshow( "Depth", scaleMat );
#endif
}