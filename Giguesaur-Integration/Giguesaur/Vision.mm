//
//  Vision.m
//  Giguesaur
//
//  Created by Local Joshua La Pine on 7/23/15.
//  Modified by Ashley Manson
//  Copyright (c) 2015 Giguesaur Team. All rights reserved.
//

#import "Vision.h"

cv::Size boardSize(9,6);
std::vector<cv::Point3f> corners;
cv::Mat cameraMatrix, distCoeffs;
cv::Mat frame; // the puzzle image, who named it input?
BOOL puzzleImageCopied = NO;
std::vector<cv::Point2f> imagePlane;
std::vector<cv::Point3f> polypoints;
GLKMatrix4 modelView;


static const unsigned char UNVISITED = 0;
static const unsigned char VISITED = 1;
static const unsigned char MARKED = 2;
static const unsigned char BLACK = 3;
static const unsigned char TEMP = 4;

static const int array_size = 10000;
static const unsigned char thresh = 120;

struct fill_queue{
    cv::Point2i queue[array_size];
    int head;
    int tail;
};

struct fill_data{
    int white_pixels;
    //made it non static, beware memory leak?
    std::vector<cv::Point2i> black_list;
    //might need to make min
    cv::Point2i max_x, max_y, min_x, min_y, max_x_p_y, min_x_m_y, min_x_p_y, max_x_m_y;
};

cv::Mat table;

fill_data max_white;
fill_data temp_fill;

@implementation Vision

@synthesize videoCaptureDevice;
@synthesize videoDataOutput;
@synthesize session;

- (void) visionInit:(Graphics *) graphics{

    // This needs to be replaced with the UIImage held in Graphics
    NSBundle *mainBundle = [NSBundle mainBundle];
    NSString *filePath = [mainBundle pathForResource: @"puppy" ofType: @"png"];
    UIImage* resImage = [UIImage imageWithContentsOfFile:filePath];

    //UIImage* resImage = self.graphics.puzzleImage;
    frame = [self cvMatFromUIImage:resImage];


    self.graphics = graphics;

    for( int i = 0; i < boardSize.height; ++i ){
        for( int j = 0; j < boardSize.width; ++j ){
            corners.push_back(cv::Point3f(float(j*28), float(i*28), 0));
        }
    }

    NSString *myFile = [mainBundle pathForResource: @"0.7params" ofType: @"xml"];
    //std::string *path = new std::string([myFile UTF8String]);
    const char *path = [myFile UTF8String];

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "File io is not working" << std::endl;

    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs.release();

    session = [[AVCaptureSession alloc]init];
    session.sessionPreset = AVCaptureSessionPresetHigh;
    videoCaptureDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];

    [videoCaptureDevice lockForConfiguration:nil];
    [videoCaptureDevice setVideoZoomFactor:1];
    [videoCaptureDevice setFocusMode:AVCaptureFocusModeLocked];
    [videoCaptureDevice unlockForConfiguration];

    NSError *error = nil;
    AVCaptureDeviceInput *videoInput = [AVCaptureDeviceInput deviceInputWithDevice:videoCaptureDevice error: &error];
    if(videoInput){
        [session addInput:videoInput];
    } else {
        NSLog(@"For some reason the device has no camera?");
    }

    videoDataOutput = [AVCaptureVideoDataOutput new];
    NSDictionary *newSettings =
    @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    videoDataOutput.videoSettings = newSettings;
    [videoDataOutput setAlwaysDiscardsLateVideoFrames:YES];

    dispatch_queue_t queue = dispatch_queue_create("MyQueue", NULL);
    [videoDataOutput setSampleBufferDelegate:self queue:queue];

    [session addOutput:videoDataOutput];

    AVCaptureVideoPreviewLayer *preview = [AVCaptureVideoPreviewLayer layerWithSession:session];
    preview.frame = self.graphics.bounds;
    preview.videoGravity = AVLayerVideoGravityResizeAspectFill;
    preview.hidden = YES;
}

// Convert from screen coordinates to world coordinates
- (CGPoint) projectedPoints: (CGPoint) screenCoords {

    double s_x = screenCoords.x*1.875; // 1920 / 1024
    double s_y = screenCoords.y*1.40625; // 1080 / 768

    cv::Mat rvec, tvec, rotationMatrix;
    cv::solvePnP(corners, imagePlane, cameraMatrix, distCoeffs, rvec, tvec, false);
    cv::Rodrigues(rvec,rotationMatrix);
    cv::Mat uvPoint = cv::Mat::ones(3,1,cv::DataType<double>::type); // u,v,1
    // image point
    uvPoint.at<double>(0,0) = s_x;
    uvPoint.at<double>(1,0) = s_y;

    cv::Mat tempMat, tempMat2;
    double s;
    tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
    tempMat2 = rotationMatrix.inv() * tvec;
    s = PIECE_Z + tempMat2.at<double>(2,0);
    s /= tempMat.at<double>(2,0);
    cv::Mat wcPoint = rotationMatrix.inv() * (s * cameraMatrix.inv() * uvPoint - tvec);

    cv::Point3f realPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0)); // point in world coordinates

    return CGPointMake(wcPoint.at<double>(0,0), wcPoint.at<double>(1, 0));
}

cv::Point2i flood_fill(cv::Mat image, unsigned char lower, unsigned char upper, cv::Point2i seed, int skip, bool white_flag){
    
    unsigned char value, state;
    cv::Point2i sum = cv::Point2i(0,0);
    int count = 0;
    fill_queue circ_queue;
    circ_queue.head = 0;
    circ_queue.tail = 0;
    
    circ_queue.queue[circ_queue.tail] = seed;
    circ_queue.tail = (circ_queue.tail + 1) % array_size;
    
    while(circ_queue.head != circ_queue.tail){
        //std::cout << "in queue while" << std::endl;
        seed = circ_queue.queue[circ_queue.head];
        circ_queue.head = (circ_queue.head + 1) % array_size;
        
        //changed values in backtrack loop to skip
        value = image.at<uchar>(seed.x, seed.y-skip);
        while(value >= lower && value < upper){
            //std::cout << "in backtrack loop" << std::endl;
            seed.y-=skip;
            value = image.at<uchar>(seed.x, seed.y);
        }
        
        //might flip out if white near edges, UPDATE: Now start scanning from skip.
        value = image.at<uchar>(seed.x-skip, seed.y);
        state = table.at<uchar>(seed.x-skip, seed.y);
        if(value >= lower && value < upper
           && (state == UNVISITED || state == TEMP)){
            circ_queue.queue[circ_queue.tail] = cv::Point2i(seed.x-skip, seed.y);
            circ_queue.tail = (circ_queue.tail + 1) % array_size;
            table.at<uchar>(seed.x-skip, seed.y) = MARKED;
        }
        
        value = image.at<uchar>(seed.x+skip, seed.y);
        state = table.at<uchar>(seed.x+skip, seed.y);
        if(value >= lower && value < upper &&
           (state == UNVISITED ||  state == TEMP)){
            circ_queue.queue[circ_queue.tail] = cv::Point2i(seed.x+skip, seed.y);
            circ_queue.tail = (circ_queue.tail + 1) % array_size;
            table.at<uchar>(seed.x+skip, seed.y) = MARKED;
        }
        
        bool mark_flag_up = false;
        bool mark_flag_down = false;
        bool run_start = true;
        do{
            //std::cout << "in do" << std::endl;
            table.at<uchar>(seed.x, seed.y) = VISITED;
            
            
            if(!white_flag){
                sum.x += seed.x;
                sum.y += seed.y;
                count++;
            }
            
            else{
                temp_fill.white_pixels++;
                //input.at<cv::Vec3b>(seed.x,seed.y) = cv::Vec3b(0,0,255);
                value = image.at<uchar>(seed.x, seed.y+skip);
                if(run_start || !(value >= lower && value < upper)){
                    run_start = false;
                    if(seed.x < temp_fill.min_x.x){
                        temp_fill.min_x.x = seed.x;
                        temp_fill.min_x.y = seed.y;
                    }
                    if(seed.y < temp_fill.min_y.y){
                        temp_fill.min_y.y = seed.y;
                        temp_fill.min_y.x = seed.x;
                    }
                    if(seed.x > temp_fill.max_x.x){
                        temp_fill.max_x.x = seed.x;
                        temp_fill.max_x.y = seed.y;
                    }
                    if(seed.y > temp_fill.max_y.y){
                        temp_fill.max_y.y = seed.y;
                        temp_fill.max_y.x = seed.x;
                    }
                    
                    //might need to change x and y around as they are non cartesian. Probably makes a difference.
                    if((seed.x + seed.y) > (temp_fill.max_x_p_y.x + temp_fill.max_x_p_y.y)){
                        temp_fill.max_x_p_y.x = seed.x;
                        temp_fill.max_x_p_y.y = seed.y;
                    }
                    if((seed.x - seed.y) < (temp_fill.min_x_m_y.x - temp_fill.min_x_m_y.y)){
                        temp_fill.min_x_m_y.x = seed.x;
                        temp_fill.min_x_m_y.y = seed.y;
                    }
                    if((seed.x + seed.y) < (temp_fill.min_x_p_y.x + temp_fill.min_x_p_y.y)){
                        temp_fill.min_x_p_y.x = seed.x;
                        temp_fill.min_x_p_y.y = seed.y;
                    }
                    if((seed.x - seed.y) > (temp_fill.max_x_m_y.x - temp_fill.max_x_m_y.y)){
                        temp_fill.max_x_m_y.x = seed.x;
                        temp_fill.max_x_m_y.y = seed.y;
                    }
                    
                    //might not be necessary
                    if(!(value >= lower && value < upper)){
                        table.at<uchar>(seed.x, seed.y+skip) = BLACK;
                        temp_fill.black_list.push_back(cv::Point2i(seed.x, seed.y+skip));
                    }
                }
                
            }
            
            
            value = image.at<uchar>(seed.x-skip, seed.y);
            state = table.at<uchar>(seed.x-skip, seed.y);
            if(mark_flag_up && value >= lower && value < upper
               && (state == UNVISITED || state == TEMP )){
                
                circ_queue.queue[circ_queue.tail] = cv::Point2i(seed.x-skip, seed.y);
                circ_queue.tail = (circ_queue.tail + 1) % array_size;
                table.at<uchar>(seed.x-skip, seed.y) = MARKED;
                mark_flag_up = false;
            }
            
            else if(!(value >= lower && value < upper)){
                mark_flag_up = true;
            }
            
            value = image.at<uchar>(seed.x+skip, seed.y);
            state = table.at<uchar>(seed.x+skip, seed.y);
            if(mark_flag_down && value >= lower && value < upper
               && (state == UNVISITED || state == TEMP)){
                
                circ_queue.queue[circ_queue.tail] = cv::Point2i(seed.x+skip, seed.y);
                circ_queue.tail = (circ_queue.tail + 1) % array_size;
                table.at<uchar>(seed.x+skip, seed.y) = MARKED;
                mark_flag_down = false;
            }
            
            else if(!(value >= lower && value < upper)){
                mark_flag_down = true;
            }
            /*std::cout << seed.x << " " << seed.y << "\n"
             << temp_fill.black_list.size() << "\n" <<
             circ_queue.head << " " << circ_queue.tail << std::endl;*/
            seed.y += skip;
            //&& seed.x < 1080 && seed.y < 1920
        }while(image.at<uchar>(seed.x, seed.y) >= lower && image.at<uchar>(seed.x, seed.y) < upper );
    }
    if(!white_flag){
        sum.x /= count;
        sum.y /= count;
        return sum;
    }
    
    return cv::Point2i(-1,-1);
}

 void calculateCharacteristics(cv::Mat &frame){
    
    int regularChar[4];
    int rotatedChar[4];
    int a, b;
    int regMin = 4000;
    int rotMin = 4000;
    
    //regular characteristics
    a = abs(max_white.min_x.x - max_white.min_y.x);
    b = abs(max_white.min_x.y - max_white.min_y.y);
    regularChar[0] = a < b ? a : b;
    regMin = regularChar[0] < regMin ? regularChar[0] : regMin;
    
    a = abs(max_white.max_y.x - max_white.min_x.x);
    b = abs(max_white.max_y.y - max_white.min_x.y);
    regularChar[1] = a < b ? a : b;
    regMin = regularChar[1] < regMin ? regularChar[1] : regMin;
    
    a = abs(max_white.max_x.x - max_white.max_y.x);
    b = abs(max_white.max_x.y - max_white.max_y.y);
    regularChar[2] = a < b ? a : b;
    regMin = regularChar[2] < regMin ? regularChar[2] : regMin;
    
    a = abs(max_white.min_y.x - max_white.max_x.x);
    b = abs(max_white.min_y.y - max_white.max_x.y);
    regularChar[3] = a < b ? a : b;
    regMin = regularChar[3] < regMin ? regularChar[3] : regMin;
    
    //rotated characteristics
    a = abs(max_white.min_x_p_y.x - max_white.min_x_m_y.x);
    b = abs(max_white.min_x_p_y.y - max_white.min_x_m_y.y);
    rotatedChar[0] = a < b ? a : b;
    rotMin = rotatedChar[0] < rotMin ? rotatedChar[0] : rotMin;
    
    a = abs(max_white.max_x_m_y.x - max_white.min_x_p_y.x);
    b = abs(max_white.max_x_m_y.y - max_white.min_x_p_y.y);
    rotatedChar[1] = a < b ? a : b;
    rotMin = rotatedChar[1] < rotMin ? rotatedChar[1] : rotMin;
    
    a = abs(max_white.max_x_p_y.x - max_white.max_x_m_y.x);
    b = abs(max_white.max_x_p_y.y - max_white.max_x_m_y.y);
    rotatedChar[2] = a < b ? a : b;
    rotMin = rotatedChar[2] < rotMin ? rotatedChar[2] : rotMin;
    
    a = abs(max_white.min_x_m_y.x - max_white.max_x_p_y.x);
    b = abs(max_white.min_x_m_y.y - max_white.max_x_p_y.y);
    rotatedChar[3] = a < b ? a : b;
    rotMin = rotatedChar[3] < rotMin ? rotatedChar[3] : rotMin;
     
     std::cout << "reg " << std::endl;
     for(int i = 0; i < 4; i ++){
         std::cout << regularChar[i] <<  " ";
     }
     
     std::cout << "\nrot " << std::endl;
     for(int i = 0; i < 4; i ++){
         std::cout << rotatedChar[i] << " ";
     }
     
     
    
    rotMin *= 1/sqrt(2);
     std::cout << "\nrotMin = " << rotMin << "\nregMin " << regMin << std::endl;
     
    
    cv::Point2i corners[8];
   // if(regMin < rotMin){
        corners[0] = max_white.max_x_p_y;
        corners[1] = max_white.min_x_m_y;
        corners[2] = max_white.min_x_p_y;
        corners[3] = max_white.max_x_m_y;
     
     corners[4] = max_white.max_y;
     corners[5] = max_white.min_x;
     corners[6] = max_white.min_y;
     corners[7] = max_white.max_x;
   // }
    
   /* else {
        corners[0] = max_white.max_y;
        corners[1] = max_white.min_x;
        corners[2] = max_white.min_y;
        corners[3] = max_white.max_x;
    }*/
    
    cv::Point corner0(corners[0].y, corners[0].x);
    cv::Point corner1(corners[1].y, corners[1].x);
    cv::Point corner2(corners[2].y, corners[2].x);
    cv::Point corner3(corners[3].y, corners[3].x);
     
     cv::Point corner4(corners[4].y, corners[4].x);
     cv::Point corner5(corners[5].y, corners[5].x);
     cv::Point corner6(corners[6].y, corners[6].x);
     cv::Point corner7(corners[7].y, corners[7].x);
    circle(frame, corner0, 8, cv::Scalar(0,255,0), -1, 8, 0);
    circle(frame, corner1, 8, cv::Scalar(0,0,255), -1, 8, 0);
    circle(frame, corner2, 8, cv::Scalar(255,255,0), -1, 8, 0);
    circle(frame, corner3, 8, cv::Scalar(255,0,0), -1, 8, 0);
     
     circle(frame, corner4, 8, cv::Scalar(0,0,0), -1, 8, 0);
     circle(frame, corner5, 8, cv::Scalar(0,0,0), -1, 8, 0);
     circle(frame, corner6, 8, cv::Scalar(0,0,0), -1, 8, 0);
     circle(frame, corner7, 8, cv::Scalar(0,0,0), -1, 8, 0);
    
}

- (void) calculatePose:(cv::Mat &)frame{

    if (self.graphics.puzzleStateRecieved && !puzzleImageCopied) {
        frame = [self cvMatFromUIImage:self.graphics.puzzleImage];
        puzzleImageCopied = YES;
    }

    std::vector<cv::Point2f> imagepoints;
    std::vector<cv::Point2f> pixelcorners;
    std::vector<cv::Point3f> worldpieces;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat rotation;
    int width = frame.cols;
    int height = frame.rows;
    bool vectors = false;
    /*bool patternfound = findChessboardCorners(frame, boardSize, pixelcorners,
                                              cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                                              + cv::CALIB_CB_FAST_CHECK);*/
    /*******************************/
    bool patternfound = false;
    cv::Mat image;
    static const int skip = 10;
    cv::cvtColor(frame, image, CV_BGR2GRAY);
    table = cv::Mat::zeros(image.rows,image.cols, CV_8UC1);
    //cv::imwrite("output.png", image);
    max_white.white_pixels = 0;
    max_white.max_x = cv::Point2i(0,0);
    max_white.max_y = cv::Point2i(0,0);
    max_white.min_x = cv::Point2i(4000,4000);
    max_white.min_y = cv::Point2i(4000,4000);
    max_white.max_x_p_y = cv::Point2i(0,0);
    max_white.min_x_m_y = cv::Point2i(4000,4000);
    max_white.min_x_p_y = cv::Point2i(4000,4000);
    max_white.max_x_m_y = cv::Point2i(0,0);
    //POTENTIALLY CHOSE BAD INITIAL VALUES FOR ROTATED COORDINATE SYSTEM
    temp_fill.white_pixels = 0;
    temp_fill.max_x = cv::Point2i(0,0);
    temp_fill.max_y = cv::Point2i(0,0);
    temp_fill.min_x = cv::Point2i(4000,4000);
    temp_fill.min_y = cv::Point2i(4000,4000);
    temp_fill.max_x_p_y = cv::Point2i(0,0);
    temp_fill.min_x_m_y = cv::Point2i(4000,4000);
    temp_fill.min_x_p_y = cv::Point2i(4000,4000);
    temp_fill.max_x_m_y = cv::Point2i(0,0);
    
    int count = 0;
    //clock_t startTime = clock();
    for(int i = skip; i < image.rows-skip; i+=skip){
        for(int j = skip; j < image.cols-skip; j+=skip){
            if(image.at<uchar>(i,j) >= thresh && table.at<uchar>(i,j) == UNVISITED){
                //std::cout << "i = " << i << " j = " << j << std::endl;
                //issue with 255 value, less than in algorithm
                // std::cout << (int)image.at<uchar>(i,j) << std::endl;
                count++;
                //std::cout << i << " " << j << std::endl;
                //cv::Point centre(j,i);
                // circle(input,centre,5,cv::Scalar(255,0,0),-1,8,0);
                flood_fill(image, 140, 255, cv::Point2i(i,j), skip, true);
                if(temp_fill.white_pixels > max_white.white_pixels){
                    max_white = temp_fill;
                }
                temp_fill.white_pixels = 0;
                temp_fill.black_list.clear();
                temp_fill.max_x = cv::Point2i(0,0);
                temp_fill.max_y = cv::Point2i(0,0);
                temp_fill.min_x = cv::Point2i(4000,4000);
                temp_fill.min_y = cv::Point2i(4000,4000);
                temp_fill.max_x_p_y = cv::Point2i(0,0);
                temp_fill.min_x_m_y = cv::Point2i(4000,4000);
                temp_fill.min_x_p_y = cv::Point2i(4000,4000);
                temp_fill.max_x_m_y = cv::Point2i(0,0);
            }
        }
    }
    //std::cout << count << std::endl;
    //std::cout << double(clock() - startTime) / (double) CLOCKS_PER_SEC << std::endl;
    //calculate dot products, no point in doing it more than once.
    
    //this might not be neccesary anymore
    /*
    for(int i = 0; i < table.rows; i++){
        for(int j = 0; j < table.cols; j++){
            unsigned char state = table.at<uchar>(i,j);
            if(state == VISITED){
                table.at<uchar>(i,j) = BLACK;
            } else if(state == BLACK){
                table.at<uchar>(i,j) = TEMP;
            }
        }
    }*/
    
    /***********************/
    /*  std::vector<cv::Point2i>dots;
     for(int i = 0; i < max_white.black_list.size(); i++){
     std::cout << "allg" << std::endl;
     cv::Point seed = max_white.black_list[i];
     std::cout << "still allg" << seed << std::endl;
     if(table.at<uchar>(seed) == TEMP){
     std::cout << table.at<uchar>(seed) << std::endl;
     cv::Point2i dot = flood_fill(image, 0, 120, seed, 1, false);
     if(dot.x > -1)
     dots.push_back(dot);
     }
     }
     // std::cout << max_white.white_pixels << std::endl;
     std::cout << dots << std::endl;
     for(int i = 0; i < dots.size(); i++){
     cv::Point centre(dots[i].y, dots[i].x);
     circle(input, centre, 5, cv::Scalar(255,255,255), -1, 8, 0);
     }*/
    /**************************/
    cv::Point centre((max_white.max_y.y + max_white.min_y.y) / 2, (max_white.max_x.x + max_white.min_x.x) /2);
    circle(frame,centre,3,cv::Scalar(255,0,0), -1,8,0);
    /*
    cv::Point max_x(max_white.max_x.y, max_white.max_x.x);
    cv::Point max_y(max_white.max_y.y, max_white.max_y.x);
    cv::Point min_x(max_white.min_x.y, max_white.min_x.x);
    cv::Point min_y(max_white.min_y.y, max_white.min_y.x);
    circle(frame, max_x, 8, cv::Scalar(0,255,0), -1, 8, 0);
    circle(frame, max_y, 8, cv::Scalar(0,0,255), -1, 8, 0);
    circle(frame, min_x, 8, cv::Scalar(255,255,0), -1, 8, 0);
    circle(frame, min_y, 8, cv::Scalar(255,0,0), -1, 8, 0);
    */
    /*std::cout <<"Green = Max x/row " << max_white.max_x <<
    "\nRed = max y/column " << max_white.max_y <<
    "\nCyan = min x/row " << max_white.min_x <<
    "\nBlue = min y/column " << max_white.min_y << std::endl;

    */
    
    calculateCharacteristics(frame);
    
    /****************************************/
    SimpleMath *simpleMath = [[SimpleMath alloc] init];
    PieceCoords pieceCoords[self.graphics.num_of_pieces][4]; // 4 = number of corners
    int num_pieces_draw = 0;

    for (int i = 0; i < self.graphics.num_of_pieces; i++) {
        // set row and col to get the sub-section of the texture
        int row = 0;
        int col = 0;
        int index = 0;
        while (index != i) {
            col++;
            index++;
            if (col >= self.graphics.puzzle_cols) {
                col = 0;
                row++;
            }
        }

        Piece tempPiece = self.graphics.pieces[i];
        NSArray *rotatedPiece = [simpleMath pointsRotated:tempPiece];
        CGPoint topLeft = [[rotatedPiece objectAtIndex:0] CGPointValue];
        CGPoint topRight = [[rotatedPiece objectAtIndex:1] CGPointValue];
        CGPoint botRight = [[rotatedPiece objectAtIndex:2] CGPointValue];
        CGPoint botLeft = [[rotatedPiece objectAtIndex:3] CGPointValue];

        pieceCoords[i][0] = (PieceCoords) {
            {static_cast<float>(botLeft.x), static_cast<float>(botLeft.y), PIECE_Z},
            //{tempPiece.x_location-SIDE_HALF, tempPiece.y_location-SIDE_HALF, PIECE_Z},
            {self.graphics.texture_width * col, self.graphics.texture_height * row}
        };
        pieceCoords[i][1] = (PieceCoords) {
            {static_cast<float>(botRight.x), static_cast<float>(botRight.y), PIECE_Z},
            //{tempPiece.x_location+SIDE_HALF, tempPiece.y_location-SIDE_HALF, PIECE_Z},
            {self.graphics.texture_width * (col + 1), self.graphics.texture_height * row}
        };
        pieceCoords[i][2] = (PieceCoords) {
            {static_cast<float>(topRight.x), static_cast<float>(topRight.y), PIECE_Z},
            //{tempPiece.x_location+SIDE_HALF, tempPiece.y_location+SIDE_HALF, PIECE_Z},
            {self.graphics.texture_width * (col + 1), self.graphics.texture_height * (row + 1)}
        };
        pieceCoords[i][3] = (PieceCoords) {
            {static_cast<float>(topLeft.x), static_cast<float>(topLeft.y), PIECE_Z},
            //{tempPiece.x_location-SIDE_HALF, tempPiece.y_location+SIDE_HALF, PIECE_Z},
            {self.graphics.texture_width * col, self.graphics.texture_height * (row + 1)}
        };

        num_pieces_draw++;
        for(int j = 0; j < 4; j++){
            float x, y, z;
            x = pieceCoords[i][j].Position[0];
            y = pieceCoords[i][j].Position[1];
            if (!self.graphics.pieces[i].held || self.graphics.holdingPiece == i)
                z = pieceCoords[i][j].Position[2];
            else
                z = pieceCoords[i][j].Position[2]-1000000;

            worldpieces.push_back(cv::Point3f(x,y,z));
        }
    }

    if (patternfound) {
        imagePlane = pixelcorners;
        vectors = solvePnP(corners, pixelcorners, cameraMatrix, distCoeffs, rvec, tvec, false);
        //cv::drawChessboardCorners(frame, boardSize, pixelcorners, patternfound);
    }

    if (vectors) {
        
        for (int piece = 0; piece < num_pieces_draw; piece++) {
            std::vector<cv::Point2f> imagePiece;
            std::vector<cv::Point3f> worldPiece;
            int corner = piece * 4;

            worldPiece.push_back(worldpieces.at(corner));
            worldPiece.push_back(worldpieces.at(corner+1));
            worldPiece.push_back(worldpieces.at(corner+2));
            worldPiece.push_back(worldpieces.at(corner+3));

            cv::projectPoints(worldPiece, rvec, tvec, cameraMatrix, distCoeffs, imagePiece);

            imagepoints.push_back(imagePiece.at(0));
            imagepoints.push_back(imagePiece.at(1));
            imagepoints.push_back(imagePiece.at(2));
            imagepoints.push_back(imagePiece.at(3));

            cv::Mat lambda(3,3, CV_32FC1);

            cv::Point2f inputQuad[4];
            cv::Point2f outputQuad[4];

            cv::Point2f tempQuad = cv::Point2f(pieceCoords[piece][0].TexCoord[0]*width, pieceCoords[piece][0].TexCoord[1]*height);
            cv::Rect crop(tempQuad.x, tempQuad.y, width*self.graphics.texture_width, height*self.graphics.texture_height);
            cv::Mat subImage = frame(crop);

            inputQuad[0] = cv::Point2f(0,0);
            inputQuad[1] = cv::Point2f(subImage.cols-1, 0);
            inputQuad[2] = cv::Point2f(subImage.cols-1, subImage.rows-1);
            inputQuad[3] = cv::Point2f(0, subImage.rows-1);
            
            outputQuad[0] = imagepoints[corner];
            outputQuad[1] = imagepoints[corner+1];
            outputQuad[2] = imagepoints[corner+2];
            outputQuad[3] = imagepoints[corner+3];

            lambda = cv::getPerspectiveTransform(inputQuad, outputQuad);
            cv::Mat output = cv::Mat::zeros(frame.size(), frame.type());
            cv::warpPerspective(subImage, output, lambda, output.size(), CV_INTER_LINEAR);

            output.copyTo(frame,output);
        }
    }

    @autoreleasepool {

        cv::cvtColor(frame, frame, CV_BGRA2RGBA);

        /* cv::Rect crop(240,0,1440,1080);
         frame = frame(crop);
         std::cout << frame.cols << " " << frame.rows << std::endl;*/
        UIImage *image = [self UIImageFromCVMat:frame];

        dispatch_async(dispatch_get_main_queue(), ^{
            [[self graphics] setupTextureImage:image];
        });
    }
    rvec.release();
    tvec.release();
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
       fromConnection:(AVCaptureConnection *)connection {

    @autoreleasepool {

        cv::Mat frame;
        [self fromSampleBuffer:sampleBuffer toCVMat: frame];
        [self calculatePose:frame];
        frame.release();
    }
}

- (void)fromSampleBuffer:(CMSampleBufferRef)sampleBuffer
                 toCVMat:(cv::Mat &)mat{

    CVImageBufferRef imgBuf = CMSampleBufferGetImageBuffer(sampleBuffer);

    // lock the buffer
    CVPixelBufferLockBaseAddress(imgBuf, 0);

    // get the address to the image data
    void *imgBufAddr = CVPixelBufferGetBaseAddress(imgBuf);

    // get image properties
    int w = (int)CVPixelBufferGetWidth(imgBuf);
    int h = (int)CVPixelBufferGetHeight(imgBuf);
    int bytesPerRow = (int)CVPixelBufferGetBytesPerRow(imgBuf);
    int size = h * bytesPerRow;


    // create the cv mat
    mat.create(h, w, CV_8UC4);
    memcpy(mat.data, imgBufAddr, size);

    // unlock again
    CVPixelBufferUnlockBaseAddress(imgBuf, 0);

}

-(UIImage *)UIImageFromCVMat:(cv::Mat)cvMat
{
    NSData *data = [NSData dataWithBytes:cvMat.data length:cvMat.elemSize()*cvMat.total()];
    CGColorSpaceRef colorSpace;

    if (cvMat.elemSize() == 1) {
        colorSpace = CGColorSpaceCreateDeviceGray();
    } else {
        colorSpace = CGColorSpaceCreateDeviceRGB();
    }

    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);

    // Creating CGImage from cv::Mat
    CGImageRef imageRef = CGImageCreate(cvMat.cols,                                 //width
                                        cvMat.rows,                                 //height
                                        8,                                          //bits per component
                                        8 * cvMat.elemSize(),                       //bits per pixel
                                        cvMat.step[0],                            //bytesPerRow
                                        colorSpace,                                 //colorspace
                                        kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                        provider,                                   //CGDataProviderRef
                                        NULL,                                       //decode
                                        false,                                      //should interpolate
                                        kCGRenderingIntentDefault                   //intent
                                        );


    // Getting UIImage from CGImage
    UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);

    return finalImage;
}

- (cv::Mat)cvMatFromUIImage:(UIImage *)image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;

    cv::Mat cvMat(rows, cols, CV_8UC4); // 8 bits per component, 4 channels (color channels + alpha)

    CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to  data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[0],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // Bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
    CGContextRelease(contextRef);
    
    return cvMat;
}

@end

