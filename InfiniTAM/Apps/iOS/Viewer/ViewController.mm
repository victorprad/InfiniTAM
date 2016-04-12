//
//  ViewController.m
//  InfiniTAM
//
//  Created by Victor Adrian Prisacariu on 29/10/2014.
//  Copyright (c) 2014 Victor Adrian Prisacariu. All rights reserved.
//

#import "ViewController.h"

#include "../../../ORUtils/MetalContext.h"

#include "../../../ITMLib/ITMLibDefines.h"
#include "../../../ITMLib/Core/ITMMainEngine.h"
#include "../../../ITMLib/Core/ITMBasicEngine.h"

#include "../../../InputSource/ImageSourceEngine.h"
#include "../../../InputSource/IMUSourceEngine.h"

using namespace InfiniTAM::Engine;
using namespace ITMLib;

@interface ViewController()

@property (nonatomic, strong) dispatch_queue_t renderingQueue;
@property (nonatomic, strong) MetalContext *context;

@end

@implementation ViewController
{
    CGColorSpaceRef rgbSpace;
    Vector2i imageSize;
    ITMUChar4Image *result;
    
    ImageSourceEngine *imageSource;
    IMUSourceEngine *imuSource;
    ITMLibSettings *internalSettings;
    ITMMainEngine *mainEngine;
    
    ITMIMUMeasurement *imuMeasurement;
    
    ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
    
    STSensorController *_sensorController;
    
    bool isDone;
    bool fullProcess;
    bool isRecording;
    bool usingSensor;
    
    int currentFrameNo;
    
    NSTimeInterval totalProcessingTime;
    int totalProcessedFrames;
    
    char documentsPath[1000], *docsPath;
}

- (void) viewDidLoad
{
    [super viewDidLoad];
    
    self.renderingQueue = dispatch_queue_create("rendering", DISPATCH_QUEUE_SERIAL);
    
    _sensorController = [STSensorController sharedController];
    _sensorController.delegate = self;
    
    _motionManager = [[CMMotionManager alloc]init];
    _motionManager.deviceMotionUpdateInterval = 1.0f / 60.0f;
    
    totalProcessingTime = 0;
    totalProcessedFrames = 0;
}

- (void) viewDidAppear:(BOOL)animated
{
    [self setupApp];
}

- (void) didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

- (void) setupApp
{
    isDone = false;
    fullProcess = false;
    isRecording = false;
    
    currentFrameNo = 0;
    
    self.context = [MetalContext instance];
    
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    docsPath = (char*)[[dirPaths objectAtIndex:0]cStringUsingEncoding:[NSString defaultCStringEncoding]];
    memcpy(documentsPath, docsPath, strlen(docsPath));
    
    NSError *error;
    NSString *dataPath = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:@"/Output"];
    if (![[NSFileManager defaultManager] fileExistsAtPath:dataPath])
        [[NSFileManager defaultManager] createDirectoryAtPath:dataPath withIntermediateDirectories:NO attributes:nil error:&error];
    
    STSensorControllerInitStatus resultSensor = [_sensorController initializeSensorConnection];
    
    BOOL didSucceed = (resultSensor == STSensorControllerInitStatusSuccess || resultSensor == STSensorControllerInitStatusAlreadyInitialized);
    
    if (!didSucceed)
    {
        char calibFile[2000];
        sprintf(calibFile, "%s/Teddy/calib.txt", documentsPath);
        
        fullProcess = true;
        
        char imageSource_part1[2000], imageSource_part2[2000];
        sprintf(imageSource_part1, "%s/Teddy/Frames/%%04i.ppm", documentsPath);
        sprintf(imageSource_part2, "%s/Teddy/Frames/%%04i.pgm", documentsPath);

        //TODO deallocate somewhere
        ImageMaskPathGenerator pathGenerator(imageSource_part1, imageSource_part2);
        imageSource = new ImageFileReader<ImageMaskPathGenerator>(calibFile, pathGenerator);
       
//        char imageSource_part1[2000], imageSource_part2[2000], imageSource_part3[2000];
//        sprintf(imageSource_part1, "%s/CAsmall/Frames/img_%%08d.ppm", documentsPath);
//        sprintf(imageSource_part2, "%s/CAsmall/Frames/img_%%08d.irw", documentsPath);
//        sprintf(imageSource_part3, "%s/CAsmall/Frames/imu_%%08d.txt", documentsPath);
        
//        imageSource = new RawFileReader(calibFile, imageSource_part1, imageSource_part2, Vector2i(320, 240), 0.5f);
        inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
//        imuSource = new IMUSourceEngine(imageSource_part3);
        
        [_tbOut setText:@"from file"];
        
        usingSensor = false;
        imuMeasurement = new ITMIMUMeasurement();
    }
    else
    {
        fullProcess = false;
        
        [_motionManager startDeviceMotionUpdates];
        
        imuMeasurement = new ITMIMUMeasurement();
        
        STStreamConfig streamConfig = STStreamConfigDepth640x480;
        
        NSError* error = nil;
        BOOL optionsAreValid = [_sensorController startStreamingWithOptions:@{kSTStreamConfigKey : @(streamConfig),
                                                                              kSTFrameSyncConfigKey : @(STFrameSyncOff)} error:&error];
        if (!optionsAreValid)
        {
            NSString *string = [NSString stringWithFormat:@"Error during streaming start: %s", [[error localizedDescription] UTF8String]];
            [_tbOut setText:@"from camera"];
            return;
        }
        
        const char *calibFile = [[[NSBundle mainBundle]pathForResource:@"calib" ofType:@"txt"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
        imageSource = new CalibSource(calibFile, Vector2i(640, 480), 1.0f);

        if (error != nil) [_tbOut setText:@"from camera -- errors"];
        else [_tbOut setText:@"from camera"];
        
        inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
        
        usingSensor = true;
    }
    
    imageSize = imageSource->getDepthImageSize();
    result = new ITMUChar4Image(imageSize, false);
    rgbSpace = CGColorSpaceCreateDeviceRGB();
    
    internalSettings = new ITMLibSettings();
    mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
    isDone = true;
}

- (IBAction)bProcessOne_clicked:(id)sender
{
    if (usingSensor)
    {
        isRecording = !isRecording;
        return;
    }
    
    if (!imageSource->hasMoreImages()) return;
    
    imageSource->getImages(inputRGBImage, inputRawDepthImage);
    
    dispatch_async(self.renderingQueue, ^{
        [self updateImage];
    });
}

- (IBAction)bProcessCont_clicked:(id)sender
{
    if (usingSensor)
    {
        fullProcess = true;
        return;
    }
    
    dispatch_async(self.renderingQueue, ^{
//        while (imageSource->hasMoreImages()&&imuSource->hasMoreMeasurements())
//        {
//            imageSource->getImages(inputRGBImage, inputRawDepthImage);
//            imuSource->getMeasurement(imuMeasurement);
//            [self updateImage];
//            
//        }
        while (imageSource->hasMoreImages())
        {
            imageSource->getImages(inputRGBImage, inputRawDepthImage);
            [self updateImage];
            
        }
    });
}

- (void) updateImage
{
    
//    if (fullProcess) mainEngine->turnOnMainProcessing();
//    else mainEngine->turnOffMainProcessing();
    
    NSDate *timerStart = [NSDate date];
    
    if (imuMeasurement != NULL) mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, imuMeasurement);
    else mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
    
    NSDate *timerStop = [NSDate date];
    NSTimeInterval executionTime = [timerStop timeIntervalSinceDate:timerStart];
    
    if (fullProcess)
    {
        totalProcessedFrames++;
        totalProcessingTime += executionTime;
    }
    
    if (fullProcess) mainEngine->GetImage(result, ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
    else mainEngine->GetImage(result, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
    
    CGContextRef cgContext = CGBitmapContextCreate(result->GetData(MEMORYDEVICE_CPU), imageSize.x, imageSize.y, 8,
                                                   4 * imageSize.x, rgbSpace, kCGImageAlphaNoneSkipLast);
    CGImageRef cgImageRef = CGBitmapContextCreateImage(cgContext);
    
    dispatch_sync(dispatch_get_main_queue(), ^{
        self.renderView.layer.contents = (__bridge id)cgImageRef;
        
        NSString *theValue = [NSString stringWithFormat:@"%5.4lf", totalProcessingTime / totalProcessedFrames];
        [self.tbOut setText:theValue];
    });

    CGImageRelease(cgImageRef);
    CGContextRelease(cgContext);
}

- (void)sensorDidDisconnect
{
    [self.tbOut setText:@"disconnected "];
}

- (void)sensorDidConnect
{
}

- (void)sensorDidLeaveLowPowerMode
{
}

- (void)sensorBatteryNeedsCharging
{
}

- (void)sensorDidStopStreaming:(STSensorControllerDidStopStreamingReason)reason
{
    [self.tbOut setText:@"stopped streaming"];
}

-(void) sensorDidOutputSynchronizedDepthFrame:(STDepthFrame *)depthFrame andColorBuffer:(CMSampleBufferRef)sampleBuffer
{
    [self.tbOut setText:@"got frame c"];
}

- (void)sensorDidOutputDepthFrame:(STDepthFrame *)depthFrame
{
    if (isDone)
    {
        isDone = false;
        
        CMRotationMatrix rotationMatrix = self.motionManager.deviceMotion.attitude.rotationMatrix;
        
        if (imuMeasurement != NULL)
        {
            imuMeasurement->R.m00 = rotationMatrix.m11; imuMeasurement->R.m01 = rotationMatrix.m12; imuMeasurement->R.m02 = rotationMatrix.m13;
            imuMeasurement->R.m10 = rotationMatrix.m21; imuMeasurement->R.m11 = rotationMatrix.m22; imuMeasurement->R.m12 = rotationMatrix.m23;
            imuMeasurement->R.m20 = rotationMatrix.m31; imuMeasurement->R.m21 = rotationMatrix.m32; imuMeasurement->R.m22 = rotationMatrix.m33;
        }
        
        memcpy(inputRawDepthImage->GetData(MEMORYDEVICE_CPU), [depthFrame shiftData], imageSize.x * imageSize.y * sizeof(short));
        
        dispatch_async(self.renderingQueue, ^{
            if (isRecording)
            {
                FILE *f; char fileName[2000];
                
                sprintf(fileName, "%s/Output/img_%08d.irw", documentsPath, currentFrameNo);
                f = fopen(fileName, "wb+");
                fwrite(inputRawDepthImage->GetData(MEMORYDEVICE_CPU), imageSize.x * imageSize.y * sizeof(short), 1, f);
                fclose(f);
                
                sprintf(fileName, "%s/Output/imu_%08d.txt", documentsPath, currentFrameNo);
                f = fopen(fileName, "w+");
                fprintf(f, "%f %f %f %f %f %f %f %f %f",
                        rotationMatrix.m11, rotationMatrix.m12, rotationMatrix.m13,
                        rotationMatrix.m21, rotationMatrix.m22, rotationMatrix.m23,
                        rotationMatrix.m31, rotationMatrix.m32, rotationMatrix.m33);
                
                fclose(f);
                
                currentFrameNo++;
            }
            
            [self updateImage];
            
            isDone = true;
        });
    }
}

@end
