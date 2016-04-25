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
@property (nonatomic, strong) UIDocumentInteractionController *controller;

@end

@implementation ViewController
{
    CGColorSpaceRef rgbSpace;
    Vector2i imageSize;
    ITMUChar4Image *resultMain;
    ITMUChar4Image *resultSide;
    
    ImageSourceEngine *imageSource;
    IMUSourceEngine *imuSource;
    ITMLibSettings *internalSettings;
    ITMBasicEngine<ITMVoxel, ITMVoxelIndex> *mainEngine;
    
    ITMIMUMeasurement *imuMeasurement;
    
    ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
    
    STSensorController *_sensorController;
    
    ITMMainEngine::GetImageType mainImageType;
    ITMMainEngine::GetImageType mainImageFreeviewType;
    
    ORUtils::SE3Pose freeviewPose;
    ITMLib::ITMIntrinsics freeviewIntrinsics;
    
    bool isDone;
    bool fullProcess;
    bool isRecording;
    bool usingSensor;
    bool freeviewActive;
    
    int currentFrameNo;
    
    NSTimeInterval totalProcessingTime;
    int totalProcessedFrames;
    
    char documentsPath[1000], *docsPath;
    
    Vector2f fingerLastTouch;
}

- (UIDocumentInteractionController *)controller {
    if (!_controller) {
        _controller = [[UIDocumentInteractionController alloc]init];
        _controller.delegate = self;

    }

    return _controller;
}

- (UIViewController *)documentInteractionControllerViewControllerForPreview:(UIDocumentInteractionController *)controller {
    return  self;
}



- (void)documentInteractionController:(UIDocumentInteractionController *)controller willBeginSendingToApplication:(NSString *)application {
    NSLog(@"Starting to send this puppy to %@", application);
}

- (void)documentInteractionController:(UIDocumentInteractionController *)controller didEndSendingToApplication:(NSString *)application {
    NSLog(@"We're done sending the document.");
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
        imuSource = NULL;
        
//        char imageSource_part1[2000], imageSource_part2[2000], imageSource_part3[2000];
//        sprintf(imageSource_part1, "%s/CAsmall/Frames/img_%%08d.ppm", documentsPath);
//        sprintf(imageSource_part2, "%s/CAsmall/Frames/img_%%08d.irw", documentsPath);
//        sprintf(imageSource_part3, "%s/CAsmall/Frames/imu_%%08d.txt", documentsPath);
//        
//        imageSource = new RawFileReader(calibFile, imageSource_part1, imageSource_part2, Vector2i(320, 240), 0.5f);
//        imuSource = new IMUSourceEngine(imageSource_part3);

        inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
        
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
//        STStreamConfig streamConfig = STStreamConfigDepth320x240;
        
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
//        imageSource = new CalibSource(calibFile, Vector2i(320, 240), 0.5f);
        imageSource = new CalibSource(calibFile, Vector2i(640, 480), 1.0f);
        
        if (error != nil) [_tbOut setText:@"from camera -- errors"];
        else [_tbOut setText:@"from camera"];
        
        inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
        
        usingSensor = true;
    }
    
    imageSize = imageSource->getDepthImageSize();
    resultMain = new ITMUChar4Image(imageSize, false);
    resultSide = new ITMUChar4Image(imageSize, false);
    rgbSpace = CGColorSpaceCreateDeviceRGB();
    
    internalSettings = new ITMLibSettings();
    mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());
    isDone = true;

    freeviewIntrinsics = imageSource->calib.intrinsics_d;
    mainImageType = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
    
    freeviewActive = false;
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
        if (imuSource != NULL)
        {
            while (imageSource->hasMoreImages()&&imuSource->hasMoreMeasurements())
            {
                imageSource->getImages(inputRGBImage, inputRawDepthImage);
                imuSource->getMeasurement(imuMeasurement);
                [self updateImage];
                
            }
            
        }
        else
        {
            while (imageSource->hasMoreImages())
            {
                imageSource->getImages(inputRGBImage, inputRawDepthImage);
                [self updateImage];
            }
        }
    });
}

- (IBAction)bGreyRenderingPressed:(id)sender {
    if (freeviewActive)
    {
        mainImageFreeviewType = ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED;
        [self refreshFreeview];
    }
    else mainImageType = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
}

- (IBAction)bConfidenceRenderingPressed:(id)sender {
    if (freeviewActive)
    {
        mainImageFreeviewType = ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE;
        [self refreshFreeview];
    }
    else mainImageType = ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE;
}

- (IBAction)bNormalsRenderingPressed:(id)sender {
    if (freeviewActive)
    {
        mainImageFreeviewType = ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL;
        [self refreshFreeview];
    }
    else mainImageType = ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL;
}

-(void)refreshFreeview
{
    CGContextRef cgContextMain, cgContextSide; CGImageRef cgImageRefMain, cgImageRefSide;
    
    NSDate *timerStart = [NSDate date];

    mainEngine->GetImage(resultMain, mainImageFreeviewType, &freeviewPose, &freeviewIntrinsics);
    
    cgContextMain = CGBitmapContextCreate(resultMain->GetData(MEMORYDEVICE_CPU), imageSize.x, imageSize.y, 8, 4 * imageSize.x, rgbSpace, kCGImageAlphaNoneSkipLast);
    cgImageRefMain = CGBitmapContextCreateImage(cgContextMain);
    
    NSDate *timerStop = [NSDate date];
    NSTimeInterval executionTime = [timerStop timeIntervalSinceDate:timerStart];
    
    self.renderView.layer.contents = (__bridge id)cgImageRefMain;
    
    NSString *theValue = [NSString stringWithFormat:@"%5.4lf", executionTime];
    [self.tbOut setText:theValue];
    
    CGImageRelease(cgImageRefMain);
    CGContextRelease(cgContextMain);
}

- (IBAction)bFreeviewRenderingPressed:(id)sender {
    freeviewActive = true;
    mainEngine->turnOffMainProcessing();
    freeviewPose.SetFrom(mainEngine->GetTrackingState()->pose_d);
    mainImageFreeviewType = ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED;
    [self refreshFreeview];
}

- (IBAction)bResetReconstructionClicked:(id)sender {
    mainEngine->resetAll();
}

- (IBAction)pinchDetected:(id)sender {
    if (freeviewActive)
    {
        UIPinchGestureRecognizer *recognizer = (UIPinchGestureRecognizer *)sender;
        
        float scale_translation = 0.5f;
        freeviewPose.SetT(freeviewPose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, 1.0f - (float)[recognizer scale]));
        
        [self refreshFreeview];
    }
}

- (void)mailComposeController:(MFMailComposeViewController*)controller didFinishWithResult:(MFMailComposeResult)result error:(NSError*)error
{
    switch (result)
    {
        case MFMailComposeResultCancelled:
            NSLog(@"Mail cancelled: you cancelled the operation and no email message was queued.");
            break;
        case MFMailComposeResultSaved:
            NSLog(@"Mail saved: you saved the email message in the drafts folder.");
            break;
        case MFMailComposeResultSent:
            NSLog(@"Mail send: the email message is queued in the outbox. It is ready to send.");
            break;
        case MFMailComposeResultFailed:
            NSLog(@"Mail failed: the email message was not saved or queued, possibly due to an error.");
            break;
        default:
            NSLog(@"Mail not sent.");
            break;
    }
    
    [controller dismissViewControllerAnimated:YES completion:nil];
}

- (IBAction)bSendMailClicked:(id)sender {
    if ([MFMailComposeViewController canSendMail])
    {
        MFMailComposeViewController *mailer = [[MFMailComposeViewController alloc] init];
        
        mailer.mailComposeDelegate = self;
        
        [mailer setSubject:@"InfiniTAM Reconstruction"];
        
        CGContextRef cgContextMain; CGImageRef cgImageRefMain;
        cgContextMain = CGBitmapContextCreate(resultMain->GetData(MEMORYDEVICE_CPU), imageSize.x, imageSize.y, 8, 4 * imageSize.x, rgbSpace, kCGImageAlphaNoneSkipLast);
        cgImageRefMain = CGBitmapContextCreateImage(cgContextMain);
        
        UIImage *myImage = [UIImage imageWithCGImage:cgImageRefMain];

        CGImageRelease(cgImageRefMain);
        CGContextRelease(cgContextMain);
        
        NSData *imageData = UIImagePNGRepresentation(myImage);
        [mailer addAttachmentData:imageData mimeType:@"image/png" fileName:@"reconstruction.png"];
        
        NSString *emailBody = @"Here is your InfiniTAM result!";
        [mailer setMessageBody:emailBody isHTML:NO];
        
        [self presentViewController:mailer animated:YES completion:nil];
    }
}

- (IBAction)bSavePhotoClicked:(id)sender {
    CGContextRef cgContextMain; CGImageRef cgImageRefMain;
    cgContextMain = CGBitmapContextCreate(resultMain->GetData(MEMORYDEVICE_CPU), imageSize.x, imageSize.y, 8, 4 * imageSize.x, rgbSpace, kCGImageAlphaNoneSkipLast);
    cgImageRefMain = CGBitmapContextCreateImage(cgContextMain);
    
    UIImage *myImage = [UIImage imageWithCGImage:cgImageRefMain];
    
    UIImageWriteToSavedPhotosAlbum(myImage, nil, nil, nil);
    
    CGImageRelease(cgImageRefMain);
    CGContextRelease(cgContextMain);
}

- (IBAction)bSendModelClicked:(id)sender {
    char modelPath[1000];
    sprintf(modelPath, "%s/model.stl", documentsPath);
    mainEngine->SaveSceneToMesh(modelPath);
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    if (freeviewActive)
    {
        UITouch *touch = [touches allObjects][0];
        if (touch.view == self.renderView)
        {
            CGPoint point = [touch locationInView:self.renderView];
            
            fingerLastTouch.x = point.x;
            fingerLastTouch.y = point.y;
        }
    }
}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{

}

static inline Matrix3f createRotation(const Vector3f & _axis, float angle)
{
    Vector3f axis = normalize(_axis);
    float si = sinf(angle);
    float co = cosf(angle);
    
    Matrix3f ret;
    ret.setIdentity();
    
    ret *= co;
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ret.at(c, r) += (1.0f - co) * axis[c] * axis[r];
    
    Matrix3f skewmat;
    skewmat.setZeros();
    skewmat.at(1, 0) = -axis.z;
    skewmat.at(0, 1) = axis.z;
    skewmat.at(2, 0) = axis.y;
    skewmat.at(0, 2) = -axis.y;
    skewmat.at(2, 1) = axis.x;
    skewmat.at(1, 2) = -axis.x;
    skewmat *= si;
    ret += skewmat;
    
    return ret;
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    if (freeviewActive && [[event allTouches]count] == 2)
    {
        NSArray<UITouch*> *allTouches = [[event allTouches] allObjects];
        
        UITouch *touch_0 = allTouches[0];
        UITouch *touch_1 = allTouches[1];
        
        CGPoint location_0 = [touch_0 locationInView:self.renderView];
        CGPoint location_1 = [touch_1 locationInView:self.renderView];
        float distance = sqrtf((location_0.x - location_1.x) * (location_0.x - location_1.x) +
                               (location_0.y - location_1.y) * (location_0.y - location_1.y));
        
        if (distance > 100.0f) return;
        
        UITouch *touch = [touches allObjects][0];
        if (touch.view == self.renderView)
        {
            CGPoint point = [touch locationInView:self.renderView];
            Vector2f movement;
            movement.x = point.x - fingerLastTouch.x;
            movement.y = point.y - fingerLastTouch.y;
            fingerLastTouch.x = point.x;
            fingerLastTouch.y = point.y;
            
            float scale_translation = 0.005f;
            freeviewPose.SetT(freeviewPose.GetT() + scale_translation * Vector3f((float)movement.x, (float)movement.y, 0.0f));
        }
        
        [self refreshFreeview];
    }
    
    if (freeviewActive && [[event allTouches]count] == 1)
    {
        UITouch *touch = [touches allObjects][0];
        if (touch.view == self.renderView)
        {
            CGPoint point = [touch locationInView:self.renderView];
            
            Vector2f movement;
            movement.x = point.x - fingerLastTouch.x;
            movement.y = point.y - fingerLastTouch.y;
            fingerLastTouch.x = point.x;
            fingerLastTouch.y = point.y;
            
            float scale_rotation = 0.005f;
            
            Vector3f axis((float)-movement.y, (float)-movement.x, 0.0f);
            float angle = scale_rotation * sqrt((float)(movement.x * movement.x + movement.y*movement.y));
            Matrix3f rot = createRotation(axis, angle);
            freeviewPose.SetR(rot * freeviewPose.GetR());
            freeviewPose.Coerce();
        }
        
        [self refreshFreeview];
    }
}

- (void) updateImage
{
    if (freeviewActive) return;
    
    if (fullProcess) mainEngine->turnOnMainProcessing();
    else mainEngine->turnOffMainProcessing();
    
    NSDate *timerStart = [NSDate date];
    
    ITMTrackingState::TrackingResult trackingResult;
    
    if (imuMeasurement != NULL)
        trackingResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, imuMeasurement);
    else
        trackingResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
    
    NSDate *timerStop = [NSDate date];
    NSTimeInterval executionTime = [timerStop timeIntervalSinceDate:timerStart];
    
    if (fullProcess)
    {
        totalProcessedFrames++;
        totalProcessingTime += executionTime;
    }
    
    CGContextRef cgContextMain, cgContextSide; CGImageRef cgImageRefMain, cgImageRefSide;
    
    if (fullProcess)
    {
        mainEngine->GetImage(resultSide, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
        mainEngine->GetImage(resultMain, mainImageType);
    }
    else
    {
        mainEngine->GetImage(resultMain, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
    }
    
    cgContextMain = CGBitmapContextCreate(resultMain->GetData(MEMORYDEVICE_CPU), imageSize.x, imageSize.y, 8, 4 * imageSize.x, rgbSpace, kCGImageAlphaNoneSkipLast);
    cgImageRefMain = CGBitmapContextCreateImage(cgContextMain);
    cgContextSide = CGBitmapContextCreate(resultSide->GetData(MEMORYDEVICE_CPU), imageSize.x, imageSize.y, 8, 4 * imageSize.x, rgbSpace, kCGImageAlphaNoneSkipLast);
    cgImageRefSide = CGBitmapContextCreateImage(cgContextSide);
    
    dispatch_sync(dispatch_get_main_queue(), ^{
        self.renderView.layer.contents = (__bridge id)cgImageRefMain;
        self.depthView.layer.contents = (__bridge id)cgImageRefSide;
        
        if (fullProcess)
        {
            if (trackingResult == ITMTrackingState::TRACKING_GOOD)
                self.renderView.backgroundColor = [UIColor greenColor];
            else
                self.renderView.backgroundColor = [UIColor redColor];
        }
        
        NSString *theValue = [NSString stringWithFormat:@"%5.4lf", totalProcessingTime / totalProcessedFrames];
        [self.tbOut setText:theValue];
    });

    CGImageRelease(cgImageRefMain);
    CGContextRelease(cgContextMain);
    CGImageRelease(cgImageRefSide);
    CGContextRelease(cgContextSide);
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
