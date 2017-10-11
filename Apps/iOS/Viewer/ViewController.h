//
//  ViewController.h
//  InfiniTAM
//
//  Created by Victor Adrian Prisacariu on 29/10/2014.
//  Copyright (c) 2014 Victor Adrian Prisacariu. All rights reserved.
//

#import <UIKit/UIKit.h>
#define HAS_LIBCXX
#import <Structure/Structure.h>
#import <CoreMotion/CoreMotion.h>
#import <MessageUI/MessageUI.h>

@interface ViewController : UIViewController <STSensorControllerDelegate, MFMailComposeViewControllerDelegate, UIDocumentInteractionControllerDelegate>

@property (weak, nonatomic) IBOutlet UIView *renderView;
@property (weak, nonatomic) IBOutlet UIView *depthView;
@property (weak, nonatomic) IBOutlet UITextField *tbOut;
@property (weak, nonatomic) IBOutlet UIButton *bProcessOne;
@property (weak, nonatomic) IBOutlet UIButton *bProcessCont;
@property (weak, nonatomic) IBOutlet UIView *bGreyRendering;
@property (weak, nonatomic) IBOutlet UIButton *bConfidenceRendering;
@property (weak, nonatomic) IBOutlet UIButton *bNormalsRendering;
@property (weak, nonatomic) IBOutlet UIButton *bFreeviewRendering;

@property (nonatomic, strong) CMMotionManager *motionManager;

- (IBAction)bProcessOne_clicked:(id)sender;
- (IBAction)bProcessCont_clicked:(id)sender;
- (IBAction)bGreyRenderingPressed:(id)sender;
- (IBAction)bConfidenceRenderingPressed:(id)sender;
- (IBAction)bNormalsRenderingPressed:(id)sender;
- (IBAction)bFreeviewRenderingPressed:(id)sender;
- (IBAction)bResetReconstructionClicked:(id)sender;
- (IBAction)pinchDetected:(id)sender;
- (IBAction)bSendMailClicked:(id)sender;
- (IBAction)bSavePhotoClicked:(id)sender;
- (IBAction)bSendModelClicked:(id)sender;

@end

