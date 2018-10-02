package application;


import java.io.IOException;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.flexfellow.FlexFellow;
import com.kuka.grippertoolbox.gripper.zimmer.ZimmerR840;
import static com.kuka.grippertoolbox.gripper.zimmer.ZimmerR840.*;

import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a
 * {@link RoboticsAPITask#run()} method, which will be called successively in
 * the application lifecycle. The application will terminate automatically after
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an
 * exception is thrown during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 *
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class UDPAutoCalibration extends RoboticsAPIApplication {
 @Inject
 private ZimmerR840 gripper;

 @Inject
 private LBR lBR_iiwa;

 @Inject
 private FlexFellow flexFELLOW_1;

 private UDPTestThread client = null; // thread
 byte[] receive_data = new byte[1024];
 byte[] send_data = new byte[1024];
 private String prevRecvStr = "";
 private String curRecvStr = "";
 private String endTag = "@l@";
 boolean done = false;

 boolean _IsRunning = true;
 private Tool _toolAttachedToLBR;
 private CartesianImpedanceControlMode modeSoftZ = null;
 //	private String frameName="/TCP/AR_marker";  			// old market aligned with Y
 private String frameName = "/TCP/AR_marker_X_Aligned"; // new marker aligned with X
 double cartVel = 50;


 double d = 100; // linear distance motion (mm) for auto calibration
 double a = Math.toRadians(10); // rotation angle motion (degree) for auto calibration

 public void closeLoop() {
  done = true;
 }
 @Override
 public void initialize() {
  // initialize your application here
  //lBR_iiwa = getContext().getDeviceFromType(LBR.class);
  _toolAttachedToLBR = getApplicationData().createFromTemplate("ZimmerR840EC02A01");
  _toolAttachedToLBR.attachTo(lBR_iiwa.getFlange());


  modeSoftZ = new CartesianImpedanceControlMode();
  modeSoftZ.parametrize(CartDOF.ROT).setStiffness(10);
  modeSoftZ.parametrize(CartDOF.TRANSL).setStiffness(10);


  IUserKeyBar socketKeyBar = getApplicationUI().createUserKeyBar("sockect_dispose");
  IUserKeyListener socketCloseListener = new IUserKeyListener() {
   @Override
   public void onKeyEvent(IUserKey socketCloseKey, UserKeyEvent event) {
    if (event == UserKeyEvent.KeyDown) {
     socketCloseKey.setText(UserKeyAlignment.MiddleLeft, "Close");
     socketCloseKey.setLED(UserKeyAlignment.BottomMiddle, UserKeyLED.Red, UserKeyLEDSize.Small);
     socketCloseKey.setLED(UserKeyAlignment.BottomLeft, UserKeyLED.Grey, UserKeyLEDSize.Small);
     done = true; // exit the while loop >> close the socket

    } else
     socketCloseKey.setText(UserKeyAlignment.MiddleLeft, "Open");
    socketCloseKey.setLED(UserKeyAlignment.BottomLeft, UserKeyLED.Green, UserKeyLEDSize.Small);
   }

  };

  IUserKey myKey = socketKeyBar.addUserKey(0, socketCloseListener, true);
  myKey.setLED(UserKeyAlignment.TopLeft, UserKeyLED.Green, UserKeyLEDSize.Small);
  socketKeyBar.publish();
 }

 @Override
 public void dispose() {
  _IsRunning = false;
  getLogger().info(" Closing Sockets in Dispose Block");
  if (client != null)
   client.kill();

  super.dispose();
 } // dispose

 public void processRecv() {
  String recvStr = client.getString(); // get data from client
  if (recvStr != prevRecvStr) {
   prevRecvStr = curRecvStr;
   curRecvStr = recvStr;
   getLogger().info("str received!: " + recvStr);
  }
 }

 @Override
 public void run() {
  //lBR_iiwa.move(ptpHome());
  try {
   client = new UDPTestThread(30002); //define the port number
   client.start(); // start the thread "UDPTestThread.java"
   getLogger().info("Client will never stop!");
   //			while (!client.hasReceived)		// wait for data to be received before proceeding with the rest of the code
   //				ThreadUtil.milliSleep(500);
   int counter = 0;
   processRecv();

   //client.sendMsg("initial sending test");
   //int Choice=99;
   getLogger().info("enter into while loop.");
   while (!done) {
    //getLogger().info(curRecvStr);
    if (curRecvStr.equals("esc")) {
     done = true;
     getLogger().info("esc received, app will exit.");
    }

    processRecv();
    //				if(counter>4000)
    //				{
    //					done=true;
    //					getLogger().info("counter maximum." );
    //				}
    //				counter++;
    String sendStr = new String();

    if (curRecvStr.contains("get_Cart_Position")) // wait for sending data command
    {
     getLogger().info("command: return cartesian coordinates of effector...");
     Frame goalFrame = new Frame();
     goalFrame = lBR_iiwa.getCurrentCartesianPosition(_toolAttachedToLBR.getFrame(frameName));
     sendStr = constructPositionStr(goalFrame);
     //System.out.printf("sendStr=%s\n" ,sendStr);
     if (client.sendMsg(sendStr)) {
      getLogger().info("cartesian coordinates sent!");
     } else {
      getLogger().info("Message sending failed");
     }
     prevRecvStr = curRecvStr;
     curRecvStr = "";
    }

    if (curRecvStr.contains("auto")) // wait for sending data command
    {
     getLogger().info(curRecvStr);

     getLogger().info("AutoCalibration...");

     Frame initFrame = new Frame();
     initFrame = lBR_iiwa.getCurrentCartesianPosition(_toolAttachedToLBR.getFrame(frameName));
     // //initFrame.setX(-360.).setY(470.).setZ(450.).setAlphaRad(Math.toRadians(-25.)).setBetaRad(Math.toRadians(0.)).setGammaRad(Math.toRadians(-175.));
     //initFrame.setX(-360.).setY(-470.).setZ(450.).setAlphaRad(Math.toRadians(-70.)).setBetaRad(Math.toRadians(0.)).setGammaRad(Math.toRadians(-175.)); //kinect 2
     initFrame.setX(-514.).setY(-421.).setZ(437.).setAlphaRad(Math.toRadians(-70.)).setBetaRad(Math.toRadians(9.)).setGammaRad(Math.toRadians(-175.)); //asus

     String actionName = curRecvStr.split(" ")[1];
     int actionNum = Integer.parseInt(actionName);

     switch (actionNum) {
      case 0:
       getLogger().info("move iiwa to a point 0 for calibration ...");
       /*
       IMotionContainer positionHoldContainer = lBR_iiwa.moveAsync((new PositionHold(modeSoftZ, -1, null)));
       getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to record the P0", "OK");
       //// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold.
       positionHoldContainer.cancel();
       */

       _toolAttachedToLBR.getFrame(frameName).move(lin(initFrame).setCartVelocity(cartVel));
       sendStr = constructPositionStr(initFrame);
       ThreadUtil.milliSleep(100);
       break;
       // Calibration box corner 1
      case 1:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(d, 0, 0, 0, 0, 0).setCartVelocity(cartVel));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       break;
      case 2:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, -a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
      case 3:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, -a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
       // Calibration box corner 2
      case 4:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, -d, 0, 0, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       break;
      case 5:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, -a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
      case 6:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, -a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
       // Calibration box corner 3
      case 7:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, d, 0, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       break;
      case 8:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, -a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
      case 9:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, -a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
       // Calibration box corner 4
      case 10:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(-d, 0, 0, 0, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       break;
      case 11:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, -a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
      case 12:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, -a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
       // Calibration box corner 5
      case 13:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, -d, 0, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       break;
      case 14:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, -a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
      case 15:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, d / 2, 0, 0, +a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, -a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
       // Calibration box corner 6
      case 16:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, d, d / 2, 0, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       break;
      case 17:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, -a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
      case 18:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, -a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, 0, 0, a).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;
       // Calibration box corner 7
      case 19:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, -d, 0, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       break;
      case 20:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, -a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       _toolAttachedToLBR.getFrame(frameName).move(linRel(0, 0, 0, a, 0, 0).setCartVelocity(cartVel).setOrientationVelocity(Math.toRadians(10)));
       break;


      case 21:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       //_toolAttachedToLBR.getFrame(frameName).move(ptp(getApplicationData().getFrame("/P1_calibration")).setJointVelocityRel(0.2));
       //_toolAttachedToLBR.getFrame(frameName).move(lin(getApplicationData().getFrame("/P1_calibration")).setCartVelocity(cartVel).setOrientationVelocity( Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       break;


      case 22:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       //_toolAttachedToLBR.getFrame(frameName).move(lin(getApplicationData().getFrame("/P2_calibration")).setCartVelocity(cartVel).setOrientationVelocity( Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       break;

      case 23:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       //_toolAttachedToLBR.getFrame(frameName).move(lin(getApplicationData().getFrame("/P3_calibration")).setCartVelocity(cartVel).setOrientationVelocity( Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);
       break;

      case 24:
       getLogger().info("moving to P" + actionName + "done! sendign position");
       //_toolAttachedToLBR.getFrame(frameName).move(lin(getApplicationData().getFrame("/P4_calibration")).setCartVelocity(cartVel).setOrientationVelocity( Math.toRadians(10)));
       sendStr = sendPosition(actionNum);
       System.out.printf("sendStr=%s\n", sendStr);
       ThreadUtil.milliSleep(500);

       _toolAttachedToLBR.getFrame(frameName).move(ptp(initFrame).setJointVelocityRel(0.2));
       break;

      default:
       throw new IllegalArgumentException("Invalid point # ");
     }



     if (client.sendMsg(sendStr)) {
      getLogger().info("cartesian coordinates sent!");
     } else {
      getLogger().info("Message sending failed");
     }

     prevRecvStr = curRecvStr;
     curRecvStr = "";
    }

    //Thread.sleep(200);
   }
   client.dispose();

  } catch (IOException e) {
   // TODO Auto-generated catch block
   e.printStackTrace();
   client.dispose();
  }
 }

 public String sendPosition(int j) {
  getLogger().info("moving to P" + String.valueOf(j) + "done! sendign position");
  Frame goalFrame = new Frame();
  goalFrame = lBR_iiwa.getCurrentCartesianPosition(_toolAttachedToLBR.getFrame(frameName));
  String sendStr = constructPositionStr(goalFrame);
  return sendStr;
 }


 public String constructPositionStr(Frame goalFrame) {
  String rtStr = new String();
  rtStr = Double.toString(goalFrame.getX());
  rtStr += " " + Double.toString(goalFrame.getY());
  rtStr += " " + Double.toString(goalFrame.getZ());
  Quaternion myQuart = tf_convertions.matrixToQuat(goalFrame.transformationFromWorld().getRotationMatrix());
  //		rtStr+=" "+Double.toString(goalFrame.getAlphaRad());
  //		rtStr+=" "+Double.toString(goalFrame.getBetaRad());
  //		rtStr+=" "+Double.toString(goalFrame.getGammaRad());
  rtStr += " " + Double.toString(myQuart.qx);
  rtStr += " " + Double.toString(myQuart.qy);
  rtStr += " " + Double.toString(myQuart.qz);
  rtStr += " " + Double.toString(myQuart.qw);

  return rtStr + endTag;
 }

 public String getMarkerId() {
  return "id1";
 }
}
