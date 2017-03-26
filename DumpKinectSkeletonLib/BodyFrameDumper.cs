using System;
using System.Globalization;
using System.IO;
using System.Linq;
using Microsoft.Kinect;


namespace DumpKinectSkeletonLib
{

    

    public class BodyFrameDumper
    {
        /// <summary>
        /// Body output file stream.
        /// </summary>
        private StreamWriter _bodyOutputStream;

        public static int[] hand_joint_numbers = new int[] { 7, 11 };

        /// <summary>
        /// Array buffer for tracked bodies.
        /// </summary>
        private Body[] _bodies;

        /// <summary>
        /// Number of currently tracked bodies.
        /// </summary>
        public int BodyCount { get; private set; }

        public TimeSpan InitialTime;

        /// <summary>
        /// Create a new body frame dumper that dumps first tracked Body data to a csv file.
        /// </summary>
        /// <param name="kinectSource"></param>
        /// <param name="outputFileName"></param>
        public BodyFrameDumper( KinectSource kinectSource, string outputFileName )
        {
            // open file for output
            try
            {
                _bodyOutputStream = new StreamWriter( new BufferedStream( new FileStream( outputFileName, FileMode.Create ) ) );

                // write header
                _bodyOutputStream.WriteLine(
                    //"# timestamp, jointType, position.X, position.Y, position.Z, orientation.X, orientation.Y, orientation.Z, orientation.W, pitch, yaw, roll, joint.tracking.state, lefthand.state, lefthand.conf, righthand.state, righthand.conf");
                    "# timestamp, jointType, position.X, position.Y, position.Z, orientation.X, orientation.Y, orientation.Z, orientation.W, joint.tracking.state, lefthand.state, lefthand.conf, righthand.state, righthand.conf");
            }
            catch ( Exception e )
            {
                Console.Error.WriteLine( "Error opening output file: " + e.Message );
                Close();
                throw;
            }
            kinectSource.BodyFrameEvent += HandleBodyFrame;
            kinectSource.FirstFrameRelativeTimeEvent += ts => InitialTime = ts;
        }

        /// <summary>
        /// Close subjacent output streams
        /// </summary>
        public void Close()
        {
            BodyCount = 0;
            _bodyOutputStream?.Close();
            _bodyOutputStream = null;
        }

        /// <summary>
        /// Handle a BodyFrame. Dumps first tracked Body to output file.
        /// </summary>
        /// <param name="frame"></param>
        public void HandleBodyFrame( BodyFrame frame )
        {
            // throw an error is dumper has been closed or output stream could not be opened or written to.
            if ( _bodyOutputStream == null )
            {
                throw new InvalidOperationException( "BodyFrameDumper is closed." );
            }
            var time = frame.RelativeTime;

            // lazy body buffer initialization
            if ( _bodies == null )
            {
                _bodies = new Body[ frame.BodyCount ];
            }
            
            // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
            // As long as those body objects are not disposed and not set to null in the array,
            // those body objects will be re-used.
            frame.GetAndRefreshBodyData( _bodies );

            // read the tracking state of bodies
            BodyCount = 0;
            Body firstBody = null;
            foreach ( var body in _bodies.Where( body => body.IsTracked ) )
            {
                BodyCount++;
                if ( firstBody == null )
                {
                    firstBody = body;
                }
            }

            // dump first tracked body
            if ( BodyCount > 0 )
            {
                try
                {
                    OutputBody( time - InitialTime, firstBody );
                }
                catch ( Exception e )
                {
                    Console.Error.WriteLine( "Error writing to output file(s): " + e.Message );
                    Close();
                    throw;
                }
            }
        }

        /// <summary>
        /// Output skeleton data to output file as [# timestamp, jointType, position.X, position.Y, position.Z, orientation.X, orientation.Y, orientation.Z, orientation.W, pitch, yaw, roll, joint.tracking.state, lefthand.state, lefthand.conf, righthand.state, righthand.conf].
        /// </summary>
        /// <param name="timestamp"></param>
        /// <param name="body"></param>
        private void OutputBody( TimeSpan timestamp, Body body )
        {

            //remnant from when I was outputting strings in addition to numbers
            //string LHState = "", RHState = "";
            int LHC = (int)body.HandLeftConfidence, RHC = (int)body.HandRightConfidence, LHState = (int)body.HandLeftState, RHState = (int)body.HandRightState;


            //remnant from when I was outputting strings in addition to numbers
            //switch (body.HandLeftState)
            //{
            //    case HandState.Open:
            //        LHState = "open";
            //        break;
            //    case HandState.Closed:
            //        LHState = "closed";
            //        break;
            //    case HandState.Lasso:
            //        LHState = "lasso";
            //        break;
            //    case HandState.Unknown:
            //        LHState = "unk";
            //        break;
            //    case HandState.NotTracked:
            //        LHState = "nt";
            //        break;
            //    default:
            //        break;
            //}


            //remnant from when I was outputting strings in addition to numbers
            //switch (body.HandRightState)
            //{
            //    case HandState.Open:
            //        RHState = "open";
            //        break;
            //    case HandState.Closed:
            //        RHState = "closed";
            //        break;
            //    case HandState.Lasso:
            //        RHState = "lasso";
            //        break;
            //    case HandState.Unknown:
            //        RHState = "unk";
            //        break;
            //    case HandState.NotTracked:
            //        RHState = "nt";
            //        break;
            //    default:
            //        break;
            //}


            var joints = body.Joints;
            var orientations = body.JointOrientations;

            // see https://msdn.microsoft.com/en-us/library/microsoft.kinect.jointtype.aspx for jointType Description
            foreach ( var jointType in joints.Keys )
            {
                var position = joints[ jointType ].Position;
                var orientation = orientations[ jointType ].Orientation;

                int joint_number = (int)jointType;

                int Output_LHState = -1, Output_RHState = -1, Output_LHC = -1, Output_RHC = -1;

                //7 is left hand, 11 is right hand
                //we specified these in a static array (hand_joint_numbers) at the beginning so that we can
                //output these variables *only* when the line that we're outputting is relevant to these specific joints
                if (hand_joint_numbers.Contains(joint_number))
                {
                    Output_LHState = LHState;
                    Output_LHC = LHC;
               
                    Output_RHState = RHState;
                    Output_RHC = RHC;
                }



                _bodyOutputStream.WriteLine( string.Format(CultureInfo.InvariantCulture.NumberFormat,
                    //"{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}, {16}",
                    "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}",
                    timestamp.TotalMilliseconds,
                    joint_number,
                    position.X, position.Y, position.Z,
                    orientation.X, orientation.Y, orientation.Z, orientation.W,
                    //calculate pitch
                    //Math.Atan2(2 * ((orientation.Y * orientation.Z) + (orientation.W * orientation.X)), (orientation.W * orientation.W) - (orientation.X * orientation.X) - (orientation.Y * orientation.Y) + (orientation.Z * orientation.Z)) / Math.PI * 180.0,
                    //calculate yaw
                    //Math.Asin(2 * ((orientation.W * orientation.Y) - (orientation.X * orientation.Z))) / Math.PI * 180.0,
                    //calculate roll
                    //Math.Atan2(2 * ((orientation.X * orientation.Y) + (orientation.W * orientation.Z)), (orientation.W * orientation.W) + (orientation.X * orientation.X) - (orientation.Y * orientation.Y) - (orientation.Z * orientation.Z)) / Math.PI * 180.0,
                    (int)joints[ jointType ].TrackingState,
                    //output hand states / confidences
                    Output_LHState, Output_LHC, Output_RHState, Output_RHC))
                    
              
                ;
            }


            




        }
    }
}



//JOINT STATE

//Member Value   Description
//Inferred    1 	The joint data is inferred.Confidence in the position data is very low.
//NotTracked  0 	The joint data is not tracked and no data is known about this joint.
//Tracked 2 	The joint data is being tracked and the data can be trusted.


//HAND STATE

//Members
//Member  Value Description
//Closed	3 	The hand is closed.
//Lasso	4 	The hand is in the lasso state.
//NotTracked	1 	Hand state is not tracked.
//Open    2 	The hand is open.
//Unknown 0 	The state of the hand is unknown.


//JOINT NUMBERS

//    Member Value   Description
//AnkleLeft	14 	Left ankle
//AnkleRight  18 	Right ankle
//ElbowLeft   5 	Left elbow
//ElbowRight  9 	Right elbow
//FootLeft    15 	Left foot
//FootRight   19 	Right foot
//HandLeft    7 	Left hand
//HandRight   11 	Right hand
//HandTipLeft 21 	Tip of the left hand
//HandTipRight	23 	Tip of the right hand
//Head	3 	Head
//HipLeft	12 	Left hip
//HipRight    16 	Right hip
//KneeLeft    13 	Left knee
//KneeRight   17 	Right knee
//Neck    2 	Neck
//ShoulderLeft	4 	Left shoulder
//ShoulderRight   8 	Right shoulder
//SpineBase   0 	Base of the spine
//SpineMid    1 	Middle of the spine
//SpineShoulder   20 	Spine at the shoulder
//ThumbLeft   22 	Left thumb
//ThumbRight  24 	Right thumb
//WristLeft   6 	Left wrist
//WristRight  10 	Right wrist



//code snippet taken from here:
//https://social.msdn.microsoft.com/Forums/en-US/245a3a09-a2e4-4e0e-8c12-b8625102376a/kinect-v2-sdk-joint-orientation?forum=kinectv2sdk

///// <summary>
///// Converts rotation quaternion to Euler angles 
///// And then maps them to a specified range of values to control the refresh rate
///// </summary>
///// <param name="rotQuaternion">face rotation quaternion</param>
///// <param name="pitch">rotation about the X-axis</param>
///// <param name="yaw">rotation about the Y-axis</param>
///// <param name="roll">rotation about the Z-axis</param>
//private static void ExtractFaceRotationInDegrees(Vector4 rotQuaternion, out int pitch, out int yaw, out int roll)
//{
//    double x = rotQuaternion.X;
//    double y = rotQuaternion.Y;
//    double z = rotQuaternion.Z;
//    double w = rotQuaternion.W;

//    // convert face rotation quaternion to Euler angles in degrees
//    double yawD, pitchD, rollD;
//    pitchD = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
//    yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
//    rollD = Math.Atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;

//    // clamp the values to a multiple of the specified increment to control the refresh rate
//    double increment = FaceRotationIncrementInDegrees;
//    pitch = (int)(Math.Floor((pitchD + ((increment / 2.0) * (pitchD > 0 ? 1.0 : -1.0))) / increment) * increment);
//    yaw = (int)(Math.Floor((yawD + ((increment / 2.0) * (yawD > 0 ? 1.0 : -1.0))) / increment) * increment);
//    roll = (int)(Math.Floor((rollD + ((increment / 2.0) * (rollD > 0 ? 1.0 : -1.0))) / increment) * increment);
//}

